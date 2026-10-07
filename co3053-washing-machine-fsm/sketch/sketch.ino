/**
 * @file    sketch.ino
 * @brief   Coin-operated washing machine controller - ESP32 (Wokwi)
 *
 * Embedded Systems (CO3053) - Assignment 2 - Firmware (Member 2)
 *
 * Architecture: "Foreground / Background" (Super-loop + Timer Interrupt)
 *   - Background : hardware Timer ISR every 10 ms (time base)
 *       + counts down the 30-minute washing timer (1 s resolution)
 *       + toggles the blink phase every 500 ms (BLED / RLED blinking)
 *       + raises flags only - no Serial, no I2C, no delay() inside the ISR
 *   - Foreground : super-loop in loop()
 *       1. scan & debounce the 7 buttons on every 10 ms tick -> events
 *       2. convert the "timer reached 0" flag into EV_TIMER_DONE
 *       3. dispatch every queued event to the switch-case FSM (wm_fsm.c)
 *       4. drive the LEDs from the Moore outputs of the current state
 *       5. refresh the LCD / Serial log when something changed
 *
 * delay() is NOT used anywhere in the application code.
 *
 * Wiring (all buttons: one leg to GPIO, the other to GND, INPUT_PULLUP)
 *   RLED  GPIO25   BLED   GPIO26
 *   RUN   GPIO32   PAUSE  GPIO33   STOP  GPIO27   FAULT GPIO14
 *   10c   GPIO4    20c    GPIO16   50c   GPIO17
 *   LCD1602 I2C (0x27): SDA GPIO21, SCL GPIO22
 */
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "wm_fsm.h"

#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

/* ========================================================================== */
/* Configuration                                                              */
/* ========================================================================== */
/* TIME_SCALE = 1  -> real time: the cycle lasts exactly 30 minutes.
 * TIME_SCALE = 60 -> demo mode: 1 simulated minute per real second, so the
 *                    whole 30-minute cycle can be observed in 30 seconds.
 *                    (any integer 1..100 is accepted)                       */
#define TIME_SCALE          1

#define TICK_US             10000UL               /* ISR period: 10 ms     */
#define TICK_MS             (TICK_US / 1000UL)
#define BLINK_HALF_TICKS    50UL                  /* 500 ms ON / 500 ms OFF */
#define DEBOUNCE_MASK       0x07u                 /* 3 stable samples=30ms */

/* Pins */
#define PIN_RLED       25
#define PIN_BLED       26
#define PIN_BTN_RUN    32
#define PIN_BTN_PAUSE  33
#define PIN_BTN_STOP   27
#define PIN_BTN_FAULT  14
#define PIN_BTN_C10     4
#define PIN_BTN_C20    16
#define PIN_BTN_C50    17

/* ========================================================================== */
/* Shared data between ISR and super-loop                                     */
/* ========================================================================== */
static hw_timer_t   *s_hwTimer = NULL;
static portMUX_TYPE  s_timerMux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t g_tickCount     = 0;     /* 10 ms ticks since boot  */
static volatile bool     g_tickFlag      = false; /* new tick -> scan inputs */
static volatile bool     g_blinkPhase    = false; /* toggles every 500 ms    */
static volatile bool     g_washActive    = false; /* countdown running       */
static volatile uint32_t g_washRemaining = 0;     /* Timer (seconds)         */
static volatile uint32_t g_washSubMs     = 0;     /* ms inside 1 wash second */
static volatile bool     g_timerDoneFlag = false; /* Timer reached 0         */
static volatile bool     g_secondFlag    = false; /* refresh display         */

/* ========================================================================== */
/* Timer ISR - every 10 ms                                                    */
/* ========================================================================== */
void IRAM_ATTR onTimerISR()
{
    static uint32_t blinkCnt = 0;

    portENTER_CRITICAL_ISR(&s_timerMux);

    g_tickCount = g_tickCount + 1;
    g_tickFlag = true;

    /* 1) Blink time base: 500 ms half-period */
    if (++blinkCnt >= BLINK_HALF_TICKS) {
        blinkCnt = 0;
        g_blinkPhase = !g_blinkPhase;
    }

    /* 2) 30-minute countdown (keeps running in RUNNING *and* PAUSED) */
    if (g_washActive) {
        g_washSubMs = g_washSubMs + TICK_MS * TIME_SCALE;  /* sim. ms */
        while (g_washSubMs >= 1000UL && g_washRemaining > 0) {
            g_washSubMs = g_washSubMs - 1000UL;
            g_washRemaining = g_washRemaining - 1;     /* Timer -= 1 s */
            g_secondFlag = true;
        }
        if (g_washRemaining == 0) {
            g_washActive    = false;
            g_timerDoneFlag = true;               /* -> EV_TIMER_DONE      */
        }
    }

    portEXIT_CRITICAL_ISR(&s_timerMux);
}

/* ========================================================================== */
/* FSM hardware port: timer actions requested by the FSM                      */
/* ========================================================================== */
static void port_timer_start(uint32_t seconds)
{
    portENTER_CRITICAL(&s_timerMux);
    g_washRemaining = seconds;
    g_washSubMs     = 0;
    g_timerDoneFlag = false;
    g_washActive    = true;
    portEXIT_CRITICAL(&s_timerMux);
}

static void port_timer_clear(void)
{
    portENTER_CRITICAL(&s_timerMux);
    g_washActive    = false;
    g_washRemaining = 0;
    g_washSubMs     = 0;
    g_timerDoneFlag = false;
    portEXIT_CRITICAL(&s_timerMux);
}

static uint32_t port_timer_remaining(void)
{
    portENTER_CRITICAL(&s_timerMux);
    uint32_t r = g_washRemaining;
    portEXIT_CRITICAL(&s_timerMux);
    return r;
}

static const wm_fsm_port_t s_port = {
    port_timer_start,
    port_timer_clear,
    port_timer_remaining
};

static wm_fsm_t s_fsm;

/* ========================================================================== */
/* Event queue (only used by the super-loop, no ISR access)                   */
/* ========================================================================== */
#define EVQ_SIZE 16
static wm_fsm_event_t s_evq[EVQ_SIZE];
static uint8_t        s_evqHead = 0, s_evqTail = 0;

static void evq_push(wm_fsm_event_id_t id, uint16_t value)
{
    uint8_t next = (uint8_t)((s_evqHead + 1u) % EVQ_SIZE);
    if (next == s_evqTail) return;                /* full -> drop event    */
    s_evq[s_evqHead].id    = id;
    s_evq[s_evqHead].value = value;
    s_evqHead = next;
}

static bool evq_pop(wm_fsm_event_t *ev)
{
    if (s_evqTail == s_evqHead) return false;
    *ev = s_evq[s_evqTail];
    s_evqTail = (uint8_t)((s_evqTail + 1u) % EVQ_SIZE);
    return true;
}

/* ========================================================================== */
/* Buttons: sampled every 10 ms, shift-register debounce, falling-edge event  */
/* ========================================================================== */
typedef struct {
    uint8_t           pin;
    wm_fsm_event_id_t event;
    uint16_t          value;      /* coin value for coin buttons          */
    const char       *label;
    uint8_t           history;    /* last 8 samples, 1 = pressed          */
    bool              pressed;    /* debounced state                      */
} button_t;

static button_t s_buttons[] = {
    { PIN_BTN_RUN,   EV_RUN,   0,  "RUN",   0, false },
    { PIN_BTN_PAUSE, EV_PAUSE, 0,  "PAUSE", 0, false },
    { PIN_BTN_STOP,  EV_STOP,  0,  "STOP",  0, false },
    { PIN_BTN_FAULT, EV_FAULT, 0,  "FAULT", 0, false },
    { PIN_BTN_C10,   EV_COIN,  10, "10c",   0, false },
    { PIN_BTN_C20,   EV_COIN,  20, "20c",   0, false },
    { PIN_BTN_C50,   EV_COIN,  50, "50c",   0, false },
};
#define NUM_BUTTONS (sizeof(s_buttons) / sizeof(s_buttons[0]))

static void scan_buttons(void)
{
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        button_t *b = &s_buttons[i];
        uint8_t raw = (digitalRead(b->pin) == LOW) ? 1u : 0u;   /* active-low */
        b->history = (uint8_t)((b->history << 1) | raw);

        if (!b->pressed && (b->history & DEBOUNCE_MASK) == DEBOUNCE_MASK) {
            b->pressed = true;                    /* stable press -> event */
            evq_push(b->event, b->value);
        } else if (b->pressed && (b->history & DEBOUNCE_MASK) == 0u) {
            b->pressed = false;                   /* stable release        */
        }
    }
}

/* ========================================================================== */
/* Outputs                                                                    */
/* ========================================================================== */
static void drive_led(uint8_t pin, led_mode_t mode, bool phase)
{
    bool on = (mode == LED_ON) || (mode == LED_BLINK && phase);
    digitalWrite(pin, on ? HIGH : LOW);
}

static void update_leds(void)
{
    wm_fsm_outputs_t out = wm_fsm_outputs(s_fsm.state);
    bool phase = g_blinkPhase;
    drive_led(PIN_RLED, out.rled, phase);
    drive_led(PIN_BLED, out.bled, phase);
}

LiquidCrystal_I2C lcd(0x27, 16, 2);
static char s_lcdLine[2][17];

static void lcd_print_line(uint8_t row, const char *text)
{
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16.16s", text);         /* pad / clip 16  */
    if (strcmp(s_lcdLine[row], buf) == 0) return;          /* unchanged      */
    strcpy(s_lcdLine[row], buf);
    lcd.setCursor(0, row);
    lcd.print(s_lcdLine[row]);
}

static void update_display(void)
{
    char l0[32], l1[32];
    uint32_t t = port_timer_remaining();

    snprintf(l0, sizeof(l0), "%-7s Coin:%3u", wm_fsm_state_name(s_fsm.state),
             (unsigned)s_fsm.coin_total);
    if (s_fsm.state == ST_RUNNING || s_fsm.state == ST_PAUSED ||
        (s_fsm.state == ST_ERROR && t > 0)) {
        snprintf(l1, sizeof(l1), "T %02u:%02u  STOP:%u",
                 (unsigned)(t / 60), (unsigned)(t % 60), (unsigned)s_fsm.stop_count);
    } else {
        snprintf(l1, sizeof(l1), "T --:--  STOP:%u", (unsigned)s_fsm.stop_count);
    }
    lcd_print_line(0, l0);
    lcd_print_line(1, l1);
}

static void log_transition(wm_fsm_state_t from, wm_fsm_event_t ev, int rule)
{
    uint32_t ms = g_tickCount * TICK_MS;
    Serial.printf("[%5lu.%02lus] %-7s --%s",
                  (unsigned long)(ms / 1000), (unsigned long)((ms % 1000) / 10),
                  wm_fsm_state_name(from), wm_fsm_event_name(ev.id));
    if (ev.id == EV_COIN) Serial.printf("(%u)", ev.value);
    if (rule == WM_FSM_IGNORED) {
        Serial.printf("--> (ignored)\n");
        return;
    }
    Serial.printf("--> %-7s #%-2d coin=%u stop=%u T=%lus\n",
                  wm_fsm_state_name(s_fsm.state), rule, s_fsm.coin_total,
                  s_fsm.stop_count, (unsigned long)port_timer_remaining());
}

/* ========================================================================== */
/* Hardware timer setup (supports ESP32 Arduino core 2.x and 3.x)             */
/* ========================================================================== */
static void timer_init(void)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    s_hwTimer = timerBegin(1000000);              /* 1 MHz -> 1 us / count */
    timerAttachInterrupt(s_hwTimer, &onTimerISR);
    timerAlarm(s_hwTimer, TICK_US, true, 0);      /* 10 ms, auto-reload    */
#else
    s_hwTimer = timerBegin(0, 80, true);          /* 80 MHz / 80 = 1 MHz   */
    timerAttachInterrupt(s_hwTimer, &onTimerISR, true);
    timerAlarmWrite(s_hwTimer, TICK_US, true);
    timerAlarmEnable(s_hwTimer);
#endif
}

/* ========================================================================== */
/* setup() / loop()                                                           */
/* ========================================================================== */
void setup()
{
    Serial.begin(115200);

    pinMode(PIN_RLED, OUTPUT);
    pinMode(PIN_BLED, OUTPUT);
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) pinMode(s_buttons[i].pin, INPUT_PULLUP);

    lcd.init();
    lcd.backlight();

    wm_fsm_init(&s_fsm, &s_port);                 /* #0 -> STANDBY         */
    timer_init();

    Serial.println();
    Serial.println("=== Coin-operated Washing Machine (CO3053 - Assignment 2) ===");
    Serial.printf("Wash time: %u s, TIME_SCALE = %d\n", WM_FSM_WASH_TIME_S, TIME_SCALE);
    Serial.println("Buttons: 10c 20c 50c | RUN PAUSE STOP | FAULT");
    Serial.println("[    0.00s] (Initial) --reset--> STANDBY #0");
    update_leds();
    update_display();
}

void loop()
{
    bool changed = false;

    /* 1) Input scanning on every 10 ms tick from the ISR */
    if (g_tickFlag) {
        g_tickFlag = false;
        scan_buttons();
    }

    /* 2) Timer reached 0 -> generate the "Timer Done" event */
    if (g_timerDoneFlag) {
        portENTER_CRITICAL(&s_timerMux);
        g_timerDoneFlag = false;
        portEXIT_CRITICAL(&s_timerMux);
        evq_push(EV_TIMER_DONE, 0);
    }

    /* 3) Run-to-completion dispatch of all pending events */
    wm_fsm_event_t ev;
    while (evq_pop(&ev)) {
        wm_fsm_state_t from = s_fsm.state;
        int rule = wm_fsm_dispatch(&s_fsm, ev);
        log_transition(from, ev, rule);
        if (rule != WM_FSM_IGNORED) changed = true;
    }

    /* 4) Moore outputs: LEDs depend only on the current state (+ blink phase) */
    update_leds();

    /* 5) Display refresh only when something changed (non-blocking budget) */
    if (g_secondFlag) {
        g_secondFlag = false;
        changed = true;
    }
    if (changed) update_display();
}
