/**
 * @file    wm_fsm.h
 * @brief   Coin-operated washing machine - Finite State Machine (portable C).
 *
 * Embedded Systems (CO3053) - Assignment 2
 *
 * The FSM is hardware-independent: every side effect that touches the
 * hardware (the 30-minute timer) goes through the wm_fsm_port_t callbacks.
 * The same file is compiled for the ESP32 (Wokwi) and for the host PC
 * unit tests (test/test_fsm.c).
 *
 * Every transition returns the row number (#) of the State-transition
 * Table in the report (rows 0..19), so the firmware and the diagram
 * can be cross-checked one-to-one.
 */
#ifndef WM_FSM_H
#define WM_FSM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */
#define WM_FSM_MIN_COIN_TOTAL   50u            /* cents needed to wash      */
#define WM_FSM_WASH_TIME_S      (30u * 60u)    /* 30-minute washing cycle   */

/* -------------------------------------------------------------------------- */
/* States                                                                     */
/* -------------------------------------------------------------------------- */
typedef enum {
    ST_STANDBY = 0,     /* RLED ON,       BLED OFF      */
    ST_READY,           /* RLED OFF,      BLED ON       */
    ST_RUNNING,         /* RLED OFF,      BLED BLINKING */
    ST_PAUSED,          /* RLED OFF,      BLED ON       */
    ST_ERROR,           /* RLED BLINKING, BLED OFF      */
    ST_COUNT
} wm_fsm_state_t;

/* -------------------------------------------------------------------------- */
/* Events                                                                     */
/* -------------------------------------------------------------------------- */
typedef enum {
    EV_NONE = 0,
    EV_COIN,            /* Insert Coins   (value = 10 / 20 / 50 cents)     */
    EV_RUN,             /* Press RUN                                       */
    EV_PAUSE,           /* Press PAUSE                                     */
    EV_STOP,            /* Press STOP                                      */
    EV_FAULT,           /* Press FAULT_BUTTON                              */
    EV_TIMER_DONE,      /* Timer Done (generated when Timer reaches 0)     */
    EV_COUNT
} wm_fsm_event_id_t;

typedef struct {
    wm_fsm_event_id_t id;
    uint16_t          value;   /* coin value (cents) for EV_COIN, else 0 */
} wm_fsm_event_t;

/* -------------------------------------------------------------------------- */
/* LED output pattern (Moore outputs)                                         */
/* -------------------------------------------------------------------------- */
typedef enum { LED_OFF = 0, LED_ON, LED_BLINK } led_mode_t;

typedef struct {
    led_mode_t rled;
    led_mode_t bled;
} wm_fsm_outputs_t;

/* -------------------------------------------------------------------------- */
/* Hardware port (dependency injection)                                       */
/* -------------------------------------------------------------------------- */
typedef struct {
    void     (*timer_start)(uint32_t seconds);  /* load & start countdown   */
    void     (*timer_clear)(void);              /* stop & clear countdown   */
    uint32_t (*timer_remaining)(void);          /* remaining seconds        */
} wm_fsm_port_t;

/* -------------------------------------------------------------------------- */
/* FSM context (extended state variables)                                     */
/* -------------------------------------------------------------------------- */
typedef struct {
    wm_fsm_state_t       state;
    uint16_t             coin_total;   /* cents inserted in this transaction */
    uint8_t              stop_count;   /* STOP presses during a cycle        */
    const wm_fsm_port_t *port;
} wm_fsm_t;

#define WM_FSM_IGNORED   (-1)   /* return value when an event is ignored   */

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */
/** Initial transition (row #0): -> STANDBY. Returns 0. */
int  wm_fsm_init(wm_fsm_t *fsm, const wm_fsm_port_t *port);

/**
 * Dispatch one event to the FSM (run-to-completion).
 * @return row number (1..19) of the transition that fired,
 *         or WM_FSM_IGNORED if the event is not handled in the current state.
 */
int  wm_fsm_dispatch(wm_fsm_t *fsm, wm_fsm_event_t ev);

/** Moore output function: LED pattern of a given state. */
wm_fsm_outputs_t wm_fsm_outputs(wm_fsm_state_t state);

/** Helpers for logging. */
const char *wm_fsm_state_name(wm_fsm_state_t state);
const char *wm_fsm_event_name(wm_fsm_event_id_t ev);
bool        wm_fsm_is_valid_coin(uint16_t cents);

#ifdef __cplusplus
}
#endif

#endif /* WM_FSM_H */
