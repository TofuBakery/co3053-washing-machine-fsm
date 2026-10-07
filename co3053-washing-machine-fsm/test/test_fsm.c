/**
 * @file    test_fsm.c
 * @brief   Host-side unit tests for the washing machine FSM (wm_fsm.c).
 *
 * Build & run:   make -C test        (or: gcc -std=c99 -Wall -Wextra
 *                -I../sketch ../sketch/wm_fsm.c test_fsm.c -o test_fsm)
 *
 * Part A: one test per row of the State-transition Table (#0..#19)
 *         -> proves 100 % transition coverage of the firmware.
 * Part B: full State x Event matrix -> every unlisted event is ignored.
 * Part C: end-to-end scenarios TC-01..TC-07 from the report, run against
 *         a simulated 1-second timer (same semantics as the ESP32 ISR).
 */
#include <stdio.h>
#include <string.h>
#include "wm_fsm.h"

/* -------------------------------------------------------------------------- */
/* Mock timer port (mirrors the ISR countdown on the ESP32)                   */
/* -------------------------------------------------------------------------- */
static uint32_t mock_remaining = 0;
static int      mock_active    = 0;
static int      mock_starts    = 0;
static int      mock_clears    = 0;

static void     m_start(uint32_t s) { mock_remaining = s; mock_active = 1; mock_starts++; }
static void     m_clear(void)       { mock_remaining = 0; mock_active = 0; mock_clears++; }
static uint32_t m_remaining(void)   { return mock_remaining; }

static const wm_fsm_port_t PORT = { m_start, m_clear, m_remaining };

/* Advance simulated time; returns 1 if the timer expired (ISR flag). */
static int mock_tick_seconds(uint32_t s)
{
    while (s-- && mock_active) {
        if (mock_remaining > 0) mock_remaining--;
        if (mock_remaining == 0) { mock_active = 0; return 1; }
    }
    return 0;
}

/* -------------------------------------------------------------------------- */
/* Tiny test framework                                                        */
/* -------------------------------------------------------------------------- */
static int g_pass = 0, g_fail = 0;
static int g_rule_hit[20];

#define CHECK(cond, ...) do {                                        \
        if (cond) { g_pass++; }                                      \
        else { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
               printf(__VA_ARGS__); printf("\n"); }                  \
    } while (0)

static wm_fsm_event_t E(wm_fsm_event_id_t id)            { wm_fsm_event_t e = { id, 0 }; return e; }
static wm_fsm_event_t COIN(uint16_t v)                   { wm_fsm_event_t e = { EV_COIN, v }; return e; }

static int send(wm_fsm_t *f, wm_fsm_event_t ev)
{
    int r = wm_fsm_dispatch(f, ev);
    if (r >= 0 && r < 20) g_rule_hit[r]++;
    return r;
}

static void reset(wm_fsm_t *f)
{
    mock_remaining = 0; mock_active = 0; mock_starts = 0; mock_clears = 0;
    int r = wm_fsm_init(f, &PORT);
    if (r == 0) g_rule_hit[0]++;
}

/* Helpers to drive the FSM into a given state through legal transitions */
static void goto_ready(wm_fsm_t *f)   { reset(f); send(f, COIN(50)); }
static void goto_running(wm_fsm_t *f) { goto_ready(f); send(f, E(EV_RUN)); }
static void goto_paused(wm_fsm_t *f)  { goto_running(f); send(f, E(EV_PAUSE)); }
static void goto_error(wm_fsm_t *f)   { reset(f); send(f, E(EV_FAULT)); }

/* ========================================================================== */
/* Part A - one test per transition row                                       */
/* ========================================================================== */
static void part_a(void)
{
    wm_fsm_t f; int r;
    printf("Part A: transition table rows #0..#19\n");

    reset(&f);                                           /* #0 */
    CHECK(f.state == ST_STANDBY && f.coin_total == 0, "#0 initial");

    reset(&f); r = send(&f, COIN(20));                   /* #1 */
    CHECK(r == 1 && f.state == ST_STANDBY && f.coin_total == 20, "#1");

    reset(&f); send(&f, COIN(20)); r = send(&f, COIN(20));
    CHECK(r == 1 && f.coin_total == 40, "#1 (accumulate)");
    r = send(&f, COIN(10));                              /* #2 exact 50 */
    CHECK(r == 2 && f.state == ST_READY && f.coin_total == 50, "#2");

    reset(&f); r = send(&f, E(EV_FAULT));                /* #3 */
    CHECK(r == 3 && f.state == ST_ERROR, "#3");

    goto_ready(&f); r = send(&f, COIN(20));              /* #4 */
    CHECK(r == 4 && f.state == ST_READY && f.coin_total == 70, "#4");

    goto_ready(&f); send(&f, COIN(10)); r = send(&f, E(EV_RUN));   /* #5 */
    CHECK(r == 5 && f.state == ST_RUNNING, "#5 state");
    CHECK(f.coin_total == 0 && f.stop_count == 0, "#5 coin_total reset, no change returned");
    CHECK(mock_active && mock_remaining == WM_FSM_WASH_TIME_S && mock_starts == 1, "#5 timer 30 min");

    goto_ready(&f); r = send(&f, E(EV_STOP));            /* #6 */
    CHECK(r == 6 && f.state == ST_STANDBY && f.coin_total == 0, "#6");

    goto_ready(&f); r = send(&f, E(EV_FAULT));           /* #7 */
    CHECK(r == 7 && f.state == ST_ERROR, "#7");

    goto_running(&f); r = send(&f, E(EV_STOP));          /* #8 */
    CHECK(r == 8 && f.state == ST_RUNNING && f.stop_count == 1 && mock_active, "#8");
    r = send(&f, E(EV_STOP));                            /* #9 */
    CHECK(r == 9 && f.state == ST_STANDBY && !mock_active && mock_remaining == 0, "#9");

    goto_running(&f); mock_tick_seconds(WM_FSM_WASH_TIME_S);
    r = send(&f, E(EV_TIMER_DONE));                      /* #10 */
    CHECK(r == 10 && f.state == ST_STANDBY, "#10");

    goto_running(&f); send(&f, E(EV_STOP)); r = send(&f, E(EV_PAUSE));  /* #11 */
    CHECK(r == 11 && f.state == ST_PAUSED && f.stop_count == 0, "#11 (stop_count reset)");
    CHECK(mock_active && mock_remaining == WM_FSM_WASH_TIME_S, "#11 timer untouched");

    goto_paused(&f); send(&f, E(EV_STOP)); r = send(&f, E(EV_RUN));     /* #13 */
    CHECK(r == 13 && f.state == ST_RUNNING && f.stop_count == 0, "#13");

    goto_running(&f); r = send(&f, E(EV_FAULT));         /* #12 */
    CHECK(r == 12 && f.state == ST_ERROR, "#12");

    goto_paused(&f); r = send(&f, E(EV_STOP));           /* #14 */
    CHECK(r == 14 && f.state == ST_PAUSED && f.stop_count == 1, "#14");
    r = send(&f, E(EV_STOP));                            /* #15 */
    CHECK(r == 15 && f.state == ST_STANDBY && !mock_active, "#15");

    goto_paused(&f); mock_tick_seconds(WM_FSM_WASH_TIME_S);
    r = send(&f, E(EV_TIMER_DONE));                      /* #16 */
    CHECK(r == 16 && f.state == ST_STANDBY, "#16");

    goto_paused(&f); r = send(&f, E(EV_FAULT));          /* #17 */
    CHECK(r == 17 && f.state == ST_ERROR, "#17");

    goto_error(&f); r = send(&f, E(EV_FAULT));           /* #18 */
    CHECK(r == 18 && f.state == ST_ERROR, "#18");

    goto_running(&f); send(&f, E(EV_FAULT));             /* ERROR with timer */
    r = send(&f, E(EV_STOP));                            /* #19 */
    CHECK(r == 19 && f.state == ST_STANDBY && f.coin_total == 0 && !mock_active
          && mock_remaining == 0 && f.stop_count == 0, "#19");

    goto_ready(&f); send(&f, E(EV_FAULT)); r = send(&f, E(EV_STOP));    /* #19 coins */
    CHECK(r == 19 && f.coin_total == 0, "#19 coin_total reset");
}

/* ========================================================================== */
/* Part B - State x Event matrix: expected rule (or ignored)                  */
/* ========================================================================== */
static void part_b(void)
{
    /* rows: state, columns: COIN(10) RUN PAUSE STOP FAULT TIMER_DONE(T>0) */
    static const int expect[ST_COUNT][6] = {
        /* STANDBY */ {  1, -1, -1, -1,  3, -1 },
        /* READY   */ {  4,  5, -1,  6,  7, -1 },
        /* RUNNING */ { -1, -1, 11,  8, 12, -1 },
        /* PAUSED  */ { -1, 13, -1, 14, 17, -1 },
        /* ERROR   */ { -1, -1, -1, 19, 18, -1 },
    };
    static const wm_fsm_event_id_t evs[6] = { EV_COIN, EV_RUN, EV_PAUSE, EV_STOP, EV_FAULT, EV_TIMER_DONE };
    void (*go[ST_COUNT])(wm_fsm_t *) = { reset, goto_ready, goto_running, goto_paused, goto_error };

    printf("Part B: state x event matrix (ignore rule)\n");
    for (int s = 0; s < ST_COUNT; s++) {
        for (int e = 0; e < 6; e++) {
            wm_fsm_t f; go[s](&f);
            wm_fsm_t before = f;
            wm_fsm_event_t ev = { evs[e], (uint16_t)(evs[e] == EV_COIN ? 10 : 0) };
            int r = wm_fsm_dispatch(&f, ev);
            CHECK(r == expect[s][e], "state %s event %s: got %d expected %d",
                  wm_fsm_state_name((wm_fsm_state_t)s), wm_fsm_event_name(evs[e]), r, expect[s][e]);
            if (expect[s][e] == WM_FSM_IGNORED) {
                CHECK(f.state == before.state && f.coin_total == before.coin_total &&
                      f.stop_count == before.stop_count,
                      "ignored event changed the context (%s/%s)",
                      wm_fsm_state_name((wm_fsm_state_t)s), wm_fsm_event_name(evs[e]));
            }
        }
    }

    /* Invalid coin values are rejected in every state */
    wm_fsm_t f; reset(&f);
    CHECK(wm_fsm_dispatch(&f, COIN(5)) == WM_FSM_IGNORED && f.coin_total == 0, "5c rejected");
    CHECK(wm_fsm_dispatch(&f, COIN(100)) == WM_FSM_IGNORED && f.coin_total == 0, "100c rejected");

    /* Moore outputs */
    CHECK(wm_fsm_outputs(ST_STANDBY).rled == LED_ON    && wm_fsm_outputs(ST_STANDBY).bled == LED_OFF,   "out STANDBY");
    CHECK(wm_fsm_outputs(ST_READY).rled   == LED_OFF   && wm_fsm_outputs(ST_READY).bled   == LED_ON,    "out READY");
    CHECK(wm_fsm_outputs(ST_RUNNING).rled == LED_OFF   && wm_fsm_outputs(ST_RUNNING).bled == LED_BLINK, "out RUNNING");
    CHECK(wm_fsm_outputs(ST_PAUSED).rled  == LED_OFF   && wm_fsm_outputs(ST_PAUSED).bled  == LED_ON,    "out PAUSED");
    CHECK(wm_fsm_outputs(ST_ERROR).rled   == LED_BLINK && wm_fsm_outputs(ST_ERROR).bled   == LED_OFF,   "out ERROR");
}

/* ========================================================================== */
/* Part C - report scenarios TC-01..TC-07                                     */
/* ========================================================================== */
static void part_c(void)
{
    wm_fsm_t f;
    printf("Part C: scenarios TC-01..TC-07\n");

    /* TC-01 Insufficient funds */
    reset(&f); send(&f, COIN(10)); send(&f, COIN(20));
    CHECK(send(&f, E(EV_RUN)) == WM_FSM_IGNORED && f.state == ST_STANDBY && f.coin_total == 30, "TC-01");

    /* TC-02 Normal start: 10+20+20 = 50 */
    reset(&f); send(&f, COIN(10)); send(&f, COIN(20)); send(&f, COIN(20));
    CHECK(f.state == ST_READY && wm_fsm_outputs(f.state).bled == LED_ON, "TC-02 READY, BLED on");
    send(&f, E(EV_RUN));
    CHECK(f.state == ST_RUNNING && f.coin_total == 0 && mock_remaining == 1800, "TC-02 RUNNING");

    /* TC-03 Pause keeps the timer counting */
    send(&f, E(EV_PAUSE)); mock_tick_seconds(5 * 60);
    CHECK(f.state == ST_PAUSED && mock_remaining == 1800 - 300, "TC-03 timer counts in PAUSED");
    send(&f, E(EV_RUN));
    CHECK(f.state == ST_RUNNING && mock_remaining == 1500, "TC-03 resume, timer not restarted");

    /* TC-04 Force stop needs exactly two presses */
    goto_running(&f); mock_tick_seconds(60);
    send(&f, E(EV_STOP));
    CHECK(f.state == ST_RUNNING && mock_remaining == 1740, "TC-04 1st STOP keeps running");
    send(&f, E(EV_STOP));
    CHECK(f.state == ST_STANDBY && mock_remaining == 0, "TC-04 2nd STOP -> STANDBY");

    /* TC-04b: PAUSE between the two STOPs resets the counter */
    goto_running(&f); send(&f, E(EV_STOP)); send(&f, E(EV_PAUSE)); send(&f, E(EV_RUN));
    send(&f, E(EV_STOP));
    CHECK(f.state == ST_RUNNING && f.stop_count == 1, "TC-04b counter reset by PAUSE/RUN");

    /* TC-05 Transaction cancel */
    reset(&f); send(&f, COIN(50)); send(&f, E(EV_STOP));
    CHECK(f.state == ST_STANDBY && f.coin_total == 0, "TC-05");

    /* TC-06 Timeout during PAUSE */
    goto_running(&f); mock_tick_seconds(1000); send(&f, E(EV_PAUSE));
    int expired = mock_tick_seconds(800);
    CHECK(expired == 1, "TC-06 timer expired while paused");
    send(&f, E(EV_TIMER_DONE));
    CHECK(f.state == ST_STANDBY && wm_fsm_outputs(f.state).rled == LED_ON, "TC-06");

    /* TC-07 Error injection from every state, only STOP leaves ERROR */
    void (*go[4])(wm_fsm_t *) = { reset, goto_ready, goto_running, goto_paused };
    for (int i = 0; i < 4; i++) {
        go[i](&f); send(&f, E(EV_FAULT));
        CHECK(f.state == ST_ERROR, "TC-07 fault from state %d", i);
        send(&f, COIN(50)); send(&f, E(EV_RUN)); send(&f, E(EV_PAUSE)); send(&f, E(EV_FAULT));
        CHECK(f.state == ST_ERROR && f.coin_total == (i == 1 ? 50 : 0), "TC-07 other buttons ignored");
        send(&f, E(EV_STOP));
        CHECK(f.state == ST_STANDBY && f.coin_total == 0 && !mock_active, "TC-07 STOP clears error");
    }

    /* Stale Timer Done (timer already cleared) after a forced stop is harmless */
    goto_running(&f); send(&f, E(EV_STOP)); send(&f, E(EV_STOP));
    CHECK(send(&f, E(EV_TIMER_DONE)) == WM_FSM_IGNORED && f.state == ST_STANDBY, "stale TIMER_DONE ignored");

    /* Guard [Timer == 0]: a spurious TIMER_DONE while time remains is ignored */
    goto_running(&f);
    CHECK(send(&f, E(EV_TIMER_DONE)) == WM_FSM_IGNORED && f.state == ST_RUNNING, "guard Timer==0");
}

int main(void)
{
    part_a();
    part_b();
    part_c();

    printf("\nTransition coverage (rows of the State-transition Table):\n");
    int covered = 0;
    for (int i = 0; i < 20; i++) {
        printf("  #%-2d %s (hit %d)\n", i, g_rule_hit[i] ? "covered" : "NOT COVERED", g_rule_hit[i]);
        if (g_rule_hit[i]) covered++;
    }
    printf("Coverage: %d/20 transitions\n", covered);
    printf("Checks  : %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0 && covered == 20) ? 0 : 1;
}
