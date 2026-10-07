/**
 * @file    wm_fsm.c
 * @brief   Coin-operated washing machine - FSM implemented with switch-case.
 *
 * Structure: an outer switch on the CURRENT STATE, an inner switch on the
 * EVENT. Each "case" corresponds to exactly one arrow of the State Machine
 * Diagram; the comment "#n" is the row number in the State-transition Table.
 * Events that are not listed for a state fall into "default" and are
 * ignored (the state does not change) - this is the "ignore rule".
 */
#include "wm_fsm.h"

#include <stddef.h>

/* -------------------------------------------------------------------------- */
/* Small helpers that wrap the hardware port (NULL-safe)                      */
/* -------------------------------------------------------------------------- */
static void act_timer_start(wm_fsm_t *fsm, uint32_t seconds)
{
    if (fsm->port && fsm->port->timer_start) fsm->port->timer_start(seconds);
}

static void act_timer_clear(wm_fsm_t *fsm)
{
    if (fsm->port && fsm->port->timer_clear) fsm->port->timer_clear();
}

static uint32_t timer_value(const wm_fsm_t *fsm)
{
    if (fsm->port && fsm->port->timer_remaining) return fsm->port->timer_remaining();
    return 0u;
}

bool wm_fsm_is_valid_coin(uint16_t cents)
{
    return (cents == 10u) || (cents == 20u) || (cents == 50u);
}

/* -------------------------------------------------------------------------- */
/* Initial transition                                                         */
/* -------------------------------------------------------------------------- */
int wm_fsm_init(wm_fsm_t *fsm, const wm_fsm_port_t *port)
{
    fsm->port       = port;
    fsm->state      = ST_STANDBY;           /* #0  (Initial) -> STANDBY    */
    fsm->coin_total = 0u;
    fsm->stop_count = 0u;
    act_timer_clear(fsm);
    return 0;
}

/* -------------------------------------------------------------------------- */
/* Event dispatcher (switch-case FSM)                                         */
/* -------------------------------------------------------------------------- */
int wm_fsm_dispatch(wm_fsm_t *fsm, wm_fsm_event_t ev)
{
    int rule = WM_FSM_IGNORED;

    /* Coins of an unsupported value are rejected before reaching the FSM. */
    if (ev.id == EV_COIN && !wm_fsm_is_valid_coin(ev.value)) {
        return WM_FSM_IGNORED;
    }

    switch (fsm->state) {

    /* ===================================================== STANDBY ====== */
    case ST_STANDBY:
        switch (ev.id) {
        case EV_COIN:
            if ((uint32_t)fsm->coin_total + ev.value < WM_FSM_MIN_COIN_TOTAL) {
                fsm->coin_total += ev.value;            /* #1 stay STANDBY */
                rule = 1;
            } else {
                fsm->coin_total += ev.value;            /* #2 -> READY     */
                fsm->state = ST_READY;
                rule = 2;
            }
            break;
        case EV_FAULT:
            fsm->state = ST_ERROR;                      /* #3 -> ERROR     */
            rule = 3;
            break;
        default:                                        /* ignored         */
            break;
        }
        break;

    /* ======================================================= READY ====== */
    case ST_READY:
        switch (ev.id) {
        case EV_COIN:
            fsm->coin_total += ev.value;                /* #4 stay READY   */
            rule = 4;
            break;
        case EV_RUN:
            if (fsm->coin_total >= WM_FSM_MIN_COIN_TOTAL) {
                fsm->coin_total = 0u;                   /* #5 -> RUNNING   */
                act_timer_start(fsm, WM_FSM_WASH_TIME_S);       /* 30-min timer    */
                fsm->stop_count = 0u;
                fsm->state = ST_RUNNING;
                rule = 5;
            }
            break;
        case EV_STOP:
            fsm->coin_total = 0u;                       /* #6 -> STANDBY   */
            fsm->state = ST_STANDBY;                    /* no refund       */
            rule = 6;
            break;
        case EV_FAULT:
            fsm->state = ST_ERROR;                      /* #7 -> ERROR     */
            rule = 7;
            break;
        default:
            break;
        }
        break;

    /* ===================================================== RUNNING ====== */
    case ST_RUNNING:
        switch (ev.id) {
        case EV_STOP:
            if (fsm->stop_count == 0u) {
                fsm->stop_count = 1u;                   /* #8 1st press    */
                rule = 8;
            } else {                                    /* stop_count == 1 */
                act_timer_clear(fsm);                   /* #9 2nd press    */
                fsm->stop_count = 0u;
                fsm->state = ST_STANDBY;
                rule = 9;
            }
            break;
        case EV_TIMER_DONE:
            if (timer_value(fsm) == 0u) {               /* guard Timer==0  */
                act_timer_clear(fsm);                   /* #10 Cycle done  */
                fsm->stop_count = 0u;
                fsm->state = ST_STANDBY;
                rule = 10;
            }
            break;
        case EV_PAUSE:
            fsm->stop_count = 0u;                       /* #11 -> PAUSED   */
            fsm->state = ST_PAUSED;                     /* timer keeps run */
            rule = 11;
            break;
        case EV_FAULT:
            fsm->state = ST_ERROR;                      /* #12 -> ERROR    */
            rule = 12;
            break;
        default:
            break;
        }
        break;

    /* ====================================================== PAUSED ====== */
    case ST_PAUSED:
        switch (ev.id) {
        case EV_RUN:
            fsm->stop_count = 0u;                       /* #13 -> RUNNING  */
            fsm->state = ST_RUNNING;                    /* timer untouched */
            rule = 13;
            break;
        case EV_STOP:
            if (fsm->stop_count == 0u) {
                fsm->stop_count = 1u;                   /* #14 1st press   */
                rule = 14;
            } else {
                act_timer_clear(fsm);                   /* #15 2nd press   */
                fsm->stop_count = 0u;
                fsm->state = ST_STANDBY;
                rule = 15;
            }
            break;
        case EV_TIMER_DONE:
            if (timer_value(fsm) == 0u) {
                act_timer_clear(fsm);                   /* #16 Timeout     */
                fsm->stop_count = 0u;
                fsm->state = ST_STANDBY;
                rule = 16;
            }
            break;
        case EV_FAULT:
            fsm->state = ST_ERROR;                      /* #17 -> ERROR    */
            rule = 17;
            break;
        default:
            break;
        }
        break;

    /* ======================================================= ERROR ====== */
    case ST_ERROR:
        switch (ev.id) {
        case EV_FAULT:
            rule = 18;                                  /* #18 stay ERROR  */
            break;
        case EV_STOP:
            act_timer_clear(fsm);                       /* #19 Clear Error */
            fsm->coin_total = 0u;                       /* Clear Timer     */
            fsm->stop_count = 0u;                       /* coin_total = 0  */
            fsm->state = ST_STANDBY;
            rule = 19;
            break;
        default:
            break;
        }
        break;

    default:                     /* corrupted state -> safe recovery       */
        (void)wm_fsm_init(fsm, fsm->port);
        break;
    }

    return rule;
}

/* -------------------------------------------------------------------------- */
/* Moore output function                                                      */
/* -------------------------------------------------------------------------- */
wm_fsm_outputs_t wm_fsm_outputs(wm_fsm_state_t state)
{
    wm_fsm_outputs_t out = { LED_OFF, LED_OFF };
    switch (state) {
    case ST_STANDBY: out.rled = LED_ON;    out.bled = LED_OFF;   break;
    case ST_READY:   out.rled = LED_OFF;   out.bled = LED_ON;    break;
    case ST_RUNNING: out.rled = LED_OFF;   out.bled = LED_BLINK; break;
    case ST_PAUSED:  out.rled = LED_OFF;   out.bled = LED_ON;    break;
    case ST_ERROR:   out.rled = LED_BLINK; out.bled = LED_OFF;   break;
    default:         break;
    }
    return out;
}

/* -------------------------------------------------------------------------- */
/* Names for logging                                                          */
/* -------------------------------------------------------------------------- */
const char *wm_fsm_state_name(wm_fsm_state_t state)
{
    static const char *const names[ST_COUNT] = {
        "STANDBY", "READY", "RUNNING", "PAUSED", "ERROR"
    };
    return (state < ST_COUNT) ? names[state] : "?";
}

const char *wm_fsm_event_name(wm_fsm_event_id_t ev)
{
    static const char *const names[EV_COUNT] = {
        "NONE", "COIN", "RUN", "PAUSE", "STOP", "FAULT", "TIMER_DONE"
    };
    return (ev < EV_COUNT) ? names[ev] : "?";
}
