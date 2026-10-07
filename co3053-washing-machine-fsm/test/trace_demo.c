/**
 * @file    trace_demo.c
 * @brief   Replays a scripted button sequence through wm_fsm.c on the PC and
 *          prints the trace in the SAME format as the ESP32 Serial Monitor.
 *
 * Build & run:  make -C test trace
 */
#include <stdio.h>
#include "wm_fsm.h"

static double   now_s      = 0.0;   /* simulated time (s)            */
static double   start_s    = 0.0;   /* time the wash timer started   */
static int      active     = 0;

static void     t_start(uint32_t s) { (void)s; start_s = now_s; active = 1; }
static void     t_clear(void)       { active = 0; }
static uint32_t t_remain(void)
{
    if (!active) return 0;
    double left = WM_FSM_WASH_TIME_S - (now_s - start_s);
    return left > 0 ? (uint32_t)(left + 0.999) : 0;
}

static const wm_fsm_port_t PORT = { t_start, t_clear, t_remain };

typedef struct { double t; wm_fsm_event_id_t id; uint16_t v; } step_t;

int main(void)
{
    static const step_t script[] = {
        {  2.40, EV_COIN, 20 }, {  3.00, EV_COIN, 50 }, {  4.10, EV_RUN,   0 },
        {  6.00, EV_PAUSE, 0 }, {  8.00, EV_STOP,  0 }, {  9.00, EV_RUN,   0 },
        { 10.50, EV_STOP,  0 }, { 11.20, EV_STOP,  0 }, { 12.00, EV_COIN, 50 },
        { 12.50, EV_STOP,  0 }, { 13.00, EV_FAULT, 0 }, { 13.50, EV_RUN,   0 },
        { 14.00, EV_STOP,  0 },
    };
    wm_fsm_t f;
    wm_fsm_init(&f, &PORT);
    printf("[    0.00s] (Initial) --reset--> STANDBY #0\n");

    for (unsigned i = 0; i < sizeof(script) / sizeof(script[0]); i++) {
        now_s = script[i].t;
        wm_fsm_event_t ev = { script[i].id, script[i].v };
        wm_fsm_state_t from = f.state;
        int rule = wm_fsm_dispatch(&f, ev);
        unsigned long ms = (unsigned long)(now_s * 1000.0 + 0.5);
        printf("[%5lu.%02lus] %-7s --%s", ms / 1000, (ms % 1000) / 10,
               wm_fsm_state_name(from), wm_fsm_event_name(ev.id));
        if (ev.id == EV_COIN) printf("(%u)", ev.value);
        if (rule == WM_FSM_IGNORED) { printf("--> (ignored)\n"); continue; }
        printf("--> %-7s #%-2d coin=%u stop=%u T=%lus\n", wm_fsm_state_name(f.state),
               rule, f.coin_total, f.stop_count, (unsigned long)t_remain());
    }
    return 0;
}
