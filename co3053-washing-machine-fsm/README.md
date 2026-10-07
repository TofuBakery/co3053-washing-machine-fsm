# Coin-operated Washing Machine Controller (ESP32 / Wokwi)

Embedded Systems (CO3053) – Assignment 2 – Firmware.

The controller is a switch-case finite state machine (STANDBY, READY, RUNNING,
PAUSED, ERROR) driven by a 10 ms hardware timer interrupt. `delay()` is not used.

## Folder layout

```
sketch/             Wokwi / Arduino project (open this folder in Wokwi)
  sketch.ino        hardware layer: Timer ISR, buttons, event queue, LEDs, LCD, super-loop
  wm_fsm.h / .c     portable switch-case FSM (no Arduino dependency)
  diagram.json      Wokwi circuit (ESP32 DevKitC V4, 7 buttons, 2 LEDs, LCD1602 I2C)
  libraries.txt     Wokwi libraries (LiquidCrystal I2C)
test/
  test_fsm.c        PC unit tests: rows #0..#19 of the transition table, state x event matrix, TC-01..TC-07
  trace_demo.c      replays a button sequence and prints it in the Serial Monitor format
  Makefile
```

## Pin mapping

| Signal | GPIO | Wokwi key |
|---|---|---|
| RLED / BLED | 25 / 26 | – |
| RUN / PAUSE / STOP | 32 / 33 / 27 | r / p / s |
| FAULT_BUTTON | 14 | f |
| Coin 10c / 20c / 50c | 4 / 16 / 17 | 1 / 2 / 5 |
| LCD1602 SDA / SCL (0x27) | 21 / 22 | – |

All buttons are active-low with the internal pull-up.

## Run on Wokwi

Online project: https://wokwi.com/projects/477230539048342529 (open and press ▶).

To recreate it from these files:

1. Go to https://wokwi.com/projects/new/esp32 (sign in so you can save).
2. Replace `sketch.ino` and `diagram.json` with the files in `sketch/`.
3. Add the files `wm_fsm.h` and `wm_fsm.c` (tab menu ▾ → *New file…*) and paste their content.
4. Library Manager → **+** → `LiquidCrystal I2C`.
5. Press ▶. Click the simulation canvas once, then use the keyboard keys above or click the buttons.

Set `#define TIME_SCALE 60` in `sketch.ino` to watch the 30-minute cycle in 30 seconds
(1 simulated minute per real second). The default is `1` (real time).

## Serial Monitor trace

Every event is printed with the row number of the transition that fired:

```
[    4.10s] READY   --RUN--> RUNNING #5  coin=0 stop=0 T=1800s
[   10.50s] RUNNING --STOP--> RUNNING #8  coin=0 stop=1 T=1794s
[   11.20s] RUNNING --STOP--> STANDBY #9  coin=0 stop=0 T=0s
```

## Unit tests (PC)

```
make -C test run      # 103 checks, 20/20 transitions covered
make -C test trace    # example event trace
```

## Build

Tested with ESP32 Arduino core 3.0.7 and 2.0.17 (the timer setup has a branch for each API).
