CC      ?= gcc
CFLAGS  ?= -std=c99 -Wall -Wextra -Wpedantic -O2 -I../sketch

all: test_fsm trace_demo

test_fsm: test_fsm.c ../sketch/wm_fsm.c ../sketch/wm_fsm.h
	$(CC) $(CFLAGS) test_fsm.c ../sketch/wm_fsm.c -o $@

trace_demo: trace_demo.c ../sketch/wm_fsm.c ../sketch/wm_fsm.h
	$(CC) $(CFLAGS) trace_demo.c ../sketch/wm_fsm.c -o $@

run: test_fsm
	./test_fsm

trace: trace_demo
	./trace_demo

clean:
	rm -f test_fsm trace_demo

.PHONY: all run trace clean
