
CC=gcc
V=valgrind
PY3="/usr/bin/python3"

CF_GLIB=`pkg-config glib-2.0 --libs --cflags`
CF_BASE=$(CF_GLIB) -pthread

V_ARGS=--tool=memcheck --leak-check=full --track-origins=yes

SRC=\
    "src/arduino.h" \
    "src/arduino.c" \
    "src/protocol.h" \
    "src/protocol.c" \
    "src/serial.h" \
    "src/serial.c" \
    "src/main.c"

OUT="./out"

all: compile run clean

clean:
	rm out
	rm vgcore.*
preprocess:
	$(PY3) make.py
compile: preprocess
	$(CC) $(SRC) $(CF_BASE) -o $(OUT)
run:
	$(OUT)
debug:
	$(V) $(V_ARGS) -v $(OUT)
