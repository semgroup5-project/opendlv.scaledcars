
PY3="/usr/bin/python3"

all: compile run clean

clean:
	rm out
preprocess:
	$(PY3) make.py
compile:
	gcc protocol.h protocol.c test.c -o out
run:
	./out
