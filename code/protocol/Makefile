
PY3="/usr/bin/python3"

all: compile run clean

clean:
	rm out
preprocess:
	$(PY3) make.py
compile:
	gcc src/protocol.h src/protocol.c src/test.c -o out
run:
	./out
