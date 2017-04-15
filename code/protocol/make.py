#!/usr/bin/python2 -tt
# -*- coding: utf-8 -*-

import re


NOTICE = '\n'.join([
    '/*',
    ' * (!) DO NOT MODIFY THIS FILE',
    ' * Run ./make.py to generate `protocol.11.c` from `protocol.14.c`',
    ' */',
] + [''] * 2)

PATH_IN = 'protocol.14.c'
PATH_OUT = 'protocol.11.c'


def main():

    def repl(match):
        repr_bin = match.group(1)
        repr_hex = hex(eval(repr_bin))
        return repr_hex

    _in = open(PATH_IN).read()
    _out = re.sub('(0b[0|1]*)', repl, _in);

    with open(PATH_OUT, 'w') as out:
        out.write(NOTICE)
        out.write(_out)


if __name__ == '__main__':
    main()
