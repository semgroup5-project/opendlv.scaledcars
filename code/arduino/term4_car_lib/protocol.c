/*
 * (!) DO NOT MODIFY THIS FILE
 * Run ./make.py to generate `protocol.11.c` from `protocol.14.c`
 */

#include "protocol.h"

protocol_frame protocol_encode_t1(protocol_data data)
{
    protocol_frame frame;

    frame.a = (
            ((data.id & 0x3) << 0) |
            ((data.value & 0x3f) << 2)
    );

    return frame;
}

protocol_frame protocol_encode_t2(protocol_data data)
{
    protocol_frame frame;

    frame.a = (
            ((data.id & 0x07) << 0) |
            ((data.value & 0x1f) << 3)
    );

    return frame;
}

protocol_data protocol_decode_t1(protocol_frame frame)
{
    protocol_data data;

    data.id = (frame.a >> 0) & 0x3;
    data.value = (frame.a >> 2) & 0x3f;

    return data;
}

protocol_data protocol_decode_t2(protocol_frame frame)
{
    protocol_data data;

    data.id = (frame.a >> 0) & 0x07;
    data.value = (frame.a >> 3) & 0x1f;

    return data;
}
