#include "protocol.h"

protocol_frame protocol_encode_t1(protocol_data data)
{
    protocol_frame frame;

    frame.a = (
        ((data.id & 0b11) << 0) |
        ((data.value & 0b111111) << 2)
    );

    return frame;
}

protocol_frame protocol_encode_t2(protocol_data data)
{
    protocol_frame frame;

    frame.a = (
            ((data.id & 0b111) << 0) |
            ((data.value & 0b111111) << 3)
    );

    return frame;
}

protocol_data protocol_decode_t1(protocol_frame frame)
{
    protocol_data data;

    data.id = (frame.a >> 0) & 0b11;
    data.value = (frame.a >> 2) & 0b111111;

    return data;
}

protocol_data protocol_decode_t2(protocol_frame frame)
{
    protocol_data data;

    data.id = (frame.a >> 0) & 0b111;
    data.value = (frame.a >> 3) & 0b11111;

    return data;
}
