#include "protocol.h"

char protocol_checksum_calculate(protocol_frame frame)
{
    // (A1-6) XOR (A7,B1-5) = R0-5
    char r = (
        ((frame.a >> 1) & 0x3F) ^
        (((frame.a >> 7) & 0x1) | ((frame.b >> 1 & 0x3F) << 1))
    );

    // C0-2 = R0-2 XOR R3-5
    char c = (
        ((r >> 0) & 0x7) ^
        ((r >> 3) & 0x7)
    );

    return c;
}

bool protocol_checksum_check(protocol_frame frame)
{
    char expected = protocol_checksum_calculate(frame);
    char obtained = (frame.b >> 5) & 0x7;

    return expected == obtained;
}

protocol_frame protocol_encode(protocol_data data)
{
    protocol_frame frame;

    frame.a = (
        (0 << 0) |
        ((data.id & 0x07) << 1) |
        ((data.value & 0x0F) << 4)
    );

    frame.b = (
        (1 << 0) |
        (((data.value >> 4) & 0x0F) << 1)
    );

    frame.b = frame.b | (
        (protocol_checksum_calculate(frame) & 0x07) << 5
    );

    return frame;
}

protocol_data protocol_decode(protocol_frame frame)
{
    protocol_data data;

    data.id = (frame.a >> 1) & 0x07;
    data.value = (
        ((frame.a >> 4) & 0x0F) |
        (((frame.b >> 1) & 0x0F) << 4)
    );

    return data;
}

char protocol_get_byte_index(char byte)
{
    return byte & 0x1;
}

void protocol_receive(protocol_state *state, char byte)
{
    char index = protocol_get_byte_index(byte);

    switch (index)
    {
        case 0: state->frame.a = byte; break;
        case 1: state->frame.b = byte; break;
    }

    state->valid = protocol_checksum_check(state->frame);

    if (state->valid) {
        state->data = protocol_decode(state->frame);
    }
}
