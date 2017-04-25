#include "protocol.h"

uint8_t protocol_checksum_calculate(protocol_frame frame)
{
    // (A1-6) XOR (A7,B1-5) = R0-5
    uint8_t r = (
        ((frame.a >> 1) & 0b111111) ^
        (((frame.a >> 7) & 0b1) | ((frame.b >> 1 & 0b1111) << 1))
    );

    // C0-2 = R0-2 XOR R3-5
    uint8_t c = (
        ((r >> 0) & 0b111) ^
        ((r >> 3) & 0b111)
    );

    return c;
}

bool protocol_checksum_check(protocol_frame frame)
{
    uint8_t expected = protocol_checksum_calculate(frame);
    uint8_t obtained = (frame.b >> 5) & 0b111;

    return expected == obtained;
}

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
        (0 << 0) |
        ((data.id & 0b111) << 1) |
        ((data.value & 0b1111) << 4)
    );

    frame.b = (
        (1 << 0) |
        (((data.value >> 4) & 0b1111) << 1)
    );

    frame.b = frame.b | (
        (protocol_checksum_calculate(frame) & 0b111) << 5
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

    data.id = (frame.a >> 1) & 0b111;
    data.value = (
        ((frame.a >> 4) & 0b1111) |
        (((frame.b >> 1) & 0b1111) << 4)
    );

    return data;
}

uint8_t protocol_get_byte_index(uint8_t b)
{
    return b & 0b1;
}

void protocol_state_init(protocol_state *state)
{
    state->frame.a = 0;
    state->frame.b = 0;

    state->valid = false;

    state->data.id = 0;
    state->data.value = 0;
}

void protocol_receive_t2(protocol_state *state, uint8_t b)
{
    uint8_t index = protocol_get_byte_index(b);

    switch (index)
    {
        case 0: state->frame.a = b; break;
        case 1: state->frame.b = b; break;
    }

    state->valid = protocol_checksum_check(state->frame);

    if (state->valid) {
        state->data = protocol_decode_t2(state->frame);
    }
}
