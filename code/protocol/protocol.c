#include "protocol.h"

#include <stdio.h>

char protocol_checksum_calculate(protocol_frame frame)
{
    // (A1-6) XOR (A7,B1-5) = R0-5
    char r = (
        ((frame.a >> 1) & 0b111111) ^
        (((frame.a >> 7) & 0b1) | ((frame.b >> 1 & 0b11111) << 1))
    );

    // C0 = (((R0) XOR (R2)) XOR (R4))
    char c0 = (((r >> 0) & 0b1) ^ ((r >> 2) & 0b1)) ^ ((r >> 4) & 0b1);

    // C1 = (((R1) XOR (R3)) XOR (R5))
    char c1 = (((r >> 1) & 0b1) ^ ((r >> 3) & 0b1)) ^ ((r >> 5) & 0b1);

    return (
        ((c0 & 0b1) << 0) |
        ((c1 & 0b1) << 1)
    );
}

bool protocol_checksum_check(protocol_frame frame)
{
    char checksum = protocol_checksum_calculate(frame);

    // Calculated checksum
    char expected_c0 = (checksum >> 0) & 0b1;
    char expected_c1 = (checksum >> 1) & 0b1;

    // Frame checksum
    char obtained_c0 = (frame.b >> 6) & 0b1;
    char obtained_c1 = (frame.b >> 7) & 0b1;

    return (
        obtained_c0 == expected_c0 &&
        obtained_c1 == expected_c1
    );
}

protocol_frame protocol_encode(protocol_data data)
{
    protocol_frame frame;

    frame.a = (
        (0 << 0) |
        ((data.type & 0b11) << 1) |
        ((data.id & 0b11) << 3) |
        ((data.value & 0b111) << 5)
    );

    frame.b = (
        (1 << 0) |
        (((data.value >> 3) & 0b11111) << 1)
    );

    frame.b = frame.b | (
        (protocol_checksum_calculate(frame) & 0b11) << 6
    );

    return frame;
}

protocol_data protocol_decode(protocol_frame frame)
{
    protocol_data data;

    data.type = (frame.a >> 1) & 0b11;
    data.id = (frame.a >> 3) & 0b11;
    data.value = (
        ((frame.a >> 5) & 0b111) |
        (((frame.b >> 1) & 0b11111) << 3)
    );

    return data;
}

char protocol_get_byte_index(char byte)
{
    return byte & 0b1;
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
