#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>

/**
 * Data transfer unit
 */
typedef struct {
    char a;
    char b;
} protocol_frame;

/**
 * Final data form
 */
typedef struct {
    int id;
    int value;
} protocol_data;

/**
 * Transfer state
 */
typedef struct {
    protocol_frame frame;
    bool valid;
    protocol_data data;
} protocol_state;

/**
 * Calculate the checksum for the specified frame
 */
char protocol_checksum_calculate(protocol_frame frame);

/**
 * Validate the checksum included in the specified frame
 */
bool protocol_checksum_check(protocol_frame frame);

/**
 * Encode data into a transferable frame
 */
protocol_frame protocol_encode(protocol_data data);

/**
 * Decode data from a transferable frame
 */
protocol_data protocol_decode(protocol_frame frame);

/**
 * Get the frame index of the specified byte
 */
char protocol_get_byte_index(char byte);

/**
 * Handle received byte
 */
void protocol_receive(protocol_state *state, char byte);

#endif
