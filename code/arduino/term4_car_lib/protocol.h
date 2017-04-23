#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>

#define ID_OUT_MOTOR                1
#define ID_OUT_SERVO                2
# define ID_OUT_ODOMETER            3

#define ID_IN_ULTRASONIC_CENTER     1
#define ID_IN_ULTRASONIC_SIDE_FRONT 2
#define ID_IN_INFRARED_SIDE_FRONT   3
#define ID_IN_INFRARED_SIDE_BACK    4
#define ID_IN_INFRARED_BACK         5
#define ID_IN_ENCODER               6

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

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif
