#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define FRAME_T1 1
#define FRAME_T2 2

#define ID_OUT_MOTOR                1
#define ID_OUT_SERVO                2
#define ID_OUT_ODOMETER             3

#define ID_IN_ULTRASONIC_CENTER     1
#define ID_IN_ULTRASONIC_SIDE_FRONT 2
#define ID_IN_INFRARED_SIDE_FRONT   3
#define ID_IN_INFRARED_SIDE_BACK    4
#define ID_IN_INFRARED_BACK         5
#define ID_IN_ENCODER               6

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Data transfer unit
 */
typedef struct {
    uint8_t a;
    uint8_t b;
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
uint8_t protocol_checksum_calculate(protocol_frame frame);

/**
 * Validate the checksum included in the specified frame
 */
bool protocol_checksum_check(protocol_frame frame);

/**
 * Encode data into a transferable frame
 */
protocol_frame protocol_encode_t1(protocol_data data);
protocol_frame protocol_encode_t2(protocol_data data);

/**
 * Decode data from a transferable frame
 */
protocol_data protocol_decode_t1(protocol_frame frame);
protocol_data protocol_decode_t2(protocol_frame frame);

/**
 * Get the frame index of the specified byte
 */
uint8_t protocol_get_byte_index(uint8_t b);

/**
 * Initialize protocol state
 */
void protocol_state_init(protocol_state *state);

/**
 * Handle received byte
 */
void protocol_receive_t2(protocol_state *state, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif