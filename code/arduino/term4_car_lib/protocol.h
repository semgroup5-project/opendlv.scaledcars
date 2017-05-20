#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define FRAME_T1 1
#define FRAME_T2 2

#define ID_OUT_MOTOR                1
#define ID_OUT_SERVO                2

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
} protocol_frame;

/**
 * Final data form
 */
typedef struct {
    int id;
    int value;
} protocol_data;

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

#ifdef __cplusplus
}
#endif

#endif