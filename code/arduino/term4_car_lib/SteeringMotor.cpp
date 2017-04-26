#include "term4_car_lib.h"

SteeringMotor::SteeringMotor() {
}

void SteeringMotor::init(unsigned short pin) {
    _pin = pin;
    setDegrees();
    attach(_pin); //attach the servo to its pin
    arm();
}

void SteeringMotor::arm() {
    write(STRAIGHT_DEGREES);
}

void SteeringMotor::setAngle(int degrees, int isRC) { // receives some degrees in the scale of MAX_LEFT_ANGLE, MAX_RIGHT_ANGLE
    //write(degrees);
    if (degrees == 0 && isRC) {
        _angle = STRAIGHT_DEGREES;
    } else{
        if (degrees <= MAX_RIGHT_ANGLE && degrees >= MAX_LEFT_ANGLE) {
            _angle = filterAngle(degrees, 0); //_angle now holds a value between MAX_LEFT_RAW_DEGREES and MAX_RIGHT_RAW_DEGREES
        } else {
            _angle = filterAngle(degrees, 1); //_angle now holds a value between MAX_LEFT_RAW_DEGREES and MAX_RIGHT_RAW_DEGREES
        }

        write(_angle); //writes the angle to the servo motor
    }
 }

void SteeringMotor::setDegrees() {
    STRAIGHT_DEGREES = 90;
    MAX_RIGHT_ANGLE = 180;
    MAX_LEFT_ANGLE = 0;
    _angle = STRAIGHT_DEGREES;
}

unsigned int SteeringMotor::filterAngle(int degrees, int isRC) {
    int filtered = degrees;
    if (isRC) {
        if (degrees == 0) //zero is also considered as the center value
        {
            filtered = center;
        } else if (degrees <= lo) //Trim Noise from bottom end
        {
            filtered = lo;
        } else if (degrees <= deadHi && degrees >= deadLo) //Create Dead-Band
        {
            filtered = center;
        } else if (degrees >= hi) //Trim Noise from top end
        {
            filtered = hi;
        }

        if (filtered >= lo && filtered <= deadLo) {
            filtered = map(filtered, lo, deadLo, 50, STRAIGHT_DEGREES);
        } else if (filtered == center) {
            filtered = STRAIGHT_DEGREES;
        } else if (filtered >= deadHi && filtered <= hi) {
            filtered = map(filtered, deadHi, hi, STRAIGHT_DEGREES, 130);
        }

    } else {
        if (degrees >= MAX_LEFT_ANGLE && degrees < STRAIGHT_DEGREES) {
            filtered = map(degrees, MAX_LEFT_ANGLE, STRAIGHT_DEGREES - 1, 50, STRAIGHT_DEGREES - 1);
        } else if (degrees == STRAIGHT_DEGREES) {
            filtered = STRAIGHT_DEGREES;
        } else if (degrees > STRAIGHT_DEGREES && degrees <= MAX_RIGHT_ANGLE) {
            filtered = map(degrees, STRAIGHT_DEGREES + 1, MAX_RIGHT_ANGLE, STRAIGHT_DEGREES + 1, 130);
        }
    }
    return filtered;
}
