#include "term4_car_lib.h"

ESCMotor::ESCMotor() {
}

void ESCMotor::setFreqsAndSpeeds() {
    FULL_FORWARD = 180;
    FULL_BACKWARD = 0;
    IDLE_SPEED = 90;
    IDLE_RAW_SPEED = 1500;
    MAX_FRONT_RAW_SPEED = 2000;
    MAX_BACK_RAW_SPEED = 1000;
    _speed = IDLE_SPEED;
}

void ESCMotor::init(unsigned short pin) {
    _pin = pin;
    _direction = 1;
    setFreqsAndSpeeds();
    attach(_pin); //attach the servo to its pin
    arm();
}

void ESCMotor::arm() {
    write(IDLE_SPEED);
}

void ESCMotor::setSpeed(int speed) { //receives a speed in the scale of -100 to 100
//    if (speed < 90) {
//        _direction = 0;
//#ifdef COMMON_ANODE
//        red = 95;
//        green = 255;
//        blue = 95;
//#endif
//        analogWrite(redPin, red);
//        analogWrite(greenPin, green);
//        analogWrite(bluePin, blue);
//    } else if (speed > 90) {
//        _direction = 1;
//#ifdef COMMON_ANODE
//        red = 0;
//        green = 0;
//        blue = 255;
//#endif
//        analogWrite(redPin, red);
//        analogWrite(greenPin, green);
//        analogWrite(bluePin, blue);
//    }
//    write(speed);

    int s = IDLE_SPEED;
    if (speed <= FULL_FORWARD && speed >= FULL_BACKWARD) {
        s = filterSpeed(speed, 0);
    } else {
        s = filterSpeed(speed, 1);
    }
    if (s != _speed) {
        _speed = s;

        write(_speed); //write the appropriate pwm signal to the servo
    }
}

void ESCMotor::brake() {
#ifdef COMMON_ANODE
    red = 0;
    green = 255;
    blue = 255;
#endif
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);

    if (_direction && _speed != IDLE_SPEED) {
        write(IDLE_SPEED + 15);
//        wait(0.3);
        write(IDLE_SPEED);
    } else if (_speed != IDLE_SPEED && !_direction) {
        write(IDLE_SPEED - 55);
//        wait(0.3);
        write(IDLE_SPEED);
    }
    _speed = IDLE_SPEED;
}

int ESCMotor::filterSpeed(int speed, int isRC) {
    int filtered = speed;
    if (isRC) {
        if (speed == 0) //zero is also considered as the center value
        {
            filtered = center;
        } else if (speed <= lo) //Trim Noise from bottom end
        {
            filtered = lo;
        } else if (speed <= deadHi && speed >= deadLo) //Create Dead-Band
        {
            filtered = center;
        } else if (speed >= hi) //Trim Noise from top end
        {
            filtered = hi;
        }

        if (filtered >= lo && filtered <= deadLo) {
            filtered = map(filtered, lo, deadLo, FULL_BACKWARD + 25, IDLE_SPEED - 1);
            _direction = 0;
#ifdef COMMON_ANODE
            red = 95;
            green = 255;
            blue = 95;
#endif
            analogWrite(redPin, red);
            analogWrite(greenPin, green);
            analogWrite(bluePin, blue);
        } else if (filtered == center) {
            filtered = IDLE_SPEED;
        } else if (filtered >= deadHi && filtered <= hi) {
            filtered = map(filtered, deadHi, hi, IDLE_SPEED + 1, FULL_FORWARD - 30);
            _direction = 1;
#ifdef COMMON_ANODE
            red = 0;
            green = 0;
            blue = 255;
#endif
            analogWrite(redPin, red);
            analogWrite(greenPin, green);
            analogWrite(bluePin, blue);
        }
    } else {
        if (speed >= FULL_BACKWARD && speed < IDLE_SPEED) {
            _direction = 0;
#ifdef COMMON_ANODE
            red = 95;
            green = 255;
            blue = 95;
#endif
            analogWrite(redPin, red);
            analogWrite(greenPin, green);
            analogWrite(bluePin, blue);
        } else if (speed == IDLE_SPEED) {

        } else if (speed > IDLE_SPEED && speed <= FULL_FORWARD) {
            _direction = 1;
#ifdef COMMON_ANODE
            red = 0;
            green = 0;
            blue = 255;
#endif
            analogWrite(redPin, red);
            analogWrite(greenPin, green);
            analogWrite(bluePin, blue);
        }
        filtered = speed;
    }
    return filtered;
}

void ESCMotor::wait(double seconds) {
    interval = seconds * 1000;
    currentMillis = millis();
    while (millis() - currentMillis <= interval);
}
