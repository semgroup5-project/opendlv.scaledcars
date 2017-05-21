/* Placeholder for an infrared sensor class */

#include "term4_car_lib.h"

InfraredSensor::InfraredSensor() {
    _sensorMedianDelay = 15; //median delay for all the sharp infrared sensors
    _pin = 99; //give it an initial (likely invalid) value
    _minDistance = 5; //GP2D120's minimum distance
    _maxDistance = 30; //GP2D120's maximum distance
}

void InfraredSensor::attach(unsigned short pin) {
    _pin = pin;
    pinMode(_pin, INPUT);
}

unsigned int InfraredSensor::getDistance() {
    unsigned int distance = voltsToCentimeters(readAnalogInput());
    if ((distance < _minDistance) || (distance > _maxDistance)) {
        return 0;
    }
    return distance;
}

unsigned int InfraredSensor::getDistance2() {
    unsigned int distance = voltsToCentimeters2(readAnalogInput());
    if ((distance < _minDistance) || (distance > _maxDistance)) {
        return 0;
    }
    return distance;
}

unsigned int InfraredSensor::readAnalogInput() {
    return analogRead(_pin);
}

unsigned int InfraredSensor::voltsToCentimeters(unsigned int volts) {
    return ((2914 / (volts + 5)) - 1);
}

unsigned int InfraredSensor::voltsToCentimeters2(unsigned int volts){
    return (1 / (0.0002391473 * volts - 0.0100251467)); //source: Jeroen Doggen http://plat.is/v3x25
}

void InfraredSensor::setMinAndMax(unsigned int min, unsigned int max) {
    _minDistance = min; //GP2D120's minimum distance
    _maxDistance = max; //GP2D120's maximum distance
}
