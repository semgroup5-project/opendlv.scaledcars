/* Placeholder for an ultrasonic sensor class */

#include "term4_car_lib.h"

const unsigned short UltrasonicSensor::DEFAULT_PING_DELAY = 70;
const unsigned short UltrasonicSensor::DEFAULT_SRF08_ADDRESS = 112;

static unsigned short FIRST_ADDRESS = 112; //please refer to: http://www.robot-electronics.co.uk/htm/srf08tech.html
static unsigned short LAST_ADDRESS = 127;

UltrasonicSensor::UltrasonicSensor() {
    _address = 0, _delay = 0; //some initial invalid values
}

void UltrasonicSensor::attach(unsigned short address) {
    Wire.begin();
    _address = constrain(address, FIRST_ADDRESS, LAST_ADDRESS); //allow only valid values, between 112 and 127
    _delay = DEFAULT_PING_DELAY;
}

void UltrasonicSensor::setGain(unsigned short gainValue) {
    Wire.beginTransmission(_address); //start i2c transmission
    Wire.write(0x01); //write to GAIN register (1)
    Wire.write(constrain(gainValue, 0, 31)); //write the value
    Wire.endTransmission(); //end transmission
}

void UltrasonicSensor::setRange(unsigned short rangeValue) {
    Wire.beginTransmission(_address); //start i2c transmission
    Wire.write(0x02); //write to range register (1)
    Wire.write(rangeValue); //write the value -> Max_Range = (rangeValue * 3.4) + 3.4 in centimeters
    Wire.endTransmission(); //end transmission
}

void UltrasonicSensor::setPingDelay(unsigned short milliseconds) {
    if (milliseconds > 64) {
        _delay = milliseconds;
    }
}

unsigned int UltrasonicSensor::getDistance() {
    int reading = 0;
    Wire.beginTransmission(_address);
    Wire.write(byte(0x00));

    // command sensor to measure in "inches" (0x50)
    // use 0x51 for centimeters
    // use 0x52 for ping microseconds
    Wire.write(byte(0x51));
    Wire.endTransmission();

    wait(_delay);
    Wire.beginTransmission(_address);
    Wire.write(byte(0x02));
    Wire.endTransmission();

    Wire.requestFrom(_address, uint8_t(2));
    if (2 <= Wire.available()) { // if two bytes were received
        reading = Wire.read();  // receive high byte (overwrites previous reading)
        reading = reading << 8;    // shift high byte to be high 8 bits
        reading |= Wire.read(); // receive low byte as lower 8 bits
    }

    if (reading > 62) {
        return 0;
    }
    return reading;
}

unsigned short UltrasonicSensor::getLightReading() {
    Wire.beginTransmission(_address);
    Wire.write(byte(0x00));
    Wire.write(byte(0x51));
    Wire.endTransmission();
    wait(_delay);
    Wire.beginTransmission(_address);
    Wire.write(byte(0x01));
    Wire.endTransmission();
    Wire.requestFrom(_address, uint8_t(1));
    while (!Wire.available());
    return Wire.read();
}

void UltrasonicSensor::changeAddress(unsigned short newAddress) {
    newAddress = constrain(newAddress, FIRST_ADDRESS, LAST_ADDRESS); //allow only valid values, between 112 and 127
    Wire.beginTransmission(_address);
    Wire.write(byte(0x00));
    Wire.write(byte(0xA0));
    Wire.endTransmission();

    Wire.beginTransmission(_address);
    Wire.write(byte(0x00));
    Wire.write(byte(0xAA));
    Wire.endTransmission();

    Wire.beginTransmission(_address);
    Wire.write(byte(0x00));
    Wire.write(byte(0xA5));
    Wire.endTransmission();

    Wire.beginTransmission(_address);
    Wire.write(byte(0x00));
    Wire.write(newAddress << 1);
    Wire.endTransmission();

    _address = newAddress;
}

