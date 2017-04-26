#pragma clang diagnostic push
#pragma ide diagnostic ignored "CannotResolve"
#ifndef TERM4_CAR_LIB_LIBRARY_H
#define TERM4_CAR_LIB_LIBRARY_H

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "protocol.h"
#include "Netstrings.h"

#define BAUD 115200
#define US_C 0x73 //front ultrasonic pin
#define US_R 0x70 //front right ultrasonic
#define ENCODER_PIN_A 2 //wheel encoder channel A pin
#define ENCODER_PIN_B 4 //wheel encoder channel B pin
#define SERVO_PIN 3 //steering servo pin
#define ESC_PIN 9 //ESC motor pin
#define SIDE_FRONT_PIN A0 //side front infrared pin
#define SIDE_BACK_PIN A1 //side back infrared pin
#define BACK_PIN A2 //back infrared pin
#define CH_1 5 //RC receiver channel 1 pin
#define CH_2 6 //RC receiver channel 2 pin
#define redPin 22 //LED red pin
#define greenPin 24 //LED green pin
#define bluePin 26 //LED blue pin
#define COMMON_ANODE //LED common
#define RELAY_PIN 12

//Signal Conditioning limits for RC controller
#define lo 1000
#define hi 2000
#define deadLo 1430
#define deadHi 1570
#define center 1500

class DistanceSensor {
public:
    DistanceSensor();
    virtual ~DistanceSensor();
    virtual unsigned int getDistance() = 0; //to be implemented by the child classes
    unsigned int getMedianDistance(short iterations = DEFAULT_ITERATIONS);
    void encodeAndWrite(int id, int value);
    void wait(long milliseconds);
    unsigned long currentMillis;
    long interval;
private:
    static const short DEFAULT_ITERATIONS;
protected:
    unsigned int _sensorMedianDelay; //delay between measurements in the sensor's getMedianDistance in milliseconds
};

class InfraredSensor : public DistanceSensor {//placeholder for a possible future ultrasonic abstract class
public:
    explicit InfraredSensor();
    void attach(unsigned short pin);
    unsigned int getDistance();
    void setMinAndMax(unsigned int min, unsigned int max);
private:
    unsigned int readAnalogInput();
    unsigned int voltsToCentimeters(unsigned int volts);
    unsigned short _pin;
protected:
    unsigned int _maxDistance, _minDistance;
    unsigned int _sensorMedianDelay; //delay between measurements in the sensor's getMedianDistance in milliseconds
};

class UltrasonicSensor : public DistanceSensor {//placeholder for a possible future ultrasonic abstract class
public:
    explicit UltrasonicSensor();
    void attach(unsigned short address = DEFAULT_SRF08_ADDRESS);
    void setGain(unsigned short gainValue);
    void setRange(unsigned short rangeValue);
    void setPingDelay(unsigned short milliseconds = DEFAULT_PING_DELAY);
    unsigned int getDistance();
    unsigned short getLightReading();
    void changeAddress(unsigned short newAddress);
private:
    uint8_t _address;
    unsigned short _delay;
    static const unsigned short DEFAULT_PING_DELAY, DEFAULT_SRF08_ADDRESS;
};

class Odometer {
public:
    explicit Odometer(unsigned long pulsesPerMeter = DEFAULT_PULSES_PER_METER);
    int attach(unsigned short odometerPin);
    int attach(unsigned short odometerPin, unsigned short directionPin, boolean forwardDirState);
    void begin();
    unsigned long getDistance();
    long getRelativeDistance();
    short getDirection();
    float getSpeed();
    void encodeAndWrite(int id, int value);

private:
    unsigned long pulsesToCentimeters(unsigned long pulses);
    int init(unsigned short odometerPin);
    unsigned long _pulsesPerMeter;
    static const unsigned int DEFAULT_PULSES_PER_METER;
    static const unsigned short DEFAULT_DIRECTION_PIN;
    unsigned short _odometerInterruptPin, _odometerID;
    unsigned int _millimetersPerPulse;
};

class SteeringMotor : public Servo {
public:
    explicit SteeringMotor();
    void setAngle(int degrees, int isRC); //to be overriden by the child classes
    void arm();
    void init(unsigned short pin);
private:
    virtual void setDegrees(); //to be overriden by the child classes
    unsigned short _pin; //the pin the Servo motor is attached to
    unsigned int filterAngle(int degrees, int isRC);
    void setAllowedAngles();
    unsigned int _angle;
    unsigned int STRAIGHT_DEGREES; //the values (in degrees) that can get written to the steering motor, with 0 being the leftmost position, 180 the rightmost and 90 around the middle
    int MAX_RIGHT_ANGLE, MAX_LEFT_ANGLE;
};

class ESCMotor : public Servo {
public:
    explicit ESCMotor();
    void setSpeed(int speed);
    void brake();
    void arm();
    void init(unsigned short pin);
private:
    void setFreqsAndSpeeds();
    unsigned short _pin; //the pin the ESC is attached to
    int filterSpeed(int speed, int isRC);
    int _speed, _direction;
    unsigned short FULL_FORWARD, FULL_BACKWARD, IDLE_SPEED; //what percentage of the motor's power is allowed be used at most
    int IDLE_RAW_SPEED, MAX_FRONT_RAW_SPEED, MAX_BACK_RAW_SPEED; //the raw value (in whatever metric, usually pwm signal) that the motors are idle, throttling full speed backwards and forward
    int red, green, blue;
    void wait(double seconds);
    unsigned long currentMillis;
    long interval;
};

class Car {
public:
    explicit Car();
    void setUp();
    void run();
    void provideSensorsData();
    void rcControl();
    void automatedDrive();
    int readChannel1();
    int readChannel2();
    int isFunctionChanged();
    int isRCControllerOn();
    void establishContact(char toSend);
    void waitConnection();
    SteeringMotor getSteeringMotor();
    ESCMotor getEscMotor();
    UltrasonicSensor getUltrasonicFront();
    UltrasonicSensor getUltrasonicRight();
    InfraredSensor getInfraredSideFront();
    InfraredSensor getInfraredSideBack();
    InfraredSensor getInfraredBack();
    Odometer getWheelEncoder();
    protocol_state getProtocolState();
    void wait(double seconds);

private:
    int func_is_changed = -1;
    int notCenter = 0; //Flag to signal of channel 2 is in the center
    int _encoderPos = 0; //Variable to store encoder position
    int _odometer = 0; //Variable to store driving distance from the requested point
    int _odometerStart = 0; //Flag to start or stop encoder values sending
    SteeringMotor steeringMotor;
    ESCMotor escMotor;
    UltrasonicSensor ultrasonicFront, ultrasonicRight;
    InfraredSensor infraredSideFront, infraredSideBack, infraredBack;
    Odometer wheelEncoder;
    protocol_state state;
    int red, green, blue;
    int encoderPos, odometerStart, odometer;
    unsigned long currentMillis;
    long interval;
};

#endif
#pragma clang diagnostic pop