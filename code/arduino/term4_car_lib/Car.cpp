#include "term4_car_lib.h"

Car::Car() {}

void Car::setUp() {
    pinMode(CH_1, INPUT);
    pinMode(CH_2, INPUT);

    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);

    escMotor.init(ESC_PIN);
    steeringMotor.init(SERVO_PIN);
    ultrasonicFront.attach(US_C);
    ultrasonicRight.attach(US_R);
    infraredSideFront.attach(SIDE_FRONT_PIN);
    infraredSideBack.attach(SIDE_BACK_PIN);
    infraredBack.attach(BACK_PIN);
    wheelEncoder.attach(ENCODER_PIN_A, ENCODER_PIN_B, true);
    wheelEncoder.begin();

    Serial.begin(BAUD); //start the serial
    waitConnection();
}

void Car::provideSensorsData() {
    infraredBack.encodeAndWrite(ID_IN_INFRARED_BACK, infraredBack.getDistance());
    infraredSideFront.encodeAndWrite(ID_IN_INFRARED_SIDE_FRONT, infraredSideFront.getDistance());
    infraredSideBack.encodeAndWrite(ID_IN_INFRARED_SIDE_BACK, infraredSideBack.getDistance());

    odometer = wheelEncoder.getDistance() - encoderPos;
    if (odometer > 255) {
        encoderPos = wheelEncoder.getDistance();
        odometer -= 255;
    }
    if (odometerStart) {
        wheelEncoder.encodeAndWrite(ID_IN_ENCODER, odometer);
    }

    ultrasonicFront.encodeAndWrite(ID_IN_ULTRASONIC_CENTER, ultrasonicFront.getDistance());
    ultrasonicRight.encodeAndWrite(ID_IN_ULTRASONIC_SIDE_FRONT, ultrasonicRight.getDistance());
}

void Car::rcControl() {
    if (!isFunctionChanged()) {
        escMotor.brake();
    }

    func_is_changed = 1;
#ifdef COMMON_ANODE
    red = 255;
    green = 0;
    blue = 255;
#endif
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);

    steeringMotor.setAngle(readChannel1(), 1); // steer channel;
    escMotor.setSpeed(readChannel2()); // drive channel;
}

void Car::automatedDrive() {
    if (isRCControllerOn()) {
        escMotor.brake();
    }

//    func_is_changed = 0;
//
//
//#ifdef COMMON_ANODE
//    red = 255;
//    green = 255;
//    blue = 0;
//#endif
//    analogWrite(redPin, red);
//    analogWrite(greenPin, green);
//    analogWrite(bluePin, blue);


    while (!Serial.available());
    //establishContact('0');

    byte in = Serial.read();
    protocol_receive(&state, in);

    if (state.valid) {
        Serial.println(state.data.id);
        Serial.println(state.data.value);

        if (state.data.id == ID_OUT_MOTOR) //incoming motor instructions
        {
            if (state.data.value > 180) escMotor.brake(); //applying values greater than 180 will be our indicative to brake
            else {
                escMotor.setSpeed(state.data.value);
            }
            wait(1);
        } else if (state.data.id == ID_OUT_SERVO) //incoming steering instructions
        {
            steeringMotor.setAngle(state.data.value, 0);
            wait(1);
        } else if (state.data.id == ID_OUT_ODOMETER) //incoming odometer instructions
        {
            odometerStart = state.data.value ? 1
                                  : 0; //change flag to to start sending the odometer data 1 to send 0 to to not send
            encoderPos = odometerStart ? wheelEncoder.getDistance()
                                       : 0; //if we are sending odometer data set the position for the actual position of the car
        }
        //state.valid = false;
    }
}

int Car::readChannel1() {
    return pulseIn(CH_1, HIGH); // steer
}

int Car::readChannel2() {
    return pulseIn(CH_2, HIGH); // steer
}

int Car::isFunctionChanged() {
    return func_is_changed;
}

int Car::isRCControllerOn() {
    return readChannel1();
}

void Car::establishContact(char toSend) {
    while (Serial.available() <= 0) {
        Serial.print(toSend);   // send a char
        wait(1);
    }
}

void Car::waitConnection() {
    while (!Serial); // wait for serial port to connect. Needed for native USB port only
}

SteeringMotor Car::getSteeringMotor(){
    return steeringMotor;
}

ESCMotor Car::getEscMotor(){
    return escMotor;
}

UltrasonicSensor Car::getUltrasonicFront(){
    return ultrasonicFront;
}

UltrasonicSensor Car::getUltrasonicRight(){
    return ultrasonicRight;
}

InfraredSensor Car::getInfraredSideFront(){
    return infraredSideFront;
}

InfraredSensor Car::getInfraredSideBack(){
    return infraredSideBack;
}

InfraredSensor Car::getInfraredBack(){
    return infraredBack;
}

Odometer Car::getWheelEncoder(){
    return wheelEncoder;
}

protocol_state Car::getProtocolState(){
    return state;
}

void Car::wait(double seconds){
    interval = seconds * 1000;
    currentMillis = millis();
    while (millis() - currentMillis <= interval);
}