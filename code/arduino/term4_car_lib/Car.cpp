#include "term4_car_lib.h"

Car::Car() {}

void Car::setUp() {
    pinMode(RELAY_PIN, OUTPUT);

    digitalWrite(RELAY_PIN, HIGH);

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
    while (Serial.available()) { //empty any rubbish value from the buffer
        red = 255;
        green = 0;
        blue = 0;
#ifdef COMMON_ANODE
        red = 255 - red;
        green = 255 - green;
        blue = 255 - blue;
#endif
        analogWrite(redPin, red);
        analogWrite(greenPin, green);
        analogWrite(bluePin, blue);
        byte clean = Serial.read();
    }
    establishContact('\n');
}

void Car::run() {
    if (!isRCControllerOn()) {
        automatedDrive();
    } else {
        rcControl();
    }
    provideSensorsData();


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
    Serial.read();

    red = 0;
    green = 255;
    blue = 0;
#ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
#endif
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);

    steeringMotor.setAngle(readChannel1(), 1); // steer channel;
    escMotor.setSpeed(readChannel2()); // drive channel;
}

void Car::automatedDrive() {
    if (isFunctionChanged()) {
        escMotor.brake();
    }

    func_is_changed = 0;

    red = 0;
    green = 0;
    blue = 255;
#ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
#endif
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);

    int value = 90, serial_size = 0, count = 0;
    byte in;
    while ((serial_size = Serial.available()) <= 0 && !isRCControllerOn());

    while (count++ < serial_size) {
        in = Serial.read();
    }

    protocol_frame frame;
    frame.a = in;
    protocol_data data = protocol_decode_t1(frame);
//    Serial.print(data.id);
//    Serial.print(" ");
//    Serial.println(data.value);

    if (data.id == ID_OUT_SERVO) {
        value = data.value * 3;
        if (value >= 0 && value <= 180) {
            steeringMotor.setAngle(value, 0);
        }
    }

    if (data.id == ID_OUT_MOTOR) {
        value = data.value * 3;
        if (value > 180){
            escMotor.brake();//applying values greater than 180 will be our indicative to brake
        }
        else {
            if (value >= 0 && value <= 180) {
                escMotor.setSpeed(value);
            }
        }
    }
    if (isRCControllerOn()) {
        escMotor.brake();
    }
}

int Car::readChannel1() {
    return pulseIn(CH_1, HIGH, 10000); // steer
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
        red = 0;
        green = 255;
        blue = 255;
#ifdef COMMON_ANODE
        red = 255 - red;
        green = 255 - green;
        blue = 255 - blue;
#endif
        analogWrite(redPin, red);
        analogWrite(greenPin, green);
        analogWrite(bluePin, blue);
        Serial.println(toSend);   // send a char
    }
}

void Car::waitConnection() {
    while (!Serial); // wait for serial port to connect. Needed for native USB port only
}

SteeringMotor Car::getSteeringMotor() {
    return steeringMotor;
}

ESCMotor Car::getEscMotor() {
    return escMotor;
}

UltrasonicSensor Car::getUltrasonicFront() {
    return ultrasonicFront;
}

UltrasonicSensor Car::getUltrasonicRight() {
    return ultrasonicRight;
}

InfraredSensor Car::getInfraredSideFront() {
    return infraredSideFront;
}

InfraredSensor Car::getInfraredSideBack() {
    return infraredSideBack;
}

InfraredSensor Car::getInfraredBack() {
    return infraredBack;
}

Odometer Car::getWheelEncoder() {
    return wheelEncoder;
}

protocol_state Car::getProtocolState() {
    return state;
}

void Car::wait(double seconds) {
    interval = seconds * 1000;
    currentMillis = millis();
    while (millis() - currentMillis <= interval);
}