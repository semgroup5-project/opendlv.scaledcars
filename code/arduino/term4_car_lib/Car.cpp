#include "term4_car_lib.h"

Car::Car() {}

void Car::setUp() {
    timer = 5000;
    noData = 0;
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
    encoderPos = wheelEncoder.getDistance();

    Serial.begin(BAUD); //start the serial
    waitConnection();
    while (Serial.available()) { //empty any rubbish value from the buffer
        byte clean = Serial.read();
    }
    establishContact('\n');
}

void Car::run() {
//    if (!isRCControllerOn()) {
        automatedDrive();
//    } else {
//        rcControl();
//    }
//    provideSensorsData();
}

void Car::provideSensorsData() {
    int count = 0;
    while (count++ < 5) {
        infraredBack.encodeAndWrite(ID_IN_INFRARED_BACK, infraredBack.getDistance());
        infraredSideFront.encodeAndWrite(ID_IN_INFRARED_SIDE_FRONT, infraredSideFront.getDistance2());
        infraredSideBack.encodeAndWrite(ID_IN_INFRARED_SIDE_BACK, infraredSideBack.getDistance());

        int ur_val = ultrasonicFront.getDistance();
        if (ur_val > 60) {
            ultrasonicFront.encodeAndWrite(ID_IN_ULTRASONIC_CENTER, 0);
        } else {
            ultrasonicFront.encodeAndWrite(ID_IN_ULTRASONIC_CENTER, ur_val);
        }
//        ultrasonicRight.encodeAndWrite(ID_IN_ULTRASONIC_SIDE_FRONT, ultrasonicRight.getDistance());

        odometer = wheelEncoder.getDistance() - encoderPos;
        if (odometer <= 255) {
            wheelEncoder.encodeAndWrite(ID_IN_ENCODER, odometer);
            encoderPos = wheelEncoder.getDistance();
        } else {
            wheelEncoder.encodeAndWrite(ID_IN_ENCODER, 255);
            encoderPos = wheelEncoder.getDistance();
        }
    }
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
//    if (isFunctionChanged()) {
//        escMotor.brake();
//    }
//
//    func_is_changed = 0;

    readSerial();
}

void Car::readSerial() {
    oldMillis = millis();

    int value = 90, serial_size = 0, count = 0;
    while ((serial_size = Serial.available()) <= 0) {
        if ((millis() - oldMillis) > timer) {
            noData = 1;
            break;
        }
    }
    dataMotor.id = 0;
    dataMotor.value = 0;
    dataServo.id = 0;
    dataServo.value = 0;
    if (!noData) {
        while (count++ < serial_size) {
            protocol_frame frame;
            frame.a = Serial.read();
            protocol_data data = protocol_decode_t1(frame);

            if (data.id == ID_OUT_SERVO) {
                dataServo = data;
            } else if (data.id == ID_OUT_MOTOR) {
                dataMotor = data;
            }
        }


        if (dataServo.id == ID_OUT_SERVO) {
            value = dataServo.value * 3;
            if (value >= 0 && value <= 180) {
                steeringMotor.setAngle(value, 0);
            }
        }

        if (dataMotor.id == ID_OUT_MOTOR) {
            value = dataMotor.value * 3;
            if (value > 185) {
                escMotor.brake();//applying values greater than 180 will be our indicative to brake
            } else {
                if (value >= 0 && value <= 180) {
                    escMotor.setSpeed(value);
                }
            }
        }
    } else {
        escMotor.brake();
        steeringMotor.setAngle(90, 0);
    }
    noData = 0;
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
        wait(0.5);
    }
    red = 0;
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
    Serial.read();
    wait(5);
    escMotor.arm();
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