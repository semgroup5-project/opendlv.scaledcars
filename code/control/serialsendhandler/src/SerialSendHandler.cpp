#include "SerialSendHandler.h"

#include <iostream>
#include <memory>
#include <stdint.h>
#include <string>

#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>

#include "protocol.c"
#include "serial.c"
#include "arduino.c"

#define pi 3.1415926535897

using namespace std;

using namespace odcore;
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::wrapper;
using namespace odcore::data::dmcp;

namespace scaledcars {
    namespace control {

        void __on_read(uint8_t b)
        {
            cout << ">> read " << (int)b << endl;
        }

        void __on_write(uint8_t b)
        {
            cout << "<< write " << (int)b << endl;
        }

        SerialSendHandler::SerialSendHandler(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "SerialSendHandler")
        {}

        SerialSendHandler::~SerialSendHandler() {}

        void SerialSendHandler::setUp() {
            cerr << "Setting up serial handler"<< endl;

            this->serial = serial_new();
            this->serial->on_write = &__on_write;
            this->serial->on_read = &__on_read;

            serial_open(this->serial, "/dev/ttyACM0", 115200);
            cerr << "serial open" << endl;
            serial_handshake(this->serial, '\n');
            cerr << "serial handshake" << endl;
            serial_start(this->serial);
            cerr << "serial start" << endl;
        }

        void SerialSendHandler::tearDown() {
            cerr << "Shutting down serial handler" << endl;

            serial_stop(this->serial);
            serial_free(this->serial);
        }

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode SerialSendHandler::body() {
            cout << "TEST" << endl;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == ModuleStateMessage::RUNNING) {
                cout << "CYCLE " << cycle << endl;
                cycle++;

                protocol_data d_motor;
                d_motor.id = ID_OUT_MOTOR;
                d_motor.value = this->motor;

                protocol_data d_servo;
                d_servo.id = ID_OUT_SERVO;
                d_servo.value = this->servo;

                serial_send(this->serial, d_motor);
                serial_send(this->serial, d_servo);
            }

            return ModuleExitCodeMessage::OKAY;
        }

        void SerialSendHandler::nextContainer(Container &c) {
                cerr << "NEXT CONTAINER " << c.getDataType() << endl;
                if (c.getDataType() == automotive::VehicleControl::ID()) {
                    cerr << "VehicleControl" << endl;
                    const automotive::VehicleControl vd =
                            c.getData<automotive::VehicleControl>();
                    int arduinoAngle = 0;
                    double angle = vd.getSteeringWheelAngle();
                    cerr << "angle radius : " << angle << endl;

                    arduinoAngle = 90 + (angle * (180 / pi));
                    if (arduinoAngle < 0) {
                        arduinoAngle = 0;
                    } else if(arduinoAngle > 180){
                        arduinoAngle = 180;
                    }
                    cerr << "angle degree " << arduinoAngle << endl;

                    int speed = vd.getSpeed();
                    cerr << "speed to arduino : " << speed << endl;

//                    // TODO: int odometer = vd.getOdometer();



//  TODO SEND
//                    string speedMessage = pack(ID_OUT_MOTOR, speed);
//                    string angleMessage = pack(ID_OUT_SERVO, arduinoAngle);
//                    TODO: string odometerMessage = pack(ID_OUT_ODOMETER, odometer);

                    this->motor = speed;
                    this->servo = arduinoAngle;

                }
        }
    }
}
