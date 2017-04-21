#include "SerialSendHandler.h"

#include <iostream>
#include <memory>
#include <stdint.h>
#include <string>

#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>

#include "protocol.c"

#define pi 3.1415926535897

using namespace std;

using namespace odcore;
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::wrapper;

namespace scaledcars {
    namespace control {

        int port = 0;
        const string SERIAL_PORTS[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"};
        const uint32_t BAUD_RATE = 115200;
        FILE *file;

        void SerialReceiveListener::nextString(const string &s) {
            cerr << "RECEIVED: " << s << " CONTAINS: " << s.length() << " BYTES!" << endl;
        }

        SerialSendHandler::SerialSendHandler(const int32_t &argc, char **argv)
                : DataTriggeredConferenceClientModule(argc, argv, "SerialSendHandler") {}

        SerialSendHandler::~SerialSendHandler() {}

        void SerialSendHandler::setUp() {
            cerr << "Connecting to port: " << SERIAL_PORTS[port] << " br: " << BAUD_RATE << endl;

            try {
                shared_ptr <SerialPort> _serialPort(
                        SerialPortFactory::createSerialPort(SERIAL_PORTS[port], BAUD_RATE));

                this->serialPort = _serialPort;
                this->serialPort->setStringListener(&(this->serialListener));
                this->serialPort->start();

                // Wait for serial port to be ready for communication
                cerr << "Sleeping for 5 secs" << endl;
                const uint32_t ONE_SECOND = 1000 * 1000;
                odcore::base::Thread::usleepFor(5 * ONE_SECOND);

            } catch (string &exception) {
                cerr << "Serial error : " << exception << endl;
                port++;
                if (port < 4) {
                    cerr << "Trying port : " << SERIAL_PORTS[port] << endl;
                    setUp();
                }
            }
        }

        void SerialSendHandler::tearDown() {
            cerr << "Shutting down port: " << SERIAL_PORTS[port] << endl;
            this->serialPort->stop();
            this->serialPort->setStringListener(NULL);
        }

        void SerialSendHandler::nextContainer(Container &c) {
            try {
                if (c.getDataType() == automotive::VehicleControl::ID()) {
                    const automotive::VehicleControl vd =
                            c.getData<automotive::VehicleControl>();
                    int arduinoAngle = 0;
                    double angle = vd.getSteeringWheelAngle();
                    cerr << "angle radius : " << angle << endl;

                    arduinoAngle = 90 + (angle * (180 / pi));

                    // introduce top and bottom limits for angle values sent to the car
                    if (arduinoAngle < 0 ){
                        arduinoAngle = 10;
                    }else if (arduinoAngle > 180){
                        arduinoAngle = 160;
                    }
                    cerr << "angle degree " << arduinoAngle << endl;

                    //(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
                   // int speed = 105;
                    int speed = vd.getSpeed();
                    cerr << "speed to arduino : " << speed << endl;
                    // TODO: int odometer = vd.getOdometer();

                    string speedMessage = pack(ID_OUT_MOTOR, speed);
                    string angleMessage = pack(ID_OUT_SERVO, arduinoAngle);

                    // TODO: string odometerMessage = pack(ID_OUT_ODOMETER, odometer);

                    send(speedMessage);
                    send(angleMessage);

                    // TODO: send(odometerMessage);
                }
            } catch (string &exception) {
                cerr << "Serial Error : " << exception << endl;
            }
        }

        string SerialSendHandler::pack(int id, int value) {
            protocol_data protocolData;
            protocol_data *pointerProtocolData = &protocolData;

            pointerProtocolData->id = id;
            pointerProtocolData->value = value;

            protocol_frame protocolFrame = protocol_encode(protocolData);
            protocol_frame *pointerProtocolFrame = &protocolFrame;

            string message = "";
            message.insert(message.end(), pointerProtocolFrame->a);
            message.insert(message.end(), pointerProtocolFrame->b);

            return message;
        }

        void SerialSendHandler::send(string message) {
            //cerr << "sending : " << message << endl;

            // baudrate-adjusted throttle (determines throughput)
            // 1 / BAUDRATE * WORDSIZE = seconds
            // WORDSIZE = 1 start + 8 bits + 1 stop = 10

            // 1 / 115200 * 10 = 87 us (use as delay)


            //odcore::base::Thread::usleepFor(87);


            this->serialPort->send(message);
            //odcore::base::Thread::usleepFor(10000);

//            const uint32_t ONE_MS = 1000 * 1;
//            odcore::base::Thread::usleepFor(100 * ONE_MS);


            //cerr << "sending done" << endl;
        }
    }
}
