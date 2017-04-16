#include "SerialSendHandler.h"

#include <iostream>
#include <memory>
#include <stdint.h>
#include <string>

#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>

#include "protocol.c"

using namespace std;

using namespace odcore;
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::wrapper;

namespace scaledcars {
    namespace control {
        void SerialReceiveListener::nextString(const string &s) {
            cerr << "recv: '" << s << "'" << endl;
        }

        SerialSendHandler::SerialSendHandler(const int32_t &argc, char **argv)
            : DataTriggeredConferenceClientModule(argc, argv, "SerialSendHandler")
        {}

        SerialSendHandler::~SerialSendHandler() {}

        void SerialSendHandler::setUp() {
            const string SERIAL_PORT = "/dev/ttyACM0";
            const uint32_t BAUD_RATE = 115200;

            cerr << "port: " << SERIAL_PORT << " br: " << BAUD_RATE << endl;

            try {
                shared_ptr<SerialPort> _serialPort(
                    SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));

                this->serialPort = _serialPort;
                this->serialPort->setStringListener(&(this->serialListener));
                this->serialPort->start();

                // Wait for serial port to be ready for communication
                cerr << "sleeping for 10 secs" << endl;
                const uint32_t ONE_SECOND = 1000 * 1000;
                odcore::base::Thread::usleepFor(10 * ONE_SECOND);

            } catch (string &exception) {
                cerr << "serial error: " << exception << endl;
            }
        }

        void SerialSendHandler::tearDown() {
            this->serialPort->stop();
            this->serialPort->setStringListener(NULL);
        }

        void SerialSendHandler::nextContainer(Container &c) {
            if (c.getDataType() == automotive::VehicleControl::ID()) {
                const automotive::VehicleControl vd =
                    c.getData<automotive::VehicleControl>();

                int angle = vd.getSteeringWheelAngle();
                int speed = vd.getSpeed();
                // TODO: int odometer = vd.getOdometer();

                string speedMessage = pack(ID_OUT_MOTOR, speed);
                string angleMessage = pack(ID_OUT_SERVO, angle);
                // TODO: string odometerMessage = pack(ID_OUT_ODOMETER, odometer);

                send(speedMessage);
                send(angleMessage);
                // TODO: send(odometerMessage);
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
            cerr << "sending!!" << endl;

            // baudrate-adjusted throttle (determines throughput)
            // 1 / BAUDRATE * WORDSIZE = seconds
            // WORDSIZE = 1 start + 8 bits + 1 stop = 10

            // 1 / 115200 * 10 = 87 us (use as delay)

            //const uint32_t ONE_MS = 1000 * 1;
            //odcore::base::Thread::usleepFor(1 * ONE_MS);
            //odcore::base::Thread::usleepFor(87);


            this->serialPort->send(message);

            cerr << "sending done" << endl;
        }
    }
}
