#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include <iostream>

#include "SerialSendHandler.h"
#include "protocol.c"

namespace scaledcars {
    namespace control {

        using namespace std;

// We add some of OpenDaVINCI's namespaces for the sake of readability.
        using namespace odcore;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace odcore::wrapper;


        SerialSendHandler::SerialSendHandler(const int32_t &argc, char **argv) :
                DataTriggeredConferenceClientModule(argc, argv, "SerialSendHandler") {}

        SerialSendHandler::~SerialSendHandler() {}

        void SerialSendHandler::setUp() {
            cout << "This method is called before the component's body is executed." << endl;
        }

        void SerialSendHandler::tearDown() {
            cout << "This method is called after the program flow returns from the component's body." << endl;
        }

        void SerialSendHandler::nextContainer(Container &c) {
            const string SERIAL_PORT = "/dev/ttyACM0";
            const uint32_t BAUD_RATE = 115200;

            // We are using OpenDaVINCI's std::shared_ptr to automatically
            // release any acquired resources.

            if (c.getDataType() == automotive::VehicleControl::ID()) {
                const automotive::VehicleControl vd = c.getData<automotive::VehicleControl>();
                int angle = vd.getSteeringWheelAngle();
                cerr << "angle: " << angle << endl;
                int speed = vd.getSpeed();
                cerr << "speed: " << speed << endl;
//                protocol_data pds, pda;
//                protocol_data *ppds = &pds;
//                protocol_data *ppda = &pda;

                string message1 = "";
                string message2 = "";

                if ((int) vd.getSpeed() != -1) {
//                    ppds->id = 1;
//                    ppds->value = speed;
//                    cerr << "speed" << endl;
                    message1 = "m" + to_string(speed) + "\n";

                }
                if ((int) vd.getSteeringWheelAngle() != -1){
//                    ppda->id = 2;
//                    ppda->value = angle;
//                    cerr << "angle" << endl;
                    message2 = "t" + to_string(angle) + "\n";
                }

                // TODO - ADD SUPPORT FOR THE ODOMETER "ID 3" "ANY VALUE OVER 0"

//                protocol_frame pfs = protocol_encode(pds);
//                protocol_frame *ppfs = &pfs;
//
//                protocol_frame pfa = protocol_encode(pda);
//                protocol_frame *ppfa = &pfa;


//                message1.insert(message1.end(), ppfs->a);
//                message1.insert(message1.end(), ppfs->b);
//
//
//                message2.insert(message2.end(), ppfa->a);
//                message2.insert(message2.end(), ppfa->b);

                try {
                    std::shared_ptr <SerialPort> serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));

                    cerr << "angleS: " << message2 << endl;

                    cerr << "speedS: " << message1 << endl;

                    serial->send(message1);

                    serial->send(message2);
                }
                catch (string &exception) {
                    cerr << "Serial port could not be created : " << exception << endl;
                }
            }
        }

    } /*---------*/
} /*NAMESPACE*/

