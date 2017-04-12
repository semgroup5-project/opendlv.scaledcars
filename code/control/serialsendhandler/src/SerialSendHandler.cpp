#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include <iostream>

#include "SerialSendHandler.h"

namespace scaledcars {
namespace control {

using namespace std;

// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace odcore;
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::wrapper;


SerialSendHandler::SerialSendHandler(const int32_t &argc, char **argv) :
    DataTriggeredConferenceClientModule(argc, argv, "SerialSendHandler")
{}

SerialSendHandler::~SerialSendHandler() {}

void SerialSendHandler::setUp() {
    cout << "This method is called before the component's body is executed." << endl;
}

void SerialSendHandler::tearDown() {
    cout << "This method is called after the program flow returns from the component's body." << endl;
}

void SerialSendHandler::nextContainer(Container &/*c*/) {
    const string SERIAL_PORT = "/dev/ttyACM1";
    const uint32_t BAUD_RATE = 11500;

    // We are using OpenDaVINCI's std::shared_ptr to automatically
    // release any acquired resources.
    
    //if (c.getDataType() == VehicleControl::ID()) {
      //  VehicleControl vd = c.getData<VehicleControl>();
    	try {
        	std::shared_ptr<SerialPort> serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));

        	serial->send("Hello World\r\n");
        	//serial->send(vd.toString());
    	}
    	catch(string &exception) {
        	cerr << "Serial port could not be created: " << exception << endl;
    	}

    //}
}


}
}

