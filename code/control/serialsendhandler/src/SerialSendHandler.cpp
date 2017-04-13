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
    DataTriggeredConferenceClientModule(argc, argv, "SerialSendHandler")
{}

SerialSendHandler::~SerialSendHandler() {}

void SerialSendHandler::setUp() {
    cout << "This method is called before the component's body is executed." << endl;
}

void SerialSendHandler::tearDown() {
    cout << "This method is called after the program flow returns from the component's body." << endl;
}

void SerialSendHandler::nextContainer(Container &c) {
	const string SERIAL_PORT = "/dev/ttyACM1";
	const uint32_t BAUD_RATE = 11500;

    // We are using OpenDaVINCI's std::shared_ptr to automatically
    // release any acquired resources.
    
   if (c.getDataType() == automotive::VehicleControl::ID()) {
      const automotive::VehicleControl vd = c.getData<automotive::VehicleControl>();
      int angle = vd.getSteeringWheelAngle();
//      int speed = vd.getSpeed();
      protocol_data pd;
      protocol_data *ppd = &pd;
      ppd->id = 2;
      ppd->value = angle;
      
      protocol_frame pf = protocol_encode(pd);
      protocol_frame *ppf = &pf;
      
      string test = "" + ppf->a + ppf->b;
      
    	try {
        	std::shared_ptr<SerialPort> serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));
			
        	serial->send(test);
        	//serial->send(vd.toString());
    	}
    	catch(string &exception) {
        	cerr << "Serial port could not be created: " << exception << endl;
    	}
	}
}

} /*---------*/
} /*NAMESPACE*/

