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
   automotive::VehicleControl vc;
   vc.setSteeringWheelAngle(0);
   

	// Create container for finally sending the set values for the control algorithm.
   Container c2(vc);
   // Send container.
   getConference().send(c2);
   cout << "This method is called before the component's body is executed." << endl;
}

void SerialSendHandler::tearDown() {
    cout << "This method is called after the program flow returns from the component's body." << endl;
}

void SerialSendHandler::nextContainer(Container &c) {
	//const string SERIAL_PORT = "/dev/ttyACM0";
	//const uint32_t BAUD_RATE = 11500;
	   cout << "Container <3" << endl;

	cout << "The correct id: " << automotive::VehicleControl::ID() << endl;
	cout << "Type of container: " << c.getDataType() << endl;

    // We are using OpenDaVINCI's std::shared_ptr to automatically
    // release any acquired resources.
    
   //if (c.getDataType() == automotive::VehicleControl::ID()) {
   	   cout << "if statement <3" << endl;
      const automotive::VehicleControl vd = c.getData<automotive::VehicleControl>();
//      int angle = vd.getSteeringWheelAngle();
//      int speed = vd.getSpeed();
      protocol_data pd;
      protocol_data *ppd = &pd;
      ppd->id = 2;
      ppd->value = 180;
      
      protocol_frame pf = protocol_encode(pd);
      protocol_frame *ppf = &pf;
      
      string test = "";
      test.insert(test.end(), ppf->a);
      test.insert(test.end(), ppf->b);
      
      cout << test << endl;
      
    	try {
        	//std::shared_ptr<SerialPort> serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));
			
        	// serial->send(test);
        		   cout << "Sexy cream sent by serial <3" << endl;
        	//serial->send(vd.toString());
    	}
    	catch(string &exception) {
        	cerr << "Serial port could not be created: " << exception << endl;
    	}
	//}
}

} /*---------*/
} /*NAMESPACE*/

