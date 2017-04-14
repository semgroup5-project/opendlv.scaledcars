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
	const string SERIAL_PORT = "/dev/ttyACM0";
	const uint32_t BAUD_RATE = 115200;

    // We are using OpenDaVINCI's std::shared_ptr to automatically
    // release any acquired resources.
    
   if (c.getDataType() == automotive::VehicleControl::ID()) {
      const automotive::VehicleControl vd = c.getData<automotive::VehicleControl>();
      int angle = vd.getSteeringWheelAngle();
      int speed = vd.getSpeed();
      //int odometer = TODO ;
      
      string speedMessage = pack(1, speed);
      string angleMessage = pack(2, angle);
     	//string odometerMessage = pack(3, odometer); TODO
     
    	try {
        	std::shared_ptr<SerialPort> serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));
			
        	serial->send(speedMessage);
        	serial->send(angleMessage);
        	//serial->send(odometerMessage); TODO
    	}
    	catch(string &exception) {
        	cerr << "Serial port could not be created: " << exception << endl;
    	}
	}
}

string SerialSendHandler::pack(int id, int value){  
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

} /*---------*/
} /*NAMESPACE*/

