#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include <iostream>
#include "automotivedata/generated/automotive/VehicleControl.h"
#include "automotivedata/generated/automotive/VehicleData.h"
#include "automotivedata/generated/automotive/miniature/SensorBoardData.h"
#include "opendavinci/odcore/data/Container.h"

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
    DataTriggeredConferenceClientModule(argc, argv, "SerialSendHandler"){}

SerialSendHandler::~SerialSendHandler() {}

void SerialSendHandler::setUp() {
   cout << "This method is called before the component's body is executed." << endl;
}

void SerialSendHandler::tearDown() {
    cout << "This method is called after the program flow returns from the component's body." << endl;
}

void SerialSendHandler::nextContainer(Container &c) {	
   if (c.getDataType() == automotive::VehicleData::ID()) {
   	send("VD");
   	cout << "This is VehicleData: " << c.getData<automotive::VehicleData>().getAbsTraveledPath() << endl;
   	
   } else if (c.getDataType() == automotive::miniature::SensorBoardData::ID()) {
   	send("SBD");
   	cout << "This is SensorBoardData: " << c.getData<automotive::miniature::SensorBoardData>().getValueForKey_MapOfDistances(0) << endl;
   
   } else if (c.getDataType() == automotive::VehicleControl::ID()) {
      const automotive::VehicleControl vd = c.getData<automotive::VehicleControl>();
      int angle = 180; //vd.getSteeringWheelAngle();
    	int speed = 105; //vd.getSpeed();
      //int odometer = TODO ;
      
    	string speedMessage = pack(1, speed);
      string angleMessage = pack(2, angle);
     	//string odometerMessage = pack(3, odometer); TODO
     
    	send(speedMessage);
    	send(angleMessage);
    	//send(odometerMessage); TODO
    	cerr << "Yay" << endl;
	}
//	cerr << "Nay" << endl;
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

void SerialSendHandler::simpleMessage(automotive::VehicleControl vd){
	int angle = vd.getSteeringWheelAngle();
	cerr << "angle: " << angle << endl;
   int speed = vd.getSpeed();
   cerr << "speed: " << speed << endl;

   string message1 = "";
   string message2 = "";

   if ((int) vd.getSpeed() != -1) {
   	message1 = "m" + to_string(speed) + "\n";
   }
   
   if ((int) vd.getSteeringWheelAngle() != -1){
   	message2 = "t" + to_string(angle) + "\n";
   }
	
	cerr << "angleS: " << message2 << endl;
	cerr << "speedS: " << message1 << endl;
	
	send(message1);
	send(message2);
}

void SerialSendHandler::send(string message){
//	const string SERIAL_PORT = "/dev/ttyACM1";
//	const uint32_t BAUD_RATE = 115200;
	
	try {
		std::shared_ptr <SerialPort> serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));
		serial->send(message);
		cerr << "I'm " << message << endl;
	}
   catch (string &exception) {
   	cerr << "Serial port could not be created: " << exception << endl;
   }
}

} /*---------*/
} /*NAMESPACE*/

