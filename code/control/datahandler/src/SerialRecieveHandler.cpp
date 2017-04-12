#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include "protocol.h"

#include "SerialRecieveHandler.hpp"

using namespace std;



void SerialRecieveHandler::filterProtocol(const protocol_data /*pd*/){
	//Container c;
	//SensorBoardData sbd;
	//VehicleData vd;
	
	//TODO SEBASTIAN filter stuff
	
	
	if(/*TODO SEBASTIAN if its sensors go here*/ 1){
	//	sbd(/* PUT THE FILTERED DATA HERE*/ NULL);
	//	c(sbd);
	//	getConference().send(c);
   } 
   
   if (/*TODO SEBASTIAN if its other vehicle data stuff go here*/ 1){ 
   //	vd(/* PUT THE FILTERED DATA HERE*/NULL);
   // 	c(vd)
	//	getConference().send(c);
   }
    	
}

void SerialRecieveHandler::nextString(const string &s) {

	//TODO MATTIAS Uncomment this when protocol is properly integrated
    for(int i = 0; i < (int) s.length(); i++){
    	//protocol_data d = protocol_decode(s[i]);
    	//filterProtocol(d); 
    }
}

// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace odcore;
using namespace odcore::wrapper;

int32_t main(int32_t /*argc*/, char **/*argv*/) {
    const string SERIAL_PORT = "/dev/ttyACM1";
    const uint32_t BAUD_RATE = 19200;

    // We are using OpenDaVINCI's std::shared_ptr to automatically
    // release any acquired resources.
    try {
        std::shared_ptr<SerialPort>
            serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));

        // This instance will handle any bytes that are received
        // from our serial port.
        SerialRecieveHandler handler;
        serial->setStringListener(&handler);

        // Start receiving bytes.
        serial->start();

        const uint32_t ONE_SECOND = 1000 * 1000;
        odcore::base::Thread::usleepFor(10 * ONE_SECOND);

        // Stop receiving bytes and unregister our handler.
        serial->stop();
        serial->setStringListener(NULL);
    }
    catch(string &exception) {
        cerr << "Error while creating serial port: " << exception << endl;
    }
}


