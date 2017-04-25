#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include "automotivedata/generated/automotive/VehicleControl.h"
#include "automotivedata/generated/automotive/VehicleData.h"
#include "automotivedata/generated/automotive/miniature/SensorBoardData.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/io/conference/ContainerConferenceFactory.h"

#include "SerialReceiveHandler.h"
#include "protocol.c"

namespace scaledcars {
	namespace control {

using namespace std;

// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore;
using namespace odcore::wrapper;
using namespace odcore::io::conference;
using namespace automotive;
using namespace automotive::miniature;

SerialReceiveHandler s_instance();

void SerialReceiveHandler::nextString(const string &s) {
    		protocol_state st;
    		protocol_state *state = &st;

    		protocol_receive_t2(state, s.at(0));
    		protocol_receive_t2(state, s.at(1));
			//int i = stoi(s,nullptr,0);
			//cout << "Received " << s.length() << " bytes containing '" << s << "'" << endl;
			
    		cerr << "[RECIEVED VALUE] ID: " << state->data.id << " VALUE: " << state->data.value << endl;
			//filterIntData(id, value);
    		
    		if(state->valid)
    			filterData(state);
    	}
    	
void SerialReceiveHandler::filterData(protocol_state *state){
	SensorBoardData sbd;
	VehicleData vd;
	map<uint32_t, double> sensor;
	
	if((state->data.id == 1 || state->data.id == 2) && state->data.value >= 1 && state->data.value <= 70){
		//US-SENSOR
		sensor[state->data.id - 1] = state->data.value;
		sbd.setMapOfDistances(sensor);
		Container c(sbd);
		getConference().send(c);
		cerr << "[VALUE TO CONFERENCE] ID: " << state->data.id << " VALUE: " << state->data.value << endl;
						
	} else if ((state->data.id == 3 || state->data.id == 4 || state->data.id == 5) && state->data.value >= 3 && state->data.value <= 40){
		//IR-SENSOR
		
		sensor[state->data.id - 1] = state->data.value;
		sbd.setMapOfDistances(sensor);
		Container c(sbd);
		getConference().send(c);
		cerr << "[VALUE TO CONFERENCE] ID: " << state->data.id << " VALUE: " << state->data.value << endl;
						
						
	} else if (state->data.id == 6 && state->data.value >= 0 && state->data.value <= 255){ 
		//ODOMETER
		vd.setAbsTraveledPath(state->data.value);
		Container c(vd);
		getConference().send(c);
		cerr << "[VALUE TO CONFERENCE] ID: " << state->data.id << " VALUE: " << state->data.value << endl;
		
	}
}

void SerialReceiveHandler::filterIntData(int id, int value){
	SensorBoardData sbd;
	VehicleData vd;
	map<uint32_t, double> sensor;
	
	if(id == 1 && value >= 1 && value <= 70){
		//US-SENSOR
		sensor[id - 1] = value;
		sbd.setMapOfDistances(sensor);
		Container c(sbd);
		getConference().send(c);
						
	} else if (id == 2 && value >= 3 && value <= 40){
		//IR-SENSOR
		
		sensor[id - 1] = value;
		sbd.setMapOfDistances(sensor);
		Container c(sbd);
		getConference().send(c);
						
						
	} else if (id == 3 && value >= 0 && value <= 255){ 
		//ODOMETER
		vd.setAbsTraveledPath(value);
		Container c(vd);
		getConference().send(c);
		
	}
}

SerialReceiveHandler::SerialReceiveHandler(const int32_t &argc, char **argv) :
    TimeTriggeredConferenceClientModule(argc, argv, "SerialReceiveHandler")
    {}

SerialReceiveHandler::~SerialReceiveHandler() {}

void SerialReceiveHandler::setUp() {
    cout << "This method is called before the component's body is executed." << endl;
}

void SerialReceiveHandler::tearDown() {
    cout << "This method is called after the program flow returns from the component's body." << endl;
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode SerialReceiveHandler::body() {
    while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    	const string SERIAL_PORT = "/dev/ttyACM2";
    	const uint32_t BAUD_RATE = 115200;

    	cout << "I'm here" << endl;
    	
    	// We are using OpenDaVINCI's std::shared_ptr to automatically
    	// release any acquired resources.
    	try {
        std::shared_ptr<SerialPort> serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));

        // This instance will handle any bytes that are received
        // from our serial port.
        
        serial->setStringListener(this);

        // Start receiving bytes.
        serial->start();

        const uint32_t ONE_SECOND = 1000 * 1000;
        odcore::base::Thread::usleepFor(10 * ONE_SECOND);

        // Stop receiving bytes and unregister our handler.
        serial->stop();
        serial->setStringListener(NULL);
        
        cout << "Now I'm here" << endl;
    	}
    	catch(string &exception) {
        cerr << "Error while creating serial port: " << exception << endl;
    	}
   }
	return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

}
}



