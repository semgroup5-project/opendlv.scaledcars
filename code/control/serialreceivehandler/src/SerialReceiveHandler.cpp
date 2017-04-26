#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>

#include "SerialReceiveHandler.h"
#include "SerialListener.h"

namespace scaledcars {
	namespace control {

using namespace std;

// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore;
using namespace odcore::wrapper;

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
    	const string SERIAL_PORT = "/dev/ttyACM0";
    	const uint32_t BAUD_RATE = 115200;

    	// We are using OpenDaVINCI's std::shared_ptr to automatically
    	// release any acquired resources.
    	try {
        std::shared_ptr<SerialPort> serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));

        // This instance will handle any bytes that are received
        // from our serial port.
        SerialListener handler;
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
	return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

}
}



