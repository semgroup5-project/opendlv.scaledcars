#include <iostream>

#include "opendavinci/odcore/data/TimeStamp.h"

#include "SerialHandler.h"

using namespace std;

// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace odcore::base::module;
using namespace odcore::data;

SerialHandler::SerialHandler(const int32_t &argc, char **argv) :
    TimeTriggeredConferenceClientModule(argc, argv, "SerialHandler")
    {}

SerialHandler::~SerialHandler() {}

void SerialHandler::setUp() {
    cout << "This method is called before the component's body is executed." << endl;
}

void SerialHandler::tearDown() {
    cout << "This method is called after the program flow returns from the component's body." << endl;
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode SerialHandler::body() {
    uint32_t i = 0;
    while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
        cout << "Sending " << i << "-th time stamp data...";
        TimeStamp ts(i, 2*i++);
        Container c(ts);
        getConference().send(c);
        cout << "done." << endl;
    }

    return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

int32_t main(int32_t argc, char **argv) {
    SerialHandler tts(argc, argv);

    return tts.runModule();
}
