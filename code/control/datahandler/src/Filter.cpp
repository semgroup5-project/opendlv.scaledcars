#include <iostream>

#include "Filter.h"
#include "opendavinci/odcore/data/TimeStamp.h"

using namespace std;

// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace odcore::base::module;
using namespace odcore::data;

Filter::Filter(const int32_t &argc, char **argv) :
    DataTriggeredConferenceClientModule(argc, argv, "Filter")
{}

Filter::~Filter() {}

void Filter::setUp() {
    cout << "This method is called before the component's body is executed." << endl;
}

void Filter::tearDown() {
    cout << "This method is called after the program flow returns from the component's body." << endl;
}

void Filter::nextContainer(Container &c) {
    cout << "Received container of type " << c.getDataType() <<
                              " sent at " << c.getSentTimeStamp().getYYYYMMDD_HHMMSSms() <<
                          " received at " << c.getReceivedTimeStamp().getYYYYMMDD_HHMMSSms() << endl;

    if (c.getDataType() == SensorBoardData::ID()) {
    	SensorBoardData sbd = c.getData<SensorBoardData>();
    	if(sbd.getValueForKey_MapOfDistances(0) >= 0 && sbd.getValueForKey_MapOfDistances(0) =< 100)
    		//TODO Conference it!
    }
    
    if (c.getDataType() == VehicleData::ID()) {
    	VehicleData vd = c.getData<VehicleData>();
    	//TODO filter thing
    }
}

int32_t main(int32_t argc, char **argv) {
    DataTriggeredReceiver dtr(argc, argv);

    return dtr.runModule();
}
