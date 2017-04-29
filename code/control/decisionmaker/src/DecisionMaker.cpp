#include "DecisionMaker.h"

#include <iostream>
#include <memory>
#include <stdint.h>
#include <string>

#include <opendavinci/odcore/base/Thread.h>
#include "automotivedata/generated/automotive/VehicleData.h"
#include "automotivedata/generated/automotive/miniature/SensorBoardData.h"

using namespace std;

using namespace odcore;
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::wrapper;
using namespace odcore::data::dmcp;

namespace scaledcars {
    namespace control {
        DecisionMaker::DecisionMaker(const int32_t &argc, char **argv) :
        TimeTriggeredConferenceClientModule(argc, argv, "DecisionMaker") {}

        DecisionMaker::~DecisionMaker() {}

        void DecisionMaker::setUp() {}

        void DecisionMaker::tearDown() {}

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode DecisionMaker::body() {
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == ModuleStateMessage::RUNNING) {
                cout << "CYCLE " << cycle << endl;
                cycle++;
            }
            return ModuleExitCodeMessage::OKAY;
        }

        void DecisionMaker::nextContainer(Container &c) {
            cerr << "NEXT CONTAINER " << c.getDataType() << endl;
        }
    }
}
