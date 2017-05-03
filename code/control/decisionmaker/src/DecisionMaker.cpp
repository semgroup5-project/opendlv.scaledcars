#include "DecisionMaker.h"

using namespace std;

using namespace odcore;
using namespace odcore::io;
using namespace odcore::io::udp;
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::wrapper;
using namespace odcore::data::dmcp;

using namespace odcore::base;
using namespace automotive;
using namespace automotive::miniature;
using namespace group5;

namespace scaledcars {
    namespace control {

        DecisionMaker::DecisionMaker(const int32_t &argc, char **argv) :
                TimeTriggeredConferenceClientModule(argc, argv, "DecisionMaker") {}

        DecisionMaker::~DecisionMaker() {}

        void DecisionMaker::setUp() {
            cout << "Starting DecisionMaker" << endl;

            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();

            decisionMakerMSG.setState(kv.getValue<int32_t>("decisionmaker.function"));
        }

        void DecisionMaker::tearDown() {
            cout << "Shutting down DecisionMaker" << endl;
        }

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode DecisionMaker::body() {
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == ModuleStateMessage::RUNNING) {
                Container container(decisionMakerMSG);
                // Send container.
                getConference().send(container);
            }
            return ModuleExitCodeMessage::OKAY;
        }

        void DecisionMaker::nextContainer(Container &c) {
            cerr << "Received container of type : " << c.getDataType() << endl;
            if (c.getDataType() == VehicleData::ID()) {
                vd = c.getData<VehicleData>();

                decisionMakerMSG.setWheelEncoder(vd.getAbsTraveledPath());
            }
            else if (c.getDataType() == SensorBoardData::ID()) {
                sbd = c.getData<SensorBoardData>();

                decisionMakerMSG.setUltraSonicFrontCenter(sbd.getValueForKey_MapOfDistances(ID_IN_ULTRASONIC_CENTER));

                decisionMakerMSG.setUltrasonicFrontRight(sbd.getValueForKey_MapOfDistances(ID_IN_ULTRASONIC_SIDE_FRONT));

                decisionMakerMSG.setInfraredSideFront(sbd.getValueForKey_MapOfDistances(ID_IN_INFRARED_SIDE_FRONT));

                decisionMakerMSG.setInfraredSideBack(sbd.getValueForKey_MapOfDistances(ID_IN_INFRARED_SIDE_BACK));

                decisionMakerMSG.setInfraredBack(sbd.getValueForKey_MapOfDistances(ID_IN_INFRARED_BACK));
            }
        }
    }
}
