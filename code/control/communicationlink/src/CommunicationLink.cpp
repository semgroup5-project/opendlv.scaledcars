#include "CommunicationLink.h"

using namespace std;

using namespace odcore;
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

        CommunicationLink::CommunicationLink(const int32_t &argc, char **argv) :
                TimeTriggeredConferenceClientModule(argc, argv, "CommunicationLink"),
                communicationLinkMSG(),
                laneFollowerMSG(),
                overtakerMSG(),
                parkerMSG(),
                vd(),
                sbd(){}

        CommunicationLink::~CommunicationLink() {}

        void CommunicationLink::setUp() {
            cout << "Starting CommunicationLink" << endl;

            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();

            communicationLinkMSG.setStateLaneFollower(kv.getValue<int32_t>("communicationlink.functionlane"));
            int func2 = kv.getValue<int32_t>("communicationlink.function2");
            if (func2) {
                communicationLinkMSG.setStateOvertaker(0);
                communicationLinkMSG.setStateParker(1);
            } else {
                communicationLinkMSG.setStateOvertaker(1);
                communicationLinkMSG.setStateParker(0);
            }
        }

        void CommunicationLink::tearDown() {
            cout << "Shutting down CommunicationLink" << endl;
        }

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode CommunicationLink::body() {
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == ModuleStateMessage::RUNNING) {
                Container vehicleDataContainer = getKeyValueDataStore().get(VehicleData::ID());
                if (vehicleDataContainer.getDataType() == VehicleData::ID()) {
                    vd = vehicleDataContainer.getData<VehicleData>();
                    communicationLinkMSG.setWheelEncoder(vd.getAbsTraveledPath());
                }

                Container sensorBoardDataContainer = getKeyValueDataStore().get(SensorBoardData::ID());
                if (sensorBoardDataContainer.getDataType() == SensorBoardData::ID()) {
                    sbd = sensorBoardDataContainer.getData<SensorBoardData>();

                    communicationLinkMSG.setUltraSonicFrontCenter(
                            sbd.getValueForKey_MapOfDistances(ID_IN_ULTRASONIC_CENTER));

                    communicationLinkMSG.setUltrasonicFrontRight(
                            sbd.getValueForKey_MapOfDistances(ID_IN_ULTRASONIC_SIDE_FRONT));

                    communicationLinkMSG.setInfraredSideFront(
                            sbd.getValueForKey_MapOfDistances(ID_IN_INFRARED_SIDE_FRONT));

                    communicationLinkMSG.setInfraredSideBack(
                            sbd.getValueForKey_MapOfDistances(ID_IN_INFRARED_SIDE_BACK));

                    communicationLinkMSG.setInfraredBack(sbd.getValueForKey_MapOfDistances(ID_IN_INFRARED_BACK));
                }

                Container overtakerMSGContainer = getKeyValueDataStore().get(OvertakerMSG::ID());
                if (overtakerMSGContainer.getDataType() == OvertakerMSG::ID()) {
                    overtakerMSG = overtakerMSGContainer.getData<OvertakerMSG>();

                    communicationLinkMSG.setStateLaneFollower(overtakerMSG.getStateStop());
                    communicationLinkMSG.setStateOvertaker(1);
                    communicationLinkMSG.setStateParker(0);
                    communicationLinkMSG.setDrivingLane(overtakerMSG.getStateLane());
                }

                Container parkerMSGContainer = getKeyValueDataStore().get(ParkerMSG::ID());
                if (parkerMSGContainer.getDataType() == ParkerMSG::ID()) {
                    parkerMSG = parkerMSGContainer.getData<ParkerMSG>();

                    communicationLinkMSG.setStateLaneFollower(parkerMSG.getStateStop());
                    communicationLinkMSG.setStateOvertaker(0);
                    communicationLinkMSG.setStateParker(1);
                    communicationLinkMSG.setDrivingLane(0);
                }

                Container laneFollowerMSGContainer = getKeyValueDataStore().get(LaneFollowerMSG::ID());
                if (laneFollowerMSGContainer.getDataType() == LaneFollowerMSG::ID()) {
                    laneFollowerMSG = laneFollowerMSGContainer.getData<LaneFollowerMSG>();

                    communicationLinkMSG.setDrivingLane(laneFollowerMSG.getStateLane());
                }

                Container container(communicationLinkMSG);
                // Send container.
                getConference().send(container);
            }
            return ModuleExitCodeMessage::OKAY;
        }
    }
}
