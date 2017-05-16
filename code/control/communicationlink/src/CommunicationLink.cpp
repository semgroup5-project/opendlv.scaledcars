#include "CommunicationLink.h"

namespace scaledcars {
    namespace control {

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

        CommunicationLink::CommunicationLink(const int32_t &argc, char **argv) :
                TimeTriggeredConferenceClientModule(argc, argv, "CommunicationLink"),
                communicationLinkMSG(),
                laneFollowerMSG(),
                overtakerMSG(),
                parkerMSG(),
                sensorsMSG(),
                UDPMSG() {}

        CommunicationLink::~CommunicationLink() {}

        void CommunicationLink::setUp() {
            cout << "Starting CommunicationLink" << endl;

            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();

            communicationLinkMSG.setStateLaneFollower(kv.getValue<int32_t>("communicationlink.functionlane"));
            int func2 = kv.getValue<int32_t>("communicationlink.function2");

            if (func2 == 1) {
                communicationLinkMSG.setStateOvertaker(0);
                communicationLinkMSG.setStateParker(1);
            } else if (func2 == 0){
                communicationLinkMSG.setStateOvertaker(1);
                communicationLinkMSG.setStateParker(0);
            } else {
                communicationLinkMSG.setStateOvertaker(0);
                communicationLinkMSG.setStateParker(0);
            }
        }

        void CommunicationLink::tearDown() {
            cout << "Shutting down CommunicationLink" << endl;
        }

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode CommunicationLink::body() {
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == ModuleStateMessage::RUNNING) {
                Container sensorBoardDataContainer = getKeyValueDataStore().get(SensorsMSG::ID());
                if (sensorBoardDataContainer.getDataType() == SensorsMSG::ID()) {
                    sensorsMSG = sensorBoardDataContainer.getData<SensorsMSG>();

                    communicationLinkMSG.setWheelEncoder(sensorsMSG.getTravelledDistance());
                    cout << "ID: 6 VALUE: " << sensorsMSG.getTravelledDistance() << endl;

                    communicationLinkMSG.setUltraSonicFrontCenter(
                            sensorsMSG.getValueForKey_MapOfDistances(ID_IN_ULTRASONIC_CENTER));
                    cout << "ID:  " << ID_IN_ULTRASONIC_CENTER << " VALUE: "
                         << sensorsMSG.getValueForKey_MapOfDistances(ID_IN_ULTRASONIC_CENTER) << endl;

                    communicationLinkMSG.setUltraSonicFrontRight(
                            sensorsMSG.getValueForKey_MapOfDistances(ID_IN_ULTRASONIC_SIDE_FRONT));
                    cout << "ID:  " << ID_IN_ULTRASONIC_SIDE_FRONT << " VALUE: "
                         << sensorsMSG.getValueForKey_MapOfDistances(ID_IN_ULTRASONIC_SIDE_FRONT) << endl;

                    communicationLinkMSG.setInfraredSideFront(
                            sensorsMSG.getValueForKey_MapOfDistances(ID_IN_INFRARED_SIDE_FRONT));
                    cout << "ID:  " << ID_IN_INFRARED_SIDE_FRONT << " VALUE: "
                         << sensorsMSG.getValueForKey_MapOfDistances(ID_IN_INFRARED_SIDE_FRONT) << endl;

                    communicationLinkMSG.setInfraredSideBack(
                            sensorsMSG.getValueForKey_MapOfDistances(ID_IN_INFRARED_SIDE_BACK));
                    cout << "ID:  " << ID_IN_INFRARED_SIDE_BACK << " VALUE: "
                         << sensorsMSG.getValueForKey_MapOfDistances(ID_IN_INFRARED_SIDE_BACK) << endl;

                    communicationLinkMSG.setInfraredBack(sensorsMSG.getValueForKey_MapOfDistances(ID_IN_INFRARED_BACK));
                    cout << "ID:  " << ID_IN_INFRARED_BACK << " VALUE: "
                         << sensorsMSG.getValueForKey_MapOfDistances(ID_IN_INFRARED_BACK) << endl;
                }

                Container overtakerMSGContainer = getKeyValueDataStore().get(OvertakerMSG::ID());
                if (overtakerMSGContainer.getDataType() == OvertakerMSG::ID()) {
                    overtakerMSG = overtakerMSGContainer.getData<OvertakerMSG>();

                    communicationLinkMSG.setStateLaneFollower(!overtakerMSG.getStateStop());
                    communicationLinkMSG.setDrivingLane(overtakerMSG.getStateLane());
                }

                Container parkerMSGContainer = getKeyValueDataStore().get(ParkerMSG::ID());
                if (parkerMSGContainer.getDataType() == ParkerMSG::ID()) {
                    parkerMSG = parkerMSGContainer.getData<ParkerMSG>();

                    communicationLinkMSG.setStateLaneFollower(!parkerMSG.getStateStop());
                }

                Container laneFollowerMSGContainer = getKeyValueDataStore().get(LaneFollowerMSG::ID());
                if (laneFollowerMSGContainer.getDataType() == LaneFollowerMSG::ID()) {
                    laneFollowerMSG = laneFollowerMSGContainer.getData<LaneFollowerMSG>();

                    communicationLinkMSG.setDrivingLane(laneFollowerMSG.getStateLane());
                    communicationLinkMSG.setDistanceToRightLane(laneFollowerMSG.getDistanceToRightLane());
                }

                Container UDPMSGContainer = getKeyValueDataStore().get(UdpMSG::ID());
                if (UDPMSGContainer.getDataType() == UdpMSG::ID()) {
                    UDPMSG = UDPMSGContainer.getData<UdpMSG>();

                    if (UDPMSG.getStateStop()) {
                        communicationLinkMSG.setStateLaneFollower(0);
                        communicationLinkMSG.setStateOvertaker(0);
                        communicationLinkMSG.setStateParker(0);
                    } else if (!UDPMSG.getStateStop()) {
                        communicationLinkMSG.setStateLaneFollower(1);
                        communicationLinkMSG.setStateOvertaker(UDPMSG.getStateFunctionOvertaker());
                        communicationLinkMSG.setStateParker(UDPMSG.getStateFunctionParker());
                        communicationLinkMSG.setUnpark(UDPMSG.getUnpark());
                    }
                }

                Container container(communicationLinkMSG);
                // Send container.
                getConference().send(container);
            }
            return ModuleExitCodeMessage::OKAY;
        }
    }
}