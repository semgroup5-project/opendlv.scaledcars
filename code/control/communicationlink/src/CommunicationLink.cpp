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
                DataTriggeredConferenceClientModule(argc, argv, "CommunicationLink"),
                communicationLinkMSG(),
                laneFollowerMSG(),
                overtakerMSG(),
                parkerMSG(),
                sensorsMSG(),
                UDPMSG(),
                udp_stop(0){}

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
            } else if (func2 == 0) {
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

        void CommunicationLink::nextContainer(Container &c) {

            if (c.getDataType() == SensorsMSG::ID()) {
                Container sensorBoardDataContainer = c.getData<SensorsMSG>();

                sensorsMSG = sensorBoardDataContainer.getData<SensorsMSG>();

                communicationLinkMSG.setWheelEncoder(sensorsMSG.getTravelledDistance());

                communicationLinkMSG.setUltraSonicFrontCenter(
                        sensorsMSG.getValueForKey_MapOfDistances(ID_IN_ULTRASONIC_CENTER));

                communicationLinkMSG.setUltraSonicFrontRight(
                        sensorsMSG.getValueForKey_MapOfDistances(ID_IN_ULTRASONIC_SIDE_FRONT));

                communicationLinkMSG.setInfraredSideFront(
                        sensorsMSG.getValueForKey_MapOfDistances(ID_IN_INFRARED_SIDE_FRONT));

                communicationLinkMSG.setInfraredSideBack(
                        sensorsMSG.getValueForKey_MapOfDistances(ID_IN_INFRARED_SIDE_BACK));

                communicationLinkMSG.setInfraredBack(sensorsMSG.getValueForKey_MapOfDistances(ID_IN_INFRARED_BACK));
                
                Container container(communicationLinkMSG);
            // Send container.
            getConference().send(container);
            }

          else  if (c.getDataType() == OvertakerMSG::ID()) {
                Container overtakerMSGContainer = c.getData<OvertakerMSG>();
                overtakerMSG = overtakerMSGContainer.getData<OvertakerMSG>();

                if (udp_stop) {
                    communicationLinkMSG.setStateLaneFollower(0);
                } else {
                    communicationLinkMSG.setStateLaneFollower(overtakerMSG.getStateStop());
                }
                communicationLinkMSG.setDrivingLane(overtakerMSG.getStateLane());

                if (overtakerMSG.getState()) {
                    communicationLinkMSG.setStateLaneFollower(1);
                    communicationLinkMSG.setStateOvertaker(0);
                    udp_stop = 0;
                }
              Container container(communicationLinkMSG);
            // Send container.
            getConference().send(container);
            }

         else   if (c.getDataType() == ParkerMSG::ID()) {
                Container parkerMSGContainer = c.getData<ParkerMSG>();
                parkerMSG = parkerMSGContainer.getData<ParkerMSG>();

                if (udp_stop) {
                    communicationLinkMSG.setStateLaneFollower(0);
                } else {
                    communicationLinkMSG.setStateLaneFollower(parkerMSG.getStateStop());
                }
             Container container(communicationLinkMSG);
            // Send container.
            getConference().send(container);
            }

        else    if (c.getDataType() == LaneFollowerMSG::ID()) {
                Container laneFollowerMSGContainer = c.getData<LaneFollowerMSG>();
                laneFollowerMSG = laneFollowerMSGContainer.getData<LaneFollowerMSG>();

                communicationLinkMSG.setDrivingLane(laneFollowerMSG.getStateLane());
                communicationLinkMSG.setDistanceToRightLane(laneFollowerMSG.getDistanceToRightLane());
                communicationLinkMSG.setStop(laneFollowerMSG.getDanger());
            Container container(communicationLinkMSG);
            // Send container.
            getConference().send(container);
            }

          else  if (c.getDataType() == UdpMSG::ID()) {
                Container UDPMSGContainer = c.getData<UdpMSG>();
                UDPMSG = UDPMSGContainer.getData<UdpMSG>();

                udp_stop = UDPMSG.getStateStop();

                if (udp_stop) {
                    communicationLinkMSG.setStateLaneFollower(0);
                    communicationLinkMSG.setStateOvertaker(0);
                    communicationLinkMSG.setStateParker(0);
                } else {
                    if (UDPMSG.getStateFunctionOvertaker()) {
                        udp_stop = UDPMSG.getStateFunctionOvertaker();
                        communicationLinkMSG.setStateOvertaker(1);
                        communicationLinkMSG.setStateLaneFollower(0);
                        communicationLinkMSG.setStateParker(0);

                    } else if (UDPMSG.getStateFunctionParker()) {
                        udp_stop = UDPMSG.getStateFunctionParker();
                        communicationLinkMSG.setStateOvertaker(0);
                        communicationLinkMSG.setStateLaneFollower(0);
                        communicationLinkMSG.setStateParker(1);
                        communicationLinkMSG.setUnpark(UDPMSG.getUnpark());

                    } else {
                        udp_stop = 0;
                        communicationLinkMSG.setStateOvertaker(0);
                        communicationLinkMSG.setStateLaneFollower(1);
                        communicationLinkMSG.setStateParker(0);
                    }
                }
              Container container(communicationLinkMSG);
            // Send container.
            getConference().send(container);
            }

            
        }
    }
}
