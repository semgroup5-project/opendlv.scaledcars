#include <cstdio>
#include <cmath>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "Overtaker.h"

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odcore::data::image;
        using namespace automotive;
        using namespace odcore::wrapper;
        using namespace odcore::data::dmcp;
        using namespace automotive::miniature;
        using namespace group5;

        const int32_t ULTRASONIC_FRONT_CENTER = 3;
        const int32_t ULTRASONIC_FRONT_RIGHT = 4;
        const int32_t INFRARED_FRONT_RIGHT = 0;
        const int32_t INFRARED_REAR_RIGHT = 2;
        const int32_t INFRARED_BACK = 1;
        const int32_t WHEEL_ENCODER = 5;

        const double OVERTAKING_DISTANCE = 55.0;
        const double HEADING_PARALLEL = 1;

        const double TURN_SPEED_SIM = 0.7;
        const double TURN_ANGLE_SIM_LEFT = -25;
        const double TURN_ANGLE_SIM_RIGHT = 25;
        const double STRAIGHT_ANGLE_SIM = 0;

        const double TURN_SPEED_CAR = 96;
        const double TURN_ANGLE_CAR_LEFT = TURN_ANGLE_SIM_LEFT;
        const double TURN_ANGLE_CAR_RIGHT = TURN_ANGLE_SIM_RIGHT;
        const double STRAIGHT_ANGLE_CAR = STRAIGHT_ANGLE_SIM;

        const int IR_FR_BLIND_COUNT = 2;

        long cycles = 0;
        const bool USE_CYCLES = true;

        int IR_FR_blindCount = 0;

        enum StateMachineMoving {
            FORWARD,
            OUT_TO_LEFT,
            OUT_TO_RIGHT,
            CONTINUE_STRAIGHT,
            IN_TO_RIGHT,
            IN_TO_LEFT
        };

        enum StateMachineMeasuring {
            DISABLE,
            FIND_OBJECT_INIT,
            FIND_OBJECT,
            FIND_OBJECT_PLAUSIBLE,
            HAVE_BOTH_IR,
            HAVE_BOTH_IR_SAME_DISTANCE,
            END_OF_OBJECT
        };

        StateMachineMoving stageMoving = FORWARD;
        StateMachineMeasuring stageMeasuring = FIND_OBJECT_INIT;

        double distanceOUTtoL_0 = 0;
        double distanceOUTtoL_1 = 0;

        double distanceOUTtoR_0 = 0;
        double distanceOUTtoR_1 = 0;

        double distanceINtoR_0 = 0;
        double distanceINtoR_1 = 0;

        double distanceINtoL_0 = 0;
        double distanceINtoL_1 = 0;

        double distanceToObstacle = 0;
        double distanceToObstacleOld = 0;

        const int OBJECT_PLAUSIBLE_COUNT = 2;
        int objectPlausibleCount = 0;

        Overtaker::Overtaker(const int32_t &argc, char **argv) :
                DataTriggeredConferenceClientModule(argc, argv, "overtaker"),
                m_vehicleControl(),
                Sim(false),
                _state(0) {
        }

        Overtaker::~Overtaker() {}

        void Overtaker::setUp() {
            KeyValueConfiguration kv = getKeyValueConfiguration();
            Sim = kv.getValue<int32_t>("global.sim") == 1;
        }

        void Overtaker::tearDown() {}

        void Overtaker::nextContainer(Container &c) {
            if (c.getDataType() == CommunicationLinkMSG::ID()) {
                Container communicationLinkContainer = c.getData<CommunicationLinkMSG>();
                const CommunicationLinkMSG communicationLinkMSG = c.getData<CommunicationLinkMSG>();
                _state = communicationLinkMSG.getStateLaneFollower();

                m_vehicleControl.setBrakeLights(false);
                m_vehicleControl.setSpeed(96);

                movingMachine(communicationLinkMSG);
                measuringMachine(communicationLinkMSG);

                Container c3(m_vehicleControl);
                getConference().send(c3);
            }
        }

        void Overtaker::movingMachine(CommunicationLinkMSG clm) {
//            // Get vehicle data
//            if(c.getDataType() == CommunicationLinkMSG::ID()){
//                Container vdContainer = c.getKeyValueDataStore().get(VehicleData::ID());
//            }else{
//                VehicleData vd = vdContainer.getData<VehicleData>();
//            }



//            // Get sensor board data
//            if(c.getDataType() == CommunicationLinkMSG::ID()) {
//                Container sbdContainer = getKeyValueDataStore().get(SensorBoardData::ID());
//            }else{
//                SensorBoardData sbd = sbdContainer.getData<SensorBoardData>();
//            }



//            // Get communication link message
//            Container clmContainer = getKeyValueDataStore().get(CommunicationLinkMSG::ID());
//            CommunicationLinkMSG clm = clmContainer.getData<CommunicationLinkMSG>();

            if (stageMoving == FORWARD) {
                cerr << "FORWARD" << endl;

                if (Sim) {
                    m_vehicleControl.setSpeed(TURN_SPEED_SIM);
                    m_vehicleControl.setSteeringWheelAngle(STRAIGHT_ANGLE_SIM);
                } else {
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(STRAIGHT_ANGLE_CAR);
                }

            } else if (stageMoving == OUT_TO_LEFT) {
                cerr << "OUT_TO_LEFT" << endl;

                if (Sim) {
                    m_vehicleControl.setSpeed(TURN_SPEED_SIM);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_SIM_LEFT);
                } else {
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_LEFT);
                }

                stageMeasuring = HAVE_BOTH_IR;

            } else if (stageMoving == OUT_TO_RIGHT) {
                cerr << "OUT_TO_RIGHT" << endl;

                if (Sim) {
                    m_vehicleControl.setSpeed(TURN_SPEED_SIM);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_SIM_RIGHT);
                } else {
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_RIGHT);
                }

                stageMeasuring = HAVE_BOTH_IR_SAME_DISTANCE;

            } else if (stageMoving == CONTINUE_STRAIGHT) {
                cerr << "CONTINUE_STRAIGHT" << endl;

                stageMeasuring = END_OF_OBJECT;

                if (Sim) {
                    m_vehicleControl.setSpeed(TURN_SPEED_SIM);
                    m_vehicleControl.setSteeringWheelAngle(STRAIGHT_ANGLE_SIM);
                } else {
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(STRAIGHT_ANGLE_CAR);
                }

            } else if (stageMoving == IN_TO_RIGHT) {
                cerr << "IN_TO_RIGHT" << endl;

                if (Sim) {
                    m_vehicleControl.setSpeed(TURN_SPEED_SIM);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_SIM_RIGHT);
                } else {
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_RIGHT);
                }

                double traveled;
                if (USE_CYCLES) {
                    traveled = cycles;
                } else {
//                    if (Sim) {
//                        traveled = vd.getAbsTraveledPath();
//                    } else {
                    traveled = clm.getWheelEncoder();
//                    }
                }

                double traveledSoFar = traveled - distanceINtoR_0;
                double traveledRequired = distanceOUTtoR_1 - distanceOUTtoR_0;

                cerr << "traveledSoFar=" << traveledSoFar << endl;
                cerr << "traveledRequired=" << traveledRequired << endl;

                if (traveledSoFar > (traveledRequired * 1.0)) {
                    stageMoving = IN_TO_LEFT;
                    if (USE_CYCLES) {
                        distanceINtoL_0 = cycles;
                    } else {
//                        if (Sim) {
//                            distanceINtoL_0 = vd.getAbsTraveledPath();
//                        } else {
                        distanceINtoL_0 = clm.getWheelEncoder();
//                        }
                    }

                }

            } else if (stageMoving == IN_TO_LEFT) {
                cerr << "IN_TO_LEFT" << endl;

                if (Sim) {
                    m_vehicleControl.setSpeed(TURN_SPEED_SIM);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_SIM_LEFT);
                } else {
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_LEFT);
                }

                double traveled;
                if (USE_CYCLES) {
                    traveled = cycles;
                } else {
//                    if (Sim) {
//                        traveled = vd.getAbsTraveledPath();
//                    } else {
                    traveled = clm.getWheelEncoder();
//                    }
                }

                double traveledSoFar = traveled - distanceINtoL_0;
                double traveledRequired = distanceOUTtoL_1 - distanceOUTtoL_0;

                cerr << "traveledSoFar=" << traveledSoFar << endl;
                cerr << "traveledRequired=" << traveledRequired << endl;

                if (traveledSoFar > (traveledRequired * 1.0)) {
                    //overtake = false;

                    stageMoving = FORWARD;
                    stageMeasuring = FIND_OBJECT_INIT;

                    distanceToObstacle = 0;
                    distanceToObstacleOld = 0;
                }
            }
        }

        void Overtaker::measuringMachine(CommunicationLinkMSG clm) {
            // Get vehicle data
//            Container vdContainer = getKeyValueDataStore().get(VehicleData::ID());
//            VehicleData vd = vdContainer.getData<VehicleData>();
//
//            // Get sensor board data
//            Container sbdContainer = getKeyValueDataStore().get(SensorBoardData::ID());
//            SensorBoardData sbd = sbdContainer.getData<SensorBoardData>();
//
//            // Get communication link message
            //Container clmContainer = getKeyValueDataStore().get(CommunicationLinkMSG::ID());
            //CommunicationLinkMSG clm = c.getData<CommunicationLinkMSG>();

            if (stageMeasuring == FIND_OBJECT_INIT) {
                cerr << "FIND_OBJECT_INIT" << endl;

                // Read initial distance to obstacle
//                if (Sim) {
//                    distanceToObstacleOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
//                } else {
                distanceToObstacleOld = clm.getUltraSonicFrontCenter();
                //}

                stageMeasuring = FIND_OBJECT;

            } else if (stageMeasuring == FIND_OBJECT) {
                cerr << "FIND_OBJECT" << endl;

//                if (Sim) {
//                    distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
//                } else {
                distanceToObstacle = clm.getUltraSonicFrontCenter();
//                }

                // Approaching an obstacle (stationary or driving slower than us).
                if ((distanceToObstacle > 0) && (((distanceToObstacleOld - distanceToObstacle) > 0) ||
                                                 (fabs(distanceToObstacleOld - distanceToObstacle) < 1e-2))) {
                    // Check if overtaking shall be started.
                    stageMeasuring = FIND_OBJECT_PLAUSIBLE;

                    objectPlausibleCount = 0;
                }

                distanceToObstacleOld = distanceToObstacle;

            } else if (stageMeasuring == FIND_OBJECT_PLAUSIBLE) {
                cerr << "FIND_OBJECT_PLAUSIBLE" << endl;

                double distance;

//                if (Sim) {
//                    distance = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
//                } else {
                distance = clm.getUltraSonicFrontCenter();
                //}

                if (distance > 0 && distance < OVERTAKING_DISTANCE) {
                    objectPlausibleCount++;

                    if (objectPlausibleCount >= OBJECT_PLAUSIBLE_COUNT) {

                        //overtake = true;

                        stageMoving = OUT_TO_LEFT;
                        if (USE_CYCLES) {
                            distanceOUTtoL_0 = cycles;
                        } else {
//                            if (Sim) {
//                                distanceOUTtoL_0 = vd.getAbsTraveledPath();
//                            } else {
                            distanceOUTtoL_0 = clm.getWheelEncoder();
//                            }
                        }

                        stageMeasuring = DISABLE;
                    }
                } else {
                    stageMeasuring = FIND_OBJECT;
                }

            } else if (stageMeasuring == HAVE_BOTH_IR) {
                cerr << "HAVE_BOTH_IR" << endl;

                // Reset counters
                IR_FR_blindCount = 0;

                double infraredFrontRightDistance;
                double infraredRearRightDistance;
//
//                if (Sim) {
//                    infraredFrontRightDistance = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
//                    infraredRearRightDistance = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
//                } else {
                infraredFrontRightDistance = clm.getInfraredSideFront();
                infraredRearRightDistance = clm.getInfraredSideBack();
                //}

                //if ((infraredFrontRightDistance > 1) &&
                //    (infraredRearRightDistance > 1)) {

                infraredRearRightDistance = infraredRearRightDistance;

                if (infraredFrontRightDistance > 1) {

                    if (USE_CYCLES) {
                        distanceOUTtoL_1 = cycles;
                    } else {
//                        if (Sim) {
//                            distanceOUTtoL_1 = vd.getAbsTraveledPath();
//                        } else {
                        distanceOUTtoL_1 = clm.getWheelEncoder();
//                        }
                    }

                    stageMoving = OUT_TO_RIGHT;
                    if (USE_CYCLES) {
                        distanceOUTtoR_0 = cycles;
                    } else {
//                        if (Sim) {
//                            distanceOUTtoR_0 = vd.getAbsTraveledPath();
//                        } else {
                        distanceOUTtoR_0 = clm.getWheelEncoder();
//                        }
                    }
                }

            } else if (stageMeasuring == HAVE_BOTH_IR_SAME_DISTANCE) {
                cerr << "HAVE_BOTH_IR_SAME_DISTANCE" << endl;

                double IR_FR;
                double IR_RR;

//                if (Sim) {
//                    IR_FR = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
//                    IR_RR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
//                } else {
                IR_FR = clm.getInfraredSideFront();
                IR_RR = clm.getInfraredSideBack();
//                }

                double traveled;
                if (USE_CYCLES) {
                    traveled = cycles;
                } else {
//                    if (Sim) {
//                        traveled = vd.getAbsTraveledPath();
//                    } else {
                    traveled = clm.getWheelEncoder();
                    //                   }
                }

                double distanceOUTtoL = distanceOUTtoL_1 - distanceOUTtoL_0;
                double distanceOUTtoR = traveled - distanceOUTtoR_0;

                cerr << "IR_FR=" << IR_FR << endl;
                cerr << "IR_RR=" << IR_RR << endl;

                bool sensorCondition = IR_FR > 0 && IR_RR > 0 && (fabs(IR_FR - IR_RR) <= HEADING_PARALLEL);
                bool distanceCondition = distanceOUTtoR > distanceOUTtoL;

                if (IR_FR < 0) {
                    IR_FR_blindCount++;
                } else {
                    IR_FR_blindCount = 0;
                }

                cerr << "IR_FR_blindCount=" << IR_FR_blindCount << endl;

                bool blindCountCondition = IR_FR_blindCount >= IR_FR_BLIND_COUNT;

                cerr << "sensorCondition=" << sensorCondition << endl;
                cerr << "distanceCondition=" << distanceCondition << endl;

                cerr << "blindCountCondition=" << distanceCondition << endl;

                if ((sensorCondition && distanceCondition) || blindCountCondition) {
                    stageMoving = CONTINUE_STRAIGHT;

                    if (USE_CYCLES) {
                        distanceOUTtoR_1 = cycles;
                    } else {
//                        if (Sim) {
//                            distanceOUTtoR_1 = vd.getAbsTraveledPath();
//                        } else {
                        distanceOUTtoR_1 = clm.getWheelEncoder();
//                        }
                    }

                    // Reset PID controller.
                    //m_eSum = 0;
                    //m_eOld = 0;
                }

            } else if (stageMeasuring == END_OF_OBJECT) {
                cerr << "END_OF_OBJECT" << endl;

//                if (Sim) {
//                    distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);
//                } else {
                distanceToObstacle = clm.getUltraSonicFrontCenter();
//                }

                double IR_FR;
                double IR_RR;

//                if (Sim) {
//                    IR_FR = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
//                    IR_RR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
//                } else {
                IR_FR = clm.getInfraredSideFront();
                IR_RR = clm.getInfraredSideBack();
//            }

                IR_FR = IR_FR;
                IR_RR = IR_RR;

                if (distanceToObstacle < 0) {
                    if (IR_FR < 0) {
                        stageMoving = IN_TO_RIGHT;
                        if (USE_CYCLES) {
                            distanceINtoR_0 = cycles;
                        } else {
//                        if (Sim) {
//                            distanceINtoR_0 = vd.getAbsTraveledPath();
//                        } else {
                            distanceINtoR_0 = clm.getWheelEncoder();
//                        }
                        }

                        stageMeasuring = DISABLE;
                    }
                }
            }

        }
    }
}
 // automotive::miniature

