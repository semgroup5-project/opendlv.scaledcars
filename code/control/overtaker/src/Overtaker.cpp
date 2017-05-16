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

        double IR_RR = 0;
        double IR_FR = 0;
        double IR_BACK = 0;
        double UR_C = 0;

        long cycles = 0;
        const bool USE_CYCLES = false;
        int odometerReal = 0;
        int oldOdometer = 0;

        int IR_FR_blindCount = 0;

        enum StateMachineMoving {
            FORWARD,
            OUT_TO_LEFT,
            OUT_TO_RIGHT,
            NONE,
            ADJUST_TO_LEFT,
            IN_TO_RIGHT,
            IN_TO_LEFT
        };

        enum StateMachineMeasuring {
            DISABLE,
            FIND_OBJECT,
            FIND_OBJECT_PLAUSIBLE,
            HAVE_BOTH_IR,
            HAVE_BOTH_IR_SAME_DISTANCE,
            HAVE_DISTANCE_IR_BACK,
            HAVE_NO_IR_FRONT,
            END_OF_OBJECT
        };

        StateMachineMoving stageMoving = FORWARD;
        StateMachineMeasuring stageMeasuring = FIND_OBJECT;

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

                odometerReal = communicationLinkMSG.getWheelEncoder() - oldOdometer;
                oldOdometer = communicationLinkMSG.getWheelEncoder();
                IR_BACK = communicationLinkMSG.getInfraredBack();
                IR_RR = communicationLinkMSG.getInfraredSideBack();
                IR_FR = communicationLinkMSG.getInfraredSideFront();
                UR_C =  communicationLinkMSG.getUltraSonicFrontCenter();

                measuringMachine();
                movingMachine();

                Container c3(m_vehicleControl);
                getConference().send(c3);
            }
        }

        void Overtaker::movingMachine() {

            if (stageMoving == FORWARD) {
                cerr << "FORWARD" << endl;

                if (Sim) {
                    m_vehicleControl.setSpeed(TURN_SPEED_SIM);
                    m_vehicleControl.setSteeringWheelAngle(STRAIGHT_ANGLE_SIM);
                } else {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0.3);
                }

            }
            else if (stageMoving == OUT_TO_LEFT) {
                cerr << "OUT_TO_LEFT" << endl;
                cerr << "ODO> " << odometerReal << endl;

                if (odometerReal < 2) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(-1.5);
                }
                stageMeasuring = HAVE_BOTH_IR;

            }
 else if (stageMoving == OUT_TO_RIGHT) {
                cerr << "OUT_TO_RIGHT" << endl;

                m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                m_vehicleControl.setSteeringWheelAngle(1.5);

                stageMeasuring = HAVE_BOTH_IR_SAME_DISTANCE;

            } else if (stageMoving == IN_TO_RIGHT) {
                cerr << "IN_TO_RIGHT" << endl;

                if (odometerReal < 2) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_RIGHT);
                }
                stageMeasuring = HAVE_DISTANCE_IR_BACK;

            } else if (stageMoving == IN_TO_LEFT) {
                cerr << "IN_TO_LEFT" << endl;
                if (IR_BACK <= 10) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_LEFT);
                } else {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0.3);
                }
                stageMoving = FORWARD;
                stageMeasuring = FIND_OBJECT;

                distanceToObstacle = 0;
                distanceToObstacleOld = 0;
            } else if (stageMoving == ADJUST_TO_LEFT) {
                cerr << "ADJUST_TO_LEFT" << endl;
                if (odometerReal < 2) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(-0.15);
                } else {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0.3);
                }
                    stageMeasuring = HAVE_NO_IR_FRONT;
            }
        }

        void Overtaker::measuringMachine() {

            if (stageMeasuring == FIND_OBJECT) {
                cerr << "FIND_OBJECT" << endl;

                distanceToObstacle = UR_C;

                // Approaching an obstacle (stationary or driving slower than us).
                if ((distanceToObstacle >= 1) && (((distanceToObstacleOld - distanceToObstacle) > 0) ||
                                                 (fabs(distanceToObstacleOld - distanceToObstacle) < 1e-2))) {
                    // Check if overtaking shall be started.
                    stageMeasuring = FIND_OBJECT_PLAUSIBLE;

                    objectPlausibleCount = 0;
                }

                distanceToObstacleOld = distanceToObstacle;

            }
            else if (stageMeasuring == FIND_OBJECT_PLAUSIBLE) {
                cerr << "FIND_OBJECT_PLAUSIBLE" << endl;

                double distance;

                distance = UR_C;

                if (distance >= 1 && distance <= OVERTAKING_DISTANCE) {
                    objectPlausibleCount++;
                    cerr << "COUNT: " << objectPlausibleCount << endl;
                    if (objectPlausibleCount >= OBJECT_PLAUSIBLE_COUNT) {

                        stageMoving = OUT_TO_LEFT;
                    }
                } else {
                    stageMeasuring = FIND_OBJECT;
                }

            }
            else if (stageMeasuring == HAVE_NO_IR_FRONT) {
                if (IR_FR < 0) {
                    stageMoving = IN_TO_RIGHT;
                }
            }
            else if (stageMeasuring == HAVE_DISTANCE_IR_BACK) {

                if (IR_RR < 0) {
                    stageMoving = IN_TO_LEFT;
                }
            }
            else if (stageMeasuring == HAVE_BOTH_IR) {
                cerr << "HAVE_BOTH_IR" << endl;

                if ((IR_FR - IR_RR <= HEADING_PARALLEL) && !(IR_FR < 1) && !(IR_RR < 1)) {
                    stageMoving = OUT_TO_RIGHT;
                }

            }
            else if (stageMeasuring == HAVE_BOTH_IR_SAME_DISTANCE) {
                cerr << "HAVE_BOTH_IR_SAME_DISTANCE" << endl;

                cerr << "IR_FR=" << IR_FR << endl;
                cerr << "IR_RR=" << IR_RR << endl;

                if ((IR_FR - IR_RR <= HEADING_PARALLEL) && !(IR_FR < 1) && !(IR_RR < 1)) {
                    stageMoving = ADJUST_TO_LEFT;
                }

            }
            else if (stageMeasuring == END_OF_OBJECT) {
                cerr << "END_OF_OBJECT" << endl;

                distanceToObstacle = IR_FR;

                stageMeasuring = DISABLE;
            }
        }
    }
}
// automotive::miniature
