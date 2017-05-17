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

        const double OVERTAKING_DISTANCE = 50.0;
        const double OVERTAKING_DISTANCE_DISPLACED = 65.0;
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
        double US_C = 0;
        double US_R = 0;

        long cycles = 0;
        const bool USE_CYCLES = false;
        int odometerReal = 0;
        int oldOdometer = 0;
        int odo = 0;

        int us_c_old = 0;
        int us_r_count = 0;

        enum StateMachine {
            FIND_OBJECT,
            FIND_OBJECT_PLAUSIBLE,
            HAVE_BOTH_IR,
            HAVE_NO_READING,
            KEEP_TURN_RIGHT,
            KEEP_TURN_RIGHT_END,
            ADJUST_TO_STRAIGHT
        };

        StateMachine stage = FIND_OBJECT;

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

        const int OBJECT_PLAUSIBLE_COUNT = 3;
        int objectPlausibleCount = 0;
        int objectDisplacedPlausibleCount = 0;

        int enoughTurn = 0;

        int _stop = 0;

        UdpMSG _udpmsg;

        Overtaker::Overtaker(const int32_t &argc, char **argv) :
                DataTriggeredConferenceClientModule(argc, argv, "overtaker"),
                m_vehicleControl(),
                Sim(false),
                _state(0),
                overtakerMSG(){
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
                _state = communicationLinkMSG.getStateOvertaker();


                if (_state) {
                    _stop = 0;
                    overtakerMSG.setStateStop(0);
                    Container co(overtakerMSG);
                    getConference().send(co);
                    odometerReal = communicationLinkMSG.getWheelEncoder() - oldOdometer;
                    oldOdometer = communicationLinkMSG.getWheelEncoder();
                    IR_BACK = communicationLinkMSG.getInfraredBack();
                    IR_RR = communicationLinkMSG.getInfraredSideBack();
                    IR_FR = communicationLinkMSG.getInfraredSideFront();
                    US_C = communicationLinkMSG.getUltraSonicFrontCenter();
                    US_R = communicationLinkMSG.getUltraSonicFrontRight();

                    measuringMachine();

                    Container c3(m_vehicleControl);
                    getConference().send(c3);
                }else if(!_state && !_stop){
                    _stop = 1;

                    stage = FIND_OBJECT;

                    distanceToObstacle = 0;
                    distanceToObstacleOld = 0;
                    odo = 0;
                    m_vehicleControl.setBrakeLights(true);
                    Container c3(m_vehicleControl);
                    // Send container.
                    getConference().send(c3);
                }
//                else
//                {
//                    US_C = communicationLinkMSG.getUltraSonicFrontCenter();
//                    double distance = US_C;
//                    double us = abs(US_C - us_c_old);
//
//                    if ((us <= 5) && distance <= 62) {
//                        objectPlausibleCount++;
//                        us_c_old = distance;
//                        if (objectPlausibleCount >= OBJECT_PLAUSIBLE_COUNT) {
//
//                            objectPlausibleCount = 0;
//                            us_c_old = 0;
//
//                            overtakerMSG.setStateStop(1);
//                            Container c1(overtakerMSG);
//                            getConference().send(c1);
//
//                            m_vehicleControl.setBrakeLights(true);
//                            Container c4(m_vehicleControl);
//                            getConference().send(c4);
//                        }
//                    }
//                }
            }
        }

        void Overtaker::measuringMachine() {

            if (stage == FIND_OBJECT) {
                cerr << "FIND_OBJECT" << endl;

                m_vehicleControl.setBrakeLights(false);
                m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                m_vehicleControl.setSteeringWheelAngle(0.3);

                distanceToObstacle = US_C;
                us_c_old = US_C;

                // Approaching an obstacle (stationary or driving slower than us).
                if ((distanceToObstacle >= 1) && (((distanceToObstacleOld - distanceToObstacle) > 0) ||
                                                  (fabs(distanceToObstacleOld - distanceToObstacle) < 1e-2))) {
                    // Check if overtaking shall be started.
                    stage = FIND_OBJECT_PLAUSIBLE;

                    objectPlausibleCount = 0;
                }

                distanceToObstacleOld = distanceToObstacle;

            } else if (stage == FIND_OBJECT_PLAUSIBLE) {
                cerr << "FIND_OBJECT_PLAUSIBLE" << endl;

                m_vehicleControl.setBrakeLights(false);
                m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                m_vehicleControl.setSteeringWheelAngle(0.3);

                double distance;

                if (US_R < 0) {
                    us_r_count++;
                }

                distance = US_C;
                double us = abs(US_C - us_c_old);

                if ((us <= 5) && distance <= OVERTAKING_DISTANCE) {
                    objectPlausibleCount++;
                    us_c_old = distance;
                    if (objectPlausibleCount >= OBJECT_PLAUSIBLE_COUNT) {

                        stage = HAVE_BOTH_IR;
                    }
                }
                else if ((us <= 5) && distance <= OVERTAKING_DISTANCE_DISPLACED && us_r_count == 3) {
                    objectDisplacedPlausibleCount++;
                    us_c_old = distance;
                    if (objectDisplacedPlausibleCount >= OBJECT_PLAUSIBLE_COUNT) {

                        stage = HAVE_BOTH_IR;
                        us_r_count = 0;
                    }
                }
                else {
                    stage = FIND_OBJECT;
                }
            } else if (stage == HAVE_NO_READING) {
                cerr << "HAVE_NO_READING" << endl;
                if (IR_FR < 0 && US_R < 0) {
                    stage = KEEP_TURN_RIGHT_END;
                } else if (IR_FR > IR_RR && IR_FR > 8) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0.7);
                } else if (IR_RR > IR_FR && IR_RR > 8) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(-0.6);
                } else if ((IR_FR - IR_RR <= HEADING_PARALLEL) && !(IR_FR < 1) && !(IR_RR < 1)) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0);
                }
            }
 else if (stage == HAVE_BOTH_IR) {
                cerr << "HAVE_BOTH_IR" << endl;

                m_vehicleControl.setBrakeLights(false);
                m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                m_vehicleControl.setSteeringWheelAngle(-1.30);

                if (IR_FR > 13) {
                    enoughTurn = 1;
                }

                if (/*enoughLeft && */(IR_FR - IR_RR <= HEADING_PARALLEL) && !(IR_FR < 1) && !(IR_RR < 1)) {
                    stage = KEEP_TURN_RIGHT;
                    odo = 0;
                    enoughTurn = 0;
                }

            }
 else if (stage == KEEP_TURN_RIGHT) {
                cerr << "KEEP_TURN_RIGHT" << endl;

                odo += odometerReal;
                m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                m_vehicleControl.setSteeringWheelAngle(1.5);

                if (odo > 4 || IR_FR < 8) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0.5);
                    stage = HAVE_NO_READING;
                    odo = 0;
                }
            }
 else if (stage == KEEP_TURN_RIGHT_END) {
                cerr << "KEEP_TURN_RIGHT_END" << endl;
                odo += odometerReal;
                m_vehicleControl.setBrakeLights(false);
                m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_RIGHT);
                if (odo > 8) {
                    stage = ADJUST_TO_STRAIGHT;
                    odo = 0;
                }

            }
 else if (stage == ADJUST_TO_STRAIGHT) {
                cerr << "ADJUST_TO_STRAIGHT" << endl;
                odo += odometerReal;

                m_vehicleControl.setBrakeLights(false);
                m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_LEFT);

                if (odo > 3) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0);

                    _udpmsg.setStateFunctionOvertaker(0);
                    _udpmsg.setStateFunctionParker(0);

                    Container co(_udpmsg);
                    getConference().send(co);

                    overtakerMSG.setStateStop(1);
                    Container c(overtakerMSG);
                    getConference().send(c);

                    _state = 0;

                    stage = FIND_OBJECT;

                    distanceToObstacle = 0;
                    distanceToObstacleOld = 0;
                    odo = 0;
                }
            }
        }
    }
}
// automotive::miniature
