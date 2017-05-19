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

        Overtaker::Overtaker(const int32_t &argc, char **argv) :
                DataTriggeredConferenceClientModule(argc, argv, "overtaker"),
                m_vehicleControl(),
                Sim(false),
                _state(0),
                overtakerMSG(),
                ULTRASONIC_FRONT_CENTER(3),
                ULTRASONIC_FRONT_RIGHT(4),
                INFRARED_FRONT_RIGHT(0),
                INFRARED_REAR_RIGHT(2),
                INFRARED_BACK(1),
                WHEEL_ENCODER(5),
                OVERTAKING_DISTANCE(67.0),
                HEADING_PARALLEL(4),
                TURN_SPEED_SIM(0.7),
                TURN_ANGLE_SIM_LEFT(-25),
                TURN_ANGLE_SIM_RIGHT(25),
                STRAIGHT_ANGLE_SIM(0),
                TURN_SPEED_CAR(96),
                TURN_ANGLE_CAR_LEFT(-1.5),
                TURN_ANGLE_CAR_RIGHT(1.5),
                STRAIGHT_ANGLE_CAR(STRAIGHT_ANGLE_SIM),
                IR_RR(0),
                IR_FR(0),
                IR_BACK(0),
                US_C(0),
                US_R(0),
                cycles(0),
                USE_CYCLES(false),
                odometerReal(0),
                oldOdometer(0),
                odo(0),
                us_c_old(0),
                _us_c_old(0),
                stage(FIND_OBJECT),
                distanceOUTtoL_0(0),
                distanceOUTtoL_1(0),
                distanceOUTtoR_0(0),
                distanceOUTtoR_1(0),
                distanceINtoR_0(0),
                distanceINtoR_1(0),
                distanceINtoL_0(0),
                distanceINtoL_1(0),
                distanceToObstacle(0),
                distanceToObstacleOld(0),
                OBJECT_PLAUSIBLE_COUNT(3),
                objectPlausibleCount(0),
                objectDangerCount(0),
                enoughTurn(0),
                _stop(0),
                valid_us(0),
                park_state(0),
                isApplied(0){}

        Overtaker::~Overtaker() {}

        void Overtaker::setUp() {
            KeyValueConfiguration kv = getKeyValueConfiguration();
            Sim = kv.getValue<int32_t>("global.sim") == 1;
        }

        void Overtaker::tearDown() {}

        void Overtaker::nextContainer(Container &c) {

            if (c.getDataType() == CommunicationLinkMSG::ID()) {
                Container communicationLinkContainer = c.getData<CommunicationLinkMSG>();
                const CommunicationLinkMSG communicationLinkMSG = communicationLinkContainer.getData<CommunicationLinkMSG>();
                _state = communicationLinkMSG.getStateOvertaker();
                park_state = communicationLinkMSG.getStateParker();

                if (_state) {
                    _stop = 0;
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
                } else if (!_state && !_stop) {
                    _stop = 1;

                    stage = FIND_OBJECT;

                    distanceToObstacle = 0;
                    distanceToObstacleOld = 0;
                    odo = 0;
                    m_vehicleControl.setBrakeLights(true);
                    Container c3(m_vehicleControl);
                    // Send container.
                    getConference().send(c3);
                } else if (!park_state) {
                    US_C = communicationLinkMSG.getUltraSonicFrontCenter();
                    double distance = US_C;
                    double us = abs(distance - _us_c_old);
                    _us_c_old = distance;
                    if ((us <= 5) && (distance > 0 && distance < 36)) {
                        objectDangerCount++;
                        if (objectDangerCount >= OBJECT_PLAUSIBLE_COUNT) {
                            cerr << "DANGER OBJECT" << endl;
                            objectDangerCount = 0;
                            _us_c_old = 0;

                            overtakerMSG.setStateStop(0);
                            Container c1(overtakerMSG);
                            getConference().send(c1);

                            m_vehicleControl.setBrakeLights(true);
                            Container c4(m_vehicleControl);
                            getConference().send(c4);
                            valid_us = 0;
                        }
                    } else {
                        objectDangerCount = 0;
                        if ((us > -1 && us < 1) && (distance < 0 || distance > 35)) {
                            valid_us++;
                        }
                        if (valid_us > 3) {
                            cerr << "OBJECT CLEAR" << endl;
                            valid_us = 0;
                            overtakerMSG.setStateStop(1);
                            Container co(overtakerMSG);
                            getConference().send(co);
                        }
                    }
                }
            }
        }

        void Overtaker::measuringMachine() {

            if (stage == FIND_OBJECT) {
                cerr << "FIND_OBJECT" << endl;

                if (!isApplied) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0);
                    isApplied = 1;
                }

                distanceToObstacle = US_C;
                us_c_old = US_C;

                // Approaching an obstacle (stationary or driving slower than us).
                if ((distanceToObstacle > 0) && (((distanceToObstacleOld - distanceToObstacle) > 0) ||
                                                 (fabs(distanceToObstacleOld - distanceToObstacle) < 1e-2))) {
                    // Check if overtaking shall be started.
                    stage = FIND_OBJECT_PLAUSIBLE;
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0);

                    objectPlausibleCount = 0;
                    isApplied = 0;
                }

                distanceToObstacleOld = distanceToObstacle;

            } else if (stage == FIND_OBJECT_PLAUSIBLE) {
                cerr << "FIND_OBJECT_PLAUSIBLE" << endl;

                double distance;

                distance = US_C;
                double us = abs(distance - us_c_old);
                us_c_old = distance;

                if ((us <= 5) && (distance > 0 && distance <= OVERTAKING_DISTANCE)) {
                    objectPlausibleCount++;
                    if (objectPlausibleCount >= OBJECT_PLAUSIBLE_COUNT) {

                        stage = HAVE_BOTH_IR;
                        m_vehicleControl.setBrakeLights(false);
                        m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                        m_vehicleControl.setSteeringWheelAngle(-1.30);
                        us_c_old = 0;
                    }
                } else {
                    stage = FIND_OBJECT;
                }
            } else if (stage == HAVE_BOTH_IR) {
                cerr << "HAVE_BOTH_IR" << endl;
                odo += odometerReal;

//                if ((IR_FR - IR_RR <= HEADING_PARALLEL) && !(IR_FR < 1) && !(IR_RR < 1)) {
                if ((IR_FR > 0 && IR_RR > 0) && odo > 3) {
                    if (IR_FR > 13) {
                        enoughTurn = 1;
                    }
                    stage = KEEP_TURN_RIGHT;
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(1.5);
                    odo = 0;
                }

            } else if (stage == KEEP_TURN_RIGHT) {
                cerr << "KEEP_TURN_RIGHT" << endl;

                odo += odometerReal;

                if (!enoughTurn) {
                    if (odo > 6) {
                        m_vehicleControl.setBrakeLights(false);
                        m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                        m_vehicleControl.setSteeringWheelAngle(0.5);
                        stage = HAVE_NO_READING;
                        odo = 0;
                    }
                } else {
                    if (odo > 6 && IR_FR > 8) {
                        m_vehicleControl.setBrakeLights(false);
                        m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                        m_vehicleControl.setSteeringWheelAngle(0.5);
                        stage = HAVE_NO_READING;
                        odo = 0;
                    }
                }
            } else if (stage == HAVE_NO_READING) {
                cerr << "HAVE_NO_READING" << endl;
                if (IR_FR < 0 && US_R < 0) {
                    stage = KEEP_TURN_RIGHT_END;
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(1.2);
                } else if (IR_FR > IR_RR && IR_FR > 7) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0.7);
                } else if (IR_RR > IR_FR && IR_RR > 7) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(-0.6);
                } else if ((IR_FR - IR_RR <= HEADING_PARALLEL) && !(IR_FR < 1) && !(IR_RR < 1)) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0);
                }
            } else if (stage == KEEP_TURN_RIGHT_END) {
                cerr << "KEEP_TURN_RIGHT_END" << endl;
                odo += odometerReal;

                if (odo > 6) {
                    stage = ADJUST_TO_STRAIGHT;
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_LEFT);
                    odo = 0;
                }

            } else if (stage == ADJUST_TO_STRAIGHT) {
                cerr << "ADJUST_TO_STRAIGHT" << endl;
                odo += odometerReal;

                if (odo > 3 && (IR_BACK < 0 || IR_BACK > 10)) {
                    _state = 0;

                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0);

                    overtakerMSG.setState(1);
                    Container c(overtakerMSG);
                    getConference().send(c);

                    stage = FIND_OBJECT;

                    distanceToObstacle = 0;
                    distanceToObstacleOld = 0;
                    odo = 0;
                    overtakerMSG.setState(0);
                }
            }
        }
    }
}
// automotive::miniature
