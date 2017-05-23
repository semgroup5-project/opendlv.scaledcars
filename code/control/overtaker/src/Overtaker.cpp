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
                OVERTAKING_DISTANCE(48.0),
                HEADING_PARALLEL(10),
                SPEED_CAR(96),
                TURN_ANGLE_CAR_LEFT(-1.5),
                TURN_ANGLE_CAR_RIGHT(1.5),
                STRAIGHT_ANGLE_CAR(0),
                IR_RR(0),
                IR_FR(0),
                IR_BACK(0),
                US_C(0),
                US_R(0),
                odometerReal(0),
                oldOdometer(0),
                odo(0),
                us_c_old(0),
                _us_c_old(0),
                stage(FIND_OBJECT),
                distanceToObstacle(0),
                distanceToObstacleOld(0),
                OBJECT_PLAUSIBLE_COUNT(3),
                objectPlausibleCount(0),
                objectDangerCount(0),
                _stop(0),
                valid_us(0),
                park_state(0),
                flag(0) {}

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

                    Container c_(m_vehicleControl);
                    getConference().send(c_);

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
                m_vehicleControl.setBrakeLights(false);
                m_vehicleControl.setSpeed(SPEED_CAR);
                m_vehicleControl.setSteeringWheelAngle(0.23);

                distanceToObstacle = US_C;
                us_c_old = US_C;

                // Approaching an obstacle (stationary or driving slower than us).
                if ((distanceToObstacle > 0) && (((distanceToObstacleOld - distanceToObstacle) > 0) ||
                                                 (fabs(distanceToObstacleOld - distanceToObstacle) < 1e-2))) {
                    // Check if overtaking shall be started.
                    stage = FIND_OBJECT_PLAUSIBLE;
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(STRAIGHT_ANGLE_CAR);

                    objectPlausibleCount = 0;
                }

                distanceToObstacleOld = distanceToObstacle;

            } else if (stage == FIND_OBJECT_PLAUSIBLE) {
                double distance = US_C;
                double us = abs(distance - us_c_old);
                us_c_old = distance;

                if ((us <= 5) && (distance > 0 && distance <= OVERTAKING_DISTANCE)) {
                    objectPlausibleCount++;
                    if (objectPlausibleCount >= OBJECT_PLAUSIBLE_COUNT) {

                        stage = HAVE_BOTH_IR;
                        m_vehicleControl.setBrakeLights(false);
                        m_vehicleControl.setSpeed(SPEED_CAR);
                        m_vehicleControl.setSteeringWheelAngle(-1.38);
                        us_c_old = 0;
                    }
                } else {
                    stage = FIND_OBJECT;
                    us_c_old = 0;
                }

            } else if (stage == HAVE_BOTH_IR) {
                odo += odometerReal;

                if ((IR_FR > 0 && IR_RR > 0) && odo > 5) {
                    stage = KEEP_TURN_RIGHT;
                    m_vehicleControl.setSpeed(SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_RIGHT);
                    odo = 0;
                }

            } else if (stage == KEEP_TURN_RIGHT) {
                odo += odometerReal;
                if (odo > 10 && (fabs(IR_FR - IR_RR) <= HEADING_PARALLEL)) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(STRAIGHT_ANGLE_CAR);
                    stage = HAVE_NO_READING;
                    odo = 0;
                }

            } else if (stage == HAVE_NO_READING) {
                if ((IR_FR < 0) && (US_R < 0) && (IR_RR < 0)) {
                    stage = KEEP_TURN_RIGHT_END;
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_RIGHT);
                    odo = 0;
                } else if (IR_FR > IR_RR && IR_FR > 7) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0.7);
                } else if (IR_RR > IR_FR && IR_RR > 7) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(-0.4);//-0.6
                } else if ((IR_FR - IR_RR <= HEADING_PARALLEL) && !(IR_FR < 1) && !(IR_RR < 1)) {
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(STRAIGHT_ANGLE_CAR);
                }
            } else if (stage == KEEP_TURN_RIGHT_END) {
                odo += odometerReal;

                if (odo > 5 && IR_RR < 0) {
                    stage = ADJUST_TO_STRAIGHT;
                    m_vehicleControl.setBrakeLights(false);
                    m_vehicleControl.setSpeed(SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(STRAIGHT_ANGLE_CAR);
                    odo = 0;
                    flag = 1;
                }

            } else if (stage == ADJUST_TO_STRAIGHT) {
                odo += odometerReal;

                if (flag) {
                    if (odo > 2) {
                        m_vehicleControl.setBrakeLights(false);
                        m_vehicleControl.setSpeed(SPEED_CAR);
                        m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_LEFT);
                        flag = 0;
                    }

                } else {
                    if (odo > 10 ) {
                        _state = 0;

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
            } else if (stage == DISABLE) {
                m_vehicleControl.setBrakeLights(true);
            }

            Container container(m_vehicleControl);
            getConference().send(container);
        }
    }
}
// automotive::miniature
