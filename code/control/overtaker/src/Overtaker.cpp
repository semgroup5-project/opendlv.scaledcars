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
        //const int IR_FR_BLIND_COUNT = 2;

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
            HAVE_DISTANCE_IR_BACK,
            HAVE_NO_IR_FRONT,
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
            // getting latest sensors' data
            if (c.getDataType() == CommunicationLinkMSG::ID()) {
                Container communicationLinkContainer = c.getData<CommunicationLinkMSG>();
                const CommunicationLinkMSG communicationLinkMSG = c.getData<CommunicationLinkMSG>();
                _state = communicationLinkMSG.getStateLaneFollower();

                odometerReal = communicationLinkMSG.getWheelEncoder() - oldOdometer;
                oldOdometer = communicationLinkMSG.getWheelEncoder();


                m_vehicleControl.setBrakeLights(false);
                m_vehicleControl.setSpeed(96);

                movingMachine(communicationLinkMSG);
                measuringMachine(communicationLinkMSG);

                Container c3(m_vehicleControl);
                getConference().send(c3);
            }
        }

        void Overtaker::movingMachine(CommunicationLinkMSG clm) {
            //move forward
            if (stageMoving == FORWARD) {
                cerr << "FORWARD" << endl;

                if (Sim) {
                    m_vehicleControl.setSpeed(TURN_SPEED_SIM);
                    m_vehicleControl.setSteeringWheelAngle(STRAIGHT_ANGLE_SIM);
                } else {
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0.3);
                }

                //car start to move the left
            } else if (stageMoving == OUT_TO_LEFT) {
                cerr << "OUT_TO_LEFT" << endl;
                cerr << "ODO> " << odometerReal << endl;

                if(odometerReal < 2){

                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_LEFT);
                }else {
                    stageMeasuring = HAVE_BOTH_IR;
                    stageMoving = NONE;
                }

            } else if (stageMoving == OUT_TO_RIGHT) {
                cerr << "OUT_TO_RIGHT" << endl;

                stageMeasuring = HAVE_BOTH_IR_SAME_DISTANCE;

                //   }else if (stageMoving == CONTINUE_STRAIGHT) {


            } else if (stageMoving == IN_TO_RIGHT) {
                cerr << "IN_TO_RIGHT" << endl;

                if(odometerReal < 2){
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_RIGHT);
                }else{
                    stageMeasuring = HAVE_DISTANCE_IR_BACK;
                }

            } else if (stageMoving == IN_TO_LEFT) {
                cerr << "IN_TO_LEFT" << endl;
                IR_BACK = clm.getInfraredSideFront();
                if(IR_BACK <=15){
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_LEFT);
                }else{
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0.3);
                }
                    stageMoving = FORWARD;
                    stageMeasuring = FIND_OBJECT_INIT;

                    distanceToObstacle = 0;
                    distanceToObstacleOld = 0;
            }
        }

        void Overtaker::measuringMachine(CommunicationLinkMSG clm) {

            if (stageMeasuring == FIND_OBJECT_INIT) {
                cerr << "FIND_OBJECT_INIT" << endl;


                distanceToObstacleOld = clm.getUltraSonicFrontCenter();

                stageMeasuring = FIND_OBJECT;

            } else if (stageMeasuring == FIND_OBJECT) {
                cerr << "FIND_OBJECT" << endl;

                distanceToObstacle = clm.getUltraSonicFrontCenter();

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

                distance = clm.getUltraSonicFrontCenter();

                if (distance > 0 && distance < OVERTAKING_DISTANCE) {
                    objectPlausibleCount++;

                    if (objectPlausibleCount >= OBJECT_PLAUSIBLE_COUNT) {

                        stageMoving = OUT_TO_LEFT;

                        stageMeasuring = DISABLE;
                    }
                } else {
                    stageMeasuring = FIND_OBJECT;
                }

            } else if (stageMeasuring ==  HAVE_NO_IR_FRONT ) {
                IR_FR = clm.getInfraredSideFront();

                if (IR_FR < 0) {
                    stageMoving = IN_TO_RIGHT;
                } else {
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0.3);
                }
            } else if( stageMeasuring == HAVE_DISTANCE_IR_BACK){

                IR_RR = clm.getInfraredSideFront();
                if(IR_RR >= 15){
                    stageMoving = IN_TO_LEFT;
                }else{
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(0.3);
                }
            } else if (stageMeasuring == HAVE_BOTH_IR) {
                cerr << "HAVE_BOTH_IR" << endl;

                IR_FR = clm.getInfraredSideFront();
                IR_RR = clm.getInfraredSideBack();

                if (IR_FR - IR_RR <= HEADING_PARALLEL ) {
                    stageMoving = OUT_TO_RIGHT;
                }else{
                    cerr << "CONTINUE_STRAIGHT" << endl;
                    m_vehicleControl.setSpeed(96);
                    m_vehicleControl.setSteeringWheelAngle(0.3);
                }
                //checking the readins from both IR sensors and adjust car's manuoeuvre accordingly
            } else if (stageMeasuring == HAVE_BOTH_IR_SAME_DISTANCE) {
                cerr << "HAVE_BOTH_IR_SAME_DISTANCE" << endl;

                IR_FR = clm.getInfraredSideFront();
                IR_RR = clm.getInfraredSideBack();


                cerr << "IR_FR=" << IR_FR << endl;
                cerr << "IR_RR=" << IR_RR << endl;

                if (IR_FR <= 10) {
                    stageMoving = NONE;
                    stageMeasuring = HAVE_NO_IR_FRONT;
                } else {
                    m_vehicleControl.setSpeed(TURN_SPEED_CAR);
                    m_vehicleControl.setSteeringWheelAngle(TURN_ANGLE_CAR_RIGHT);
                }

                //
            } else if (stageMeasuring == END_OF_OBJECT) {
                cerr << "END_OF_OBJECT" << endl;

                distanceToObstacle = clm.getUltraSonicFrontCenter();

                IR_FR = clm.getInfraredSideFront();
                IR_RR = clm.getInfraredSideBack();

                IR_FR = IR_FR;
                IR_RR = IR_RR;

                stageMeasuring = DISABLE;

            }

        }
    }
}
 // automotive::miniature

