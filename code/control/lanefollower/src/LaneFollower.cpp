/**
 * lanefollower - Sample application for following lane markings.
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


/**
 * docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev:latest /opt/opendlv.scaledcars/bin/scaledcars-control-lanefollwer --cid=111 --freq=10
 */

#include <iostream>
#include <stdint.h>//trying to print out
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/base/Lock.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/wrapper/SharedMemoryFactory.h"

#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include "LaneFollower.h"


//debugging

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odcore::data::image;
        using namespace odcore::data::dmcp;
        using namespace automotive;
        using namespace automotive::miniature;
        using namespace odcore::wrapper;


        LaneFollower::LaneFollower(const int32_t &argc, char **argv) : TimeTriggeredConferenceClientModule(argc, argv, "lanefollower"),
                                                                       m_hasAttachedToSharedImageMemory(false),
                                                                       m_sharedImageMemory(),
                                                                       m_image(NULL),
                                                                       m_debug(false),
                                                                       m_font(),
                                                                       m_previousTime(),
                                                                       m_eSum(0),
                                                                       m_eOld(0),
                                                                       m_vehicleControl() {}
                // Initialize fonts.
        const double hscale = 0.4;
        const double vscale = 0.3;//0.3;
        const double shear = 0.2;
        const int thickness = 1;
        const int lineType = 6;




        // Overall state machines for moving and measuring.
        enum StateMachineMoving {
            FORWARD,
            TO_LEFT_LANE_LEFT_TURN,
            TO_LEFT_LANE_RIGHT_TURN,
            CONTINUE_ON_LEFT_LANE,
            TO_RIGHT_LANE_RIGHT_TURN,
            TO_RIGHT_LANE_LEFT_TURN
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

        // State counter for dynamically moving back to right lane.
        int32_t stageToRightLaneRightTurn = 0;
        int32_t stageToRightLaneLeftTurn = 0;
        int32_t traveled_distance = 0;

        double distanceLtoL_0 = 0;
        double distanceLtoL_1 = 0;

        double distanceLtoR_0 = 0;
        double distanceLtoR_1 = 0;

        double distanceRtoR_0 = 0;
        double distanceRtoR_1 = 0;

        double distanceRtoL_0 = 0;
        double distanceRtoL_1 = 0;


        //int32_t stageToRightLaneRightTurn2= 88;
        //int32_t stageToRightLaneLeftTurn2 = 44;

        // Distance variables to ensure we are overtaking only stationary or slowly driving obstacles.
        double distanceToObstacle = 0;
        double distanceToObstacleOld = 0;

        bool overtake = true;
        LaneFollower::~LaneFollower() {}

        void LaneFollower::setUp() {
            // This method will be call automatically _before_ running body().
            if (m_debug) {
                // Create an OpenCV-window.
                cvNamedWindow("WindowShowImage", CV_WINDOW_AUTOSIZE);
                cvMoveWindow("WindowShowImage", 300, 100);
            }
        }

        void LaneFollower::tearDown() {
            // This method will be call automatically _after_ return from body().
            if (m_image != NULL) {
                cvReleaseImage(&m_image);
            }

            if (m_debug) {
                cvDestroyWindow("WindowShowImage");
            }
        }

        bool LaneFollower::readSharedImage(Container &c) { //pointer to the container

            bool retVal = false;

            if (c.getDataType() == SharedImage::ID()) {
                SharedImage si = c.getData<SharedImage> ();//if is =shareImage the container get the data from the container


                //cerr << si << endl; //

                // Check if we have already attached to the shared memory.
                if (!m_hasAttachedToSharedImageMemory) {

                    m_sharedImageMemory
                            = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(
                            si.getName());

                }

                // Check if we could successfully attach to the shared memory.
                if (m_sharedImageMemory->isValid()) {

                    // Lock the memory region to gain exclusive access using a scoped lock.
                    Lock l(m_sharedImageMemory);
                    const uint32_t numberOfChannels = 3;
                    // For example, simply show the image.
                    if (m_image == NULL) {
                        m_image = cvCreateImage(cvSize(si.getWidth(), si.getHeight()), IPL_DEPTH_8U, numberOfChannels);
                    }

                    // Copying the image data is very expensive...
                    if (m_image != NULL) {
                        memcpy(m_image->imageData,
                               m_sharedImageMemory->getSharedMemory(),
                               si.getWidth() * si.getHeight() * numberOfChannels);
                    }
                    // Mirror the image.
                    cvFlip(m_image, 0, -1);

                    retVal = true; //return Value
                }
            }

            return retVal;
        }
        //
        void LaneFollower::processImage() {
            static bool useRightLaneMarking = true;
            double e = 0;

            const int32_t CONTROL_SCANLINE = 462; // calibrated length to right: 280px
            const int32_t distance = 300;

            TimeStamp beforeImageProcessing;
            for(int32_t y = m_image->height - 8; y > m_image->height * .6; y -= 10) { //->contain , Datatype
                // Search from middle to the left://y axes every time goes down when the car is moving
                CvScalar pixelLeft;
                CvPoint left;
                left.y = y;
                left.x = -1;
                for(int x = m_image->width/2; x > 0; x--) {//from the midle to the left
                    pixelLeft = cvGet2D(m_image, y, x);//updating
                    if (pixelLeft.val[0] >= 200) {
                        left.x = x;//1st white pixel to the left
                        break;
                    }
                }

                // Search from middle to the right://function that find the white lines in the route//same above for the right
                CvScalar pixelRight;
                CvPoint right;
                right.y = y;
                right.x = -1;
                for(int x = m_image->width/2; x < m_image->width; x++) {
                    pixelRight = cvGet2D(m_image, y, x);
                    if (pixelRight.val[0] >= 200) {
                        right.x = x;
                        break;
                    }
                }
                //printing the line
                if (m_debug) {
                    if (left.x > 0) {
                        CvScalar green = CV_RGB(0, 255, 0); //green lines that appears in the window
                        cvLine(m_image, cvPoint(m_image->width/2, y), left, green, 1, 8);

                        stringstream sstr;
                        sstr << (m_image->width/2 - left.x);
                        //the 0 pixel on the image is at the left
                        cvPutText(m_image, sstr.str().c_str(), cvPoint(m_image->width/2 - 100, y - 2), &m_font, green);
                    }
                    if (right.x > 0) {
                        CvScalar red = CV_RGB(255, 0, 0); //red lines that appear in the window measuring the distance
                        cvLine(m_image, cvPoint(m_image->width/2, y), right, red, 1, 8);

                        stringstream sstr;
                        sstr << (right.x - m_image->width/2);
                        cvPutText(m_image, sstr.str().c_str(), cvPoint(m_image->width/2 + 100, y - 2), &m_font, red);
                    }
                }

                if (y == CONTROL_SCANLINE) {
                    // Calculate the deviation error.
                    //Push the image to the left so that we can find the left lane.
                    if (right.x > 0) right.x += 30;
                    if (left.x > 0) left.x += 30;


                    if (right.x > 0) {
                        if (!useRightLaneMarking) {//if the right part is missing we consider the left one
                            m_eSum = 0;
                            m_eOld = 0;
                        }

                        e = ((right.x - m_image->width/2.0) - distance)/distance; //ERROR VALUE The left on the right and left is 0 ans is save in E

                        useRightLaneMarking = true;
                    }
                    else if (left.x > 0) {
                        if (useRightLaneMarking) {
                            m_eSum = 0;
                            m_eOld = 0;
                        }

                        e = (distance - (m_image->width/2.0 - left.x))/distance;

                        useRightLaneMarking = false;
                    }
                    else {
                        // If no measurements are available, reset PID controller.
                        m_eSum = 0;
                        m_eOld = 0;
                    }
                }
            }

            TimeStamp afterImageProcessing;//they are just numbers
            cerr << "Processing time: " << (afterImageProcessing.toMicroseconds() - beforeImageProcessing.toMicroseconds())/1000.0 << "ms." << endl;

            // Show resulting features.
            if (m_debug) {
                if (m_image != NULL) {
                    cvShowImage("WindowShowImage", m_image);
                    cvWaitKey(10);
                }
            }

            TimeStamp currentTime;
            double timeStep = (currentTime.toMicroseconds() - m_previousTime.toMicroseconds()) / (1000.0 * 1000.0);
            m_previousTime = currentTime;
            //a more soft way to handle instead of resetting to 0 a way to mittigate togling in corners
            if (fabs(e) < 1e-1) {
                m_eSum = m_eSum * 0.01;
            }
            if (fabs(e) < 1e-2) {
                m_eSum = m_eSum * 0.0001;
            }
            if (fabs(e) < 1e-3) {
                m_eSum = 0;
            }
            else {
                m_eSum += e;
            }


            //Good values don't change
//            const double Kp = 1.3;
//            const double Ki = 0.01;
//            const double Kd = 0.1;


            const double Kp = 1.3;
            const double Ki = 0.01;
            const double Kd = 0.1;

            // The following values have been determined by Twiddle algorithm.
            //Kp -> Proportional -> how big of a turn when the car try to "fix" the error
            //const double Kp = 0.4482626884328734;
            //Ki -> Integral -> Current -> might be the middle position of the car
            //const double Ki = 3.103197570937628;
            //Kd -> derivative -> how frequent the reaction the car will be -> the smaller the better.
            //const double Kd = 0.030450210485408566;

            const double p = Kp * e;
            const double i = Ki * timeStep * m_eSum;
            const double d = Kd * (e - m_eOld) / timeStep;
            m_eOld = e;

            const double y = p + i + d;
            double desiredSteering = 0;
            if (fabs(e) > 1e-2) {
                desiredSteering = y;

                if (desiredSteering > 25.0) {
                    // desiredSteering = 25.0;
                }
                if (desiredSteering < -25.0) {
                    //desiredSteering = -25.0;
                }
            }

//            cerr << "PID: " << "e = " << e << ", eSum = " << m_eSum << ", desiredSteering = " << desiredSteering
//                 << ", y = " << y << endl;


            // Go forward
            m_vehicleControl.setSpeed(2);//5
            m_vehicleControl.setSteeringWheelAngle(desiredSteering);
        }

        void LaneFollower::measuring_state_machine() {
            // Parameters for overtaking.
            const int32_t ULTRASONIC_FRONT_CENTER = 3;
            const int32_t ULTRASONIC_FRONT_RIGHT = 4;
            const int32_t INFRARED_FRONT_RIGHT = 0;
            const int32_t INFRARED_REAR_RIGHT = 2;
            //const int32_t INFRARED_BACK = 1;
            //const int32_t WHEEL_ENCODER = 5;

            const double OVERTAKING_DISTANCE = 5.0;
            const double HEADING_PARALLEL = 0.04;
            //Get most recent vehicle data:
            Container containerVehicleData = getKeyValueDataStore().get(VehicleData::ID());
            VehicleData vd = containerVehicleData.getData<VehicleData> ();
            //Get most recent sensor board data:
            Container containerSensorBoardData = getKeyValueDataStore().get(SensorBoardData::ID());
            SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

            if (stageMeasuring == FIND_OBJECT_INIT) {
                cerr << "FIND_OBJECT_INIT" << endl;
                distanceToObstacleOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                stageMeasuring = FIND_OBJECT;
            } else if (stageMeasuring == FIND_OBJECT) {
                cerr << "FIND_OBJECT" << endl;
                distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                // Approaching an obstacle (stationary or driving slower than us).
                if ((distanceToObstacle > 0) && (((distanceToObstacleOld - distanceToObstacle) > 0) ||
                                                 (fabs(distanceToObstacleOld - distanceToObstacle) < 1e-2))) {
                    // Check if overtaking shall be started.
                    stageMeasuring = FIND_OBJECT_PLAUSIBLE;
                }

                distanceToObstacleOld = distanceToObstacle;
            } else if (stageMeasuring == FIND_OBJECT_PLAUSIBLE) {
                cerr << "FIND_OBJECT_PLAUSIBLE" << endl;
                if (sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER) < OVERTAKING_DISTANCE) {
                    stageMoving = TO_LEFT_LANE_LEFT_TURN;
                    distanceLtoL_0 = vd.getAbsTraveledPath();
                    overtake = true;
                    // Disable measuring until requested from moving state machine again.
                    stageMeasuring = DISABLE;
                } else {
                    stageMeasuring = FIND_OBJECT;
                }
            } else if (stageMeasuring == HAVE_BOTH_IR) {
                cerr << "HAVE_BOTH_IR" << endl;
                // Remain in this stage until both IRs see something.
                if ((sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0) &&
                    (sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0)) {
                    // Turn to right.
                    stageMoving = TO_LEFT_LANE_RIGHT_TURN;
                    distanceLtoL_1 = vd.getAbsTraveledPath();
                    distanceLtoR_0 = vd.getAbsTraveledPath();
                }
            } else if (stageMeasuring == HAVE_BOTH_IR_SAME_DISTANCE) {
                cerr << "HAVE_BOTH_IR_SAME_DISTANCE" << endl;
                // Remain in this stage until both IRs have the similar distance to obstacle (i.e. turn car)
                // and the driven parts of the turn are plausible.
                const double IR_FR = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                const double IR_RR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);

                bool sensorCondition = (fabs(IR_FR - IR_RR) < HEADING_PARALLEL);
                bool stepsCondition = ((stageToRightLaneLeftTurn - stageToRightLaneRightTurn) > 0);

                cerr << "sensorCondition=" << sensorCondition << " stepsCondition=" << stepsCondition;

                if (sensorCondition &&
                    stepsCondition) {
                    // Straight forward again.
                    stageMoving = CONTINUE_ON_LEFT_LANE;
                    distanceLtoR_1 = vd.getAbsTraveledPath();

                    // Reset PID controller.
                    m_eSum = 0;
                    m_eOld = 0;
                }
            } else if (stageMeasuring == END_OF_OBJECT) {
                cerr << "END_OF_OBJECT" << endl;

                // Find end of object.
                distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);

                if (distanceToObstacle < 0) {
                    // Move to right lane again.
                    stageMoving = TO_RIGHT_LANE_RIGHT_TURN;
                    distanceRtoR_0 = vd.getAbsTraveledPath();

                    // Disable measuring until requested from moving state machine again.
                    stageMeasuring = DISABLE;
                }

            }
        }

        void LaneFollower::movingMachine(bool has_next_frame){

            Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
            VehicleData vd = containerVehicleData.getData<VehicleData>();

            // 2. Get most recent sensor board data:
            Container containerSensorBoardData = getKeyValueDataStore().get(SensorBoardData::ID());//
            SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData>();
            traveled_distance = vd.getAbsTraveledPath();
            //overtake = false;
            // Moving state machine.
            if (stageMoving == FORWARD && has_next_frame) {
                // Use m_vehicleControl data from image processing.
                //processImage();
                traveled_distance = 0;
                stageToRightLaneLeftTurn = 0;
                stageToRightLaneRightTurn = 0;
            } else if (stageMoving == TO_LEFT_LANE_LEFT_TURN) {
                // Move to the left lane: Turn left part until both IRs see something.
                m_vehicleControl.setSpeed(0.7);
                m_vehicleControl.setSteeringWheelAngle(-15);


                // State machine measuring: Both IRs need to see something before leaving this moving state.
                stageMeasuring = HAVE_BOTH_IR;

                stageToRightLaneRightTurn++;
                traveled_distance++;
                cout << "StageToRightLaneRightTurn is" << stageToRightLaneRightTurn << endl;
                cout << "Distance is " << traveled_distance<< endl;
            } else if (stageMoving == TO_LEFT_LANE_RIGHT_TURN) {
                // Move to the left lane: Turn right part until both IRs have the same distance to obstacle.
                // State machine measuring: Both IRs need to have the same distance before leaving this moving state.
                stageMeasuring = HAVE_BOTH_IR_SAME_DISTANCE;
                if(stageToRightLaneLeftTurn <150) {
                    m_vehicleControl.setSpeed(0.7);
                    m_vehicleControl.setSteeringWheelAngle(15);
                    stageToRightLaneLeftTurn++;
                }

                cout << "Stage RightLanelLeftTurn is" << stageToRightLaneLeftTurn << endl;
            } else if (stageMoving == CONTINUE_ON_LEFT_LANE) {
                // Move to the left lane: Passing stag
                // Use m_vehicleControl data from image processing.

                // Find end of object.
                stageMeasuring = END_OF_OBJECT;
            } else if (stageMoving == TO_RIGHT_LANE_RIGHT_TURN) {
                // Move to the right lane: Turn right part.
                m_vehicleControl.setSpeed(0.7);
                m_vehicleControl.setSteeringWheelAngle(15);

                traveled_distance--;
                stageToRightLaneRightTurn--;
                cout << "Stage RightLaneRightTurn is" << stageToRightLaneRightTurn << endl;
                cout <<"Distance "<< traveled_distance << endl;

                /*
                if (stageToRightLaneRightTurn < 17) {
                    cout << "Going left" << endl;
                    stageMoving = TO_RIGHT_LANE_LEFT_TURN;

                }*/

                double traveledSoFar = vd.getAbsTraveledPath() - distanceRtoR_0;
                double traveledRequired = distanceLtoR_1 - distanceLtoR_0;
                cerr << "traveledRequired=" << traveledRequired << endl;
                if (traveledSoFar > (traveledRequired * 1.0)) {
                    stageMoving = TO_RIGHT_LANE_LEFT_TURN;
                    distanceRtoL_0 = vd.getAbsTraveledPath();
                }
            } else if (stageMoving == TO_RIGHT_LANE_LEFT_TURN) {
                // Move to the left lane: Turn left part.
                cout<< "Reached here , start to turn "<< endl;
                m_vehicleControl.setSpeed(0.7);
                m_vehicleControl.setSteeringWheelAngle(-15);

                double traveledSoFar = vd.getAbsTraveledPath() - distanceRtoL_0;
                double traveledRequired = distanceLtoL_1 - distanceLtoL_0;
                if (traveledSoFar > (traveledRequired * 0.0)) {
                    overtake = false;
                    stageMoving = FORWARD;

                    stageMeasuring = FIND_OBJECT_INIT;

                    distanceToObstacle = 0;
                    distanceToObstacleOld = 0;

                    // Reset PID controller.
                    m_eSum = 0;
                    m_eOld = 0;
                }

                /*

                stageToRightLaneLeftTurn--;
                cout << "Stage  is" << stageToRightLaneLeftTurn << endl;
                //TESTING steps
                if (stageToRightLaneLeftTurn == 0) {
                    // Start over.
                    overtake = false;
                    stageMoving = FORWARD;

                    stageToRightLaneLeftTurn = 0;
                    stageToRightLaneRightTurn = 0;

                    stageMeasuring = FIND_OBJECT_INIT;

                    distanceToObstacle = 0;
                    distanceToObstacleOld = 0;

                    // Reset PID controller.
                    m_eSum = 0;
                    m_eOld = 0;
                }
                 */
            }


        }

        // This method will do the main data processing job.
        // Therefore, it tries to open the real camera first. If that fails, the virtual camera images from camgen are used.
        ModuleExitCodeMessage::ModuleExitCode LaneFollower::body() {
            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();
            m_debug = kv.getValue<int32_t>("lanefollower.debug") == 1;

            cvInitFont(&m_font, CV_FONT_HERSHEY_DUPLEX, hscale, vscale, shear, thickness, lineType);

            // Overall state machine handler.
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == ModuleStateMessage::RUNNING) {
                bool has_next_frame = false;


                // Get the most recent available container for a SharedImage.
                Container c = getKeyValueDataStore().get(SharedImage::ID());

                if (c.getDataType() == SharedImage::ID()) {
                    // Example for processing the received container.
                    has_next_frame = readSharedImage(c);
                }

                // Process the read image and calculate regular lane following set values for control algorithm.
                if (has_next_frame) {
                    processImage();
                }


                 //Overtaking part.
                movingMachine(has_next_frame) ;
                measuring_state_machine();

                if(overtake){
                    Container c1(m_vehicleControl);
                    getConference().send(c1);
                }else {// Create container for finally sending the set values for the control algorithm.
                    Container c2(m_vehicleControl);
                    getConference().send(c2);
                }
                    // Send container.

            }

            return ModuleExitCodeMessage::OKAY;
        }

    }
} // scaledcars::control
