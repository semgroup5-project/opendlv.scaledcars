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

#include <unistd.h>
#include <math.h>

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


namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace cv;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odcore::data::image;
        using namespace automotive;
        using namespace odcore::wrapper;
        using namespace odcore::data::dmcp;
        using namespace automotive::miniature;

        Mat m_image_new, m_image_equ, m_image_processed;
        bool stop = false;
        double stopCounter = 0;
        String state = "moving";
        bool inRightLane = true;   //the overtaker alters this value to signal a lane change

        LaneFollower::LaneFollower(const int32_t &argc, char **argv) :
                TimeTriggeredConferenceClientModule(argc, argv, "lanefollower"),
                m_sharedImageMemory(),
                m_sharedProcessedImageMemory(),
                m_sharedProcessedImage(),
                m_hasAttachedToSharedImageMemory(false),
                m_debug(false),
                Sim(false),
                m_image(),
                m_previousTime(),
                m_eSum(0),
                m_eOld(0),
                m_vehicleControl(),
                m_threshold1(50),
                m_threshold2(150),
                m_control_scanline(400),//needs testing with real c
                m_stop_scanline(200),//needs testing with real car
                m_distance(180),  //needs testing with real car as well
                p_gain(0),       // the gain values can be adjusted here outside of simulation scenario (see @setUp() )
                i_gain(0),
                d_gain(0) {}

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

        //int32_t distance = 0;
        //int32_t stageToRightLaneRightTurn2= 88;
        //int32_t stageToRightLaneLeftTurn2 = 44;

        // Distance variables to ensure we are overtaking only stationary or slowly driving obstacles.
        double distanceToObstacle = 0;
        double distanceToObstacleOld = 0;

        bool overtake = true;
        LaneFollower::~LaneFollower() {}


        // This method will be call automatically _before_ running body().
        void LaneFollower::setUp() {

            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();
            m_debug = kv.getValue<int32_t>("lanefollower.debug") == 1;
            Sim = kv.getValue<int32_t>("lanefollower.sim") == 1;
            p_gain = kv.getValue<double>("lanefollower.p");
            d_gain = kv.getValue<double>("lanefollower.d");
            i_gain = kv.getValue<double>("lanefollower.i");


            // debug, make sure we get the correct values
            cerr << "Sim is" << Sim << endl;
            cerr << "p is " << p_gain << endl;
            cerr << "d is " << d_gain << endl;
            cerr << "i is " << i_gain << endl;
            // setup window for debugging
            if (m_debug) {
                cvNamedWindow("Debug Image", CV_WINDOW_AUTOSIZE);
                cvMoveWindow("Debug Image", 300, 100);
            }
            if (Sim) {
                m_control_scanline = 462; // calibrated length to right: 280px
                m_distance = 250;  // distance from right lane marking
            }
        }

        // This method will be call automatically _after_ return from body().
        void LaneFollower::tearDown() {
            if (!m_image.empty()) {
                m_image.deallocate();
            }
            if (m_debug) {
                cvDestroyWindow("WindowShowImage");
            }
        }

        // This method returns a boolean true if it gets an image from the shared image memory
        bool LaneFollower::readSharedImage(Container &c) {
            bool retVal = false;

            if (c.getDataType() == SharedImage::ID()) {
                SharedImage si = c.getData<SharedImage>();

                // Check if we have already attached to the shared memory.
                if (!m_hasAttachedToSharedImageMemory) {

                    m_sharedImageMemory = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(si.getName());

                    m_hasAttachedToSharedImageMemory = true;

                }

                // Check if we could successfully attach to the shared memory.
                if (m_sharedImageMemory->isValid()) {
                    // Lock the memory region to gain exclusive access using a scoped lock.
                    m_sharedImageMemory->lock();

                    if (m_image.empty()) { // If image is empty, create a new cv::Mat image
                        m_image.create(si.getHeight(), si.getWidth(),
                                       CV_8UC3); //From opencv documentation: CV_8UC3 = unsigned integer 8bit matrix/image wih 3 channels (typically RGB or BRG in opencv case)
                    } else { // Copying the image data
                        memcpy(m_image.data, m_sharedImageMemory->getSharedMemory(),
                               si.getWidth() * si.getHeight() * si.getBytesPerPixel());
                    }
                    m_sharedImageMemory->unlock(); // Release the memory region lock

                    if (Sim) { // If in Sim mode, flip the image
                        flip(m_image, m_image, -1);
                    }
                    retVal = true;
                }

            }

            return retVal;
        }


        // Process Image
        void LaneFollower::processImage() {
            // New image
            m_image_new = Mat(m_image.rows, m_image.cols, CV_8UC1);



            // Copy the original image to the new image as greyscale
            cvtColor(m_image, m_image_new, COLOR_BGR2GRAY);


           equalizeHist(m_image_new,  m_image_equ);



            Canny(m_image_new, m_image_new, m_threshold1, m_threshold2, 3); // see header for algorithm and threshold explanation

        }

        // Calculate deviation from goal
        double LaneFollower::errorCalculation() {
            double e = 0;

            int32_t y = m_control_scanline;

            uchar pixelLeft, pixelRight;
            Point left, right;  // P(x, y) = P (column, row)

            left.y = y;
            left.x = -1;

            // Search from middle to the left
            for (int x = m_image_new.cols / 2; x > 0; x--) {//from the middle to the left
                pixelLeft = m_image_new.at<uchar>(Point(x, y));
                if (pixelLeft >= 150) {   //tentative value, might need adjustment: lower it closer to 100
                    left.x = x;
                    break;
                }
            }


            right.y = y;
            right.x = -1;
            // Search from middle to the right
            for (int x = m_image_new.cols / 2; x < m_image_new.cols - 50; x++) {  //cols - 50 to stop it from
                pixelRight = m_image_new.at<uchar>(Point(x, y));
                if (pixelRight >= 150) {   //tentative value, might need adjustment: lower it closer to 100
                    right.x = x;
                    break;
                }
            }

            if (y == m_control_scanline) {

                if (inRightLane) {
                    if (right.x > 0) {
                        e = ((right.x - m_image_new.cols / 2.0) - m_distance) / m_distance;
                    } else if (left.x > 0) {
                        e = (m_distance - (m_image_new.cols / 2.0 - left.x)) / m_distance;
                    }
                } else {  //Adapt to following the left lane
                    if (left.x > 0) {
                        m_eSum = 0;
                        m_eOld = 0;
                        e = (m_distance - (m_image_new.cols / 2.0 - left.x)) / m_distance;
                    } else if (right.x > 0) {
                        m_eSum = 0;
                        m_eOld = 0;
                        e = ((right.x - m_image_new.cols / 2.0) - m_distance) / m_distance;
                    }
                }
            }


            //prints the lines for debugging purposes if debug flag is set to true
            if (m_debug) {

                if (left.x > 0) {
                    line(m_image_equ, Point(m_image.cols / 2, y), left, Scalar(255, 0, 0), 1, 8);

                    stringstream sstr;
                    sstr << (m_image_new.cols / 2 - left.x);

                    putText(m_image_new, sstr.str().c_str(), Point(m_image.cols / 2 - 100, y - 2), FONT_HERSHEY_PLAIN, 1,
                            CV_RGB(255, 0, 0));
                }
                if (right.x > 0) {
                    line(m_image_equ, cvPoint(m_image.cols / 2, y), right, Scalar(255, 0, 0), 1, 8);

                    stringstream sstr;
                    sstr << (right.x - m_image_new.cols / 2);
                    putText(m_image_new, sstr.str().c_str(), cvPoint(m_image.cols / 2 + 100, y - 2), FONT_HERSHEY_PLAIN, 1,
                            CV_RGB(255, 0, 0));
                }
            }


            // stopline logic

            uchar front_left, front_right;
            Point stop_left, stop_right;

            int left_dist = 0;

            stop_left.x = (m_image_new.cols / 2) - 50;
            stop_left.y = m_control_scanline;

            // Find first grey pixel in the front of the car
            for (int i = m_control_scanline; i > m_stop_scanline; i--) {
                front_left = m_image_new.at<uchar>(Point(stop_left.x, i));
                if (front_left > 150) {
                    stop_left.y = i;
                    left_dist = m_control_scanline - stop_left.y;
                    break;
                }
            }

            int right_dist = 0;

            stop_right.x = (m_image_new.cols / 2) + 50;
            stop_right.y = m_control_scanline;

            // Find first grey pixel in front of the car
            for (int i = m_control_scanline; i > m_stop_scanline; i--) {
                front_right = m_image_new.at<uchar>(Point(stop_right.x, i));
                if (front_right > 150) {
                    stop_right.y = i;
                    right_dist = m_control_scanline - stop_right.y;
                    break;
                }
            }

            // Draw lines if debug true
            if (m_debug) {
                if (stop_left.y < m_control_scanline) {
                    line(m_image, Point(stop_left.x, m_control_scanline), stop_left, Scalar(128, 0, 0));
                }

                if (stop_right.y < m_control_scanline) {
                    line(m_image, Point(stop_right.x, m_control_scanline), stop_right, Scalar(128, 0, 0));
                }
            }


            static int counter = 0;

            // is the detected stopline at a similar distance on both sides

            if (counter < 5 && (left_dist - right_dist) > -10 && (left_dist - right_dist) < 10 && left_dist != 0 &&
                right_dist != 0) {
                counter++;
            } else {
                counter = 0;
            }

            if (counter > 4) {
                stop = true;
            } else {
                stop = false;
            }
            return e;
        }


        void LaneFollower::laneFollower(double e) {

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
            } else {
                m_eSum += e;
            }

            // The following values have been determined by Twiddle algorithm.
            //Kp = p_gain -> Proportional -> how big of a turn when the car try to "fix" the error
            //const double Kp = 0.4482626884328734;
            //Ki = i_gain-> Integral -> Current -> might be the middle position of the car
            //const double Ki = 3.103197570937628;
            //Kd = d_gain-> derivative -> how frequent the reaction the car will be -> the smaller the better.
            //const double Kd = 0.030450210485408566;


            const double p = p_gain * e;
            const double i = i_gain * timeStep * m_eSum;
            const double d = d_gain * (e - m_eOld) / timeStep;
            m_eOld = e;

            const double y = p + i + d;

            double desiredSteering = 0;

            if (fabs(e) > 1e-2) {
                desiredSteering = y;

                // set an upper and lower limit for the desired steering
                if (desiredSteering > 1.5) {
                    desiredSteering = 1.5;
                }
                if (desiredSteering < -1.5) {
                    desiredSteering = -1.5;
                }

            }

            // Show resulting features.
            if (m_debug) {
                if (m_image.data != NULL) {
                    imshow("Debug Image",
                           m_image_equ);  //m_image = image without canny || m_image_new = fully processed image
                    waitKey(10);
                }
            }
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
            //int32_t covered_distance = 0;
            //Get most recent vehicle data:
            Container containerVehicleData = getKeyValueDataStore().get(VehicleData::ID());
            VehicleData vd = containerVehicleData.getData<VehicleData> ();
            //Get most recent sensor board data:
            Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
            SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

            if (stageMeasuring == FIND_OBJECT_INIT) {
                distanceToObstacleOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                stageMeasuring = FIND_OBJECT;
            } else if (stageMeasuring == FIND_OBJECT) {
                distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                //covered_distance =sbd.getValueForKey_MapOfDistances(WHEEL_ENCODER);
                //cerr << "Distance  " << covered_distance << endl;


                // Approaching an obstacle (stationary or driving slower than us).
                if ((distanceToObstacle > 0) && (((distanceToObstacleOld - distanceToObstacle) > 0) ||
                                                 (fabs(distanceToObstacleOld - distanceToObstacle) < 1e-2))) {
                    // Check if overtaking shall be started.
                    stageMeasuring = FIND_OBJECT_PLAUSIBLE;
                }

                distanceToObstacleOld = distanceToObstacle;
            } else if (stageMeasuring == FIND_OBJECT_PLAUSIBLE) {
                if (sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER) < OVERTAKING_DISTANCE) {
                    stageMoving = TO_LEFT_LANE_LEFT_TURN;
                    overtake = true;
                    // Disable measuring until requested from moving state machine again.
                    stageMeasuring = DISABLE;
                } else {
                    stageMeasuring = FIND_OBJECT;
                }
            } else if (stageMeasuring == HAVE_BOTH_IR) {
                // Remain in this stage until both IRs see something.
                if ((sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0) &&
                    (sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0)) {
                    // Turn to right.
                    stageMoving = TO_LEFT_LANE_RIGHT_TURN;
                }
            } else if (stageMeasuring == HAVE_BOTH_IR_SAME_DISTANCE) {
                // Remain in this stage until both IRs have the similar distance to obstacle (i.e. turn car)
                // and the driven parts of the turn are plausible.
                const double IR_FR = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                const double IR_RR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);

                if ((fabs(IR_FR - IR_RR) < HEADING_PARALLEL) &&
                    ((stageToRightLaneLeftTurn - stageToRightLaneRightTurn) > 0)) {
                    // Straight forward again.
                    stageMoving = CONTINUE_ON_LEFT_LANE;

                    // Reset PID controller.
                    m_eSum = 0;
                    m_eOld = 0;
                }
            } else if (stageMeasuring == END_OF_OBJECT) {
                // Find end of object.
                distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);

                if (distanceToObstacle < 0) {
                    // Move to right lane again.
                    stageMoving = TO_RIGHT_LANE_RIGHT_TURN;

                    // Disable measuring until requested from moving state machine again.
                    stageMeasuring = DISABLE;
                }

            }
        }

        void LaneFollower::movingMachine(bool has_next_frame){

            Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
            VehicleData vd = containerVehicleData.getData<VehicleData>();

            // 2. Get most recent sensor board data:
            Container containerSensorBoardData = getKeyValueDataStore().get(
                    automotive::miniature::SensorBoardData::ID());
            SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData>();
            overtake = false;
            // Moving state machine.
            if (stageMoving == FORWARD && has_next_frame) {
                // Use m_vehicleControl data from image processing.
                //processImage();

                stageToRightLaneLeftTurn = 0;
                stageToRightLaneRightTurn = 0;
            } else if (stageMoving == TO_LEFT_LANE_LEFT_TURN) {
                // Move to the left lane: Turn left part until both IRs see something.
                m_vehicleControl.setSpeed(0.5);
                m_vehicleControl.setSteeringWheelAngle(-10);


                // State machine measuring: Both IRs need to see something before leaving this moving state.
                stageMeasuring = HAVE_BOTH_IR;

                stageToRightLaneRightTurn++;
                cout << "StageToRightLaneRightTurn is " << stageToRightLaneRightTurn << endl;
            } else if (stageMoving == TO_LEFT_LANE_RIGHT_TURN) {
                // Move to the left lane: Turn right part until both IRs have the same distance to obstacle.
                // State machine measuring: Both IRs need to have the same distance before leaving this moving state.
                stageMeasuring = HAVE_BOTH_IR_SAME_DISTANCE;
                if(stageToRightLaneLeftTurn <130) {
                    m_vehicleControl.setSpeed(0.5);//0.5
                    m_vehicleControl.setSteeringWheelAngle(15);//15
                    stageToRightLaneLeftTurn++;
                }

                cout << "stageToRightLaneLeftTurn is " << stageToRightLaneLeftTurn << endl;
            } else if (stageMoving == CONTINUE_ON_LEFT_LANE) {
                // Move to the left lane: Passing stag
                // Use m_vehicleControl data from image processing.

                // Find end of object.
                stageMeasuring = END_OF_OBJECT;
            } else if (stageMoving == TO_RIGHT_LANE_RIGHT_TURN) {
                // Move to the right lane: Turn right part.
                m_vehicleControl.setSpeed(0.5);
                m_vehicleControl.setSteeringWheelAngle(10);



                stageToRightLaneRightTurn--;
                cout << "Stage RightLaneRightTurn is " << stageToRightLaneRightTurn << endl;

                if (stageToRightLaneRightTurn < 80) {
                    cout << "Going left " << endl;
                    stageMoving = TO_RIGHT_LANE_LEFT_TURN;

                }
            } else if (stageMoving == TO_RIGHT_LANE_LEFT_TURN) {
                // Move to the left lane: Turn left part.
                m_vehicleControl.setSpeed(0.5);
                m_vehicleControl.setSteeringWheelAngle(-10);//-10

                stageToRightLaneLeftTurn--;
                cout << "Stage  is " << stageToRightLaneLeftTurn << endl;
                //TESTING steps
                if (stageToRightLaneLeftTurn < 30) {
                    // Start over.
                    overtake = false;
                    stageMoving = FORWARD;

                    stageMeasuring = FIND_OBJECT_INIT;

                    distanceToObstacle = 0;
                    distanceToObstacleOld = 0;

                    // Reset PID controller.
                    m_eSum = 0;
                    m_eOld = 0;
                }
            }


        }

        // This method will do the main data processing job.
        // Therefore, it tries to open the real camera first. If that fails, the virtual camera images from camgen are used.
          // this method still needs
        ModuleExitCodeMessage::ModuleExitCode LaneFollower::body() {
            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();
            m_debug = kv.getValue<int32_t>("lanefollower.debug") == 1;

            //cvInitFont(&m_font, CV_FONT_HERSHEY_DUPLEX, hscale, vscale, shear, thickness, lineType);



            // Overall state machine handler.
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == ModuleStateMessage::RUNNING) {
                bool has_next_frame = false;


                // Get the most recent available container for a SharedImage.
                Container image_container = getKeyValueDataStore().get(SharedImage::ID());


                if (image_container.getDataType() == SharedImage::ID()) {
                    has_next_frame = readSharedImage(image_container);
                }

                // If we have an image from the previous call, it is then processed
                if (has_next_frame) {
                    processImage();
                    double error = errorCalculation();
                    laneFollower(error);
                    movingMachine(has_next_frame);
                    measuring_state_machine();


                    // overtaking part

                    // State control for intersection stop
                    if (state == "moving") {
                        if (Sim) {
                            m_vehicleControl.setSpeed(1);
                        } else {
                            m_vehicleControl.setSpeed(100);
                        }

                        if (stop) {
                            if (stopCounter < 6.0) {
                                stopCounter += 0.5;
                                cerr << stopCounter << endl;
                            } else {
                                state = "stop";
                                cerr << "Stopline found ! " << endl;
                            }
                        }

                    }else if (state == "stop") {
                        m_vehicleControl.setSteeringWheelAngle(0);
                        if (Sim) {
                            m_vehicleControl.setSpeed(0);
                        } else {
                            m_vehicleControl.setSpeed(90);
                        }
                        stopCounter += 0.5;

                        if (stopCounter > 29.9999) {
                            state = "resume";
                            if (Sim) {
                                m_vehicleControl.setSpeed(1);
                            } else {
                                m_vehicleControl.setSpeed(99);
                            }
                            cerr << "Resuming!" << endl;
                        }
                    }else if (state == "resume") {
                        if (stopCounter < 50) {
                            stopCounter += 0.5;
                            cerr << "counter " << stopCounter << endl;
                        } else {
                            stopCounter = 0;
                            state = "moving";
                            cerr << "Moving!" << endl;
                        }
                    }


                }
                // Create container for finally sending the set values for the control algorithm.
                //Container c2(m_vehicleControl);
                // Send container.
                //getConference().send(c2);
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
} // scaledcars::contr