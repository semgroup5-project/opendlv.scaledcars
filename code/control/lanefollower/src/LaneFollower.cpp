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
#include <algorithm>

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
        using namespace odcore::data::dmcp;
        using namespace automotive;
        using namespace odcore::wrapper;
        using namespace odcore::data::dmcp;
        using namespace automotive::miniature;
        using namespace group5;

        Mat m_image_mat, m_image_new;
        bool stop = false;
        double stopCounter = 0, counter = 0;
        String state = "moving", prevState = "moving";
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
                laneFollowerMSG(),
                m_threshold1(50),  //50
                m_threshold2(200),  // 150
                m_control_scanline(400),//needs testing with real c
                m_stop_scanline(250),//needs testing with real car
                m_distance(245),  //needs testing with real car as well
                p_gain(0),       // the gain values can be adjusted here outside of simulation scenario (see @setUp() )
                i_gain(0),
                d_gain(0),
                _state(0) {}

        // Initialize fonts.
        const double hscale = 0.4;
        const double vscale = 0.3;//0.3;
        const double shear = 0.2;
        const int thickness = 1;
        const int lineType = 6;



        // Parameters for overtaking.
        const int32_t ULTRASONIC_FRONT_CENTER = 3;
        const int32_t ULTRASONIC_FRONT_RIGHT = 4;
        const int32_t INFRARED_FRONT_RIGHT = 0;
        const int32_t INFRARED_REAR_RIGHT = 2;
        //const int32_t INFRARED_BACK = 1;
        //const int32_t WHEEL_ENCODER = 5;

        const double OVERTAKING_DISTANCE = 5.0;
        const double HEADING_PARALLEL = 0.04;

        const double TURN_SPEED_SIM = 0.7;
        const double TURN_ANGLE_SIM_LEFT = -15;
        const double TURN_ANGLE_SIM_RIGHT = 15;

        const double TURN_SPEED_CAR = 100;
        const double TURN_ANGLE_CAR_LEFT = 60;
        const double TURN_ANGLE_CAR_RIGHT = 120;

        // Overall state machines for moving and measuring.
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

        // State counter for dynamically moving back to right lane.
        //int32_t stageToRightLaneRightTurn = 0;
        //int32_t stageToRightLaneLeftTurn = 0;
        //int32_t traveled_distance = 0;

        double distanceOUTtoL_0 = 0;
        double distanceOUTtoL_1 = 0;

        double distanceOUTtoR_0 = 0;
        double distanceOUTtoR_1 = 0;

        double distanceINtoR_0 = 0;
        double distanceINtoR_1 = 0;

        double distanceINtoL_0 = 0;
        double distanceINtoL_1 = 0;


        /*
        double distanceLtoL_0 = 0;
        double distanceLtoL_1 = 0;

        double distanceLtoR_0 = 0;
        double distanceLtoR_1 = 0;

        double distanceRtoR_0 = 0;
        double distanceRtoR_1 = 0;

        double distanceRtoL_0 = 0;
        double distanceRtoL_1 = 0;
        */

        // Distance variables to ensure we are overtaking only stationary or slowly driving obstacles.
        double distanceToObstacle = 0;
        double distanceToObstacleOld = 0;

        bool overtake = true;
        LaneFollower::~LaneFollower() {}

        // This method will be call automatically _before_ running body().
        void LaneFollower::setUp() {

            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();
            m_debug = kv.getValue<int32_t>("global.debug") == 1;
            Sim = kv.getValue<int32_t>("global.sim") == 1;
            p_gain = kv.getValue<double>("lanefollower.p");
            d_gain = kv.getValue<double>("lanefollower.d");
            i_gain = kv.getValue<double>("lanefollower.i");

            // setup window for debugging
            if (m_debug) {
                cvNamedWindow("Debug Image", CV_WINDOW_AUTOSIZE);
                cvMoveWindow("Debug Image", 300, 100);
            }
            if (Sim) {
                m_control_scanline = 462; // calibrated length to right: 280px
                m_distance = 250;  // distance from right lane marking
            }

            m_vehicleControl.setBrakeLights(true);
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
                                       CV_8UC3); //From opencv documentation: CV_8UC3 = unsigned integer 8bit matrix/image wih 3 mats (typically RGB or BRG in opencv case)
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
            m_image_mat = Mat(m_image.rows, m_image.cols, CV_8UC1);
            // Copy the original image to the new image as greyscale

            cvtColor(m_image, m_image_mat, COLOR_BGR2GRAY);

            GaussianBlur(m_image_mat, m_image_new, Size(5, 5), 0, 0);
            // calc median of pixel color
            double median;
            median = Median(m_image_new);

            m_threshold1 = max(static_cast<double>(0), ((1.0 - 0.33) * median));
            m_threshold2 = min(static_cast<double>(255), (1.0 + 0.33) * median);

            Canny(m_image_new, m_image_new, m_threshold1, m_threshold2,
                  3); // see header for algorithm and threshold explanation
        }

        double LaneFollower::Median(Mat mat) {
            double m = (mat.rows * mat.cols) / 2;
            int bin = 0;
            double med = -1.0;

            int histSize = 256;
            float range[] = {0, 256};
            const float *histRange = {range};
            bool uniform = true;
            bool accumulate = false;
            Mat hist;
            calcHist(&mat, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

            for (int i = 0; i < histSize && med < 0.0; ++i) {
                bin += cvRound(hist.at<float>(i));
                if (bin > m && med < 0.0)
                    med = i;
            }

            return med;
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
            for (int x = m_image_new.cols / 2;
                 x < m_image_new.cols ; x++) {  //cols - 50 to stop it from finding the wall
                pixelRight = m_image_new.at<uchar>(Point(x, y));
                if (pixelRight >= 150) {   //tentative value, might need adjustment: lower it closer to 100
                    right.x = x;
                    break;
                }
            }

            if (right.x == -1 && left.x == -1) {  //setting state if the car does not see any line
                state = "danger";
            }
            else{
                if(state != "stop" && state != "resume"){
                    state = "moving";
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
//                        m_eSum = 0;
//                        m_eOld = 0;
                        e = (m_distance - (m_image_new.cols / 2.0 - left.x)) / m_distance;
                    } else if (right.x > 0) {
//                        m_eSum = 0;
//                        m_eOld = 0;
                        e = ((right.x - m_image_new.cols / 2.0) - m_distance) / m_distance;
                    }
                }
            }

            // stopline logic

            uchar front_left, front_right;
            Point stop_left, stop_right;

            int left_dist = 0;

            stop_left.x = (m_image_new.cols / 2) - 40;   // stop line checker needs to be moved more towards the left side
            stop_left.y = m_control_scanline;

            // Find first grey pixel in the front of the car left side
            for (int i = m_control_scanline; i > m_stop_scanline; i--) {
                front_left = m_image_new.at<uchar>(Point(stop_left.x, i));
                if (front_left > 150) {
                    stop_left.y = i;
                    left_dist = m_control_scanline - stop_left.y;
                    break;
                }
            }

            int right_dist = 0;

            stop_right.x = (m_image_new.cols / 2) + 40;  // stop line checker needs to be moved more towards the left side
            stop_right.y = m_control_scanline;

            // Find first grey pixel in front of the car right side
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
                    line(m_image_new, Point(stop_left.x, m_control_scanline), stop_left, Scalar(255, 0, 0));
                }

                if (stop_right.y < m_control_scanline) {
                    line(m_image_new, Point(stop_right.x, m_control_scanline), stop_right, Scalar(255, 0, 0));
                }
            }

            //prints the lines for debugging purposes if debug flag is set to true
            if (m_debug) {
                std::string speed = std::to_string(m_vehicleControl.getSpeed());
                putText(m_image_new, "Speed is " + speed , Point(m_image_new.cols - 150, 20), FONT_HERSHEY_PLAIN, 1,
                        CV_RGB(255, 255, 255));

                std::string steer = std::to_string(90+ (m_vehicleControl.getSteeringWheelAngle() * (180/3.14)));
                putText(m_image_new, "Steering: " + steer , Point(m_image_new.cols - 150, 40), FONT_HERSHEY_PLAIN, 1,
                        CV_RGB(255, 255, 255));

                putText(m_image_new, "State: " + state , Point(m_image_new.cols - 150, 60), FONT_HERSHEY_PLAIN, 1,
                        CV_RGB(255, 255, 255));

                putText(m_image_new, "Old state: " + prevState  , Point(m_image_new.cols - 150, 80), FONT_HERSHEY_PLAIN, 1,
                      CV_RGB(255, 255, 255));


                putText(m_image_new, "Stop counter :" + std::to_string(stopCounter) , Point(m_image_new.cols - 150, 100), FONT_HERSHEY_PLAIN, 1,
                        CV_RGB(255, 255, 255));

//                putText(m_image_new, state , Point(m_image_new.cols - 80, 20), FONT_HERSHEY_PLAIN, 1,
//                        CV_RGB(255, 255, 255));
//
//                std::string speed = std::to_string(m_vehicleControl.getSpeed());
//                putText(m_image_new, speed , Point(m_image_new.cols - 80, 40), FONT_HERSHEY_PLAIN, 1,
//                        CV_RGB(255, 255, 255));
//

                putText(m_image_new, "Distance " + std::to_string(m_distance) , Point(m_image_new.cols - 150, 120), FONT_HERSHEY_PLAIN, 1,
                        CV_RGB(255, 255, 255));

                if (left.x > 0) {
                    line(m_image_new, Point(m_image.cols / 2, y), left, Scalar(255, 0, 0), 1, 8);
                    std::string left_reading = std::to_string((m_image_new.cols / 2 - left.x));


                    putText(m_image_new, left_reading, Point(m_image_new.cols / 2 - 100, y - 2), FONT_HERSHEY_PLAIN, 1,
                            CV_RGB(255, 255, 255));
                }
                if (right.x > 0) {
                    line(m_image_new, cvPoint(m_image.cols / 2, y), right, Scalar(255, 0, 0), 1, 8);
                    std::string right_reading = std::to_string((right.x - m_image_new.cols / 2));

                    putText(m_image_new, right_reading, Point(m_image_new.cols / 2 + 100, y - 2), FONT_HERSHEY_PLAIN, 1,
                            CV_RGB(255, 255, 255));
                }
            }

            // is the detected stopline at a similar distance on both sides
            if (counter < 5 && (left_dist - right_dist) > -10 && (left_dist - right_dist) < 10 && left_dist != 0 &&
                right_dist != 0) {
                counter++;
            }else{
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

            //a more soft way to handle instead of resetting to 0 a way to mitigate toggling in corners
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
                           m_image_new);  //m_image = image without canny || m_image_new = fully processed image
                    waitKey(10);
                }
            }

            if (!m_vehicleControl.getBrakeLights()) {
                m_vehicleControl.setSteeringWheelAngle(desiredSteering);
            }


        void LaneFollower::measuring_state_machine() {
            // Get vehicle data
            Container vdContainer = getKeyValueDataStore().get(VehicleData::ID());
            VehicleData vd = vdContainer.getData<VehicleData>();

            // Get sensor board data
            Container sbdContainer = getKeyValueDataStore().get(SensorBoardData::ID());
            SensorBoardData sbd = sbdContainer.getData<SensorBoardData>();

            // Get communication link message
            Container clmContainer = getKeyValueDataStore().get(CommunicationLinkMSG::ID());
            CommunicationLinkMSG clm = clmContainer.getData<CommunicationLinkMSG>();

            if (stageMeasuring == FIND_OBJECT_INIT) {
                cerr << "FIND_OBJECT_INIT" << endl;

                // Read initial distance to obstacle
                if (Sim) {
                    distanceToObstacleOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                } else {
                    distanceToObstacleOld = clm.getUltraSonicFrontCenter();
                }

                stageMeasuring = FIND_OBJECT;

            } else if (stageMeasuring == FIND_OBJECT) {
                cerr << "FIND_OBJECT" << endl;

                if (Sim) {
                    distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                } else {
                    distanceToObstacle = clm.getUltraSonicFrontCenter();
                }

                // Approaching an obstacle (stationary or driving slower than us).
                if ((distanceToObstacle > 0) && (((distanceToObstacleOld - distanceToObstacle) > 0) ||
                                                 (fabs(distanceToObstacleOld - distanceToObstacle) < 1e-2))) {
                    // Check if overtaking shall be started.
                    stageMeasuring = FIND_OBJECT_PLAUSIBLE;
                }

                distanceToObstacleOld = distanceToObstacle;

            } else if (stageMeasuring == FIND_OBJECT_PLAUSIBLE) {
                cerr << "FIND_OBJECT_PLAUSIBLE" << endl;

                double distance;

                if (Sim) {
                    distance = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                } else {
                    distance = clm.getUltraSonicFrontCenter();
                }

                if (distance < OVERTAKING_DISTANCE) {
                    overtake = true;

                    stageMoving = OUT_TO_LEFT;
                    if (Sim) {
                        distanceOUTtoL_0 = vd.getAbsTraveledPath();
                    } else {
                        distanceOUTtoL_0 = clm.getWheelEncoder();
                    }

                    stageMeasuring = DISABLE;
                } else {
                    stageMeasuring = FIND_OBJECT;
                }

            } else if (stageMeasuring == HAVE_BOTH_IR) {
                cerr << "HAVE_BOTH_IR" << endl;

                double infraredFrontRightDistance;
                double infraredRearRightDistance;

                if (Sim) {
                    infraredFrontRightDistance = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                    infraredRearRightDistance = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
                } else {
                    infraredFrontRightDistance = clm.getInfraredSideFront();
                    infraredRearRightDistance = clm.getInfraredSideBack();
                }

                if ((infraredFrontRightDistance > 0) &&
                    (infraredRearRightDistance > 0)) {

                    if (Sim) {
                        distanceOUTtoL_1 = vd.getAbsTraveledPath();
                    } else {
                        distanceOUTtoL_1 = clm.getWheelEncoder();
                    }

                    stageMoving = OUT_TO_RIGHT;
                    if (Sim) {
                        distanceOUTtoR_0 = vd.getAbsTraveledPath();
                    } else {
                        distanceOUTtoR_0 = clm.getWheelEncoder();
                    }
                }

            } else if (stageMeasuring == HAVE_BOTH_IR_SAME_DISTANCE) {
                cerr << "HAVE_BOTH_IR_SAME_DISTANCE" << endl;

                double IR_FR;
                double IR_RR;

                if (Sim) {
                    IR_FR = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                    IR_RR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
                } else {
                    IR_FR = clm.getInfraredSideFront();
                    IR_RR = clm.getInfraredSideBack();
                }

                double traveled;
                if (Sim) {
                    traveled = vd.getAbsTraveledPath();
                } else {
                    traveled = clm.getWheelEncoder();
                }

                double distanceOUTtoL = distanceOUTtoL_1 - distanceOUTtoL_0;
                double distanceOUTtoR = traveled - distanceOUTtoR_0;

                bool sensorCondition = (fabs(IR_FR - IR_RR) < HEADING_PARALLEL);
                bool distanceCondition = distanceOUTtoR > distanceOUTtoL;

                if (sensorCondition && distanceCondition) {
                    stageMoving = CONTINUE_STRAIGHT;

                    if (Sim) {
                        distanceOUTtoR_1 = vd.getAbsTraveledPath();
                    } else {
                        distanceOUTtoR_1 = clm.getWheelEncoder();
                    }

                    // Reset PID controller.
                    m_eSum = 0;
                    m_eOld = 0;
                }

            } else if (stageMeasuring == END_OF_OBJECT) {
                cerr << "END_OF_OBJECT" << endl;

                if (Sim) {
                    distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);
                } else {
                    distanceToObstacle = clm.getUltraSonicFrontCenter();
                }

                if (distanceToObstacle < 0) {
                    stageMoving = IN_TO_RIGHT;
                    if (Sim) {
                        distanceINtoR_0 = vd.getAbsTraveledPath();
                    } else {
                        distanceINtoR_0 = clm.getWheelEncoder();
                    }

                    stageMeasuring = DISABLE;
                }
            }
        }

        void LaneFollower::movingMachine(bool has_next_frame) {
            // Get vehicle data
            Container vdContainer = getKeyValueDataStore().get(VehicleData::ID());
            VehicleData vd = vdContainer.getData<VehicleData>();

            // Get sensor board data
            Container sbdContainer = getKeyValueDataStore().get(SensorBoardData::ID());
            SensorBoardData sbd = sbdContainer.getData<SensorBoardData>();

            // Get communication link message
            Container clmContainer = getKeyValueDataStore().get(CommunicationLinkMSG::ID());
            CommunicationLinkMSG clm = clmContainer.getData<CommunicationLinkMSG>();

            if (stageMoving == FORWARD && has_next_frame) {
                cerr << "FORWARD" << endl;

                // Reset counters

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
                if (Sim) {
                    traveled = vd.getAbsTraveledPath();
                } else {
                    traveled = clm.getWheelEncoder();
                }

                double traveledSoFar = traveled - distanceINtoR_0;
                double traveledRequired = distanceOUTtoR_1 - distanceOUTtoR_0;

                if (traveledSoFar > (traveledRequired * 0.8)) {
                    stageMoving = IN_TO_LEFT;
                    if (Sim) {
                        distanceINtoL_0 = vd.getAbsTraveledPath();
                    } else {
                        distanceINtoL_0 = clm.getWheelEncoder();
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
                if (Sim) {
                    traveled = vd.getAbsTraveledPath();
                } else {
                    traveled = clm.getWheelEncoder();
                }

                double traveledSoFar = traveled - distanceINtoL_0;
                double traveledRequired = distanceOUTtoL_1 - distanceOUTtoL_0;

                if (traveledSoFar > (traveledRequired * 0.0)) {
                    overtake = false;

                    stageMoving = FORWARD;
                    stageMeasuring = FIND_OBJECT_INIT;

                    distanceToObstacle = 0;
                    distanceToObstacleOld = 0;

                    // Reset PID controller
                    m_eSum = 0;
                    m_eOld = 0;
                }
            }
        }


        // This method will do the main data processing job.
        // Therefore, it tries to open the real camera first. If that fails, the virtual camera images from camgen are used.
        ModuleExitCodeMessage::ModuleExitCode LaneFollower::body() {    // this method still needs
            // Overall state machine handler.
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == ModuleStateMessage::RUNNING) {

                Container communicationLinkContainer = getKeyValueDataStore().get(CommunicationLinkMSG::ID());
                if (communicationLinkContainer.getDataType() == CommunicationLinkMSG::ID()) {
                    const CommunicationLinkMSG communicationLinkMSG = communicationLinkContainer.getData<CommunicationLinkMSG>();
                    _state = communicationLinkMSG.getStateLaneFollower();
                }

                cerr << "STATE IS : " << _state << endl;
                if (_state == 1) {
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
                    }

                    movingMachine(has_next_frame) ;
                    measuring_state_machine();

                    // State control for intersection stop
                    if (state == "moving")
                    {
                        m_vehicleControl.setBrakeLights(false);
                        if (Sim)
                        {
                            m_vehicleControl.setSpeed(1);
                        }
                        else
                        {
                            if (stop)
                            {
                                if (stopCounter < 2.0)
                                {
                                    stopCounter += 0.5;
                                }
                                else
                                {
                                    if (prevState == "stopLine")
                                    {
                                        m_vehicleControl.setSpeed(100);
                                    }
                                    else
                                    {
                                        state = "stop";
                                        stopCounter = 0;
                                    }
                                }
                            }
                            else
                            {
                                m_vehicleControl.setSpeed(100);
                                prevState = "moving";
                            }
                        }
                    }
                    if (state == "stop")
                    {
                        m_vehicleControl.setSteeringWheelAngle(0);
                        m_vehicleControl.setBrakeLights(true);
                        if (Sim)
                        {
                            m_vehicleControl.setSpeed(0);
                        }
                        else
                        {
                            m_vehicleControl.setSpeed(190);
                        }

                        stopCounter += 0.5;

                        if (stopCounter > 20.9999)
                        {
                            state = "resume";
                            prevState = "stopLine";
                            m_vehicleControl.setBrakeLights(false);
                            if (Sim)
                            {
                                m_vehicleControl.setSpeed(1);
                            }
                            else
                            {
                                m_vehicleControl.setSpeed(100);
                            }

                        }
                    }
                    if (state == "resume")
                    {
                        if (stopCounter < 50)
                        {
                            stopCounter += 0.5;
                        }
                        else
                        {
                            stopCounter = 0;
                            state = "moving";
                            cerr << "Moving!" << endl;
                        }
                    }
                    if (state == "danger")
                    {
                        if (prevState == "stopLine")
                        {  // The idea here is, after a stop line, go forward and dont steer at all, it is expected to not find any reference line markings
                            m_vehicleControl.setSpeed(100);
                            m_vehicleControl.setSteeringWheelAngle(0);
                            m_vehicleControl.setBrakeLights(false);
                        }
                        if (prevState == "moving")
                        {
                            prevState = "danger";
                            m_vehicleControl.setBrakeLights(true);
                            m_vehicleControl.setSpeed(190);
                            m_vehicleControl.setSteeringWheelAngle(0);
                        }
                    }
                    // Create container for finally sending the set values for the control algorithm.
                    Container c2(m_vehicleControl);
                    // Send container.
                    getConference().send(c2);

                }
            }
            return ModuleExitCodeMessage::OKAY;
        }
    }
} // scaledcars::contr