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
#include <algorithm>

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
        using namespace group5;

        Mat m_image_mat, m_image_new;
        bool stop = false;
        double stopCounter = 0, counter = 0;
        String state = "moving", prevState = "moving";
        bool inRightLane = true;   //Flip this value to indicate lane change

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
                m_control_scanline(250),//needs testing with real c
                m_stop_scanline(170),//needs testing with real car
                m_distance(130),  //needs testing with real car as well
                p_gain(0),       // the gain values can be adjusted here outside of simulation scenario (see @setUp() )
                i_gain(0),
                d_gain(0),
                _state(0) {}

        LaneFollower::~LaneFollower() {}

        // This method will be call automatically _before_ running body().
        void LaneFollower::setUp() {
            // Get configuration data for this class.
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
            Point left, right, left2, right2;  // P(x, y) = P (column, row)

            left.y = y;
            left2.y = 50;
            left.x = -1;
            left2.x = -1;

            // Search from middle to the left
            for (int x = m_image_new.cols / 2; x > 0; x--) {//from the middle to the left
                pixelLeft = m_image_new.at<uchar>(Point(x, y));
                if (pixelLeft >= 150) {   //tentative value, might need adjustment: lower it closer to 100
                    left.x = x;
                    break;
                }
            }

            right.y = y;
            right2.y = 50;
            right.x = -1;
            right2.x = -1;
            // Search from middle to the right
            for (int x = m_image_new.cols / 2;
                 x < m_image_new.cols ; x++) {
                pixelRight = m_image_new.at<uchar>(Point(x, y));
                if (pixelRight >= 150) {   //tentative value, might need adjustment: lower it closer to 100
                    right.x = x;
                    break;
                }
            }

            if (right.x == -1 && left.x == -1) {  //setting state if the car does not see any line
                for (int x = m_image_new.cols / 2; x > 0; x--) {//from the middle to the left
                    pixelLeft = m_image_new.at<uchar>(Point(x, y));
                    if (pixelLeft >= 150) {   //tentative value, might need adjustment: lower it closer to 100
                        left2.x = x;
                        break;
                    }
                }

                // Search from middle to the right
                for (int x = m_image_new.cols / 2;
                     x < m_image_new.cols - 30; x++) {
                    pixelRight = m_image_new.at<uchar>(Point(x, y));
                    if (pixelRight >= 150) {   //tentative value, might need adjustment: lower it closer to 100
                        right2.x = x;
                        break;
                    }
                }
                if (right2.x == -1 && left2.x == -1)
                state = "danger";
            }
            else{
                if(state != "stop" && state != "resume"){
                    state = "moving";
                }
            }

            if (right.x > 0) right.x += 20;
            if (left.x > 0) left.x += 20;

            if (y == m_control_scanline) {

                if (inRightLane) {
                    if (right.x > 0) {
                       // m_distance = 210;
                        e = ((right.x - m_image_new.cols / 2.0) - m_distance) / m_distance;
                    } else if (left.x > 0) {
                       // m_distance = 190;
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

            stop_left.x = (m_image_new.cols / 2) - 70;   // stop line checker needs to be moved more towards the left side
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

            stop_right.x = (m_image_new.cols / 2) ;  // stop line checker needs to be moved more towards the left side
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
            if (counter < 5 && (left_dist - right_dist) > -15 && (left_dist - right_dist) < 15 && left_dist != 0 &&
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
            if(fabs(e) < 1e-2) {
                m_eSum = 0;
            } else {
                m_eSum += e;
            }

            // PID control algorithm uses the following values, with the meaning:
            //Kp = p_gain -> Proportional -> how big of a turn when the car try to "fix" the error
            //Ki = i_gain-> Integral -> Current -> might be the middle position of the car
            //Kd = d_gain-> derivative -> how frequent the reaction the car will be -> the smaller the better.


            const double p = p_gain * e;
            const double i = i_gain * timeStep * m_eSum;
            const double d = d_gain * (e - m_eOld) / timeStep;
            m_eOld = e;

            const double y = p + i + d;

            double desiredSteering = 0;

            if (fabs(e) > 1e-2) {
                desiredSteering = y;
            }
            // set an upper and lower limit for the desired steering
            if (desiredSteering > 1.5) {
                desiredSteering = 1.5;
            }
            if (desiredSteering < -1.5) {
                desiredSteering = -1.5;
            }

            // Show resulting image from image processing
            if (m_debug) {
                if (m_image.data != NULL) {
                    imshow("Debug Image",
                           m_image_new);
                    waitKey(10);
                }
            }
            m_vehicleControl.setSteeringWheelAngle(desiredSteering);


            // change this to use distance readings from image processing, same as line drawing
//            int curveCheckerRight, curveCheckerLeft;
//
//            if (desiredSteering < 0) {
//                curveCheckerLeft++;
//            }
//            if (desiredSteering > 0) {
//                curveCheckerRight++;
//            }
//
//            if (curveCheckerLeft > 5) {
//                m_distance = 220;
//            } else if (curveCheckerRight > 5) {
//                m_distance = 190;
//            }
        }

        // Body method does the main data processing job.
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

                    // State control for intersection stop
                    if (state == "moving") {
                        if (Sim) {
                            m_vehicleControl.setSpeed(1);
                        } else {
                            if (stop) {
                                if (stopCounter < 1.0) {
                                    stopCounter += 0.5;
                                }else{
                                    if (prevState == "stopLine"){
                                        m_vehicleControl.setSpeed(99);
                                    }else{
                                        state = "stop";
                                        stopCounter = 0;
                                    }
                                }
                            } else {
                                m_vehicleControl.setSpeed(99);
                                prevState = "moving";
                            }
                        }
                    }
                    if (state == "stop") {
                        m_vehicleControl.setSteeringWheelAngle(0);
                        if (Sim) {
                            m_vehicleControl.setSpeed(0);
                        } else {
                            m_vehicleControl.setSpeed(190);
                        }

                        stopCounter += 0.5;

                        if (stopCounter > 30.9999) {
                            state = "resume";
                            prevState = "stopLine";


                        }
                    }
                    if (state == "resume"){
                        if (stopCounter < 50.0) {
                            stopCounter += 0.5;
                            if (Sim) {
                                m_vehicleControl.setSpeed(1);
                            } else {
                                m_vehicleControl.setSpeed(99);
                                m_vehicleControl.setSteeringWheelAngle(0);
                            }
                        } else {
                                stopCounter = 0;
                                state = "moving";
                                cerr << "Moving!" << endl;
                        }
                    }
                    if (state == "danger") {
                        if (prevState ==
                            "stopLine") {  // The idea here is, after a stop line, go forward and dont steer at all, it is expected to not find any reference line markings
                            m_vehicleControl.setSpeed(100);
                            m_vehicleControl.setSteeringWheelAngle(0);
                        }
                        if (prevState ==
                            "moving") {
                            prevState = "danger";
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