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

#include "LaneFollower.h"
#include <opendavinci/odtools/player/Player.h>

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

        Mat m_image_new;
        bool stop = false;
        double stopCounter = 0;
        String state = "moving";

        LaneFollower::LaneFollower(const int32_t &argc, char **argv) :
                TimeTriggeredConferenceClientModule(argc, argv, "lanefollower"),
                m_sharedImageMemory(),
                m_sharedProcessedImageMemory(),
                m_sharedProcessedImage(),
                m_hasAttachedToSharedImageMemory(false),
                m_debug(false),
                Sim(true),
                m_image(),
                m_previousTime(),
                m_eSum(0),
                m_eOld(0),
                m_vehicleControl(),
                m_threshold1(50),
                m_threshold2(190),
                m_control_scanline(200),//needs testing with real c
                m_stop_scanline(250),//needs testing with real car
                m_distance(190),  //needs testing with real car as well
                p_gain(1.30),       // the gain values can be adjusted here outside of simulation scenario (see @setUp() )
                i_gain(0.01),
                d_gain(0.10){}

        LaneFollower::~LaneFollower() {}


        // This method will be call automatically _before_ running body().
        void LaneFollower::setUp() {
            // setup window for debugging
            if (m_debug) {
                cvNamedWindow("WindowShowImage", CV_WINDOW_AUTOSIZE);
                cvMoveWindow("WindowShowImage", 300, 100);
            }
            if (Sim){
                m_control_scanline = 462; // calibrated length to right: 280px
                m_distance = 250;  // distance from right lane marking

                p_gain = 1.30;
                i_gain = 0;
                d_gain = 0;

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
                SharedImage si = c.getData<SharedImage> ();


                // Check if we have already attached to the shared memory.
                if (!m_hasAttachedToSharedImageMemory) {

                    m_sharedImageMemory = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(si.getName());  // non valid when using camera

//
//                    cerr << "sharedmemory.get is " << m_sharedImageMemory.get() << endl;
//                      cerr << "si.getWidth is " << si.getName() << endl;
//                    cerr << "si.getWidth is " << si.getWidth() << endl;
//                    cerr << "sharedmemory.getSM is " << m_sharedImageMemory->getSharedMemory() << endl;
//                    cerr << "name at m_sharedMemory " << m_sharedImageMemory->getName() << endl;
//                    cerr << "size at m_sharedMemory " << m_sharedImageMemory->getSize() << endl;
//                    cerr << "value for m_sharedMemory -> valid " << m_sharedImageMemory->isValid() << endl;



                    m_hasAttachedToSharedImageMemory = true;

                }

                // Check if we could successfully attach to the shared memory.
                if (m_sharedImageMemory->isValid()) {
                    // Lock the memory region to gain exclusive access using a scoped lock.
                    m_sharedImageMemory->lock();

                    if (m_image.empty()) { // If image is empty, create a new cv::Mat image
                        m_image.create(si.getHeight(), si.getWidth(), CV_8UC3); //From opencv documentation: CV_8UC3 = unsigned integer 8bit matrix/image wih 3 channels (typically RGB or BRG in opencv case)
                    }

                    else { // Copying the image data
                        memcpy(m_image.data, m_sharedImageMemory->getSharedMemory(), si.getWidth() * si.getHeight() * si.getBytesPerPixel());
                    }
                    m_sharedImageMemory-> unlock(); // Release the memory region lock

                    if (Sim){ // If in Sim mode, flip the image
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

            Canny(m_image_new, m_image_new, m_threshold1, m_threshold2, 3); // see header for algorithm and threshold explanation

//            if (!Sim){ //If not in sim mode, copy processed image to shared memory
//                if (m_sharedProcessedImageMemory.get() && m_sharedProcessedImageMemory->isValid()) {
//                    m_sharedProcessedImageMemory->lock();
//                    cerr << "inside image process and fixing shared process image for display" << endl;
//                    memcpy(m_sharedProcessedImageMemory->getSharedMemory(), m_image_new.data, 640*480);
//                    m_sharedProcessedImageMemory->unlock();
//                }
//            }
        }

        // Calculate deviation from goal
        double LaneFollower::errorCalculation() {
            static bool useRightLaneMarking = true;   // we can the overtaker alter this value to signal a lane change
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
            for (int x = m_image_new.cols / 2; x < m_image_new.cols; x++) {
                pixelRight = m_image_new.at<uchar>(Point(x, y));
                if (pixelRight >= 150) {   //tentative value, might need adjustment: lower it closer to 100
                    right.x = x;
                    break;
                }
            }

            if (y == m_control_scanline) {

                if (useRightLaneMarking) {
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


            //prints the lines for debugging purposes if debug flat is set to true
            if (m_debug) {

                if (left.x > 0) {
                    line(m_image, Point(m_image.cols / 2, y), left, Scalar(0, 255, 0));

                    stringstream sstr;
                    sstr << (m_image.cols / 2 - left.x);

                    putText(m_image, sstr.str().c_str(), Point(m_image.cols / 2 - 100, y - 2), FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 255, 0));
                }
                if (right.x > 0) {
                    line(m_image, cvPoint(m_image.cols / 2, y), right, Scalar( 0, 0, 255));

                    stringstream sstr;
                    sstr << (right.x - m_image.cols / 2);
                    putText(m_image, sstr.str().c_str(), cvPoint(m_image.cols / 2 + 100, y - 2), FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 0, 0));
                }
            }


            // stopline logic

            uchar pixelFrontLeft, pixelFrontRight;
            Point stop_left, stop_right;

            int left_dist = 0;

            stop_left.x = (m_image_new.cols/2) - 50;
            stop_left.y = m_control_scanline;

            // Find first grey pixel in the front of the car
            for(int i = m_control_scanline; i > m_stop_scanline; i--) {
                pixelFrontLeft = m_image_new.at<uchar>(Point(stop_left.x, i));
                if(pixelFrontLeft > 150) {
                    stop_left.y = i;
                    left_dist = m_control_scanline - stop_left.y;
                    break;
                }
            }

            int right_dist = 0;

            stop_right.x = (m_image_new.cols/2) + 50;
            stop_right.y = m_control_scanline;

            // Find first grey pixel in front of the car
            for(int i = m_control_scanline; i > m_stop_scanline; i--) {
                pixelFrontRight = m_image_new.at<uchar>(Point(stop_right.x, i));
                if(pixelFrontRight > 150) {
                    stop_right.y = i;
                    right_dist = m_control_scanline - stop_right.y;
                    break;
                }
            }

            // Draw lines if debug true
            if(m_debug) {
                if(stop_left.y < m_control_scanline) {
                    line(m_image, Point(stop_left.x, m_control_scanline), stop_left, Scalar(128, 0, 0));
                }

                if(stop_right.y < m_control_scanline) {
                    line(m_image, Point(stop_right.x, m_control_scanline), stop_right, Scalar(128, 0, 0));
                }
            }


            static int counter = 0;


            if(counter < 5 && (left_dist - right_dist) > -10 && (left_dist - right_dist) < 10 && left_dist != 0 && right_dist != 0) {  // is the detected stopline at a similar distance on both sides
                counter ++;
            } else {
                counter = 0;
            }

            if(counter > 4) {
                stop = true;
            }
            else{
                stop = false;
            }
            return e;
        }



        void  LaneFollower::laneFollower(double e){

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

                if (desiredSteering > 25.0) {
                    // desiredSteering = 25.0;
                }
                if (desiredSteering < -25.0) {
                    // desiredSteering = -25.0;
                }
            }

            // Show resulting features.
            if (m_debug) {
                if (m_image.data != NULL) {
                    imshow("WindowShowImage", m_image);
                    waitKey(10);
                }
            }

            m_vehicleControl.setSteeringWheelAngle(desiredSteering);


        }


        // This method will do the main data processing job.
        // Therefore, it tries to open the real camera first. If that fails, the virtual camera images from camgen are used.
        ModuleExitCodeMessage::ModuleExitCode LaneFollower::body() {    // this method still needs
            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();
            m_debug = kv.getValue<int32_t>("lanefollower.debug") == 1;


            // Overall state machine handler.
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == ModuleStateMessage::RUNNING) {
                bool has_next_frame = false;

                // Get the most recent available container for a SharedImage.
                Container imageC = getKeyValueDataStore().get(SharedImage::ID());


                if (imageC.getDataType() == SharedImage::ID()) {
                    has_next_frame = readSharedImage(imageC);
                }

                // If we have an image from the previous call, it is then processed
                if (has_next_frame) {
                    processImage();
                    double error = errorCalculation();
                    laneFollower(error);
                }
//                if (!Sim){
//                    Container processedC(m_sharedProcessedImage);
//                    getConference().send(processedC);
//                }

                // State control for intersection stop
                if (state == "moving"){
                    m_vehicleControl.setSpeed(1);

                    if (stop){
                        if (stopCounter < 6.0){
                            stopCounter+= 0.5;
                            cerr << stopCounter << endl;
                        }else{
                            state = "stop";
                            cerr << "Stopline found ! " << endl;
                        }
                    }

                }
                if (state == "stop"){
                    m_vehicleControl.setSpeed(0);
                    m_vehicleControl.setSteeringWheelAngle(0);
                    stopCounter+= 0.5;

                    if (stopCounter > 29.9999){
                        state = "resume";
                        m_vehicleControl.setSpeed(1);
                        cerr << "Resuming!" << endl;
                    }
                }
                if (state == "resume"){
                    if (stopCounter < 50){
                        stopCounter+= 0.5;
                        cerr << "counter " << stopCounter << endl;
                    }else{
                        stopCounter = 0;
                        state = "moving";
                        cerr << "Moving!" << endl;
                    }
                }

                // Create container for finally sending the set values for the control algorithm.
                Container c2(m_vehicleControl);
                // Send container.
                getConference().send(c2);
            }

            return ModuleExitCodeMessage::OKAY;
        }

    }
} // scaledcars::control
