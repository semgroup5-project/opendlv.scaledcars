/**
 * Example - Example code.
 * Copyright (C) 2016 Christian Berger
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

#include <stdint.h>

#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>

#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>
#include <automotivedata/GeneratedHeaders_AutomotiveData.h>

#include "odvdscaledcarsdatamodel/generated/chalmersrevere/scaledcars/ExampleMessage.h"

#include "Example.h"

namespace scaledcars {
namespace perception {

using namespace std;
using namespace odcore::base;
using namespace odcore::data;
using namespace odcore::data::image;
        using namespace automotive;
        using namespace automotive::miniature;
        using namespace odcore::wrapper;

Example::Example(const int &argc, char **argv) : DataTriggeredConferenceClientModule(argc, argv, "scaledcars-perception-example"),
    m_hasAttachedToSharedImageMemory(false),
    m_sharedImageMemory(),
    m_image(NULL),
    m_debug(false),
    m_font(),
    m_previousTime(),
    m_eSum(0),
    m_eOld(0){}

Example::~Example() {}

void Example::setUp() {
    cvNamedWindow("Camera Feed Image", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("Camera Feed Image", 300, 100);
}

void Example::tearDown() {
    if (m_image != NULL) {
        cvReleaseImage(&m_image);
    }

    cvDestroyWindow("Camera Feed Image");
}

void Example::nextContainer(odcore::data::Container &c) {

   
    if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
        SharedImage si = c.getData<SharedImage> ();
        cerr << "here" << endl;
        // Check if we have already attached to the shared memory containing the image from the virtual camera.
        if (!m_hasAttachedToSharedImageMemory) {
            m_sharedImageMemory = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(si.getName());
        }

        // Check if we could successfully attach to the shared memory.
        if (m_sharedImageMemory->isValid()) {
            cerr << "IS valid" << endl;
            // Lock the memory region to gain exclusive access using a scoped lock.
            Lock l(m_sharedImageMemory);

            if (m_image == NULL) {
                m_image = cvCreateImage(cvSize(si.getWidth(), si.getHeight()), IPL_DEPTH_8U, si.getBytesPerPixel());
            }

            // Example: Simply copy the image into our process space.
            if (m_image != NULL) {
            cerr << "image not null" << endl;
                memcpy(m_image->imageData, m_sharedImageMemory->getSharedMemory(), si.getWidth() * si.getHeight() * si.getBytesPerPixel());
            }

            // Mirror the image.
            cvFlip(m_image, 0, -1);


        }
         drive();
        // Do some image processing.
        processImage();

    }
}

void Example::processImage() {

        static bool useRightLaneMarking = true;
    //    double e = 0;

        const int32_t CONTROL_SCANLINE = 462; // calibrated length to right: 280px
   //     const int32_t distance = 280;

        TimeStamp beforeImageProcessing;
        for(int32_t y = m_image->height - 8; y > m_image->height * .6; y -= 10) {
            // Search from middle to the left:
            CvScalar pixelLeft;
            CvPoint left;
            left.y = y;
            left.x = -1;
            for(int x = m_image->width/2; x > 0; x--) {
                pixelLeft = cvGet2D(m_image, y, x);
                if (pixelLeft.val[0] >= 200) {
                    left.x = x;
                    break;
                }
            }

            // Search from middle to the right:
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

            if (m_debug) {
                if (left.x > 0) {
                    CvScalar green = CV_RGB(0, 255, 0);
                    cvLine(m_image, cvPoint(m_image->width/2, y), left, green, 1, 8);

                    stringstream sstr;
                    sstr << (m_image->width/2 - left.x);
                    cvPutText(m_image, sstr.str().c_str(), cvPoint(m_image->width/2 - 100, y - 2), &m_font, green);
                }
                if (right.x > 0) {
                    CvScalar red = CV_RGB(255, 0, 0);
                    cvLine(m_image, cvPoint(m_image->width/2, y), right, red, 1, 8);

                    stringstream sstr;
                    sstr << (right.x - m_image->width/2);
                    cvPutText(m_image, sstr.str().c_str(), cvPoint(m_image->width/2 + 100, y - 2), &m_font, red);
                }
            }

            if (y == CONTROL_SCANLINE) {
                // Calculate the deviation error.
                if (right.x > 0) {
                    if (!useRightLaneMarking) {
                        m_eSum = 0;
                        m_eOld = 0;
                    }

            //        e = ((right.x - m_image->width/2.0) - distance)/distance;

                    useRightLaneMarking = true;
                }
                else if (left.x > 0) {
                    if (useRightLaneMarking) {
                        m_eSum = 0;
                        m_eOld = 0;
                    }

            //        e = (distance - (m_image->width/2.0 - left.x))/distance;

                    useRightLaneMarking = false;
                }
                else {
                    // If no measurements are available, reset PID controller.
                    m_eSum = 0;
                    m_eOld = 0;
                }
            }
        }
    if (m_image != NULL) {

    cerr << "Image was processed" << endl;
    cvShowImage("Camera Feed Image", m_image);
    cvWaitKey(10);
    }
}

    void Example::drive(){
        VehicleControl vc;
        vc.setSpeed(1);
        vc.setSteeringWheelAngle(0);
        Container c(vc);
        getConference().send(c);

    }

}
} // scaledcars::perception
