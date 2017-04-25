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

Example::Example(const int &argc, char **argv)
    : DataTriggeredConferenceClientModule(argc, argv, "scaledcars-perception-example"),
    m_hasAttachedToSharedImageMemory(false),
    m_sharedImageMemory(),
    m_image(NULL) {}

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

        // Check if we have already attached to the shared memory containing the image from the virtual camera.
        if (!m_hasAttachedToSharedImageMemory) {
            m_sharedImageMemory = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(si.getName());
        }

        // Check if we could successfully attach to the shared memory.
        if (m_sharedImageMemory->isValid()) {
            // Lock the memory region to gain exclusive access using a scoped lock.
            Lock l(m_sharedImageMemory);

            if (m_image == NULL) {
                m_image = cvCreateImage(cvSize(si.getWidth(), si.getHeight()), IPL_DEPTH_8U, si.getBytesPerPixel());
            }

            // Example: Simply copy the image into our process space.
            if (m_image != NULL) {
                memcpy(m_image->imageData, m_sharedImageMemory->getSharedMemory(), si.getWidth() * si.getHeight() * si.getBytesPerPixel());
            }

            // Mirror the image.
            cvFlip(m_image, 0, -1);
        }

        // Do some image processing.
        processImage();
    }
}

void Example::processImage() {
    // Example: Show the image.
    if (m_image != NULL) {
        cvShowImage("Camera Feed Image", m_image);
        cvWaitKey(10);
    }
}


}
} // scaledcars::perception
