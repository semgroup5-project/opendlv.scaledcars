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
#include <opencv/highgui.h>
#include <iostream>
#include <memory>
#include <opencv/cv.h>

#include <automotivedata/GeneratedHeaders_AutomotiveData.h>
#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>
#include <opendavinci/odcore/base/Lock.h>
#include "odvdscaledcarsdatamodel/generated/chalmersrevere/scaledcars/ExampleMessage.h"
#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include "Example.h"




namespace scaledcars {
namespace control {

using namespace std;
using namespace odcore::base;
using namespace odcore::data;
    using namespace odcore::data::image;
    using namespace automotive;
    using namespace automotive::miniature;
    using namespace odcore::wrapper;

Example::Example(const int &argc, char **argv)
    : TimeTriggeredConferenceClientModule(argc, argv, "scaledcars-control-example"),
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
//attempt to get a sharedImage and process it
void Example::nextContainer(odcore::data::Container &c) {

   // cout << "Received Container of type = " << c.getDataType() << endl;
    if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
        SharedImage si = c.getData<SharedImage>();

        // Check if we have already attached to the shared memory containing the image from the virtual camera.
        if (!m_hasAttachedToSharedImageMemory) {
            m_sharedImageMemory = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(si.getName());
        }

        // Check if we could successfully attach to the shared memory.
        if (m_sharedImageMemory->isValid()) {
            ;
            // Lock the memory region to gain exclusive access using a scoped lock.
            Lock l(m_sharedImageMemory);

            if (m_image == NULL) {
                m_image = cvCreateImage(cvSize(si.getWidth(), si.getHeight()), IPL_DEPTH_8U, si.getBytesPerPixel());
            }

            // Example: Simply copy the image into our process space.
            if (m_image != NULL) {

                memcpy(m_image->imageData, m_sharedImageMemory->getSharedMemory(),
                       si.getWidth() * si.getHeight() * si.getBytesPerPixel());
            }

            // Mirror the image.
            //cvFlip(m_image, 0, -1);
            if (m_image != NULL) {

                cerr << "Image was processed" << endl;
                cvShowImage("Camera Feed Image", m_image);
                cvWaitKey(10);
            }
        }
    }
}
    //body with vehicle control
odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Example::body() {

    while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
        cerr << "driving" << endl;
        // Example how to send commands to the vehicle.
        automotive::VehicleControl vc;
        vc.setSpeed(-2);
        vc.setSteeringWheelAngle(25);
        Container ac(vc);
        getConference().send(ac);

    }
    return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}
}
} // scaledcars::control

