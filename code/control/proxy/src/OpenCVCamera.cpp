/**
 * proxy - Sample application to encapsulate HW/SW interfacing with embedded systems.
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

#include <iostream>

#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"

#include "OpenCVCamera.h"

namespace scaledcars {
    namespace control {

        OpenCVCamera::OpenCVCamera(const string &name, const uint32_t &id, const uint32_t &width,
                                   const uint32_t &height, const uint32_t &bpp) :
                Camera(name, id, width, height, bpp),
                m_capture(NULL),
                m_image(NULL),
                m_sharedImageMemory() {
            int _id = id;
            while (!(m_capture = cvCaptureFromCAM(_id)) && _id < 9) {
                if (m_capture) {
                    cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_WIDTH, width);
                    cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_HEIGHT, height);
                    cvSetCaptureProperty(m_capture, CV_CAP_PROP_FPS, 30);
                } else {
                    cerr << "proxy: Could not open camera '" << name << "' with ID: " << _id << endl;
                    _id++;
                }
            }
        }

        OpenCVCamera::~OpenCVCamera() {
            if (m_capture) {
                cvReleaseCapture(&m_capture);
                m_capture = NULL;
            }
        }

        bool OpenCVCamera::isValid() const {
            return (m_capture != NULL);
        }

        bool OpenCVCamera::captureFrame() {
            bool retVal = false;
            if (m_capture != NULL) {
                if (cvGrabFrame(m_capture)) {
                    if (getBPP() == 1) {
                        IplImage *tmpFrame = cvRetrieveFrame(m_capture);

                        if (m_image == NULL) {
                            m_image = cvCreateImage(cvGetSize(tmpFrame), IPL_DEPTH_8U, 1);
                        }

                        cvCvtColor(tmpFrame, m_image, CV_BGR2GRAY);
                    } else {
                        m_image = cvRetrieveFrame(m_capture);
                    }

                    retVal = true;
                }
            }
            return retVal;
        }

        bool OpenCVCamera::copyImageTo(char *dest, const uint32_t &size) {
            bool retVal = false;

            if ((dest != NULL) && (size > 0) && (m_image != NULL)) {
                memcpy(dest, m_image->imageData, size);

//                cvShowImage("WindowShowImage", m_image);
//                cvWaitKey(10);

                retVal = true;
            }

            return retVal;
        }

    }   // control
} // scaledcars
