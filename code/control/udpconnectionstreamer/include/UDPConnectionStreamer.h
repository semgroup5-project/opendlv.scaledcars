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

#ifndef UDPCONNECTIONSTREAMER_H
#define UDPCONNECTIONSTREAMER_H

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/base/Lock.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>

#include <opendavinci/odcore/io/udp/UDPSender.h>
#include <opendavinci/odcore/io/udp/UDPFactory.h>

#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>

#include <opendavinci/odcore/wrapper/jpg/JPG.h>
#include "opendavinci/odcore/data/image/CompressedImage.h"
#include "opendavinci/generated/odcore/data/image/SharedImage.h"

#include <iostream>
#include <memory>
#include <stdint.h>
#include <string>
#include <opendavinci/odcore/base/Thread.h>

#include <opendavinci/odcore/base/KeyValueConfiguration.h>

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore;
        using namespace odcore::io;
        using namespace odcore::io::udp;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace odcore::wrapper;

        class UDPConnectionStreamer : public odcore::base::module::DataTriggeredConferenceClientModule {

        private:
            /**
             * "Forbidden" copy constructor.
             *
             * Goal: The compiler should warn already at compile time
             * for unwanted bugs caused by any misuse of the copy
             * constructor.
             *
             * @param obj Reference to an object of this class.
             */
            UDPConnectionStreamer(const UDPConnectionStreamer &/*obj*/);

            /**
             * "Forbidden" assignment operator.
             *
             * Goal: The compiler should warn already at compile time
             * for unwanted bugs caused by any misuse of the assignment
             * operator.
             *
             * @param obj Reference to an object of this class.
             * @return Reference to this instance.
             */
            UDPConnectionStreamer &operator=(const UDPConnectionStreamer &/*obj*/);

        public:
            /**
             * Constructor.
             *
             * @param argc Number of command line arguments.
             * @param argv Command line arguments.
             */
            UDPConnectionStreamer(const int &argc, char **argv);

            virtual ~UDPConnectionStreamer();

        protected:
            /**
             * This method is called to process an incoming container.
             *
             * @param c Container to process.
             * @return true if c was successfully processed.
             */
            bool readSharedImage(odcore::data::Container &c);

        private:
            void setUp();

            void tearDown();

            virtual void nextContainer(odcore::data::Container &c);

            void processImage();

            double Median(cv::Mat mat);

            void stream();

            bool m_hasAttachedToSharedImageMemory;

            cv::Mat m_image;
            cv::Mat m_image_mat;
            cv::Mat m_image_new;

            shared_ptr <odcore::wrapper::SharedMemory> m_sharedImageMemory;
            shared_ptr <odcore::wrapper::SharedMemory> m_sharedProcessedImageMemory;
            odcore::data::image::SharedImage m_sharedProcessedImage;

            int32_t m_threshold1;
            int32_t m_threshold2;

            odcore::data::image::SharedImage si;
        };
    }
} // scaledcars::control

#endif /*UDPCONNECTIONSTREAMER_H*/