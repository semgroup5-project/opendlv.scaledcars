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

#ifndef LANEFOLLOWER_H_
#define LANEFOLLOWER_H_

#include <iostream>
#include <stdint.h>
#include <memory>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opendavinci/odcore/base/KeyValueConfiguration.h>
#include <opendavinci/odcore/base/Lock.h>

#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include <opendavinci/odcore/data/Container.h>

#include <opendavinci/odcore/data/TimeStamp.h>

#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>

#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>
#include <automotivedata/GeneratedHeaders_AutomotiveData.h>


namespace scaledcars {
namespace control {

        using namespace std;

        /**
         * This class is an exemplary skeleton for processing video data.
         */
        class LaneFollower: public odcore::base::module::TimeTriggeredConferenceClientModule {
            private:
	            /**
	             * "Forbidden" copy constructor. Goal: The compiler should warn
	             * already at compile time for unwanted bugs caused by any misuse
	             * of the copy constructor.
	             *
	             * @param obj Reference to an object of this class.
	             */
	            LaneFollower(const LaneFollower &/*obj*/);

	            /**
	             * "Forbidden" assignment operator. Goal: The compiler should warn
	             * already at compile time for unwanted bugs caused by any misuse
	             * of the assignment operator.
	             *
	             * @param obj Reference to an object of this class.
	             * @return Reference to this instance.
	             */
	            LaneFollower& operator=(const LaneFollower &/*obj*/);

            public:
	            /**
	             * Constructor.
	             *
	             * @param argc Number of command line arguments.
	             * @param argv Command line arguments.
	             */
	            LaneFollower(const int32_t &argc, char **argv);

	            virtual ~LaneFollower();

	            odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

            protected:
	            /**
	             * This method is called to process an incoming container.
	             *
	             * @param c Container to process.
	             * @return true if c was successfully processed.
	             */
	            bool readSharedImage(odcore::data::Container &c);

            private:

	            shared_ptr<odcore::wrapper::SharedMemory> m_sharedImageMemory;
				shared_ptr<odcore::wrapper::SharedMemory> m_sharedProcessedImageMemory;
			    odcore::data::image::SharedImage m_sharedProcessedImage;

                bool m_hasAttachedToSharedImageMemory;
                bool m_debug;
                bool Sim;

                cv::Mat m_image;
                odcore::data::TimeStamp m_previousTime;

                double m_eSum;
                double m_eOld;

                automotive::VehicleControl m_vehicleControl;
                /**
                 *Canny algoithm
                 * http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html
                 *
                 *If a pixel gradient is higher than the upper threshold, the pixel is accepted as an edge
                 *If a pixel gradient value is below the lower threshold, then it is rejected.
                 *If the pixel gradient is between the two thresholds, then it will be accepted only if it is connected to a pixel that is above the upper threshold.
                 *
                 **/
                int32_t m_threshold1;
                int32_t m_threshold2;
                int32_t m_control_scanline;
                int32_t m_stop_scanline;
                int32_t m_distance;


                double p_gain;
                double i_gain;
                double d_gain;

	            virtual void setUp();

	            virtual void tearDown();

                void processImage();
                double errorCalculation();
                void laneFollower(double e);
        };

    } //control
} // scaledcars

#endif /*LANEFOLLOWER_H_*/
