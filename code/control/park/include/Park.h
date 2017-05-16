/**
 * Parallelparker - Sample application for realizing a Parallel parking car.
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

#ifndef PARK_H_
#define PARK_H_

#include <cstdio>
#include <cmath>
#include <iostream>
#include <math.h>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"
#include "odvdscaledcarsdatamodel/generated/group5/CommunicationLinkMSG.h"
#include "odvdscaledcarsdatamodel/generated/group5/ParkerMSG.h"
#include <automotivedata/GeneratedHeaders_AutomotiveData.h>
#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>

#define IR 0
#define US 1
#define GAP 13
#define RIGHT_TURN 1
#define LEFT_TURN 2
#define INGAP_RIGHT_TURN 3
#define END 4

//MS
//#define WAITFORWHEELANGLECHANGE 100
//MS

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;
        using namespace group5;


        /**
         * This class is a skeleton to send driving commands to Hesperia-light's vehicle driving dynamics simulation.
         */
        class Park : public odcore::base::module::TimeTriggeredConferenceClientModule {
        private:
            /**
             * "Forbidden" copy constructor. Goal: The compiler should warn
             * already at compile time for unwanted bugs caused by any misuse
             * of the copy constructor.
             *
             * @param obj Reference to an object of this class.
             */
            Park(const Park &/*obj*/);

            /**
             * "Forbidden" assignment operator. Goal: The compiler should warn
             * already at compile time for unwanted bugs caused by any misuse
             * of the assignment operator.
             *
             * @param obj Reference to an object of this class.
             * @return Reference to this instance.
             */
            Park &operator=(const Park &/*obj*/);

        public:
            /**
             * Constructor.
             *
             * @param argc Number of command line arguments.
             * @param argv Command line arguments.
             */
            Park(const int32_t &argc, char **argv);

            virtual ~Park();

            odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

            // Methods
        private:
            virtual void setUp();

            virtual void tearDown();

            void parkingFinder();

            void parallelPark();

            void setParkingState(int state);

            void setParkingType(int type);

            double adjDistCalculation(double start);

            bool obstacleDetection(int i, int id);

            void park();

            void unpark();

            void sendParkerMSG();

            CommunicationLinkMSG communicationLinkMSG;

            VehicleControl vc;

            double parkingSpace;

            bool IRRObstacle;

            bool USFObstacle;

            bool IRFRObstacle;

            bool IRRRObstacle;

            double odometer;

            double usFront;

            double usFrontRight;

            double irFrontRight;

            double irRear;

            double irRearRight;

            int parkingState;

            int parkingType;

            int parkingCounter;

            int parkingStart;

            double backDist;

            double backStart;

            double backEnd;

            double adjDist;

            bool isParking;


        };

    }
} // scaledcars::control

#endif /*SIDEWAYSPARKER_H_*/
