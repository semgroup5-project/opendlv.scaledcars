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

#include <cstdio>
#include <cmath>
#include <iostream>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "ParallelParker.h"

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;

        ParallelParker::ParallelParker(const int32_t &argc, char **argv) :
                TimeTriggeredConferenceClientModule(argc, argv, "ParallelParker") {
        }

        ParallelParker::~ParallelParker() {}

        void ParallelParker::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void ParallelParker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        bool ParallelParker::obstacleDetect(int i) {
            bool ifObstacle;
            if (i < 0) {
                ifObstacle = false;
            } else if (i >= 0) {
                ifObstacle = true;
            }
            return ifObstacle;
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode ParallelParker::body() {

            bool IFFRObstacle;
            //bool IFRRObstacle;
            const double INFRARED_FRONT_RIGHT = 0;
            const double INFRARED_REAR = 1;
            // const double INFRARED_REAR_RIGHT = 2;
            double GAP_SIZE = 0;
            double distance = 0;
            double absPathStart = 0;
            double absPathEnd = 0;

//            double gapStart = 0;
//            double gapEnd = 0;
//            double gapSize = 0;

            int stageMeasuring = 0;


            // double irFrontRight = 0;
            double irRear = 0;
            // double irRearRight = 0;


            int stageMoving = 0;


            while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
                   odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData>();

                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(
                        automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData>();

                // Create vehicle control data.
                VehicleControl vc;

                if (GAP_SIZE < 7 && stageMoving != 1) {
                    // Go forward.
                    vc.setSpeed(2);
                    vc.setSteeringWheelAngle(0);
                }
                if (GAP_SIZE > 7 && stageMoving == 1) {
                    cerr << "irRear: " << irRear << endl;
                    cerr << "StageMoving " << stageMoving << endl;
//                    if ((irRear > 0) && (irRear < 1)) {
//                        // Stop.
//                        cerr << "yayy" << endl;
//                        vc.setSpeed(0);
//                        vc.setSteeringWheelAngle(0);
//                    }

                    if ((stageMoving > 0) && (stageMoving < 40)) {
                        // Move slightly forward.
                        vc.setSpeed(1);
                        vc.setSteeringWheelAngle(0);
                        stageMoving++;
                    }
                    if ((stageMoving >= 40) && (stageMoving < 50)) {
                        // Stop.
                        vc.setSpeed(0);
                        vc.setSteeringWheelAngle(0);
                        stageMoving++;
                    }
                    if ((stageMoving >= 50) && (stageMoving < 130)) {
                        // Backwards, steering wheel to the right.
                        vc.setSpeed(-2);
                        vc.setSteeringWheelAngle(25);
                        stageMoving++;
                    }
                    if ((stageMoving >= 130)) {
                        // Backwards, steering wheel to the left.
                        vc.setSpeed(-2);
                        vc.setSteeringWheelAngle(-25);
                        stageMoving++;
                    }


                }
                // Measuring state machine.
                switch (stageMeasuring) {
                    case 0: {
                        // Initialize measurement.
                        distance = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                        if ((IFFRObstacle)) {
                            stageMeasuring++;
                        }
                    }
                        break;
                    case 1: {

                        // Checking for sequence +, -.
                        if ((distance > 0) && (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 0)) {
                            // Found sequence +, -.
                            stageMeasuring = 2;
                            absPathStart = vd.getAbsTraveledPath();
                        }
                        distance = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                    }
                        break;
                    case 2: {
                        // Checking for sequence -, +.
                       double PathEnd = vd.getAbsTraveledPath();
                       double gapSize = (PathEnd - absPathStart);

                        if ((distance < 0) && (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0)) {
                            // Found sequence -, +.
                            stageMeasuring = 1;
                            absPathEnd = vd.getAbsTraveledPath();

                            GAP_SIZE = (absPathEnd - absPathStart);

                            cerr << "Size = " << GAP_SIZE << endl;

                            if ((stageMoving < 1) && (GAP_SIZE > 7)) {
                                cerr << "set stage moving to 1" << endl;
                                stageMoving = 1;
                            }
                        }
                        if ((distance < 0) && (gapSize > 9)) {

                            stageMoving = 1;
                        }
                        distance = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                    }
                        break;
                }

                // Create container for finally sending the data.
                Container c(vc);
                // Send container.
                getConference().send(c);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    }
} // automotive::miniature

