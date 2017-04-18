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

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode ParallelParker::body() {
            const double ULTRASONIC_FRONT_RIGHT = 0;
            double distanceOld = 0;
            double absPathStart = 0;
            double absPathEnd = 0;

            int stageMoving = 0;
            int stageMeasuring = 0;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                // Create vehicle control data.
                VehicleControl vc;

                // Moving state machine.
                if (stageMoving == 0) {
                    // Go forward.
                    vc.setSpeed(2);
                    vc.setSteeringWheelAngle(0);
                }
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
                if ((stageMoving >= 130) && (stageMoving <180)) {
                    // Backwards, steering wheel to the left.
                    vc.setSpeed(-2);
                    vc.setSteeringWheelAngle(-25);
                    stageMoving++;
                }
                if (stageMoving >= 180) {
                    // Stop.
                    vc.setSpeed(0);
                    vc.setSteeringWheelAngle(0);
                }

                // Measuring state machine.
                switch (stageMeasuring) {
                    case 0:
                        {
                            // Initialize measurement.
                            distanceOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);
                            stageMeasuring++;
                        }
                    break;
                    case 1:
                        {
                            // Checking for sequence +, -.
                            if ((distanceOld > 0) && (sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT) < 0)) {
                                // Found sequence +, -.
                                stageMeasuring = 2;
                                absPathStart = vd.getAbsTraveledPath();
                            }
                            distanceOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);
                        }
                    break;
                    case 2:
                        {
                            // Checking for sequence -, +.
                            if ((distanceOld < 0) && (sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT) > 0)) {
                                // Found sequence -, +.
                                stageMeasuring = 1;
                                absPathEnd = vd.getAbsTraveledPath();

                                const double GAP_SIZE = (absPathEnd - absPathStart);

                                cerr << "Size = " << GAP_SIZE << endl;

                                if ((stageMoving < 1) && (GAP_SIZE > 7)) {
                                    stageMoving = 1;
                                }
                            }
                            distanceOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);
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

