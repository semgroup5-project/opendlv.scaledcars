/**
 * boxparker - Sample application for realizing a box parking car.
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

#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "BoxParker.h"

namespace scaledcars {
    namespace control
    {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace automotive;

        BoxParker::BoxParker(const int32_t &argc, char **argv) :
            DataTriggeredConferenceClientModule(argc, argv, "BoxParker"),
            m_foundGaps() {}

        BoxParker::~BoxParker() {}

        void BoxParker::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void BoxParker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        vector<double> BoxParker::getFoundGaps() const {
            return m_foundGaps;
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode BoxParker::body() {
            double distanceOld = 0;
            double absPathStart = 0;
            double absPathEnd = 0;
            const double parkingSpace;//ParkingSpace
            int stageMeasuring = 0;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data describing virtual sensor data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();
                //IR sensor
                double FRONT_SENSOR = sbd.getValueForKey_MapOfDistances(3);
                double RIGHT_SENSOR = sbd.getValueForKey_MapOfDistances(0);
                double DISTANCE_CAR = vd.getAbsTraveledPath();
                //double parkingSpace = absPathStart - absPathEnd;
                //Status
                cout << "Front Sensor = " << FRONT_SENSOR << endl;
                cout << "Right Front Sensor = " << RIGHT_SENSOR << endl;
                cout << "distance = " << DISTANCE_CAR << endl;

                //Create vehicle control data.
                VehicleControl vc;

                // Moving state machine.
                if (parkingSpace < 50) {
                    // Go forward.
                    // Change here to adapt lanefollower
                    cout << "moving forward... " << endl;
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(0);
                }
                if (parkingSpace > 50) {
                    // Move slightly forward.
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(0);
                    stageMoving++;
                }
                if ((stageMoving >= 20) && (stageMoving < 25)) {
                    // Stop.
                    vc.setSpeed(0);
                    vc.setSteeringWheelAngle(0);
                    stageMoving++;
                }
                if ((stageMoving >= 25) && (stageMoving < 80)) {
                    // Backwards, steering wheel to the right.
                    vc.setSpeed(-2);
                    vc.setSteeringWheelAngle(25);
                    stageMoving++;
                }
                if (stageMoving >= 80) {
                    // Stop.
                    vc.setSpeed(0);
                    vc.setSteeringWheelAngle(0);

                    stageMoving++;
                }
                if (stageMoving >= 150) {
                    // End component.
                    break;
                }

                // Measuring state machine.
                switch (stageMeasuring) {
                    case 0:
                        {
                            // Initialize measurement.
                            distanceOld = sbd.getValueForKey_MapOfDistances(2);
                            stageMeasuring++;
                        }
                    break;
                    case 1:
                        {
                            // Checking for distance sequence +, -.
                            if ((sbd.getValueForKey_MapOfDistances(0) < 300)) {
                                // Found distance sequence +, -.
                                cout << "Stage 1 the distance = " << endl;
                                stageMeasuring = 2;
                                absPathStart = vd.getAbsTraveledPath();
                            }
                            distanceOld = sbd.getValueForKey_MapOfDistances(2);
                        }
                    break;
                    case 2:
                        {
                            // Checking for distance sequence -, +.
                            if ((sbd.getValueForKey_MapOfDistances(2) > 200)) {
                                // Found distance sequence -, +.
                                stageMeasuring = 1;
                                absPathEnd = vd.getAbsTraveledPath();

                                const double parkingSpace = (absPathEnd - absPathStart);
                                cerr << "parkingSpace = " << parkingSpace<< endl;

                                m_foundGaparkingSpace.push_back(parkingSpace);

                                if ((parkingSpace > 50)) {
                                    stageMoving = 1;
                                    vc.setSpeet(-1);
                                    absPathStart = vd.getAbsTravledPath();
                                    m_foundGaparkingSpace.push_back(parkingSpace);
                                }
                            }
                            distanceOld = sbd.getValueForKey_MapOfDistances(2);
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
} // automotive

