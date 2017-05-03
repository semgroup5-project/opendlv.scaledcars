/**
 * parallelparker - Sample application for realizing a Parallel parking car.
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
#include <math.h>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "parallelparker.h"

#define PI 3.14159265359
namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;

        parallelparker::parallelparker(const int32_t &argc, char **argv) :
                TimeTriggeredConferenceClientModule(argc, argv, "parallelparker"),
                sim(false) {}

        parallelparker::~parallelparker() {}

        void parallelparker::setUp() {
            // This method will be call automatically _before_ running body().
            // sim = getKeyValueConfiguration().getValue<int32_t>("parallelparking.simu");
            KeyValueConfiguration kv = getKeyValueConfiguration();
            sim = (kv.getValue<int32_t>("parallelparker.sim") == 0);
        }

        void parallelparker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        bool parallelparker::obstacleDetect(int i) {
            bool ifObstacle;
            if (i < 0) {
                ifObstacle = false;
            } else if (i >= 0) {
                ifObstacle = true;
            }
            return ifObstacle;
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode parallelparker::body() {

            bool IFFRObstacle;
            //bool IFRRObstacle;
            const double INFRARED_FRONT_RIGHT = 0;
            const double INFRARED_REAR = 1;
            const double INFRARED_REAR_RIGHT = 2;
            const double ULTRASONIC_FRONT = 3;

            double GAP_SIZE = 0;
            double distance = 0;
            double absPathStart = 0;
            double absPathEnd = 0;
            double backDist = 0;
            double parkStart = 0;
            double parkEnd = 0;
            double parkingGap = 0;
            double counter = 1;
            double i = 0;

            //double irFrontRight = 0;
            double usFront = 0;
            double irRear = 0;
            double irRearRight = 0;
            int parkingSit = 0; //0 -> Two boxes, 1 -> Enough space from beginning,
            int gap = 0;
            int stageMeasuring = 0;
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
                //irFrontRight = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);

                if (sim) {
                    irRear = sbd.getValueForKey_MapOfDistances(INFRARED_REAR);
                    irRearRight = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
                    usFront = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT);
                    IFFRObstacle = obstacleDetect(sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT));
                    if (stageMoving == 0) {
                        // Go forward.
                        vc.setSpeed(2);
                        vc.setSteeringWheelAngle(0);
                        parkStart = vd.getAbsTraveledPath();
                    }
                    if (stageMoving == 1) {
                        vc.setSpeed(0);
                        vc.setSteeringWheelAngle(0);
                        if (irRearRight > 0 && parkingSit == 1) {
                            stageMoving = 2;
                        }
                        if (irRearRight < 0 && parkingSit == 2) {
                            stageMoving = 3;
                            cerr << "parkingSit ==" << parkingSit << " moving to next stage" << endl;
                        }
                    }
                    if (stageMoving == 2) {
                        vc.setSpeed(1);
                        if (irRear < 0) {
                            stageMoving = 3;
                            cerr << "irRear smaller than 0 Stage Moving = 3" << endl;
                        }
                    }
                    if (stageMoving == 3 && (irRear < 0)) {
                        //Parking
                        parkEnd = vd.getAbsTraveledPath();
                        backDist = parkEnd - parkStart;
                        double cosVal = cos(30 * PI / 180.0);
                        double adjDist = cosVal * backDist;
                        if (gap == 1) {
                            if (irRear < 0 && adjDist < parkingGap / 2) {
                                vc.setSpeed(-1);
                                vc.setSteeringWheelAngle(2);
                                counter++;
                            }
                            if (irRear < 0 && adjDist > parkingGap / 2 && counter > 1) {
                                vc.setSpeed(-1);
                                vc.setSteeringWheelAngle(-2);
                                counter -= 1.6;
                            }
                        }
                        if (gap == 2) {
                            if (irRear < 0 && adjDist < parkingGap / 3) {
                                vc.setSpeed(-1);
                                vc.setSteeringWheelAngle(.3);
                                counter++;
                            }
                            if (irRear < 0 && adjDist > parkingGap / 3 && counter > 1) {
                                vc.setSpeed(-1);
                                vc.setSteeringWheelAngle(-.3);
                                counter -= 1.6;
                            }
                        }
                    }
                    if (counter < 1) {
                        i = vd.getAbsTraveledPath();
                        stageMoving = 4;
                    }
                    if (stageMoving == 4) {
                        cerr << "Move one last bit" << endl;
                        double j = vd.getAbsTraveledPath();
                        vc.setSpeed(1);
                        vc.setSteeringWheelAngle(.2);
                        if (parkingSit == 2 && j - i < 2) {
                            cerr << "stage4 #1 if" << endl;
                            stageMoving = 5;
                        }
                        if (usFront < 7 && usFront > 0) {
                            stageMoving = 5;
                            cerr << "stage4 #2 if" << endl;
                        }
                    }
                    if (stageMoving == 5) {
                        cerr << "car stopped!!" << endl;
                        vc.setSpeed(0);
                        vc.setSteeringWheelAngle(0);
                    }
                    if (irRear > 0 && irRear < 4) {
                        //Emergency stop
                        vc.setSpeed(0);
                        cerr << "emergency stop!" << endl;
                    }
                } else if (!sim) {
                    cerr << "This is for the real car!" << endl;
                }
                if (stageMoving == 0) {
                    switch (stageMeasuring) {
                        case 0: {
                            distance = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                            double parking = vd.getAbsTraveledPath();
                            absPathStart = vd.getAbsTraveledPath();
                            //when there is no car parked for 7 units
                            if (!IFFRObstacle && parking > 15) {
                                GAP_SIZE = parking;
                                parkingGap = parking;
                                gap = 1;
                                stageMoving = 1;
                                parkingSit = 2;
                                stageMeasuring++;
                            }
                            //when there is something detected within 7 units
                            if (parking < 7 && IFFRObstacle) {
                                stageMeasuring++;
                                parkingSit = 1;
                            }
                        }
                            break;
                        case 1: {
                            // Checking for sequence +, -.
                            if (parkingSit == 1) {
                                if ((distance > 0) && (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 0)) {
                                    // Found sequence +, -.
                                    stageMeasuring = 2;
                                    absPathStart = vd.getAbsTraveledPath();
                                }
                                distance = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                            }
                        }
                            break;
                        case 2: {
                            // Checking for sequence -, +.
                            if (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0) {
                                absPathEnd = vd.getAbsTraveledPath();
                                GAP_SIZE = (absPathEnd - absPathStart);
                                cerr << "Size = " << GAP_SIZE << endl;
                                if (GAP_SIZE <= 10) {
                                    gap = 1;
                                } else if (GAP_SIZE > 10) {
                                    gap = 2;
                                }
                                stageMeasuring = 1;
                                if ((stageMoving < 1) &&
                                    (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0) && (GAP_SIZE > 7)) {
                                    stageMeasuring = 1;
                                    parkingGap = GAP_SIZE;
                                    parkingSit = 1;
                                    stageMoving = 1;
                                    cerr << "Size = " << GAP_SIZE << endl;
                                }
                                //Extra, when there is only one box on the track
//                                if ((stageMoving < 1) &&
//                                    (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 0) && (GAP_SIZE > 7)) {
//                                    cerr << "case 2 if 2" << endl;
//                                    stageMeasuring = 1;
//                                    parkingGap = GAP_SIZE;
//                                    cerr << "set stage moving to 1 gapSize" << endl;
//                                    stageMoving = 1;
//                                    cerr << "Size = " << GAP_SIZE << endl;
//                                }
                            }
                            distance = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                        }
                            break;
                    }
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


