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
            sim = (kv.getValue<int32_t>("global.sim") == 1);

        }

        void parallelparker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        bool parallelparker::obstacleDetect(int i) {
            bool ifObstacle;
            if (sim) {
                if (i < 0) {
                    ifObstacle = false;
                } else if (i >= 0) {
                    ifObstacle = true;
                }
            } else if (!sim) {
                if (i > 30 || i == 0) {
                    ifObstacle = false;
                } else if (i > 0 && i <= 30) {
                    ifObstacle = true;
                }
            }

            return ifObstacle;
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode parallelparker::body() {

            const int INFRARED_FRONT_RIGHT = 0;
            const int INFRARED_REAR = 1;
            const int INFRARED_REAR_RIGHT = 2;
            const int ULTRASONIC_FRONT = 3;
            const int CAR_INFRARED_FRONT_RIGHT = 3;
            const int CAR_INFRARED_REAR = 5;
            const int CAR_INFRARED_REAR_RIGHT = 4;
            const int CAR_ULTRASONIC_FRONT = 1;

            bool IFFRObstacle;
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
            // double travleDist = 0;

            //double irFrontRight = 0;
            double usFront = 0;
            double irRear = 0;
            double irRearRight = 0;
            int parkingSit = 0; //1 -> Two boxes, 0 -> Enough space from beginning,
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
                        if (irRearRight > 0 && parkingSit == 0) {
                            stageMoving = 3;
                        }
                        if (irRearRight < 0 && parkingSit == 1) {
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
                        cerr << "parking gap " << parkingGap << endl;
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
                        if (parkingSit == 1 && j - i < 2) {
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
                    double a = sbd.getValueForKey_MapOfDistances(CAR_INFRARED_FRONT_RIGHT);
                    cerr << "This is the IFFRObs reading!!!" << a << endl;
                    irRear = sbd.getValueForKey_MapOfDistances(CAR_INFRARED_REAR);
                    irRearRight = sbd.getValueForKey_MapOfDistances(CAR_INFRARED_REAR_RIGHT);
                    usFront = sbd.getValueForKey_MapOfDistances(CAR_ULTRASONIC_FRONT);
                    IFFRObstacle = obstacleDetect(sbd.getValueForKey_MapOfDistances(CAR_INFRARED_FRONT_RIGHT));

                    //Car is using degree.
                    if (stageMoving == 0) {
                        // Go forward.
                        vc.setSpeed(100);
                        vc.setSteeringWheelAngle(0);
                        parkStart = vd.getAbsTraveledPath();
                    }
                    if (stageMoving == 1) {

                        //to make the car stop
                        vc.setSpeed(190);
                        vc.setSteeringWheelAngle(0);
                        if (irRearRight > 1 && parkingSit == 0) {
                            stageMoving = 2;
                        }
                        if (irRearRight < 1 && parkingSit == 1) {
                            stageMoving = 3;
                            cerr << "parkingSit ==" << parkingSit << " moving to next stage" << endl;
                        }
                    }
                    if (stageMoving == 2) {
                        vc.setSpeed(100);
                        if (irRear < 20) {
                            stageMoving = 3;
                            cerr << "irRear smaller than 0 Stage Moving = 3" << endl;
                        }
                    }
                    if (stageMoving == 3 && (irRear < 5)) {
                        //Parking
                        parkEnd = vd.getAbsTraveledPath();
                        backDist = parkEnd - parkStart;
                        double cosVal = cos(60 * PI / 180.0);
                        double adjDist = cosVal * backDist;
                        cerr << "GAP: " << gap << endl;
                        if (gap == 1) {
                            if (irRear < 5 && adjDist < parkingGap / 2) {
                                vc.setSpeed(-100);
                                vc.setSteeringWheelAngle(60);
                                counter++;
                            }
                            if (irRear < 5 && adjDist > parkingGap / 2 && counter > 1) {
                                vc.setSpeed(-100);
                                vc.setSteeringWheelAngle(-60);
                                counter -= 1.6;
                            }
                        }
                        if (gap == 2) {
                            if (irRear < 5 && adjDist < parkingGap / 3) {
                                vc.setSpeed(-100);
                                vc.setSteeringWheelAngle(20);
                                counter++;
                            }
                            if (irRear < 5 && adjDist > parkingGap / 3 && counter > 1) {
                                vc.setSpeed(-100);
                                vc.setSteeringWheelAngle(-20);
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
                        vc.setSpeed(100);
                        vc.setSteeringWheelAngle(12);
                        if (parkingSit == 1 && j - i < 2) {
                            cerr << "stage4 #1 if" << endl;
                            stageMoving = 5;
                        }
                        if (usFront < 25 && usFront > 1) {
                            stageMoving = 5;
                            cerr << "stage4 #2 if" << endl;
                        }
                    }
                    if (stageMoving == 5) {
                        cerr << "car stopped!!" << endl;
                        vc.setSpeed(190);
                        vc.setSteeringWheelAngle(0);
                    }
                    if (irRear > 3 && irRear < 15) {
                        //Emergency stop
                        vc.setSpeed(190);
                        cerr << "emergency stop!" << endl;
                    }
                }




                //Measuring parking gap
                if (stageMoving == 0 && sim) {
                    switch (stageMeasuring) {
                        case 0: {

                            distance = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                            double parking = vd.getAbsTraveledPath();
                            absPathStart = vd.getAbsTraveledPath();
                            cerr << "parkingSit detection" << endl;
                            //when there is no car parked for 15 units
                            if (!IFFRObstacle && parking >= 15) {
                                GAP_SIZE = parking;
                                parkingGap = parking;
                                gap = 1;
                                stageMoving = 1;
                                parkingSit = 1;
                                stageMeasuring++;
                                cerr << "parkingSit 15 units" << endl;
                            }
                            //when there is something detected within 15 units
                            if (IFFRObstacle && parking < 15) {
                                parkingSit = 0;
                                stageMeasuring++;
                                cerr << "parkingSit 2 < 15 units" << endl;
                            }
                        }
                            break;
                        case 1: {
                            // Checking for sequence +, -.
                            if (parkingSit == 1) {
                                cerr << "in case 1 parkingSit sim ==1" << endl;
                                if ((distance > 0) && (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 0)) {
                                    // Found sequence +, -.
                                    cerr << "Setting stageMeasuring to 2 " << endl;
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
                                cerr << "Sizeeee = " << GAP_SIZE << endl;
                                if (GAP_SIZE <= 15) {
                                    gap = 1;
                                    cerr << "set GAP = 1 from case 2" << endl;
                                } else if (GAP_SIZE > 15) {
                                    gap = 2;
                                    cerr << "set GAP = 2 from case 2" << endl;

                                }
                                stageMeasuring = 1;
                                if ((stageMoving < 1) &&
                                    (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0) && (GAP_SIZE > 7)) {
                                    stageMeasuring = 1;
                                    parkingGap = GAP_SIZE;
                                    parkingSit = 1;
                                    stageMoving = 1;
                                    cerr << "Sizeeee = " << GAP_SIZE << endl;
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
                } else if (stageMoving == 0 && !sim) {
                    switch (stageMeasuring) {
                        case 0: {
                            cerr << "case 0! " << endl;
                            distance = sbd.getValueForKey_MapOfDistances(CAR_INFRARED_FRONT_RIGHT);
                            double parking = vd.getAbsTraveledPath();
                            absPathStart = vd.getAbsTraveledPath();
                            cerr << "IFFRObstacle :" << IFFRObstacle << endl;
                            if (!IFFRObstacle && parking >= 100) {
                                GAP_SIZE = parking;
                                parkingGap = parking;
                                gap = 1;
                                stageMoving = 1;
                                parkingSit = 1;
                                stageMeasuring = 1;
                                cerr << "parkingSit 1 100 units" << endl;
                            }
                            if (IFFRObstacle && parking < 100) {
                                parkingSit = 0;
                                stageMeasuring = 1;
                                cerr << "parkingSit 2 < 100 units" << endl;
                            }
                        }
                            break;
                        case 1: {
                            cerr << "case 1! " << endl;
                            cerr << "parking sit  " << parkingSit << endl;
                            // Checking for sequence +, -.
                            if (parkingSit == 1) {
                                cerr << "in case 1 parkingSit==1 real car " << endl;
                                if ((distance > 0) &&
                                    (sbd.getValueForKey_MapOfDistances(CAR_INFRARED_FRONT_RIGHT) < 5)) {
                                    // Found sequence +, -.
                                    cerr << "Setting stageMeasuring to 2 " << endl;
                                    stageMeasuring = 2;
                                    absPathStart = vd.getAbsTraveledPath();
                                }
                                distance = sbd.getValueForKey_MapOfDistances(CAR_INFRARED_FRONT_RIGHT);
                            }
                        }
                            break;
                        case 2: {
                            cerr << "case 2! " << endl;
                            // Checking for sequence -, +.
                            if (sbd.getValueForKey_MapOfDistances(CAR_INFRARED_FRONT_RIGHT) > 10) {
                                absPathEnd = vd.getAbsTraveledPath();
                                GAP_SIZE = (absPathEnd - absPathStart);
                                cerr << "Sizeeee = " << GAP_SIZE << endl;
                                if (GAP_SIZE <= 150) {
                                    gap = 1;
                                    cerr << "set GAP = 1 from case 2" << endl;
                                } else if (GAP_SIZE > 150) {
                                    gap = 2;
                                    cerr << "set GAP = 2 from case 2" << endl;

                                }
                                stageMeasuring = 1;
                                if ((stageMoving < 1) &&
                                    (sbd.getValueForKey_MapOfDistances(CAR_INFRARED_FRONT_RIGHT) > 10) &&
                                    (GAP_SIZE > 150)) {
                                    stageMeasuring = 1;
                                    parkingGap = GAP_SIZE;
                                    parkingSit = 1;
                                    stageMoving = 1;
                                    cerr << "Sizeeee = " << GAP_SIZE << endl;
                                }
                            }
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


