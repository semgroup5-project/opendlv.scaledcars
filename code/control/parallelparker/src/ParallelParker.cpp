/**
 * ParallelParker - Sample application for realizing a Parallel parking car.
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
#include "ParallelParker.h"

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;
        using namespace group5;

        ParallelParker::ParallelParker(const int32_t &argc, char **argv) :
                TimeTriggeredConferenceClientModule(argc, argv, "ParallelParker"),
                sim(false),
                guardGoodV(0),
                guardBadV(0),
                communicationLinkMSG(),
                vc() {}

        ParallelParker::~ParallelParker() {}

        void ParallelParker::setUp() {
            // This method will be call automatically _before_ running body().
            // sim = getKeyValueConfiguration().getValue<int32_t>("parallelparking.simu");
            KeyValueConfiguration kv = getKeyValueConfiguration();
            sim = (kv.getValue<int32_t>("global.sim") == 1);

        }

        void ParallelParker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        bool ParallelParker::obstacleDetect(int i, int id) {
            // This method will return true/false basedon the sensor data input whether if
            // it detects a obstacle. id 1 -> ultrasonic, 2 -> infrared.
            bool ifObstacle;
            switch (id) {
                case 1 : {
                    if (sim) {
                        if (i < 0) {
                            ifObstacle = false;
                        } else if (i >= 0) {
                            ifObstacle = true;
                        }
                    } else if (!sim) {
                        if (i > 70 || i < 0) {
                            guardBadV++;
                            if (guardBadV > 8) {
                                ifObstacle = false;
                                guardGoodV = 0;
                            }
                        } else if (i <= 70 && i > 0) {
                            guardGoodV++;
                            if (guardGoodV > 8) {
                                ifObstacle = true;
                                guardBadV = 0;
                            }
                        }
                    }
                }
                    break;
                case 2 : {
                    if (sim) {
                        if (i < 0) {
                            ifObstacle = false;
                        } else if (i >= 0) {
                            ifObstacle = true;
                        }
                    } else if (!sim) {
                        if (i > 38 || i < 0) {
                            guardBadV++;
                            if (guardBadV > 8) {
                                ifObstacle = false;
                                guardGoodV = 0;
                            }
                        } else if (i <= 38 && i > 0) {
                            guardGoodV++;
                            if (guardGoodV > 8) {
                                ifObstacle = true;
                                guardBadV = 0;
                            }
                        }
                    }
                }
                    break;
            }
            return ifObstacle;
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode ParallelParker::body() {

            const int INFRARED_FRONT_RIGHT = 0;
            const int INFRARED_REAR = 1;
            const int INFRARED_REAR_RIGHT = 2;
            const int ULTRASONIC_FRONT = 3;

//            const int CAR_INFRARED_FRONT_RIGHT = 3;
//            const int CAR_INFRARED_REAR = 5;
//            const int CAR_INFRARED_REAR_RIGHT = 4;
//            const int CAR_ULTRASONIC_FRONT = 1;

            bool IRFRObstacle;
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
            double timer = 0;

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

                Container communicationLinkMSGContainer = getKeyValueDataStore().get(CommunicationLinkMSG::ID());
                communicationLinkMSG = communicationLinkMSGContainer.getData<CommunicationLinkMSG>();

                if (sim) { // The following is for the parking simulation
                    irRear = sbd.getValueForKey_MapOfDistances(INFRARED_REAR);
                    irRearRight = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
                    usFront = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT);
                    IRFRObstacle = obstacleDetect(sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT), 2);

                    if (stageMoving == 0) {
                        // Go forward.
                        vc.setSpeed(2);
                        vc.setSteeringWheelAngle(0);
                        parkStart = vd.getAbsTraveledPath();
                    }
                    if (stageMoving == 1) {
                        // Move a bit forward after a parking space found
                        vc.setSpeed(1);
                        vc.setSteeringWheelAngle(0);
                        if (irRearRight > 0 && parkingSit == 0) {
                            // Move to stage 2 after the irRear reached the second obstacle
                            stageMoving = 2;
                        }
                        if (irRearRight < 0 && parkingSit == 1) {
                            // Move to stage 2 if the irRear reads remains 0
                            stageMoving = 2;
                            cerr << "parkingSit ==" << parkingSit << " moving to next stage " << endl;
                        }
                    }
                    if (stageMoving == 2) {
                        //stop the car
                        vc.setSpeed(0);
                        if (irRear < 0) {
                            // move ths stage to 3 when there is nothing on the back of the car
                            stageMoving = 3;
                            cerr << "irRear smaller than 0 Stage Moving = 3" << endl;
                        }
                    }
                    if (stageMoving == 3 && (irRear < 0)) {
                        // Parking starts
                        cerr << "parking gap " << parkingGap << endl;

                        parkEnd = vd.getAbsTraveledPath();    // keep updating the "end" of the parking (current odometer reads)
                        backDist = parkEnd - parkStart;       // distance traveled while parking
                        double cosVal = cos(30 * PI / 180.0); // cos value with the proximity angle
                        double adjDist = cosVal * backDist;   // proximity value of the car traveled distance (paralleled to the road)

                        if (gap == 1) {
                            // first step (turning right) of the parking start
                            if (irRear < 0 && adjDist < parkingGap / 2) {
                                vc.setSpeed(-1);
                                vc.setSteeringWheelAngle(2);
                                counter++;
                            }
                            // second step (turning left) of the parking start
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
                    if (counter < 1) {  //if the counter is less than 1, move the stage to 4
                        i = vd.getAbsTraveledPath(); // records the current position.
                        stageMoving = 4;
                    }
                    if (stageMoving == 4) { // the final adjustment to the car.
                        cerr << "Move one last bit" << endl;
                        double j = vd.getAbsTraveledPath(); // records the current position
                        vc.setSpeed(1);
                        vc.setSteeringWheelAngle(.2);
                        if (parkingSit == 0 && j - i < 2) { // move the car two units forwards
                            cerr << "stage4 #1 if" << endl;
                            stageMoving = 5; // move the stage to 5
                        }
                        if (usFront < 7 && usFront > 0) { // move the car forwards until ultrasonic front sensor reads below 7
                            stageMoving = 5;
                            cerr << "stage4 #2 if" << endl;
                        }
                    }
                    if (stageMoving == 5) { // stop the car, parking endded
                        cerr << "car stopped!!" << endl;
                        vc.setSpeed(0);
                        vc.setSteeringWheelAngle(0);
                    }
                    if (irRear > 0 && irRear < 4) { //Emergency stop whenever the irRear reads smaller than 4
                        vc.setSpeed(0);
                        cerr << "emergency stop!" << endl;
                    }
                } else if (!sim) {
                    irRear = communicationLinkMSG.getInfraredBack();
                    usFront = communicationLinkMSG.getUltraSonicFrontCenter();
                    int a = communicationLinkMSG.getInfraredSideFront();
                    bool IRRObstacle = obstacleDetect(irRear, 2);
                    bool USFObstacle = obstacleDetect(usFront, 1);
                    cerr << "This is the IFFRObs reading!!!" << a << endl;
                    IRFRObstacle = obstacleDetect(communicationLinkMSG.getInfraredSideFront(), 2);

                    //Car is using degree.
                    if (stageMoving == 0) {
                        // Go forward.
                        vc.setBrakeLights(false);
                        vc.setSpeed(100);
                        vc.setSteeringWheelAngle(0);
                        parkStart = communicationLinkMSG.getWheelEncoder();
                    }
                    if (stageMoving == 1) {
                        //to make the car stop
                        vc.setBrakeLights(false);
                        vc.setSpeed(100);
                        vc.setSteeringWheelAngle(0);
                        if (!IRRObstacle && parkingSit == 0) {
                            stageMoving = 2;
                        }
                        if (IRRObstacle && parkingSit == 1) {
                            stageMoving = 2;
                            cerr << "parkingSit ==" << parkingSit << " moving to next stage" << endl;
                        }
                    }
                    if (stageMoving == 2) {
                        vc.setBrakeLights(true);
                        //vc.setSpeed(190);
                        if (parkingSit == 1) {
                            //vc.setSpeed(190);
                            timer++;
                            if (timer > 30) {
                                stageMoving = 3;
                            }
                        }
                        if (parkingSit == 0 && irRear < 20) {
                            stageMoving = 3;
                            cerr << "irRear smaller than 0 Stage Moving = 3" << endl;
                        }
                    }
                    if (stageMoving == 3 && !IRRObstacle) {
                        //Parking
                        parkEnd = communicationLinkMSG.getWheelEncoder();
                        backDist = parkEnd - parkStart;
                        double cosVal = cos(60 * PI / 180.0);
                        double adjDist = cosVal * backDist;

                        if (gap == 1) {
                            cerr << "GAP: " << gap << endl;
                            if (irRear < 10 && adjDist < parkingGap / 2) {
                                vc.setBrakeLights(false);
                                vc.setSpeed(75);
                                vc.setSteeringWheelAngle(1.5);
                                counter++;
                            }
                            if (irRear < 10 && adjDist > parkingGap / 2 && counter > 1) {
                                vc.setBrakeLights(false);
                                vc.setSpeed(75);
                                vc.setSteeringWheelAngle(-1.5);
                                counter -= 1.6;
                            }
                        }
                        if (gap == 2) {
                            cerr << "GAP: " << gap << endl;
                            if (irRear < 10 && adjDist < parkingGap / 3) {
                                vc.setBrakeLights(false);
                                vc.setSpeed(75);
                                vc.setSteeringWheelAngle(.7);
                                counter++;
                            }
                            if (irRear < 10 && adjDist > parkingGap / 3 && counter > 1) {
                                vc.setBrakeLights(false);
                                vc.setSpeed(75);
                                vc.setSteeringWheelAngle(-.7);
                                counter -= 1.6;
                            }
                        }
                    }
                    if (counter < 1 && stageMoving == 3) {
                        i = communicationLinkMSG.getWheelEncoder();
                        stageMoving = 4;
                    }
                    if (stageMoving == 4) {
                        cerr << "Move one last bit" << endl;
                        double j = communicationLinkMSG.getWheelEncoder();
                        vc.setBrakeLights(false);
                        vc.setSpeed(100);
                        vc.setSteeringWheelAngle(.1);
                        if (parkingSit == 1 && j - i < 20) {
                            cerr << "stage4 #1 if" << endl;
                            stageMoving = 5;
                        }
                        if (usFront < 25 && USFObstacle) {
                            stageMoving = 5;
                            cerr << "stage4 #2 if" << endl;
                        }
                    }
                    if (stageMoving == 5) {
                        cerr << "car stopped!!" << endl;
                        vc.setBrakeLights(true);
                       // vc.setSpeed(190);
                        vc.setSteeringWheelAngle(0);
                    }
                    if (irRear > 3 && irRear < 15 && IRRObstacle) {
                        //Emergency stop
                        vc.setBrakeLights(true);
                       // vc.setSpeed(190);
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
                            if (!IRFRObstacle && parking >= 15) {
                                GAP_SIZE = parking;
                                parkingGap = parking;
                                gap = 1;
                                stageMoving = 1;
                                parkingSit = 1;
                                stageMeasuring++;
                                cerr << "parkingSit 15 units" << endl;
                            }
                            //when there is something detected within 15 units
                            if (IRFRObstacle && parking < 15) {
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
                            communicationLinkMSG.getWheelEncoder();
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
                            distance = communicationLinkMSG.getInfraredSideFront();
                            double empty = communicationLinkMSG.getWheelEncoder();
                            absPathStart = communicationLinkMSG.getWheelEncoder();
                            cerr << "IRFRObstacle :" << IRFRObstacle << endl;
                            if (!IRFRObstacle && empty >= 50) {
                                GAP_SIZE = empty;
                                parkingGap = empty;
                                gap = 1;
                                stageMoving = 1;
                                parkingSit = 0;
                                stageMeasuring = 1;
                                cerr << "parkingSit 1 = 50 units" << endl;
                            }
                            if (IRFRObstacle && empty < 50) {
                                parkingSit = 1;
                                stageMeasuring = 1;
                                cerr << "parkingSit 2 < 50 units" << endl;
                            }
                        }
                            break;
                        case 1: {
                            // Checking for sequence +, -.
                            if (parkingSit == 1) {
                                cerr << "case 1! parkSit 1 " << endl;
                                cerr << "in case 1 parkingSit==1 real car " << endl;
                                cerr<< "distance "<<distance<<endl;
                                cerr<< "IRFRObstacle !"<<IRFRObstacle<<endl;
                                if ((distance > 0) && (!IRFRObstacle)) {
                                    // Found sequence +, -.
                                    cerr << "Setting stageMeasuring to 2 " << endl;
                                    stageMeasuring = 2;
                                    absPathStart = communicationLinkMSG.getWheelEncoder();
                                }
                                distance = communicationLinkMSG.getInfraredSideFront();
                            }
                        }
                            break;
                        case 2: {
                            cerr << "case 2! " << endl;
                            // Checking for sequence -, +.
                            if (IRFRObstacle) {
                                absPathEnd = communicationLinkMSG.getWheelEncoder();
                                GAP_SIZE = (absPathEnd - absPathStart);
                                cerr << "Sizeeee = " << GAP_SIZE << endl;

                                if (stageMoving < 1 && GAP_SIZE <= 30) {
                                    gap = 1;
                                    cerr << "Start Parking!!!!! gap 1" << endl;
                                } else if (stageMoving < 1 && GAP_SIZE > 30) {
                                    gap = 2;
                                    cerr << "Start Parking!!!!! gap 2" << endl;
                                }
                                stageMeasuring = 1;
                                if ((stageMoving < 1) && (IRFRObstacle) && (GAP_SIZE > 30)) {
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

