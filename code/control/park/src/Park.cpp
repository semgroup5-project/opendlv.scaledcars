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
#include "Park.h"

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;
        using namespace group5;


        //*****************************//
        //	DIFFERENT PARKING STATES   //
        //*****************************//
        const int PARALLEL = 0;
        const int BOX = 1;
        double counterS = 0;
        double counterO = 0;
        int counter = 0;
        double timer = 0;
        double turnCount = 0;

        Park::Park(const int32_t &argc, char **argv) :
                TimeTriggeredConferenceClientModule(argc, argv, "Park"),
                communicationLinkMSG(),
                vc(),
                parkingSpace(0),
                IRRObstacle(false),
                USFObstacle(false),
                IRFRObstacle(false),
                IRRRObstacle(false),
                odometer(0),
                usFront(0),
                irFrontRight(0),
                irRear(0),
                irRearRight(0),
                parkingState(0),
                parkingType(0),
                parkingCounter(0),
                parkingStart(0),
                backDist(0),
                backStart(0),
                backEnd(0),
                adjDist(0),
                isParking(false) {}

        Park::~Park() {}

        void Park::setUp() {


        }

        void Park::tearDown() {
            // This method will be call automatically _after_ return from body().
        }


        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Park::body() {


            while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
                   odcore::data::dmcp::ModuleStateMessage::RUNNING) {

                Container communicationLinkMSGContainer = getKeyValueDataStore().get(CommunicationLinkMSG::ID());
                communicationLinkMSG = communicationLinkMSGContainer.getData<CommunicationLinkMSG>();

                //     setParkingType(communicationLinkMSG.getParkingType());
                irRear = communicationLinkMSG.getInfraredBack();
                usFront = communicationLinkMSG.getUltraSonicFrontCenter();
                irFrontRight = communicationLinkMSG.getInfraredSideFront();
                irRearRight = communicationLinkMSG.getInfraredSideBack();
                odometer = communicationLinkMSG.getWheelEncoder();

                IRRObstacle = obstacleDetection(irRear, IR);
                IRFRObstacle = obstacleDetection(irFrontRight, IR);
                IRRRObstacle = obstacleDetection(irRearRight, IR);
                USFObstacle = obstacleDetection(usFront, US);

                if (IRRObstacle && irRear < 5 && irRear > 0) {
                    vc.setBrakeLights(true);
                    cerr << "TOO CLOSE AT THE BACK, EMERGENCY STOP!!" << endl;
                }
//                if (USFObstacle && usFront > 0 && usFront < 10) {
//                    vc.setBrakeLights(true);
//                    cerr << "TOO CLOSE AT THE FRONT, EMERGENCY STOP!!" << endl;
//                }
                if (isParking) {
                    parallelPark();
                    cout << "PARKING : Now I'm parking" << endl;
                } else if (!isParking) {
                    parkingFinder();
                    cout << "PARKING : Finding values" << endl;
                }

                Container c(vc);
                getConference().send(c);

            }
            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

        void Park::parkingFinder() {
            // Parking space starting point
            vc.setSteeringWheelAngle(.2);

            vc.setBrakeLights(false);
            vc.setSpeed(96);
            cout << "IRRRObstacle : " << IRRRObstacle << endl;

            if (!IRRRObstacle) {
                counterS++;
                if (counterS > 3) {
                    counterO = 0;
                    parkingSpace = odometer - parkingStart;
                    cout << "ParkingSpace : " << parkingSpace << endl;

                }
            } else if (IRRRObstacle) {
                counterO++;
                if (counterO > 3) {
                    parkingStart = odometer;
                    parkingSpace = 0;
                    counterS = 0;
                }
            }
            if (parkingSpace >= GAP) {
                backStart = odometer;
                timer += 0.5;
                if (timer >= 8) {
                    vc.setBrakeLights(true);
                    isParking = true;
                }
            }


//            if ((IRRRObstacle == false) /*&& !IRFRObstacle */&& parkingStart == 0) {
//                parkingStart = odometer;
//
//                vc.setSteeringWheelAngle(.2);
//                cout << "PARKING : Here starts freedom      " << parkingStart << endl;
//            }
//            cout << "IRRR: " << IRRRObstacle << endl;
//            cout << "IRFR: " << IRFRObstacle << endl;
//            cout << "OD-Park: " << odometer - parkingStart << endl;
//            cout << " odometer: " << odometer << endl;
//            cout << "parkingStart: " << parkingStart << endl;
//
//            // Gap is too narrow
//            if (IRRRObstacle && (odometer - parkingStart) < GAP && parkingStart > 0) {
//                parkingStart = 0;
//                vc.setSteeringWheelAngle(0);
//                isParking = false;
//                cout << "PARKING : No freedom" << endl;
//            }
//
//            // Gap is sufficient
//            if (((odometer - parkingStart) >= GAP) && (parkingStart > 0)) {
//                backStart = odometer;
//                isParking = true;
//                //sendParkerMSG();
//
//                vc.setBrakeLights(true);
//                vc.setSpeed(190);
//                cout << "PARKING : Insertion time" << endl;
//            }

        }

        /*void Park::park() {
            switch (parkingState) {
                case START: {
                    setParkingState(RIGHT_TURN);
                    vc.setBrakeLights(false);

                }
                    break;
                case RIGHT_TURN: {
                    vc.setSpeed(70);
                    vc.setSteeringWheelAngle(1.5);
                    parkingCounter++;
                    cout << "PARKING : Turning right" << endl;

                    if (parkingCounter == 100) {
                        if (parkingType == PARALLEL) {
                            setParkingState(LEFT_TURN);
                        } else if (parkingType == BOX) {
                            setParkingState(END);
                        }
                    }
                }
                    break;

                case LEFT_TURN: {
                    vc.setSpeed(70);
                    vc.setSteeringWheelAngle(-1.5);
                    parkingCounter++;
                    cout << "PARKING : Turning left" << endl;

                    if (parkingCounter == 200) {
                        setParkingState(END);
                    }
                }
                    break;

                case END: {
                    vc.setBrakeLights(true);
                    setParkingState(START);
                    isParking = false;
                    cout << "PARKING : I'm parked" << endl;
                }
            }
        }
*/
        /*void Park::unpark() {
            switch (parkingState) {
                case START: {

                    if (parkingType == PARALLEL) {
                        setParkingState(LEFT_TURN);
                    } else if (parkingType == BOX) {
                        setParkingState(RIGHT_TURN);
                    }
                    vc.setBrakeLights(false);
                }
                    break;
                case LEFT_TURN: {
                    vc.setSpeed(96);

                    vc.setSteeringWheelAngle(-1.5);
                    parkingCounter++;
                    cout << "UNPARKING : Turning left" << endl;

                    if (parkingCounter == 100) {
                        setParkingState(RIGHT_TURN);
                    }
                }
                    break;

                case RIGHT_TURN: {
                    vc.setSpeed(96);
                    vc.setSteeringWheelAngle(1.5);
                    parkingCounter++;
                    cout << "UNPARKING : Turning right" << endl;

                    if (parkingCounter == 200) {
                        setParkingState(END);
                    }
                }
                    break;

                case END: {
                    sendParkerMSG();
                    setParkingState(START);
                    cout << "UNPARKING : I'm unparked" << endl;
                }
            }
        }
*/

        void Park::setParkingType(int type) {
            parkingType = type;
        }


        void Park::parallelPark() {
            backEnd = odometer;
            adjDist = adjDistCalculation(backStart, backEnd);
            switch (parkingState) {
                case START: {
                    vc.setBrakeLights(true);
                    setParkingState(RIGHT_TURN);
                }
                    break;
                case RIGHT_TURN: {
                    vc.setBrakeLights(false);
                    vc.setSpeed(60);
                    vc.setSteeringWheelAngle(1.5);

                    cout << "PARKING : Turning right" << endl;
                    cout << "adjDist" << adjDist << endl;
                    if (adjDist >= GAP / 1.2  /*&& parkingCounter >= 40*/) {
                        setParkingState(LEFT_TURN);
                    }
                }
                    break;

                case LEFT_TURN: {
                    vc.setBrakeLights(false);
                    vc.setSpeed(60);
                    vc.setSteeringWheelAngle(-1.5);

                    cout << "PARKING : Turning left" << endl;
                    cout << "adjDist" << adjDist << endl;
                    if (adjDist >= (GAP * 1.5)/*&& parkingCounter < 0 */ ) {
                        setParkingState(INGAP_RIGHT_TURN);
                    }
                }
                    break;
                case INGAP_RIGHT_TURN: {
                    vc.setBrakeLights(false);
                    vc.setSpeed(96);
                    vc.setSteeringWheelAngle(1);
                    cout << "PARKING : In Gap Turning right" << endl;
                    cout << "turnCount" << turnCount << endl;
                    turnCount += 0.5;
                    if (turnCount > 10) {
                        setParkingState(END);
                    }
                }
                case END: {
                    vc.setBrakeLights(true);
                    cout << "PARKING : I'm parked" << endl;
                }
            }
        }

        void Park::setParkingState(int state) {
            parkingState = state;
        }

        bool Park::obstacleDetection(int i, int id) {
            bool ifObstacle;
            switch (id) {
                case (US) : {
                    if (i > 70 || i < 0) {
                        ifObstacle = false;
                    } else if (i <= 70 && i > 0) {
                        cout << "YOYO! US Object" << endl;
                        ifObstacle = true;
                    }
                }
                    break;
                case (IR) : {
                    if (i > 28 || i < 0) {
                        cout << "YO! IR Object" << endl;
                        ifObstacle = false;
                    } else if (i <= 28 && i > 0) {
                        ifObstacle = true;
                    }
                }
                    break;
            }
            return ifObstacle;
        }

        double Park::adjDistCalculation(double start, double end) {
            backDist = end - start;

            double cosVal = cos(backDist / (GAP / 2)); // cos value with the proximity angle
            adjDist = abs(cosVal) * backDist;   // proximity value of the car traveled distance (paralleled to the road)

            return adjDist;
        }

        void Park::sendParkerMSG() {
            ParkerMSG p;
            p.setStateStop(0);
            Container c(p);
            getConference().send(c);
        }
    }
} // automotive::miniature

