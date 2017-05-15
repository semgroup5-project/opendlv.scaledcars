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
        const int PARALLEL = 1;
       // const int BOX = 0;
        double counterS = 0;
        double counterO = 0;
        int counter = 0;
        //MS
        //int ChangeWheelAngleCounter =0;
        //MS
        double timer = 0;
        double turnCount = 0;

        Park::Park(const int32_t &argc, char **argv) :
                DataTriggeredConferenceClientModule(argc, argv, "Park"),
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
                isParking(false),
                isParked(false) {}

        Park::~Park() {}

        void Park::setUp() {


        }

        void Park::tearDown() {
            // This method will be call automatically _after_ return from body().
        }


        // This method will do the main data processing job.
        void Park::nextContainer(Container &c){

            if (c.getDataType() == CommunicationLinkMSG::ID()){
                CommunicationLinkMSG communicationLinkMSG = c.getData<CommunicationLinkMSG>();
                
                // DO THE PARKING IF PARKING IS SET
                if(isOkay(communicationLinkMSG)){

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
                	
                	// PARK IF THE CAR IS NOT PARKED
                	if(communicationLinkMSG.getUnpark() == 0 && !isParked){
               		if (isParking) {
                  		parallelPark();
                  		cout << "PARKING : Now I'm parking" << endl;
                		} else if (!isParking) {
                  		parkingFinder();
                  		cout << "PARKING : Finding values" << endl;
                		}
                	
                	// UNPARK IF THE CAR IS PARKED
                	} else if (communicationLinkMSG.getUnpark() == 1 && isParked){
                		unpark();
                	}

                	Container ct(vc);
                	getConference().send(ct);
                
					}
            }
        }

       //going forwards till finds a gap
        void Park::parkingFinder() {
            // Parking space starting point


            vc.setBrakeLights(false);
            vc.setSpeed(96);
            cout << "IRRRObstacle : " << IRRRObstacle << endl;

            if (!IRRRObstacle) {
                counterS++;
                if (counterS > 3) {
                    counterO = 0;
                    parkingSpace = odometer - parkingStart;
                    cout << "ParkingSpace : " << parkingSpace << endl;
                    cout << "counterO: " <<counterO << endl;
                }

            }
            else if (IRRRObstacle) {
                counterO++;
                if (counterO > 3) {
                    parkingStart = odometer;
                    parkingSpace = 0;
                    counterS = 0;
                    cout << "counterS: " << counterS << endl;
                }
            }
            if (parkingSpace >= GAP) {
                cout <<  "parking Space  : " << parkingSpace << endl;
                backStart = odometer;
                //timer += 0.5;
                //if (timer >= 4) {
                    vc.setBrakeLights(true);
                    isParking = true;
                //}
            }
            /*void Park::unpark(){
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

            }*/

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

        ) {
*/
        
        void Park::unpark(){
            /*switch (parkingState) {
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
            }*/
        }


        void Park::setParkingType(int type) {
            parkingType = type;
        }


        void Park::parallelPark() {
            backEnd = odometer;
            adjDist = adjDistCalculation(parkingSpace);
            cout << "adjDist:   " << adjDist << endl;

            switch (parkingState) {
                case START: {
                    vc.setBrakeLights(true);
                    setParkingState(RIGHT_TURN);
                }
                    break;
                /*case INTHEMIDD: {
                    vc.setBrakeLights(false);
                    vc.setSteeringWheelAngle(1.4);
                    vc.setSpeed(96);
                    cout << "In the middle" << endl;
                    setParkingState(RIGHT_TURN);
                }
                break;*/
                case RIGHT_TURN: {
                    vc.setBrakeLights(false);
                    vc.setSteeringWheelAngle(1.5);
                     /*//MS change angle before moving the car to allow for a tighter turn
                      ChangeWheelAngleCounter++;
                      if(ChangeWheelAngleCounter >= 50){
                          vc.setSpeed(60);
                          ChangeWheelAngleCounter = 0;
                      }
                      //MS END*/
                    vc.setSpeed(60);
                    cout << "PARKING : Turning right" << endl;
                    cout << "adjDist:   " << adjDist << endl;
                    if (backEnd - parkingSpace >= (adjDist * 0.9)) {

                        setParkingState(LEFT_TURN);
                    }
                    //if (adjDist >= GAP / 1.1  /*&& parkingCounter >= 40*/) {

                    //  setParkingState(LEFT_TURN);
                    //}

                    break;
                }

                case LEFT_TURN: {
                    vc.setBrakeLights(false);
                    vc.setSteeringWheelAngle(-1.5);
                    //MS change angle before moving the car to allow for a tighter turn
                    /*ChangeWheelAngleCounter++;
                    if(ChangeWheelAngleCounter >= WAITFORWHEELANGLECHANGE){
                        vc.setSpeed(60);
                        ChangeWheelAngleCounter = 0;
                    }
                    //MS END*/
                    vc.setSpeed(60);
                    cout << "PARKING : Turning left" << endl;
                    cout << "adjDist" << adjDist << endl;
                    //if (adjDist >= (GAP / 1.5)/*&& parkingCounter < 0 */ ) {
                    if (backEnd - parkingSpace >= (adjDist * 1.0)) {

                        setParkingState(INGAP_RIGHT_TURN);

                    }

                    break;
                }
                case INGAP_RIGHT_TURN: {
                    vc.setBrakeLights(false);
                    vc.setSteeringWheelAngle(1);
                    //MS change angle before moving the car to allow for a tighter turn
                    /*ChangeWheelAngleCounter++;
                    if(ChangeWheelAngleCounter >= WAITFORWHEELANGLECHANGE){
                        vc.setSpeed(96);
                        ChangeWheelAngleCounter = 0;
                    }*/
                    //MS END
                    vc.setSpeed(96);
                    cout << "PARKING : In Gap Turning right" << endl;
                    cout << "turnCount" << turnCount << endl;
                    turnCount += 0.5;
                    if (turnCount > 10) {
                        setParkingState(END);
                    }
                }
                case END: {
                    vc.setBrakeLights(true);
                    isParked = true;
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

        double Park::adjDistCalculation(double start) {
       /*     backDist = end - start;
            //turn degrees toradians
            double cosVal = cos(backDist / (GAP / 2)); // cos value with the proximity angle
            adjDist = abs(cosVal) * backDist;   // proximity value of the car traveled distance (paralleled to the road)

       */
            adjDist=start/cos(40);
            return abs(adjDist);
        }

        void Park::sendParkerMSG() {
            ParkerMSG p;
            p.setStateStop(0);
            Container c(p);
            getConference().send(c);
        }
        
        bool Park::isOkay(CommunicationLinkMSG c){
        		if(c.getStateParker() == 1 && c.getStateLaneFollower() == 0 && c.getStateOvertaker() == 0)
        			return true;
        		
        		return false;
        }
        
    }
} // automotive::miniature


