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
        double counterS = 0;
        double counterO = 0;
        int counter = 0;

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
                usFrontRight(0),
                irFrontRight(0),
                irRear(0),
                irRearRight(0),
                parkingState(0),
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
                
                // Do parking if it's activated in the CommunicationLink
                if(isOkay(communicationLinkMSG)){

                	sensorSetup(communicationLinkMSG);
                	obstacleSetup();
                	
                	// Emergency stop if an object is detected to close to the back of the vehicle.
                	if (IRRObstacle && irRear < 5 && irRear > 0) {
                    	vc.setBrakeLights(true);
                  	cerr << "TOO CLOSE AT THE BACK, EMERGENCY STOP!!" << endl;
                	}
                	
                	// Parking
                	if(communicationLinkMSG.getUnpark() == 0 && !isParked){
               		if (isParking) {
                  		park();
                  		cout << "PARKING : Now I'm parking" << endl;
                		} else if (!isParking) {
                  		parkingFinder();
                  		cout << "PARKING : Finding values" << endl;
                		}
                	
                	// Unparking
                	} else if (communicationLinkMSG.getUnpark() == 1 && isParked){
                		unpark();
                	}

                	Container ct(vc);
                	getConference().send(ct);
                
					}
            }
        }

        /**
        * Checks if there is enough space to park
        */
        void Park::parkingFinder() {
            // Parking space starting point
            vc.setBrakeLights(false);
            vc.setSpeed(96);
            double angle = (irRearRight - irFrontRight) / 10;
            
            // Adjust the distance of the vehicle to a potential object
            if (irFrontRight < 10 && irFrontRight > 0 && (int) (irRearRight - irFrontRight) != 0) {
                vc.setSteeringWheelAngle(angle);
            } else if (irFrontRight < 0 && (int) (irRearRight - irFrontRight) == 0) {
                if (usFrontRight < 20 && usFrontRight > 0) {
                    vc.setSteeringWheelAngle(-0.26);
                } else {
                    vc.setSteeringWheelAngle(angle);
                }
            }

				// If it is no obstacle in the way
            if (!IRRRObstacle) {
                counterS++;
                if (counterS > 3) {
                    counterO = 0;
                    parkingSpace = odometer - parkingStart;
                    cout << "ParkingSpace : " << parkingSpace << endl;

                }

				// If it is an obstacle in the way
            } else if (IRRRObstacle) {
                counterO++;
                if (counterO > 3) {
                    parkingStart = odometer;
                    parkingSpace = 0;
                    counterS = 0;

                }
            }
            
            // If the gap for parking is sufficient
            if (parkingSpace >= GAP) {
                cout << "parking Space  : " << parkingSpace << endl;
                backStart = odometer;
                
                isParking = true;
                parkingState = START;
                
            }
        }

			/**
			* Parks an unparked vehicle
			*/
        void Park::park() {
            
            adjDist = adjDistCalculation(parkingSpace);
            cout << "adjDist:   " << adjDist << endl;

            switch (parkingState) {
                case START: {
                	cout << "PARKING : Start parking" << endl;
                	setParkingState(RIGHT_TURN);
                }
                case RIGHT_TURN: {

                    vc.setBrakeLights(false);
                    if (odometer - backStart >= 4) {
                        vc.setSpeed(60);
                        vc.setSteeringWheelAngle(1.5);
                        cout << "PARKING : Turning right" << endl;
                        cout << "calc " << (odometer - backStart) << endl;
                        if (odometer - backStart >= (adjDist * 0.60)) {
                            
                            setParkingState(LEFT_TURN);
                        }
                    }

                    break;
                }

                case LEFT_TURN: {

                    vc.setBrakeLights(false);
                    vc.setSteeringWheelAngle(-1.5);
                    
                    vc.setSpeed(60);
                    cout << "PARKING : Turning left" << endl;
                    cout << "adjDist" << adjDist << endl;
                    
                    if (obstacleDetection(irRear, 3) || (odometer - backStart >= (adjDist * 1.125))) {
                        
                        setParkingState(INGAP_RIGHT_TURN);
                        backEnd = odometer;

                    }

                    break;
                }
                case INGAP_RIGHT_TURN: {
                    
                    cout << "odo - backEnd" << (odometer - backEnd) << endl;
                    if ((abs(odometer - backEnd) <= 3) || (obstacleDetection(usFront, 4))) {
                        vc.setSpeed(96);
                        vc.setSteeringWheelAngle(0.24);

                    } else {
                        setParkingState(END);
                    }
						break;
                }

                case END: {
                    vc.setBrakeLights(true);
                    isParked = true;
                    cout << "PARKING : I'm parked" << endl;
                }

            }
        }
        
        /**
        * Unparks a parked vehicle
        */
        void Park::unpark() {
           	
            adjDist = adjDistCalculation(parkingSpace);
            cout << "adjDist:   " << adjDist << endl;

            switch (parkingState) {
                case START: {

                	cout << "UNPARKING: Start" << endl;
                	if(IRRObstacle && irRear < 5 && irRear > 0){
							vc.setBrakeLights(true);
                		setParkingState(LEFT_TURN);
                	} else {    
                    vc.setBrakeLights(false);
                    vc.setSpeed(60);
                  }
                  
                	break;
                }
                case LEFT_TURN: {

                    
                    vc.setSpeed(96);
                    vc.setSteeringWheelAngle(-1.5);
                    
                    cout << "UNPARKING : Turning left" << endl;
                    
                    if (obstacleDetection(irRear, 3) || (odometer - backStart >= (adjDist * 1.125))) {
                        
                        setParkingState(END);
                        backEnd = odometer;

                    }
                    

                    break;
                }

                case RIGHT_TURN: {
                
                    vc.setBrakeLights(false);
                    if (odometer - backStart >= 4) {
                        vc.setSpeed(96);
                        vc.setSteeringWheelAngle(1.5);
                        
                        cout << "UNPARKING : Turning right" << endl;
                        
                        if (odometer - backStart >= (adjDist * 0.60)) {
                            
                            setParkingState(LEFT_TURN);
                        }
                    }

                    break;
                }
                case END: {
                    
                    setParkingState(START);
                    isParked = false;
                    isParking = false;
                    vc.setSpeed(96);
                    vc.setSteeringWheelAngle(0.24);
                    cout << "UNPARKING : Car is unparked" << endl;

                }
            }
        }

			/**
			* Set the state of the parking and unparking functions
			*/
        void Park::setParkingState(int state) {
            parkingState = state;
        }

			/**
			* Detects if there is an obstacle
			*
			* @return true if there is an obstacle detected
			*/
        bool Park::obstacleDetection(int value, int id) {
            bool ifObstacle;
            switch (id) {
                case (US) : {
                    if (value > 70 || value < 0) {
                        ifObstacle = false;
                    } else if (value <= 70 && value > 0) {
                        ifObstacle = true;
                    }
                }
                    break;
                case (IR) : {
                    if (value > 28 || value < 0) {
                        ifObstacle = false;
                    } else if (value <= 28 && value > 0) {
                        ifObstacle = true;
                    }
                }
                    break;
                case (3) : {
                    if ((value > 2) && (value < 29)) {
                        ifObstacle = true;
                    } else {
                        ifObstacle = false;
                    }
                }
                    break;
                case (4) : {
                    if ((value > 1) && (value < 30)) {
                        ifObstacle = true;
                    } else if (value < 1) {
                        ifObstacle = false;
                    }
                }
                    break;
            }
            return ifObstacle;
        }

			/**
			* @return the estimated maximum parking distance, maximum distance for the vehicle to travel.
			*/
        double Park::adjDistCalculation(double start) {
           
            adjDist = start / cos(40);
            return abs(adjDist);
        }

			/**
			* Send a ParkerMSG to to the conference to let the parking algorithm take over.
			*/
        void Park::sendParkerMSG() {
            ParkerMSG p;
            p.setStateStop(0);
            Container c(p);
            getConference().send(c);
        }
        
        /**
        * @return true if only the parker state is active in the CommunicationLinkMSG provided.
        */
        bool Park::isOkay(CommunicationLinkMSG c){
        		if(c.getStateParker() == 1 && c.getStateLaneFollower() == 0 && c.getStateOvertaker() == 0)
        			return true;
        		
        		return false;
        }
        
        /**
        * Setup private global sensors with the CommunicationLinkMSG provided.
        */
        void Park::sensorSetup(CommunicationLinkMSG communicationLinkMSG){
        		irRear = communicationLinkMSG.getInfraredBack();
            usFront = communicationLinkMSG.getUltraSonicFrontCenter();
            irFrontRight = communicationLinkMSG.getInfraredSideFront();
            irRearRight = communicationLinkMSG.getInfraredSideBack();
            odometer = communicationLinkMSG.getWheelEncoder();
        }
        
        /**
        * Setup the global private obstacle indicators with the private global sensor values.
        */
        void Park::obstacleSetup(){
        		IRRObstacle = obstacleDetection(irRear, IR);
            IRFRObstacle = obstacleDetection(irFrontRight, IR);
            IRRRObstacle = obstacleDetection(irRearRight, IR);
            USFObstacle = obstacleDetection(usFront, US);
        }
 
    }
}
// automotive::miniature


