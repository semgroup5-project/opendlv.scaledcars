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
        bool parked = false;
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

//                if (IRRObstacle && irRear < 5 && irRear > 0) {
//                    vc.setBrakeLights(true);
//                    cerr << "TOO CLOSE AT THE BACK, EMERGENCY STOP!!" << endl;
//                }
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
                } else if(parked){
                    vc.setBrakeLights(true);
                    cout << "Car Parked!" << endl;
                }

                Container c(vc);
                getConference().send(c);

            }
            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

        //going forwards till finds a gap
        void Park::parkingFinder() {
            // Parking space starting point


            vc.setBrakeLights(false);
            vc.setSpeed(96);
            vc.setSteeringWheelAngle(0.28);


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
                cout << "parking Space  : " << parkingSpace << endl;
                backStart = odometer;
                //timer += 0.5;
                //if (timer >= 4) {
              //  vc.setBrakeLights(true);
                isParking = true;
                parkingState = RIGHT_TURN;
                //}
            }
        }

        void Park::setParkingType(int type) {
            parkingType = type;
        }


        void Park::parallelPark() {
          //  backEnd = odometer;
            adjDist = adjDistCalculation(parkingSpace);
            cout << "adjDist:   " << adjDist << endl;

            switch (parkingState) {
                case START: {
                    cout<<"start and parkingSpace is "<< parkingSpace << endl;
                //    vc.setBrakeLights(false);
               //     vc.setSpeed(96);
                   // timer+=0.5;

//                        vc.setBrakeLights(true);
                       // vc.setSpeed(96);


                }
                    break;
                case RIGHT_TURN: {

                  //  vc.setBrakeLights(false);

                    /*//MS change angle before moving the car to allow for a tighter turn
                     ChangeWheelAngleCounter++;
                     if(ChangeWheelAngleCounter >= 50){
                         vc.setSpeed(54);
                         ChangeWheelAngleCounter = 0;
                     }
                     //MS END*/

                    if (odometer - backStart >= 5) {
                        vc.setSpeed(60);
                        vc.setSteeringWheelAngle(1.5);
                        cout << "PARKING : Turning right" << endl;
                        cout<<  "calc "<<(odometer - backStart)<<endl;
                        if (odometer - backStart >= (adjDist * 0.8)) {
                           // vc.setBrakeLights(true);
                             setParkingState(LEFT_TURN);
                        }
                    }

                    //if (adjDist >= GAP / 1.1  /*&& parkingCounter >= 40*/) {

                    //  setParkingState(LEFT_TURN);
                    //}

                    break;
                }

                case LEFT_TURN: {
                 //   vc.setBrakeLights(false);
                    vc.setSteeringWheelAngle(-1.5);
                    //MS change angle before moving the car to allow for a tighter turn
                    /*ChangeWheelAngleCounter++;
                    if(ChangeWheelAngleCounter >= WAITFORWHEELANGLECHANGE){
                        vc.setSpeed(54);
                        ChangeWheelAngleCounter = 0;
                    }
                    //MS END*/
                    vc.setSpeed(60);
                    cout << "PARKING : Turning left" << endl;
                    cout << "adjDist" << adjDist << endl;
                    //if (adjDist >= (GAP / 1.5)/*&& parkingCounter < 0 */ ) {

               //     if (odometer - backStart >= (adjDist * 1.1)) {
                    if (obstacleDetection(irRear, 3)){
                         //  vc.setBrakeLights(true);

                        setParkingState(INGAP_RIGHT_TURN);


                    }

                    break;
                }
                case INGAP_RIGHT_TURN: {
               //     vc.setBrakeLights(false);

                    //MS change angle before moving the car to allow for a tighter turn
                    /*ChangeWheelAngleCounter++;
                    if(ChangeWheelAngleCounter >= WAITFORWHEELANGLECHANGE){
                        vc.setSpeed(96);
                        ChangeWheelAngleCounter = 0;
                    }*/
                    //MS END
                   if (!(obstacleDetection(usFront, 4))){
                       vc.setSpeed(96);
                       vc.setSteeringWheelAngle(0.24);
                   }else{
                       vc.setBrakeLights(true);
                       parked = true;
                   }

                }
//                case END: {
//                    vc.setBrakeLights(true);
//                    parked = true;
//                    cout << "PARKING : I'm parked" << endl;
//                }
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
                        ifObstacle = true;
                    }
                }
                    break;
                case (IR) : {
                    if (i > 28 || i < 0) {
                        ifObstacle = false;
                    } else if (i <= 28 && i > 0) {
                        ifObstacle = true;
                    }
                }
                    break;
                case (3) : {
                    if ((i > 2 ) && (i < 29) ){
                        ifObstacle = true;
                    } else  {
                        ifObstacle = false;
                    }
                }
                    break;
                case (4) : {
                    if ((i > 1 ) && (i < 30) ){
                        ifObstacle = true;
                    } else  {
                        ifObstacle = false;
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
            adjDist = start / cos(40);
            return abs(adjDist);
        }

        void Park::sendParkerMSG() {
            ParkerMSG p;
            p.setStateStop(0);
            Container c(p);
            getConference().send(c);
        }
    }
}
// automotive::miniature


