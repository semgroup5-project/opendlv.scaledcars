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


        Park::Park(const int32_t &argc, char **argv) :
                TimeTriggeredConferenceClientModule(argc, argv, "Park"),
                communicationLinkMSG(),
                vc(),
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

                usFront = communicationLinkMSG.getUltraSonicFrontCenter();
                irFrontRight = communicationLinkMSG.getInfraredSideFront();
                irRearRight = communicationLinkMSG.getInfraredSideBack();
                odometer = communicationLinkMSG.getWheelEncoder();

                IRRObstacle = obstacleDetection(irRear, IR);
                IRFRObstacle = obstacleDetection(irFrontRight, IR);
                IRRRObstacle = obstacleDetection(irRearRight, IR);
                USFObstacle = obstacleDetection(usFront,US);

                if(IRRObstacle && irRear < 15 && irRear > 5){
                    vc.setBrakeLights(true);
                    cerr << "TOO CLOSE AT THE BACK, EMERGENCY STOP!!"<<endl;
                }
                if(USFObstacle && usFront >0 && usFront < 20){
                    vc.setBrakeLights(true);
                    cerr << "TOO CLOSE AT THE BACK, EMERGENCY STOP!!"<<endl;
                }
                if (isParking) {
                    parallelPark();
                    cout << "PARKING : Now I'm parking" << endl;
                } else {
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
            if (IRRRObstacle && parkingStart == 0) {
                parkingStart = odometer;
                cout << "PARKING : Here starts freedom      " << parkingStart << endl;
            }

            // Gap is too narrow
            if (IRRRObstacle && IRFRObstacle && parkingStart > 0 && (odometer - parkingStart) < GAP) {
                parkingStart = 0;
                isParking = false;
                cout << "PARKING : No freedom" << endl;
            }

            // Gap is sufficient
            if (parkingStart > 0 && (odometer - parkingStart) >= GAP) {
                backStart = odometer;
                isParking = true;
                sendParkerMSG();
                vc.setBrakeLights(true);
                cout << "PARKING : Insertion time" << endl;
            }

        }

        void Park::parallelPark() {
            backEnd = odometer;
            adjDist = adjDistCalculation(backStart, backEnd);
            switch (parkingState) {
                case START: {
                    vc.setBrakeLights(false);
                    setParkingState(RIGHT_TURN);
                }
                    break;
                case RIGHT_TURN: {
                    vc.setBrakeLights(false);
                    vc.setSpeed(72);
                    vc.setSteeringWheelAngle(1.5);
                    parkingCounter++;
                    cout << "PARKING : Turning right" << endl;
                    if (adjDist < GAP / 2) {
                        setParkingState(LEFT_TURN);
                    }
                }
                    break;

                case LEFT_TURN: {
                    vc.setBrakeLights(false);
                    vc.setSpeed(72);
                    vc.setSteeringWheelAngle(-1.5);
                    parkingCounter++;
                    cout << "PARKING : Turning left" << endl;
                    if (adjDist > GAP / 2) {
                        setParkingState(END);
                    }
                }
                    break;
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
                        ifObstacle = true;
                    }
                }
                    break;
                case (IR) : {
                    if (i > 28 || i < 0) {
                        ifObstacle = true;
                    } else if (i <= 28 && i > 0) {
                        ifObstacle = false;
                    }
                }
                    break;
            }
            return ifObstacle;
        }

        double Park::adjDistCalculation(double start, double end) {
            backDist = end - start;

            double cosVal = cos(backDist / (GAP / 2)); // cos value with the proximity angle
            adjDist = cosVal * backDist;   // proximity value of the car traveled distance (paralleled to the road)

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

