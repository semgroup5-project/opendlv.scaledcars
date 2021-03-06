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

        //  DIFFERENT PARKING STATES   //

        //*****************************//

        const int PARALLEL = 1;
        double counterS = 0;
        double counterO = 0;
        int counter = 0, _state = 0;
        bool parked = false;
        double timer = 0;
        double turnCount = 0;
        CommunicationLinkMSG communicationLinkMSG;
        int _stop = 0;
        Park::Park(const int32_t &argc, char **argv) :
                DataTriggeredConferenceClientModule(argc, argv, "Park"),vc(),parkingSpace(0),
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
/**
 * if there is a next container, proceed with the method inside
 * if _state is set to true, then run the parking algorithm
 *
 * @param c container received from the conference
 */

        void Park::nextContainer(Container &c) {

            if (c.getDataType() == CommunicationLinkMSG::ID()) {
                Container communicationLinkContainer = c.getData<CommunicationLinkMSG>();
                communicationLinkMSG = communicationLinkContainer.getData<CommunicationLinkMSG>();
                _state = communicationLinkMSG.getStateParker();
                if (_state) {
                    _stop = 0;
                    // setParkingType(communicationLinkMSG.getParkingType());
                    irRear = communicationLinkMSG.getInfraredBack();
                    usFront = communicationLinkMSG.getUltraSonicFrontCenter();
                    usFrontRight = communicationLinkMSG.getUltraSonicFrontRight();
                    irFrontRight = communicationLinkMSG.getInfraredSideFront();
                    irRearRight = communicationLinkMSG.getInfraredSideBack();
                    odometer = communicationLinkMSG.getWheelEncoder();
                    IRRObstacle = obstacleDetection(irRear, IR);
                    IRFRObstacle = obstacleDetection(irFrontRight, IR);
                    IRRRObstacle = obstacleDetection(irRearRight, IR);
                    USFObstacle = obstacleDetection(usFront, US);
                    if (isParking) {
                        parallelPark();
                        cout << "PARKING : Now I'm parking" << endl;
                    } else if (!isParking) {
                        parkingFinder();
                        cout << "PARKING : Finding values" << endl;
                    } else if (parked) {
                        vc.setBrakeLights(true);
                        cout << "Car Parked!" << endl;
                    }
                    Container c1(vc);
                    getConference().send(c1);
                }
                else if (!_state && !_stop) {
                    _stop = 1;
                    parkingSpace = 0;
                    IRRObstacle = false;
                    USFObstacle = false;
                    IRFRObstacle = false;
                    IRRRObstacle = false;
                    odometer = 0;
                    parkingState = 0;
                    parkingType = 0;
                    parkingCounter = 0;
                    parkingStart = 0;
                    backDist = 0;
                    backStart = 0;
                    backEnd = 0;
                    adjDist = 0;
                    isParking = false;
                    vc.setBrakeLights(true);
                    Container c3(vc);
                    // Send container.
                    getConference().send(c3);
                }

            }

        }
/**
 * This method will make the car move forward while checking for a paking gap,
 * moreover the it calibrates the relative position of the car reacting to the
 * the obstacle on the right side .
 */

        void Park::parkingFinder() {
            // Parking space starting point
            vc.setBrakeLights(false);
            vc.setSpeed(96);
            if (irFrontRight < 10 && irFrontRight > 0 && (int) (irRearRight - irFrontRight) != 0) {
                vc.setSteeringWheelAngle(0.33);
            }
            else if (irFrontRight < 0 && (int) (irRearRight - irFrontRight) == 0) {
                if (usFrontRight < 20 && usFrontRight > 0) {
                    vc.setSteeringWheelAngle(0.35);
                }
                else {
                    vc.setSteeringWheelAngle(0.33);
                }
            }
            if (!IRRRObstacle) {
                counterS++;
                if (counterS > 3) {
                    counterO = 0;
                    parkingSpace = odometer - parkingStart;
                    cout << "ParkingSpace : " << parkingSpace << endl;
                }
            }
            else if (IRRRObstacle) {
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
                isParking = true;
                parkingState = RIGHT_TURN;
            }
        }

/**
 * this method will set the parameter "parkingType" according to method call
 * @param type parking type
 */

        void Park::setParkingType(int type) {
            parkingType = type;
        }

/**
 * This method is the logic of the parking maneuver
 * it uses the function adjDistCalcualtion() to calculate
 * the adjacent distance that the car will turn into the gap
 * while parking. The
 */

        void Park::parallelPark() {

            adjDist = adjDistCalculation(parkingSpace);
            cout << "adjDist:   " << adjDist << endl;

            switch (parkingState) {

                case RIGHT_TURN: {
                    vc.setBrakeLights(false);

                    if (odometer - backStart >= 8){
                        vc.setSpeed(60);
                        vc.setSteeringWheelAngle(1.5);
                        cout << "PARKING : Turning right" << endl;
                        cout << "calc " << (odometer - backStart) << endl;
                        //tweeking the adjacent distance by multiplaying it to 1.15
                        //to get the desire turn
                        if (odometer - backStart >= (adjDist * 1.15)) {
                            setParkingState(LEFT_TURN);
                        }
                    }
                    break;
                }

                case LEFT_TURN: {
                    vc.setSteeringWheelAngle(-1.5);
                    vc.setSpeed(60);
                    cout << "PARKING : Turning left" << endl;
                    cout << "adjDist" << adjDist << endl;
                    //tweeking the adjacent distance by multiplaying it to 1.65
                    //to get the desire turn
                    if (obstacleDetection(irRear, 3) || (odometer - backStart >= (adjDist * 1.65))) {
                        cout<<"moving in gap forward"<<endl;
                        setParkingState(INGAP_RIGHT_TURN);
                        backEnd = odometer;
                    }
                    break;
                }

                case INGAP_RIGHT_TURN: {
                    cout << "odo - backEnd" << (odometer - backEnd) << endl;
                    if ((obstacleDetection(usFront, 4))) {
                        vc.setSpeed(96);
                        vc.setSteeringWheelAngle(1);
                    } else {
                        cout<<"hallo"<<endl;
                        vc.setBrakeLights(true);
                        parked = true;
                        _state = 0;
                    }
                }

            }

        }

/**
 * This method set the parkingState according to function call
 * @param state the state of parking procedure
 */

        void Park::setParkingState(int state) {
            parkingState = state;
        }

/**
 * This method takes an int i as sensor(ultrasonic & infrared) readings and int id as sensor type
 * then detects whether if it's within the range for obstacle
 * @param i the sensor reads data
 * @param id the sensor type
 * @return true for obstacle detected, false for not.
 */

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
                    if ((i > 2) && (i < 29)) {
                        ifObstacle = true;
                    } else {
                        ifObstacle = false;
                    }
                }
                    break;

                case (4) : {
                    if (i < 0 || i > 30) {
                        ifObstacle = true;
                    } else if ((i > 1) && (i < 30)) {
                        ifObstacle = false;
                    }
                }
                    break;
            }
            return ifObstacle;
        }

/**
 * This method takes a double start (parkinfg space) as the gap size and return a estimated path that the
 * car should travel while parking, the numbers 40 is the approximate best angle for the car to start turning
 * in order to get into the parking gap
 * car to turn into the gap.
 * @param start parking gap size
 * @return estimated parking travel distance
 */
        double Park::adjDistCalculation(double start) {

            /*     backDist = end - start;
                  turn degrees to radians
                  double cosVal = cos(backDist / (GAP / 2)); // cos value with the proximity angle
                  adjDist = abs(cosVal) * backDist;   // proximity value of the car traveled distance
                  (paralleled to the road)
            */

            adjDist = start / cos(40);
            return abs(adjDist);
        }
/**
 * Send the container, with all information/instrucctions to the conference
 */

        void Park::sendParkerMSG() {
            ParkerMSG p;
            p.setStateStop(0);
            Container c(p);
            getConference().send(c);
        }
    }
}

// automotive::miniature