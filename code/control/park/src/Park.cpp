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
        
        const int START = 0;
        const int RIGHT_TURN = 1;
        const int LEFT_TURN = 2;
        const int END = 3;

        Park::Park(const int32_t &argc, char **argv) :
                TimeTriggeredConferenceClientModule(argc, argv, "Park"),
                communicationLinkMSG(),
                vc(),
                parkingState(0),
                parkingCounter(0),
                parkingStart(0),
                isParking(false) {}

        Park::~Park() {}

        void Park::setUp() {
            

        }

        void Park::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        
        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Park::body() {
				Container communicationLinkMSGContainer = getKeyValueDataStore().get(CommunicationLinkMSG::ID());
            communicationLinkMSG = communicationLinkMSGContainer.getData<CommunicationLinkMSG>();
            

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
                   odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                   
                   
            	if(isParking){
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
        
        void Park::parkingFinder(){
        		// Parking space starting point
        		if((communicationLinkMSG.getInfraredSideBack() < 3 || communicationLinkMSG.getInfraredSideBack() > 10) && parkingStart == 0){
        			parkingStart = communicationLinkMSG.getWheelEncoder();
        			vc.setSpeed(70);
        			vc.setSteeringWheelAngle(0);
        			cout << "PARKING : Here starts freedom" << endl;
        		}
        		
        		// Gap is too narrow
        		if(communicationLinkMSG.getInfraredSideBack() <= 10 && communicationLinkMSG.getInfraredSideFront() <= 10 
        			&& parkingStart > 0 && (communicationLinkMSG.getWheelEncoder() - parkingStart) < 100){
        			parkingStart = 0;
        			isParking = false;
        			vc.setSpeed(70);
        			vc.setSteeringWheelAngle(0);
        			cout << "PARKING : No freedom" << endl;
        		}
        		
        		// Gap is sufficient
        		if(parkingStart > 0 && (communicationLinkMSG.getWheelEncoder() - parkingStart) >= 100){
        			isParking = true;
        			//sendParkerMSG();
        			vc.setBrakeLights(true);
        			cout << "PARKING : Insertion time" << endl;
        		}
        		
        }
        
        void Park::parallelPark(){
        		switch(parkingState){
        			case START: {
        				setParkingState(RIGHT_TURN);
        				vc.setBrakeLights(false);
        			}
        			break;
        			
        			case RIGHT_TURN:{
        				vc.setSpeed(70);
               	vc.setSteeringWheelAngle(1.5);
               	parkingCounter++;
               	cout << "PARKING : Turning right" << endl;
        			
        				if(parkingCounter == 10){
        					setParkingState(LEFT_TURN);
        				}
        			}
        			break;
        		
        			case LEFT_TURN:{
        				vc.setSpeed(70);
               	vc.setSteeringWheelAngle(-1.5);
        				parkingCounter++;
        				cout << "PARKING : Turning left" << endl;
        			
        				if(parkingCounter == 20){
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
        
        void Park::setParkingState(int state){
        		parkingState = state;
        }
        
        void Park::sendParkerMSG(){
        		ParkerMSG p;
        		Container c(p);
        		getConference().send(c);
        }
    }
} // automotive::miniature

