#include "SerialSendHandler.h"

#include <iostream>
#include <memory>
#include <stdint.h>
#include <string>

#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>

#include "automotivedata/generated/automotive/VehicleData.h"
#include "automotivedata/generated/automotive/miniature/SensorBoardData.h"

#include "protocol.c"
#include "serial.c"
#include "arduino.c"

#define pi 3.1415926535897

using namespace std;

using namespace odcore;
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::wrapper;
using namespace odcore::data::dmcp;
using namespace automotive;
using namespace automotive::miniature;

namespace scaledcars {
    namespace control {
        int port = 0;
        const string SERIAL_PORTS[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"};
        int BAUD_RATE = 115200;

        void __on_read(uint8_t b)
        {
            cout << ">> read " << (int)b << endl;
        }

        void __on_write(uint8_t b)
        {
            cout << "<< write " << (int)b << endl;
        }

        SerialSendHandler::SerialSendHandler(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "SerialSendHandler")
        {}

        SerialSendHandler::~SerialSendHandler() {}

        void SerialSendHandler::setUp() {
            try {
                cerr << "Setting up serial handler to port " << SERIAL_PORTS[port] << endl;

                this->serial = serial_new();

                this->serial->incoming_frame_t = FRAME_T2;
                this->serial->outgoing_frame_t = FRAME_T1;

                this->serial->on_write = &__on_write;
                this->serial->on_read = &__on_read;

                const char * _port = SERIAL_PORTS[port].c_str();
                serial_open(this->serial, _port, BAUD_RATE);
                cerr << "serial open" << endl;
                serial_handshake(this->serial, '\n');
                cerr << "serial handshake" << endl;

                const uint32_t ONE_SECOND = 1000 * 1000;
                odcore::base::Thread::usleepFor(2 * ONE_SECOND);

                serial_start(this->serial);
                cerr << "serial start" << endl;
            }catch (const char* msg){
                cerr << "Serial error : " << msg << endl;
                port++;
                if (port < 4) {
                    cerr << "Trying port : " << SERIAL_PORTS[port] << endl;
                    setUp();
                }
            }
        }

        void SerialSendHandler::tearDown() {
            cerr << "Shutting down serial handler" << endl;

            protocol_data d_motor;
            d_motor.id = ID_OUT_MOTOR;
            d_motor.value = 90 / 3;

            protocol_data d_servo;
            d_servo.id = ID_OUT_SERVO;
            d_servo.value = 90 / 3;
            serial_send(this->serial, d_motor);
            serial_send(this->serial, d_servo);

            serial_stop(this->serial);
            serial_free(this->serial);
        }

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode SerialSendHandler::body() {
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == ModuleStateMessage::RUNNING) {
                cout << "CYCLE " << cycle << endl;
                cycle++;

                protocol_data d_motor;
                d_motor.id = ID_OUT_MOTOR;
                d_motor.value = this->motor / 3;

                protocol_data d_servo;
                d_servo.id = ID_OUT_SERVO;
                d_servo.value = this->servo / 3;

                serial_send(this->serial, d_motor);
                serial_send(this->serial, d_servo);

                int pending = g_async_queue_length(this->serial->incoming_queue);
                protocol_data m_list[10]; // TODO fix this shit
                protocol_data incoming;
                for (int i = 0; i < pending; i++) {
                    if (serial_receive(this->serial, &incoming)) {
                        cerr << "RECEIVED : id=" << incoming.id << " value=" << incoming.value << endl;
                        m_list[i] = incoming;
                    }
                }
                // Send to filter
                filterData(m_list, pending);
            }

            return ModuleExitCodeMessage::OKAY;
        }

        void SerialSendHandler::nextContainer(Container &c) {
                cerr << "NEXT CONTAINER " << c.getDataType() << endl;
                if (c.getDataType() == automotive::VehicleControl::ID()) {
                    const automotive::VehicleControl vd =
                            c.getData<automotive::VehicleControl>();
                    int arduinoAngle = 0;
                    double angle = vd.getSteeringWheelAngle();
                    cerr << "angle radius : " << angle << endl;

                    arduinoAngle = 90 + (angle * (180 / pi));
                    if (arduinoAngle < 0) {
                        arduinoAngle = 0;
                    } else if(arduinoAngle > 180){
                        arduinoAngle = 180;
                    }
                    cerr << "angle degree " << arduinoAngle << endl;

                    int speed = vd.getSpeed();
                    cerr << "speed to arduino : " << speed << endl;

//                    // TODO: int odometer = vd.getOdometer();



//  TODO SEND
//                    string speedMessage = pack(ID_OUT_MOTOR, speed);
//                    string angleMessage = pack(ID_OUT_SERVO, arduinoAngle);
//                    TODO: string odometerMessage = pack(ID_OUT_ODOMETER, odometer);

                    this->motor = speed;
                    this->servo = arduinoAngle;

                }
        }
        
        /**
        * Devides the protocol_data according to what sensor it represents.
        *
        * Send as
        * 	SensorBoardData -> Ultrasonic sensor and IR-sensor
        * 	VehicleData	->	Odometer
        *
        * @param data to filter
        */
        void SerialSendHandler::filterData(protocol_data *list, int size){
				double us1, us2, ir3, ir4, ir5;
				int us1_size, us2_size, ir3_size, ir4_size, ir5_size;
				
				for(int i = 0; i < size; i++){
					//US-SENSOR [ID 1] [ID 2] with value between 1 - 70
        			if(list[i].id == 1 && list[i].value >= 1 && list[i].value <= 70){
        				us1 += list[i].value;
        				us1_size++;
        				
        			} else if (list[i].id == 2 && list[i].value >= 1 && list[i].value <= 70){
						us2 += list[i].value;
						us2_size++;
						
					//IR-SENSOR [ID 3] [ID 4] with value between 3 - 40
					} else if (list[i].id == 3 && list[i].value >= 3 && list[i].value <= 40){
						ir3 += list[i].value;
						ir3_size++;
					
					} else if (list[i].id == 4 && list[i].value >= 3 && list[i].value <= 40){
						ir4 += list[i].value;
						ir4_size++;
					
					} else if (list[i].id == 5 && list[i].value >= 3 && list[i].value <= 40){
						ir5 += list[i].value;
						ir5_size++;
							
					//ODOMETER [ID 6] with value between 0 - 255
					} else if (list[i].id == 6 && list[i].value >= 0 && list[i].value <= 255){ 
						sendVehicleData(list[i]);	
						
					} else {
						cerr << "[Filter no sensor] ID: " << list[i].id << " VALUE: " << list[i].value << endl;
        			}
				}
				
				protocol_data one, two, three, four, five;
				
				one.id = 1;
				one.value = us1/us1_size;
				
				two.id = 2;
				two.value = us2/us2_size;
				
				three.id = 3;
				three.value = ir3/ir3_size;
				
				four.id = 4;
				four.value = ir4/ir4_size;
				
				five.id = 5;
				five.value = ir5/ir5_size;
				
				sendSensorBoardData(one);
				sendSensorBoardData(two);
				sendSensorBoardData(three);
				sendSensorBoardData(four);
				sendSensorBoardData(five);
			}
        
        /**
      	* Pack a sensor id and a sensor value as a SensorBoardData.
      	* Then put the SensorBoardData into a Container and send the
      	* Container to the Conference.
      	*
      	* @param id and value of either a ultrasonic or ir-sensor
			*/        
        void SerialSendHandler::sendSensorBoardData(protocol_data data){
        		SensorBoardData sbd;
        		map<uint32_t, double> sensor;
        		
        		sensor[data.id] = data.value;
				sbd.setMapOfDistances(sensor);
				
				Container c(sbd);
				getConference().send(c);
				
				cout << "[SensorBoardData to conference] ID: " << data.id << " VALUE: " << data.value << endl;
        }
          
        /**
      	* Pack a sensor value as a VehicleData. Then put the VehicleData 
      	* into a Container and send the Container to the Conference.
      	*
      	* @param value of a odometer sensor
			*/      
        void SerialSendHandler::sendVehicleData(protocol_data data){
        		VehicleData vd;
        		
        		vd.setAbsTraveledPath(data.value);
        		
				Container c(vd);
				getConference().send(c);
				
				cout << "[VehicleData to conference] VALUE: " << data.value << endl;
        }
    }
}
