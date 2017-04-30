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
        
        double odometerCounter;
        double odometerOldValue;

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
                double valuesToNormalize[5];
                int numbers[5];
                bool isSensorValues = false;
                protocol_data incoming;
                for (int i = 0; i < pending; i++) {
                    if (serial_receive(this->serial, &incoming)) {
                        cerr << "RECEIVED : id=" << incoming.id << " value=" << incoming.value << endl;
                        filterData(incoming, valuesToNormalize, numbers);
                        isSensorValues = true;
                    }
                }
                
              	 if(isSensorValues){
                	map<uint32_t, double> sensor;
                	for(int i = 0; i < 5; i++){
                		protocol_data d;
                		d.id = i+1;
                		if(numbers[i] != 0){
                			d.value = valuesToNormalize[i] / numbers[i];
                		} else {
                			d.value = valuesToNormalize[i];
                		}
                		sensor[d.id] = d.value;
                		cout << "[SensorBoardData to conference] ID: " << d.id << " VALUE: " << d.value << endl;
                	}
                	sendSensorBoardData(sensor);
                }
                
                
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
        * Filters the data according to what sensor it represents and that sensors ranges.
        * Ultrasonic and IR-sensor values are added to the "values", incrementing the "numbers".
        * Every odometer value is passed forward for packing and sending.
        *
        * @param data to filter
        */
        void SerialSendHandler::filterData(protocol_data data, double *values, int *numbers){
				
					//US-SENSOR [ID 1] [ID 2] with value between 1 - 70
        			if((data.id == 1 || data.id == 2) && data.value >= 1 && data.value <= 70){
        				values[data.id - 1] += data.value;
        				numbers[data.id - 1] += 1;
        				cout << "filter " << data.id << "  " << data.value << endl;
        				
					//IR-SENSOR [ID 3] [ID 4] with value between 3 - 40
					} else if ((data.id == 3 || data.id == 4 || data.id == 5) && data.value >= 3 && data.value <= 40){
						values[data.id - 1] += data.value;
        				numbers[data.id - 1] += 1;
        				cout << "filter " << data.id << "  " << data.value << endl;
							
					//ODOMETER [ID 6] with value between 0 - 255
					} else if (data.id == 6 && data.value >= 0 && data.value <= 255){ 
						
						if((int)odometerOldValue > data.value){
							odometerCounter += (odometerOldValue - data.value);
						} else {
							odometerCounter += data.value;
						}
						odometerOldValue = data.value;
						sendVehicleData();
						//cout << "filter " << data.id << "  " << odometerCounter << endl;	
						
					} else {
						cerr << "[Filter no sensor] ID: " << data.id << " VALUE: " << data.value << endl;
        			}
			}
        
        /**
      	* Pack a map of sensor values ad SensorBoardData.
      	* Then put the SensorBoardData into a Container and send the
      	* Container to the Conference.
      	*
      	* @param a map of sensor data from every ultrasonic and ir-sensor
			*/        
        void SerialSendHandler::sendSensorBoardData(map<uint32_t, double> sensor){
        		SensorBoardData sbd;
        		
				sbd.setMapOfDistances(sensor);
				
				Container c(sbd);
				getConference().send(c);
				
        }
          
        /**
      	* Pack the odometerCounter value as a VehicleData. Then put the VehicleData 
      	* into a Container and send the Container to the Conference.
			*/      
        void SerialSendHandler::sendVehicleData(){
        		VehicleData vd;
        		
        		vd.setAbsTraveledPath(odometerCounter);
        		
				Container c(vd);
				getConference().send(c);
				
				cout << "[VehicleData to conference] VALUE: " << odometerCounter << endl;
        }
    }
}
