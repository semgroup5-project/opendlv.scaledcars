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
                protocol_data incoming;
                for (int i = 0; i < pending; i++) {
                    if (serial_receive(this->serial, &incoming)) {
                        cerr << "RECEIVED : id=" << incoming.id << " value=" << incoming.value << endl;
                        filterData(incoming);
                    }
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
        
        /*
        * Devides the protocol_data according to what sensor it represents.
        *
        * Send as
        * 	SensorBoardData -> Ultrasonic sensor and IR-sensor
        * 	VehicleData	->	Odometer
        */
        void SerialSendHandler::filterData(protocol_data data){
				
				//US-SENSOR [ID 1] [ID 2] with value between 1 - 70
        		if((data.id == 1 || data.id == 2) && data.value >= 1 && data.value <= 70){
        			sendSensorBoardData(data.id, data.value);
						
				//IR-SENSOR [ID 3] [ID 4] with value between 3 - 40
				} else if ((data.id == 3 || data.id == 4 || data.id == 5) && data.value >= 3 && data.value <= 40){
					sendSensorBoardData(data.id, data.value);
							
				//ODOMETER [ID 6] with value between 0 - 255
				} else if (data.id == 6 && data.value >= 0 && data.value <= 255){ 
					sendVehicleData(data.value);	
				}
        }
        
        /*
      	*	Pack a sensor id and a sensor value as a SensorBoardData.
      	*	Then put the SensorBoardData into a Container and send the
      	*	Container to the Conference.
			*/        
        void SerialSendHandler::sendSensorBoardData(int id, int value){
        		SensorBoardData sbd;
        		map<uint32_t, double> sensor;
        		sensor[id - 1] = value;
				sbd.setMapOfDistances(sensor);
				Container c(sbd);
				getConference().send(c);
				cerr << "[SensorBoardData to conference] ID: " << id << " VALUE: " << value << endl;
        }
          
        /*
      	*	Pack a sensor value as a VehicleData. Then put the VehicleData 
      	*	into a Container and send the Container to the Conference.
			*/      
        void SerialSendHandler::sendVehicleData(int value){
        		VehicleData vd;
        		vd.setAbsTraveledPath(value);
				Container c(vd);
				getConference().send(c);
				cerr << "[VehicleData to conference] VALUE: " << value << endl;
        }
    }
}
