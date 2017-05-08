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
#include "Filter.h"

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
            TimeTriggeredConferenceClientModule(argc, argv, "SerialSendHandler"),
            serial(),
            motor(90),
            servo(90)
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

            const uint32_t ONE_SECOND = 1000 * 1000;
            odcore::base::Thread::usleepFor(5 * ONE_SECOND);

            serial_stop(this->serial);
            serial_free(this->serial);
        }

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode SerialSendHandler::body() {
           while (getModuleStateAndWaitForRemainingTimeInTimeslice() == ModuleStateMessage::RUNNING) {

               Container vehicleControlContainer = getKeyValueDataStore().get(automotive::VehicleControl::ID());
               if (vehicleControlContainer.getDataType() == automotive::VehicleControl::ID()) {
                   const automotive::VehicleControl vd =
                           vehicleControlContainer.getData<automotive::VehicleControl>();

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

                   this->motor = speed;
                   this->servo = arduinoAngle;
               }

                protocol_data d_motor;
                d_motor.id = ID_OUT_MOTOR;
                d_motor.value = this->motor / 3;

                protocol_data d_servo;
                d_servo.id = ID_OUT_SERVO;
                d_servo.value = this->servo / 3;

                serial_send(this->serial, d_motor);
                serial_send(this->serial, d_servo);

                int pending = g_async_queue_length(this->serial->incoming_queue);
                Filter filter;
                bool isSensorValues = false;
                protocol_data incoming;
                for (int i = 0; i < pending; i++) {
                    if (serial_receive(this->serial, &incoming)) {
                        cerr << "RECEIVED : id=" << incoming.id << " value=" << incoming.value << endl;
                        if(filter.isOdometer(incoming)){
                        	sendVehicleData(filter.handleOdometerValue(incoming));
                        } else {
                        	filter.filter(incoming);
                        	isSensorValues = true;
                        }
                        
                    }
                }
                
              	 if(isSensorValues){
                	map<uint32_t, double> sensor;
                	sensor = filter.normalize();
                	sendSensorBoardData(sensor);
                }
            }

            return ModuleExitCodeMessage::OKAY;
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
        void SerialSendHandler::sendVehicleData(double value){
        		VehicleData vd;
        		
        		vd.setAbsTraveledPath(value);
        		
				Container c(vd);
				getConference().send(c);
				
				//cout << "[VehicleData to conference] VALUE: " << odometerCounter << endl;
        }
    }
}
