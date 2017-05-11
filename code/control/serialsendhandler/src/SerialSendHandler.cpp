#include "SerialSendHandler.h"
#include "protocol.c"
#include "serial.c"
#include "arduino.c"

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace odcore::wrapper;
        using namespace odcore::data::dmcp;
        using namespace automotive;
        using namespace automotive::miniature;

        int port = 0;
        const string SERIAL_PORTS[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"};
        int BAUD_RATE = 115200;
        serial_state *serial_;
        SensorsMSG sbd;
        map<uint32_t, double> sensors;
        int realOdometer = 0;
        long counter = 0;
        bool isSensorValues = false;
        int count_values[] = {0, 0, 0, 0, 0};
        int _values[] = {0, 0, 0, 0, 0};

        // Your class needs to implement the method void beforeStop().
        void MyService::beforeStop() {
            // This block is executed right before
            // the thread will be stopped.
            cout << "This method is called right before "
                 << "isRunning will return false." << endl;
        }

        // Your class needs to implement the method void run().
        void MyService::run() {
            // Here, you can do some initialization of
            // resources (e.g. data structures and the like).
            cout << "Starting listener thread. . . ." << endl;

            serviceReady();

            // This is the body of the concurrently executed method.
            while (isRunning()) {
                cout << "This message is printed every second." << endl;
                int pending = g_async_queue_length(serial_->incoming_queue);
                protocol_data incoming;
                for (int j = 0; j < 5; ++j) {
                    count_values[j] = 0;
                    _values[j] = 0;
                }
                for (int i = 0; i < pending; i++) {
                    incoming.value = 0;
                    if (serial_receive(serial_, &incoming)) {
                        cerr << "RECEIVED : id=" << incoming.id << " value=" << incoming.value << endl;
                        filterData(incoming.id, incoming.value);
                    }
                    for (int j = 0; j < 5; ++j) {
                        sensors[j + 1] = _values[j];
                    }
                    isSensorValues = true;
                }

                const uint32_t ONE_SECOND = 1000 * 1000;
                odcore::base::Thread::usleepFor(ONE_SECOND/2);
            }
        }

        /**
       * Filters the data according to what sensor it represents and that sensors ranges.
       * Ultrasonic and IR-sensor values are added to the "values", incrementing the "numbers".
       * Every odometer value is passed forward for packing and sending.
       *
       * @param data to filter
       */
        void MyService::filterData(int id, int value) {

            //US-SENSOR [ID 1] [ID 2] with value between 1 - 70
            if ((id == 1 || id == 2) && value > 0) {
                if (sensors[id] > -1) {
                    _values[id-1] += value;
                } else {
                    _values[id-1]= value;
                }
                count_values[id-1] += 1;

                //IR-SENSOR [ID 3] [ID 4] with value between 3 - 30
            } else if ((id == 1 || id == 2) && value == 0) {
                _values[id-1] = -1;
                cout << "[SensorBoardData to conference] ID: " << id << " VALUE: " << -1 << endl;

                //IR-SENSOR [ID 3] [ID 4] with value between 3 - 40
            } else if ((id == 3 || id == 4 || id == 5) && value > 2) {
                if (sensors[id] > -1) {
                    _values[id-1] += value;
                } else {
                    _values[id-1] = value;
                }
                count_values[id-1] += 1;

                //ODOMETER [ID 6] with value between 0 - 255
            } else if ((id == 3 || id == 4 || id == 5) && value == 0) {
                _values[id-1] = -1;
                cout << "[SensorBoardData to conference] ID: " << id << " VALUE: " << -1 << endl;

                //ODOMETER [ID 6] with value between 0 - 255
            } else if (id == 6 && (value >= 0 && value <= 255)) {
                cout << "ODOMETER VALUE IS : " << value << endl;
                realOdometer += value;
                cout << "ODOMETER REAL IS : " << realOdometer << endl;

                if (realOdometer >= KM_IN_CM) {
                    realOdometer -= KM_IN_CM;
                    counter++;
                    cout << "LOOPED +1 KM! " << endl;
                }

                cout << "[VehicleData to conference] VALUE: " << realOdometer << endl;
                sbd.setTravelledDistance(realOdometer);
                sbd.setTravelledKm(counter);
            } else {
                cerr << "[Filter no sensor] ID: " << id << " VALUE: " << value << endl;
            }
        }

        void __on_read(uint8_t b) {
            cout << ">> read " << (int) b << endl;
        }

        void __on_write(uint8_t b) {
            cout << "<< write " << (int) b << endl;
        }

        SerialSendHandler::SerialSendHandler(const int32_t &argc, char **argv) :
                TimeTriggeredConferenceClientModule(argc, argv, "SerialSendHandler"),
                serial(),
                motor(90),
                servo(90),
                arduinoStopAngle(90),
                arduinoBrake(190),
                arduinoAngle(90),
                speed(190),
                s() {}

        SerialSendHandler::~SerialSendHandler() {}

        void SerialSendHandler::setUp() {
            try {
                cerr << "Setting up serial handler to port " << SERIAL_PORTS[port] << endl;

                this->serial = serial_new();

                this->serial->incoming_frame_t = FRAME_T2;
                this->serial->outgoing_frame_t = FRAME_T1;

                this->serial->on_write = &__on_write;
                this->serial->on_read = &__on_read;

                const char *_port = SERIAL_PORTS[port].c_str();
                serial_open(this->serial, _port, BAUD_RATE);

                cerr << "serial open" << endl;
                serial_handshake(this->serial, '\n');
                cerr << "serial handshake" << endl;

                const uint32_t ONE_SECOND = 1000 * 1000;
                odcore::base::Thread::usleepFor(5 * ONE_SECOND);

                protocol_data d_motor;
                d_motor.id = ID_OUT_MOTOR;
                d_motor.value = 90 / 3;

                protocol_data d_servo;
                d_servo.id = ID_OUT_SERVO;
                d_servo.value = 90 / 3;
                serial_send(this->serial, d_motor);
                serial_send(this->serial, d_servo);

                const uint32_t ONE_SECOND = 1000 * 1000;
                odcore::base::Thread::usleepFor(2 * ONE_SECOND);

                serial_start(this->serial);
                cerr << "serial start" << endl;

                serial_ = this->serial;
                s.start();
            } catch (const char *msg) {
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

            s.stop();

            protocol_data d_motor;
            d_motor.id = ID_OUT_MOTOR;
            d_motor.value = 90 / 3;

            protocol_data d_servo;
            d_servo.id = ID_OUT_SERVO;
            d_servo.value = 90 / 3;
            serial_send(this->serial, d_motor);
            serial_send(this->serial, d_servo);

            const uint32_t ONE_SECOND = 1000 * 1000;
            odcore::base::Thread::usleepFor(15 * ONE_SECOND);

            serial_stop(this->serial);
            serial_free(this->serial);
        }

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode SerialSendHandler::body() {
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == ModuleStateMessage::RUNNING) {

                Container vehicleControlContainer = getKeyValueDataStore().get(automotive::VehicleControl::ID());
                if (vehicleControlContainer.getDataType() == automotive::VehicleControl::ID()) {
                    const automotive::VehicleControl vc =
                            vehicleControlContainer.getData<automotive::VehicleControl>();

                    cerr << "BRAKE LIGHTS " << vc.getBrakeLights() << endl;
                    if (!vc.getBrakeLights()) {
                        double angle = vc.getSteeringWheelAngle();
                        cerr << "angle radius : " << angle << endl;

                        arduinoAngle = 90 + (angle * (180 / PI));
                        if (arduinoAngle < 0) {
                            arduinoAngle = 0;
                        } else if (arduinoAngle > 180) {
                            arduinoAngle = 180;
                        }

                        speed = vc.getSpeed();

                        cerr << "angle degree " << arduinoAngle << endl;
                        cerr << "speed to arduino : " << speed << endl;

                        this->motor = speed;
                        this->servo = arduinoAngle;

                    } else {
                        cerr << "Brake signal sent..." << endl;
                        this->motor = arduinoBrake;
                        this->servo = arduinoStopAngle;
                    }
                }


                protocol_data d_motor;
                d_motor.id = ID_OUT_MOTOR;
                d_motor.value = this->motor / 3;

                serial_send(this->serial, d_motor);

                protocol_data d_servo;
                d_servo.id = ID_OUT_SERVO;
                d_servo.value = this->servo / 3;

                serial_send(this->serial, d_servo);

                if (isSensorValues) {
                    for (int i = 1; i < 6; ++i) {
                       if (sensors[i] > 0) {
                            cerr << "Normalizing ID:" << i << " value total:" << sensors[i] << " divided by:" << count_values[i-1] << endl;
                            sensors[i] /= count_values[i-1];
                           cout << "[SensorBoardData to conference] ID: " << i << " VALUE: " << sensors[i] << " RECEIVED: " << count_values[i-1] << " TIMES" << endl;

                       }
                    }
                    sendSensorBoardData(sensors);
                }

//                const uint32_t ONE_SECOND = 1000 * 1000;
//                odcore::base::Thread::usleepFor(2 * ONE_SECOND);
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
        void SerialSendHandler::sendSensorBoardData(map<uint32_t, double> sensor) {
            sbd.setMapOfDistances(sensor);
            Container c(sbd);
            getConference().send(c);
            isSensorValues = false;
        }
    }
}
