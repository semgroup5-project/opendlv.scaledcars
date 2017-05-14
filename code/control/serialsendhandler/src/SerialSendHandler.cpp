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
        serial_state *serial_incoming_;
        serial_state *serial_outgoing_;
        SensorsMSG sbd;
        map<uint32_t, double> sensors;
        int realOdometer = 0;
        long counter = 0;
        bool isSensorValues = false;
        vector<int> ur_list_values;
        vector<int> ur2_list_values;
        vector<int> ir_side_front_list_values;
        vector<int> ir_side_back_list_values;
        vector<int> ir_back_list_values;
        const uint32_t ONE_SECOND = 1000 * 1000;

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
                int pending = g_async_queue_length(serial_incoming_->incoming_queue);
                protocol_data incoming;
                ur_list_values.clear();
                ur2_list_values.clear();
                ir_side_front_list_values.clear();
                ir_side_back_list_values.clear();
                ir_back_list_values.clear();
                for (int i = 0; i < pending; i++) {
                    if (serial_receive(serial_incoming_, &incoming)) {
                        cerr << "RECEIVED : id=" << incoming.id << " value=" << incoming.value << endl;
                        filterData(incoming.id, incoming.value);
                    }

                    if (ur_list_values.size() > 0) {
                        sort(ur_list_values.begin(), ur_list_values.end());
                        int med = (int) ur_list_values.size() / 2;
                        sensors[ID_IN_ULTRASONIC_CENTER] = ur_list_values[med];
                    } else {
                        sensors[ID_IN_ULTRASONIC_CENTER] = 0;
                    }
                    cout << "[SensorBoardData to conference] ID: " << ID_IN_ULTRASONIC_CENTER << " VALUE: "
                         << sensors[ID_IN_ULTRASONIC_CENTER] << endl;

                    if (ur2_list_values.size() > 0) {
                        sort(ur2_list_values.begin(), ur2_list_values.end());
                        int med = (int) ur2_list_values.size() / 2;
                        sensors[ID_IN_ULTRASONIC_SIDE_FRONT] = ur2_list_values[med];
                    } else {
                        sensors[ID_IN_ULTRASONIC_SIDE_FRONT] = 0;
                    }
                    cout << "[SensorBoardData to conference] ID: " << ID_IN_ULTRASONIC_SIDE_FRONT << " VALUE: "
                         << sensors[ID_IN_ULTRASONIC_SIDE_FRONT] << endl;


                    if (ir_side_front_list_values.size() > 0) {
                        sort(ir_side_front_list_values.begin(), ir_side_front_list_values.end());
                        int med = (int) ir_side_front_list_values.size() / 2;
                        sensors[ID_IN_INFRARED_SIDE_FRONT] = ir_side_front_list_values[med];
                    } else {
                        sensors[ID_IN_INFRARED_SIDE_FRONT] = 0;
                    }
                    cout << "[SensorBoardData to conference] ID: " << ID_IN_INFRARED_SIDE_FRONT << " VALUE: "
                         << sensors[ID_IN_INFRARED_SIDE_FRONT] << endl;

                    if (ir_side_back_list_values.size() > 0) {
                        sort(ir_side_back_list_values.begin(), ir_side_back_list_values.end());
                        int med = (int) ir_side_back_list_values.size() / 2;
                        sensors[ID_IN_INFRARED_SIDE_BACK] = ir_side_back_list_values[med];
                    } else {
                        sensors[ID_IN_INFRARED_SIDE_BACK] = 0;
                    }
                    cout << "[SensorBoardData to conference] ID: " << ID_IN_INFRARED_SIDE_BACK << " VALUE: "
                         << sensors[ID_IN_INFRARED_SIDE_BACK] << endl;


                    if (ir_back_list_values.size() > 0) {
                        sort(ir_back_list_values.begin(), ir_back_list_values.end());
                        int med = (int) ir_back_list_values.size() / 2;
                        sensors[ID_IN_INFRARED_BACK] = ir_back_list_values[med];
                    } else {
                        sensors[ID_IN_INFRARED_BACK] = 0;
                    }
                    cout << "[SensorBoardData to conference] ID: " << ID_IN_INFRARED_BACK << " VALUE: "
                         << sensors[ID_IN_INFRARED_BACK] << endl;

                    isSensorValues = true;
                }

                odcore::base::Thread::usleepFor(ONE_SECOND / 2);
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

            if ((id == ID_IN_ULTRASONIC_CENTER || id == ID_IN_ULTRASONIC_SIDE_FRONT) && value > 0) {
                if (id == ID_IN_ULTRASONIC_CENTER) {
                    ur_list_values.push_back(value);
                } else {
                    ur2_list_values.push_back(value);
                }

                //IR-SENSOR [ID 3] [ID 4] with value between 3 - 30
            } else if ((id == ID_IN_ULTRASONIC_CENTER || id == ID_IN_ULTRASONIC_SIDE_FRONT) && value == 0) {
                if (id == ID_IN_ULTRASONIC_CENTER) {
                    ur_list_values.push_back(-1);
                } else {
                    ur2_list_values.push_back(-1);
                }

                //IR-SENSOR [ID 3] [ID 4] with value between 3 - 40
            } else if (
                    (id == ID_IN_INFRARED_SIDE_FRONT || id == ID_IN_INFRARED_SIDE_BACK || id == ID_IN_INFRARED_BACK) &&
                    value > 2) {
                if (id == ID_IN_INFRARED_SIDE_FRONT) {
                    ir_side_front_list_values.push_back(value);
                } else if (id == ID_IN_INFRARED_SIDE_BACK) {
                    ir_side_back_list_values.push_back(value);
                } else {
                    ir_back_list_values.push_back(value);
                }

            } else if (
                    (id == ID_IN_INFRARED_SIDE_FRONT || id == ID_IN_INFRARED_SIDE_BACK || id == ID_IN_INFRARED_BACK) &&
                    value == 0) {
                if (id == ID_IN_INFRARED_SIDE_FRONT) {
                    ir_side_front_list_values.push_back(-1);
                } else if (id == ID_IN_INFRARED_SIDE_BACK) {
                    ir_side_back_list_values.push_back(-1);
                } else {
                    ir_back_list_values.push_back(-1);
                }

                //ODOMETER [ID 6] with value between 0 - 255
            } else if (id == ID_IN_ENCODER) {
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
                serial_incoming(),
                serial_outgoing(),
                motor(90),
                servo(90),
                arduinoStopAngle(90),
                arduinoBrake(190),
                arduinoAngle(90),
                speed(190),
                s() {}

        SerialSendHandler::~SerialSendHandler() {}

        void SerialSendHandler::setUp() {

            cerr << "Setting up serial handler to port " << endl;

            this->serial_incoming = serial_new();
            this->serial_outgoing = serial_new();

            this->serial_incoming->incoming_frame_t = FRAME_T2;
            this->serial_incoming->outgoing_frame_t = FRAME_T1;

            this->serial_outgoing->incoming_frame_t = FRAME_T2;
            this->serial_outgoing->outgoing_frame_t = FRAME_T1;

            this->serial_incoming->on_write = &__on_write;
            this->serial_incoming->on_read = &__on_read;

            this->serial_outgoing->on_write = &__on_write;
            this->serial_outgoing->on_read = &__on_read;

            //const char *_port = SERIAL_PORTS[port].c_str();
            serial_open(this->serial_incoming, "/dev/ttyACM1", BAUD_RATE);
            serial_open(this->serial_outgoing, "/dev/ttyACM0", BAUD_RATE);

            cerr << "serial open" << endl;
            //serial_handshake(this->serial_incoming, '\n');
            serial_handshake(this->serial_outgoing, '\n');
            cerr << "serial handshake" << endl;

            odcore::base::Thread::usleepFor(5 * ONE_SECOND);

            protocol_data d_motor;
            d_motor.id = ID_OUT_MOTOR;
            d_motor.value = 90 / 3;

            protocol_data d_servo;
            d_servo.id = ID_OUT_SERVO;
            d_servo.value = 90 / 3;
            serial_send(this->serial_outgoing, d_motor);
            serial_send(this->serial_outgoing, d_servo);

            odcore::base::Thread::usleepFor(2 * ONE_SECOND);

            serial_start(this->serial_incoming);
            serial_start(this->serial_outgoing);
            cerr << "serial start" << endl;

            serial_incoming_ = this->serial_incoming;
            serial_outgoing_ = this->serial_outgoing;
            s.start();
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
            serial_send(this->serial_outgoing, d_motor);
            serial_send(this->serial_outgoing, d_servo);

            odcore::base::Thread::usleepFor(15 * ONE_SECOND);

            serial_stop(this->serial_incoming);
            serial_free(this->serial_incoming);

            serial_stop(this->serial_outgoing);
            serial_free(this->serial_outgoing);
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

                serial_send(this->serial_outgoing, d_motor);

                protocol_data d_servo;
                d_servo.id = ID_OUT_SERVO;
                d_servo.value = this->servo / 3;

                serial_send(this->serial_outgoing, d_servo);

                if (isSensorValues) {
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
