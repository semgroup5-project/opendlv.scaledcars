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
        using namespace odcore::base;

        string SERIAL_PORT = "";
        int BAUD_RATE = 115200;
        serial_state *serial_;
        const uint32_t ONE_SECOND = 1000 * 1000;

        void __on_read(uint8_t b) {
            cout << ">> read " << (int) b << endl;
        }

        void __on_write(uint8_t b) {
            cout << "<< write " << (int) b << endl;
        }

        SerialSendHandler::SerialSendHandler(const int32_t &argc, char **argv) :
                DataTriggeredConferenceClientModule(argc, argv, "SerialSendHandler"),
                serial(),
                motor(90),
                servo(90),
                arduinoStopAngle(90),
                arduinoBrake(190),
                arduinoAngle(90),
                speed(190) {}

        SerialSendHandler::~SerialSendHandler() {}

        void SerialSendHandler::setUp() {
            try {
                KeyValueConfiguration kv = getKeyValueConfiguration();

                SERIAL_PORT =  kv.getValue<string>("serialsendhandler.actuators");

                const string _S_PORT = SERIAL_PORT;

                cerr << "Setting up serial handler to port " << SERIAL_PORT << endl;

                this->serial = serial_new();

                this->serial->incoming_frame_t = FRAME_T2;
                this->serial->outgoing_frame_t = FRAME_T1;

                this->serial->on_write = &__on_write;
                this->serial->on_read = &__on_read;

                const char *_port = _S_PORT.c_str();
                serial_open(this->serial, _port, BAUD_RATE);

                serial_handshake(this->serial, '\n');

                odcore::base::Thread::usleepFor(5 * ONE_SECOND);

                protocol_data d_motor;
                d_motor.id = ID_OUT_MOTOR;
                d_motor.value = 90 / 3;

                protocol_data d_servo;
                d_servo.id = ID_OUT_SERVO;
                d_servo.value = 90 / 3;
                serial_send(this->serial, d_motor);
                serial_send(this->serial, d_servo);

                odcore::base::Thread::usleepFor(2 * ONE_SECOND);

                serial_start(this->serial);

                serial_ = this->serial;
            } catch (const char *msg) {
                cerr << "Serial error : " << msg << endl;
            }
        }

        void SerialSendHandler::tearDown() {

            protocol_data d_motor;
            d_motor.id = ID_OUT_MOTOR;
            d_motor.value = 90 / 3;

            protocol_data d_servo;
            d_servo.id = ID_OUT_SERVO;
            d_servo.value = 90 / 3;
            serial_send(this->serial, d_motor);
            serial_send(this->serial, d_servo);

            odcore::base::Thread::usleepFor(15 * ONE_SECOND);

            serial_stop(this->serial);
            serial_free(this->serial);
        }

        void SerialSendHandler::nextContainer(Container &c) {
            if (c.getDataType() == automotive::VehicleControl::ID()) {
                const automotive::VehicleControl vc =
                        c.getData<automotive::VehicleControl>();

                if (!vc.getBrakeLights()) {
                    double angle = vc.getSteeringWheelAngle();

                    arduinoAngle = 90 + (angle * (180 / PI));
                    if (arduinoAngle < 0) {
                        arduinoAngle = 0;
                    } else if (arduinoAngle > 180) {
                        arduinoAngle = 180;
                    }

                    speed = vc.getSpeed();

                    this->motor = speed;
                    this->servo = arduinoAngle;

                } else {
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

        }
    }
}
