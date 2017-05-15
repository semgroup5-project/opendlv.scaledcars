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
        const string SERIAL_PORTS[] = {"/dev/ttyACM0"};
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
                TimeTriggeredConferenceClientModule(argc, argv, "SerialSendHandler"),
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
                cerr << "serial start" << endl;

                serial_ = this->serial;
            } catch (const char *msg) {
                cerr << "Serial error : " << msg << endl;
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
            }

            return ModuleExitCodeMessage::OKAY;
        }
    }
}
