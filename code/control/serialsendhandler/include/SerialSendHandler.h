#include <automotivedata/GeneratedHeaders_AutomotiveData.h>

#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>
#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>

#include "serial.h"

using namespace std;

using namespace odcore;
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::wrapper;

namespace scaledcars {
    namespace control {

        void __on_read(uint8_t b);
        void __on_write(uint8_t b);

        class SerialSendHandler :
            public odcore::base::module::TimeTriggeredConferenceClientModule {

            private:
                /**
                 * "Forbidden" copy constructor.
                 *
                 * Goal: The compiler should warn already at compile time
                 * for unwanted bugs caused by any misuse of the copy
                 * constructor.
                 *
                 * @param obj Reference to an object of this class.
                 */
                SerialSendHandler(const SerialSendHandler &/*obj*/);

                /**
                 * "Forbidden" assignment operator.
                 *
                 * Goal: The compiler should warn already at compile time
                 * for unwanted bugs caused by any misuse of the assignment
                 * operator.
                 *
                 * @param obj Reference to an object of this class.
                 * @return Reference to this instance.
                 */
                SerialSendHandler& operator=(const SerialSendHandler &/*obj*/);

                serial_state *serial;

            public:
                /**
                 * Constructor.
                 *
                 * @param argc Number of command line arguments.
                 * @param argv Command line arguments.
                 */
                SerialSendHandler(const int32_t &argc, char **argv);

                virtual ~SerialSendHandler();

                odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

                virtual void nextContainer(odcore::data::Container &c);

                int motor;
                int servo;

                int cycle = 0;

            private:

                virtual void setUp();

                virtual void tearDown();
                
                void filterData(protocol_data data, double *values, int *numbers);
                
                void sendSensorBoardData(protocol_data data);
                
                void sendVehicleData(protocol_data data);
        };
    }
}
