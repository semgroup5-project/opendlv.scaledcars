#include <automotivedata/GeneratedHeaders_AutomotiveData.h>

#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>
#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>

using namespace std;

using namespace odcore;
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::wrapper;

namespace scaledcars {
    namespace control {
        class SerialReceiveListener : public odcore::io::StringListener {
            virtual void nextString(const string &s);
        };

        class SerialSendHandler : public DataTriggeredConferenceClientModule {
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

                shared_ptr<odcore::wrapper::SerialPort> serialPort;

                SerialReceiveListener serialListener;

            public:
                /**
                 * Constructor.
                 *
                 * @param argc Number of command line arguments.
                 * @param argv Command line arguments.
                 */
                SerialSendHandler(const int32_t &argc, char **argv);

                virtual ~SerialSendHandler();

                virtual void nextContainer(odcore::data::Container &c);

                virtual string pack(int id, int value);

                virtual void send(string Message);

            private:

                virtual void setUp();

                virtual void tearDown();
        };
    }
}
