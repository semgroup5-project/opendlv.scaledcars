#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include "protocol.h"

namespace scaledcars {
	namespace control {

// This class will handle the bytes received via a serial link.
class SerialReceiveHandler : public odcore::base::module::TimeTriggeredConferenceClientModule {
	
		private:
        /**
         * "Forbidden" copy constructor. Goal: The compiler should warn
         * already at compile time for unwanted bugs caused by any misuse
         * of the copy constructor.
         *
         * @param obj Reference to an object of this class.
         */
        SerialReceiveHandler(const SerialReceiveHandler &/*obj*/);

        /**
         * "Forbidden" assignment operator. Goal: The compiler should warn
         * already at compile time for unwanted bugs caused by any misuse
         * of the assignment operator.
         *
         * @param obj Reference to an object of this class.
         * @return Reference to this instance.
         */
        SerialReceiveHandler& operator=(const SerialReceiveHandler &/*obj*/);

    public:
        /**
         * Constructor.
         *
         * @param argc Number of command line arguments.
         * @param argv Command line arguments.
         */
        SerialReceiveHandler(const int32_t &argc, char **argv);

        virtual ~SerialReceiveHandler();

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

    private:
    
        	virtual void setUp();

        	virtual void tearDown();
};

}
}