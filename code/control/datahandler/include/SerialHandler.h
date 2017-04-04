#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>

class SerialHandler : public odcore::base::module::TimeTriggeredConferenceClientModule {
    private:
        /**
         * "Forbidden" copy constructor. Goal: The compiler should warn
         * already at compile time for unwanted bugs caused by any misuse
         * of the copy constructor.
         *
         * @param obj Reference to an object of this class.
         */
        SerialHandler(const SerialHandler &/*obj*/);

        /**
         * "Forbidden" assignment operator. Goal: The compiler should warn
         * already at compile time for unwanted bugs caused by any misuse
         * of the assignment operator.
         *
         * @param obj Reference to an object of this class.
         * @return Reference to this instance.
         */
        SerialHandler& operator=(const SerialHandler &/*obj*/);

    public:
        /**
         * Constructor.
         *
         * @param argc Number of command line arguments.
         * @param argv Command line arguments.
         */
        SerialHandler(const int32_t &argc, char **argv);

        virtual ~SerialHandler();

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

    private:
        virtual void setUp();

        virtual void tearDown();
};
