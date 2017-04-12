#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
namespace scaledcars {
	namespace control {
	
class SerialSendHandler : public odcore::base::module::DataTriggeredConferenceClientModule {
    private:
        /**
         * "Forbidden" copy constructor. Goal: The compiler should warn
         * already at compile time for unwanted bugs caused by any misuse
         * of the copy constructor.
         *
         * @param obj Reference to an object of this class.
         */
        SerialSendHandler(const SerialSendHandler &/*obj*/);

        /**
         * "Forbidden" assignment operator. Goal: The compiler should warn
         * already at compile time for unwanted bugs caused by any misuse
         * of the assignment operator.
         *
         * @param obj Reference to an object of this class.
         * @return Reference to this instance.
         */
        SerialSendHandler& operator=(const SerialSendHandler &/*obj*/);

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

    private:
        virtual void setUp();

        virtual void tearDown();
};
}
}
