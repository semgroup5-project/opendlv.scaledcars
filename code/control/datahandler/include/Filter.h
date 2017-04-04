#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>

class Filter : public odcore::base::module::DataTriggeredConferenceClientModule {
    private:
        /**
         * "Forbidden" copy constructor. Goal: The compiler should warn
         * already at compile time for unwanted bugs caused by any misuse
         * of the copy constructor.
         *
         * @param obj Reference to an object of this class.
         */
        Filter(const Filter &/*obj*/);

        /**
         * "Forbidden" assignment operator. Goal: The compiler should warn
         * already at compile time for unwanted bugs caused by any misuse
         * of the assignment operator.
         *
         * @param obj Reference to an object of this class.
         * @return Reference to this instance.
         */
        Filter& operator=(const DataTriggeredReceiver &/*obj*/);

    public:
        /**
         * Constructor.
         *
         * @param argc Number of command line arguments.
         * @param argv Command line arguments.
         */
        Filter(const int32_t &argc, char **argv);

        virtual ~Filter();

        virtual void nextContainer(odcore::data::Container &c);

    private:
        virtual void setUp();

        virtual void tearDown();
};
