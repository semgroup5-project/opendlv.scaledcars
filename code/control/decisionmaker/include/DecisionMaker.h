#ifndef DECISIONMAKER_H_
#define DECISIONMAKER_H_

#include <automotivedata/GeneratedHeaders_AutomotiveData.h>

#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>
#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>
#include <opendavinci/odcore/io/StringListener.h>

#include "defines.h"

using namespace std;

using namespace odcore;
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::wrapper;

namespace scaledcars {
    namespace control {

        class UDPReceiveBytes : public odcore::io::StringListener {

            virtual void nextString(const string &s);
        };

        class DecisionMaker :
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
            DecisionMaker(const DecisionMaker &/*obj*/);

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
            DecisionMaker &operator=(const DecisionMaker &/*obj*/);

        public:
            /**
             * Constructor.
             *
             * @param argc Number of command line arguments.
             * @param argv Command line arguments.
             */
            DecisionMaker(const int32_t &argc, char **argv);

            virtual ~DecisionMaker();

            odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

            virtual void nextContainer(odcore::data::Container &c);

            int cycle = 0;

        private:

            virtual void setUp();

            virtual void tearDown();

            const string RECEIVER = "0.0.0.0";

            const uint32_t PORT = 1234;
        };
    }//control
}//scaledcars

#endif /*DECISIONMAKER_H_*/
