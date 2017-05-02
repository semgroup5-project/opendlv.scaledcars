#ifndef DECISIONMAKER_H_
#define DECISIONMAKER_H_


#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>

#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>

#include <opendavinci/odcore/io/StringListener.h>
#include <opendavinci/odcore/io/udp/UDPReceiver.h>
#include <opendavinci/odcore/io/udp/UDPSender.h>
#include <opendavinci/odcore/io/udp/UDPFactory.h>

#include "automotivedata/generated/automotive/VehicleData.h"
#include "automotivedata/generated/automotive/miniature/SensorBoardData.h"
#include <automotivedata/GeneratedHeaders_AutomotiveData.h>
#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>
#include "odvdscaledcarsdatamodel/generated/group5/DecisionMakerMSG.h"

#include <iostream>
#include <memory>
#include <stdint.h>
#include <string>
#include <opendavinci/odcore/base/Thread.h>

#include <opendavinci/odcore/base/KeyValueConfiguration.h>

#include "defines.h"

using namespace std;

using namespace odcore;
using namespace odcore::io;
using namespace odcore::io::udp;
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::wrapper;

namespace scaledcars {
    namespace control {

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

            group5::DecisionMakerMSG decisionMakerMSG;

            automotive::VehicleData vd;

            automotive::miniature::SensorBoardData sbd;

            virtual void setUp();

            virtual void tearDown();

        };
    }//control
}//scaledcars

#endif /*DECISIONMAKER_H_*/
