#ifndef COMMUNICATIONLINK_H_
#define COMMUNICATIONLINK_H_


#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>

#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>

#include "automotivedata/generated/automotive/VehicleData.h"
#include "automotivedata/generated/automotive/miniature/SensorBoardData.h"
#include <automotivedata/GeneratedHeaders_AutomotiveData.h>
#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>
#include "odvdscaledcarsdatamodel/generated/group5/CommunicationLinkMSG.h"
#include "odvdscaledcarsdatamodel/generated/group5/LaneFollowerMSG.h"
#include "odvdscaledcarsdatamodel/generated/group5/OvertakerMSG.h"
#include "odvdscaledcarsdatamodel/generated/group5/ParkerMSG.h"

#include "odvdscaledcarsdatamodel/generated/group5/SensorsMSG.h"

#include <iostream>
#include <memory>
#include <stdint.h>
#include <string>
#include <opendavinci/odcore/base/Thread.h>

#include <opendavinci/odcore/base/KeyValueConfiguration.h>

#include "defines.h"

using namespace std;

using namespace odcore;
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::wrapper;

namespace scaledcars {
    namespace control {

        class CommunicationLink :
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
            CommunicationLink(const CommunicationLink &/*obj*/);

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
            CommunicationLink &operator=(const CommunicationLink &/*obj*/);

        public:
            /**
             * Constructor.
             *
             * @param argc Number of command line arguments.
             * @param argv Command line arguments.
             */
            CommunicationLink(const int32_t &argc, char **argv);

            virtual ~CommunicationLink();

        private:
            group5::CommunicationLinkMSG communicationLinkMSG;
            group5::LaneFollowerMSG laneFollowerMSG;
            group5::OvertakerMSG overtakerMSG;
            group5::ParkerMSG parkerMSG;
            group5::SensorsMSG sensorsMSG;

            virtual void setUp();

            virtual void tearDown();

            odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

        };
    }//control
}//scaledcars

#endif /*COMMUNICATIONLINK_H_*/
