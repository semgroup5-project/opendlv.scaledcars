#ifndef OVERTAKER_H_
#define OVERTAKER_H_

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"

#include <iostream>
#include <stdint.h>
#include <memory>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opendavinci/odcore/base/KeyValueConfiguration.h>
#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/io/conference/ContainerConference.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>

#include <automotivedata/GeneratedHeaders_AutomotiveData.h>

#include "automotivedata/generated/automotive/VehicleData.h"
#include "automotivedata/generated/automotive/miniature/SensorBoardData.h"
#include "odvdscaledcarsdatamodel/generated/group5/CommunicationLinkMSG.h"
#include "odvdscaledcarsdatamodel/generated/group5/LaneFollowerMSG.h"

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace group5;

        class Overtaker : public odcore::base::module::TimeTriggeredConferenceClientModule {
            private:
                Overtaker(const Overtaker &/*obj*/);
                Overtaker& operator=(const Overtaker &/*obj*/);

            public:
                Overtaker(const int32_t &argc, char **argv);
                virtual ~Overtaker();
                odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

                automotive::VehicleControl m_vehicleControl;
                bool Sim;
                int _state;

            private:
                virtual void setUp();
                virtual void tearDown();

                void movingMachine(bool /*hasNextFrame*/);
                void measuringMachine();
        };

    }
}

#endif
