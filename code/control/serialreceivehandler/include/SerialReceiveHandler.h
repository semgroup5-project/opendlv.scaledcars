#ifndef SERIALRECEIVEHANDLER_H
#define SERIALRECEIVEHANDLER_H

#include <automotivedata/GeneratedHeaders_AutomotiveData.h>
#include "odvdscaledcarsdatamodel/generated/group5/SensorsMSG.h"

#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>
#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>

#include <opendavinci/odcore/base/Service.h>

#include <iostream>
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>
#include <cctype>
#include <algorithm>

#include "serial.h"

#include "defines.h"

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace odcore::wrapper;
        using namespace automotive;
        using namespace automotive::miniature;
        using namespace group5;

        void __on_read(uint8_t b);

        void __on_write(uint8_t b);

        class SerialReceiveHandler :
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
            SerialReceiveHandler(const SerialReceiveHandler &/*obj*/);

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
            SerialReceiveHandler &operator=(const SerialReceiveHandler &/*obj*/);

            serial_state *serial;

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

            void filterData(int id, int value);

            void sendSensorBoardData(std::map<uint32_t, double> sensor);
        };
    }
}
#endif
