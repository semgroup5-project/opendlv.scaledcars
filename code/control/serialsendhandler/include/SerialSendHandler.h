#ifndef SERIALSENDHANDLER_H
#define SERIALSENDHANDLER_H

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

        class SerialSendHandler :
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
            SerialSendHandler &operator=(const SerialSendHandler &/*obj*/);

            serial_state *serial;

        public:
            /**
             * Constructor.
             *
             * @param argc Number of command line arguments.
             * @param argv Command line arguments.
             */
            SerialSendHandler(const int32_t &argc, char **argv);

            virtual ~SerialSendHandler();

            odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

        private:

            virtual void setUp();

            virtual void tearDown();

            int motor;
            int servo;

            int arduinoStopAngle;
            int arduinoBrake;

            int arduinoAngle;
            int speed;
        };
    }
}
#endif
