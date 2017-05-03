/**
 * Example - Example code.
 * Copyright (C) 2016 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef UDPCONNECTION_H
#define UDPCONNECTION_H

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
#include "odvdscaledcarsdatamodel/generated/group5/CommunicationLinkMSG.h"
#include "odvdscaledcarsdatamodel/generated/group5/LaneFollowerMSG.h"
#include "odvdscaledcarsdatamodel/generated/group5/OvertakerMSG.h"
#include "odvdscaledcarsdatamodel/generated/group5/ParkerMSG.h"

#include <iostream>
#include <memory>
#include <stdint.h>
#include <string>
#include <opendavinci/odcore/base/Thread.h>

#include <opendavinci/odcore/base/KeyValueConfiguration.h>

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore;
        using namespace odcore::io;
        using namespace odcore::io::udp;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace odcore::wrapper;

        class UDPReceiveBytes : public odcore::io::StringListener {

            virtual void nextString(const std::string &s);
        };

        class UDPConnection : public odcore::base::module::TimeTriggeredConferenceClientModule {

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
            UDPConnection(const UDPConnection &/*obj*/);

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
            UDPConnection &operator=(const UDPConnection &/*obj*/);

        public:
            /**
             * Constructor.
             *
             * @param argc Number of command line arguments.
             * @param argv Command line arguments.
             */
            UDPConnection(const int &argc, char **argv);

            virtual ~UDPConnection();

        private:
            void setUp();

            void tearDown();

            odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

            shared_ptr <UDPReceiver> udp_receiver;

        };
    }
} // scaledcars::control

#endif /*UDPCONNECTION_H*/