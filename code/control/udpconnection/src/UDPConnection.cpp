/**
 * UDPConnection - UDPConnection code.
 * Copyright (C) 2017 Raphael
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

#include "UDPConnection.h"

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odcore::data::image;
        using namespace automotive;
        using namespace automotive::miniature;
        using namespace odcore::wrapper;
        using namespace odcore;
        using namespace odcore::io;
        using namespace odcore::io::udp;

        void UDPReceiveBytes::nextString(const string &s) {
            cout << "RECEIVED : " << s.length() << " bytes containing >>>>>>>>>>>>>>>>>>>>>'" << s << "'<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
        }

        UDPConnection::UDPConnection(const int &argc, char **argv)
                : TimeTriggeredConferenceClientModule(argc, argv, "UDPConnection"),
                  udp_receiver(){}


        UDPConnection::~UDPConnection() {}

        void UDPConnection::setUp() {
            cout << "Starting UDPConnection" << endl;
        }

        void UDPConnection::tearDown() {
            cout << "Shutting down UDPConnection" << endl;
            // Stop receiving bytes and unregister our handler.
            udp_receiver->stop();
            udp_receiver->setStringListener(NULL);
        }

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode UDPConnection::body() {

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
                   odcore::data::dmcp::ModuleStateMessage::RUNNING) {

                const string RECEIVER = "0.0.0.0";
                const uint32_t PORT = 8888;

                try {
                    std::shared_ptr<UDPReceiver>
                            udpreceiver(UDPFactory::createUDPReceiver(RECEIVER, PORT));
                    udp_receiver = udpreceiver;
                    // This instance will handle any bytes that are received
                    // by our UDP socket.
                    UDPReceiveBytes handler;
                    udp_receiver->setStringListener(&handler);

                    // Start receiving bytes.
                    udp_receiver->start();

                    const uint32_t ONE_SECOND = 1000 * 1000;
                    odcore::base::Thread::usleepFor(10 * ONE_SECOND);

                    // Stop receiving bytes and unregister our handler.
                    udp_receiver->stop();
                    udp_receiver->setStringListener(NULL);
                }
                catch(string &exception) {
                    cerr << "Error while creating UDP receiver: " << exception << endl;
                }
            }
            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    }
} // scaledcars::control

