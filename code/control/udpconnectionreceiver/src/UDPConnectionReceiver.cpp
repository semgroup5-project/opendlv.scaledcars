/**
 * UDPConnectionReceiver - UDPConnectionReceiver code.
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

#include "UDPConnectionReceiver.h"

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace odcore::data::image;
        using namespace automotive;
        using namespace automotive::miniature;
        using namespace odcore::wrapper;
        using namespace odcore;
        using namespace odcore::io;
        using namespace odcore::io::udp;
        using namespace odcore::io::conference;
        using namespace group5;
        using namespace odcore::data::dmcp;

        int notValid = 1;
        UDPMSG *_udpmsg;

        void UDPReceiveBytes::nextString(const string &s) {
            cout << "RECEIVED : " << s.length() << " bytes containing '" << s << "'" << endl;
            string received = decodedNetString(s).c_str();

            if (!(notValid = received.compare("move"))) {
                _udpmsg->setStateStop(0);
            } else if (!(notValid = received.compare("stop"))) {
                _udpmsg->setStateStop(1);
            } else if (!(notValid = received.compare("overtake"))) {
                _udpmsg->setStateFunctionOvertaker(1);
                _udpmsg->setStateFunctionParker(0);
            } else if (!(notValid = received.compare("parker"))) {
                _udpmsg->setStateFunctionOvertaker(0);
                _udpmsg->setStateFunctionParker(1);
            }
        }

        UDPConnectionReceiver::UDPConnectionReceiver(const int &argc, char **argv)
                : TimeTriggeredConferenceClientModule(argc, argv, "UDPConnectionReceiver"),
                  UDPMSG(),
                  udp_receiver() {}


        UDPConnectionReceiver::~UDPConnectionReceiver() {}

        void UDPConnectionReceiver::setUp() {
            cout << "Starting UDPConnectionReceiver" << endl;
            _udpmsg = &UDPMSG;
        }

        void UDPConnectionReceiver::tearDown() {
            cout << "Shutting down UDPConnectionReceiver" << endl;

            // Stop receiving bytes and unregister our handler.
            udp_receiver->stop();
            udp_receiver->setStringListener(NULL);
        }

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode UDPConnectionReceiver::body() {

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
                   odcore::data::dmcp::ModuleStateMessage::RUNNING) {

                const string RECEIVER = "0.0.0.0";
                const uint32_t PORT = 8888;

                try {
                    std::shared_ptr <UDPReceiver>
                            udpreceiver(UDPFactory::createUDPReceiver(RECEIVER, PORT));
                    udp_receiver = udpreceiver;
                    // This instance will handle any bytes that are received
                    // by our UDP socket.
                    UDPReceiveBytes handler;
                    udp_receiver->setStringListener(&handler);

                    // Start receiving bytes.
                    udp_receiver->start();

                    const uint32_t ONE_SECOND = 1000 * 1000;
                    odcore::base::Thread::usleepFor(ONE_SECOND);

                    // Stop receiving bytes and unregister our handler.
                    udp_receiver->stop();

                    udp_receiver->setStringListener(NULL);

                    if (!notValid) {
                        cout << "SENDING CONTAINER" << endl;
                        Container container(UDPMSG);
                        // Send container.
                        getConference().send(container);
                    }
                    notValid = 1;
                }
                catch (string &exception) {
                    cerr << "Error while creating UDP receiver: " << exception << endl;
                }
            }
            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    }
} // scaledcars::control

