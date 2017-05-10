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

#include "UDPConnectionStreamer.h"

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace odcore::data::image;
        using namespace odcore::wrapper;
        using namespace odcore;
        using namespace odcore::io;
        using namespace odcore::io::udp;
        using namespace odcore::data::dmcp;
        using namespace cv;

        string imageName = "";

        UDPConnectionStreamer::UDPConnectionStreamer(const int &argc, char **argv)
                : DataTriggeredConferenceClientModule(argc, argv, "UDPConnectionStreamer"),
                  m_sharedImageMemory(),
                  m_sharedProcessedImageMemory(),
                  m_sharedProcessedImage(),
                  m_hasAttachedToSharedImageMemory(false),
                  m_image(),
                  m_image_mat(),
                  m_image_new(),
                  m_threshold1(50),  //50
                  m_threshold2(200) {}  // 150


        UDPConnectionStreamer::~UDPConnectionStreamer() {}

        void UDPConnectionStreamer::setUp() {
            cout << "Starting UDPConnectionStreamer" << endl;
            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();

//            emergencyStop = kv.getValue<int32_t>("communicationlink.functionlane");
//
//            changeFunction =kv.getValue<int32_t>("communicationlink.function2");
        }

        void UDPConnectionStreamer::tearDown() {
            cout << "Shutting down UDPConnectionStreamer" << endl;
            if (!m_image.empty()) {
                m_image.deallocate();
            }
        }

        // This method returns a boolean true if it gets an image from the shared image memory
        bool UDPConnectionStreamer::readSharedImage(Container &c) {
            bool retVal = false;

            if (c.getDataType() == SharedImage::ID()) {
                SharedImage si = c.getData<SharedImage>();

                if ((1 == si.getBytesPerPixel()) ||
                    (3 == si.getBytesPerPixel())) {

                    int compressedSize = si.getWidth() * si.getHeight() * si.getBytesPerPixel();
                    void *buffer = ::malloc(compressedSize);
                    if (buffer != NULL) {
                        // As we are transforming a SharedImage into a CompressedImage, attached to the shared memory segment.
                        std::shared_ptr <odcore::wrapper::SharedMemory> memory = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(
                                si.getName());
                        if (memory->isValid()) {
                            Lock l(memory);
                            retVal = odcore::wrapper::jpg::JPG::compress(buffer, compressedSize, si.getWidth(),
                                                                         si.getHeight(), si.getBytesPerPixel(),
                                                                         static_cast<const unsigned char *>(memory->getSharedMemory()),
                                                                         50);
                        }

                    }
                    // Check if the size of the compressed image fits in a UDP packet.
                    const int32_t MAX_SIZE_UDP_PAYLOAD = 65000;
                    if (retVal && (compressedSize < MAX_SIZE_UDP_PAYLOAD)) {
                        // Create the CompressedImage data structure.
                        odcore::data::image::CompressedImage ci(si.getName(), si.getWidth(), si.getHeight(),
                                                                si.getBytesPerPixel(), compressedSize);
                        ::memcpy(ci.getRawData(), buffer, compressedSize);

                        string s(reinterpret_cast<char const *>(buffer), compressedSize);

                        const string SEND_TO = "127.0.0.1";
                        const uint32_t _PORT = 1234;

                        // We are using OpenDaVINCI's std::shared_ptr to automatically
                        // release any acquired resources.
                        try {
                            std::shared_ptr <UDPSender> udpsender(UDPFactory::createUDPSender(SEND_TO, _PORT));
                            cerr << "Creating regular UDP sender at " << SEND_TO << ":" << _PORT << endl;

                            udpsender->send(s);
                        }
                        catch (string &exception) {
                            cerr << "Data could not be sent: " << exception << endl;
                        }

                        // Write the CompressedImage container to STDOUT.
//                        odcore::data::Container container(ci);
//                        container.setSentTimeStamp(c.getSentTimeStamp());
//                        container.setReceivedTimeStamp(c.getReceivedTimeStamp());
//                        container.setSampleTimeStamp(c.getSampleTimeStamp());
//                        std::cout << container;
                    }
                    if (compressedSize >= MAX_SIZE_UDP_PAYLOAD) {
                        cerr << "Warning! Compressed image too large (" << compressedSize
                             << " bytes) to fit in a UDP packet. Image skipped." << std::endl;
                    }
                    if (!retVal) {
                        cerr << "Warning! Failed to compress image. Image skipped." << std::endl;
                    }
                    // Free pointer to compressed data.
                    OPENDAVINCI_CORE_FREE_POINTER(buffer);
                } else {
                    cerr << "Warning! Color space not supported. Image skipped." << std::endl;
                }
            }
            return retVal;
        }

        void UDPConnectionStreamer::nextContainer(Container &c) {
            bool has_next_frame = readSharedImage(c);

            // If we have an image from the previous call, it is then processed
            if (has_next_frame) {
                cerr << "Image sent!" << std::endl;
            }
        }
    }
} // scaledcars::control

