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

        int changeFunction;
        int emergencyStop;

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
        using namespace group5;
        using namespace odcore::data::dmcp;
        using namespace cv;

        void UDPReceiveBytes::nextString(const string &s) {
            cout << "RECEIVED : " << s.length() << " bytes containing '" << s << "'" << endl;
            changeFunction = atoi(decodedNetString(s).c_str());
        }

        UDPConnection::UDPConnection(const int &argc, char **argv)
                : TimeTriggeredConferenceClientModule(argc, argv, "UDPConnection"),
                  laneFollowerMSG(),
                  udp_receiver(),
                  m_sharedImageMemory(),
                  m_sharedProcessedImageMemory(),
                  m_sharedProcessedImage(),
                  m_hasAttachedToSharedImageMemory(false),
                  m_image(),
                  m_image_mat(),
                  m_image_new(),
                  m_threshold1(50),  //50
                  m_threshold2(200) {}  // 150


        UDPConnection::~UDPConnection() {}

        void UDPConnection::setUp() {
            cout << "Starting UDPConnection" << endl;
            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();

            emergencyStop = kv.getValue<int32_t>("communicationlink.functionlane");
            changeFunction = kv.getValue<int32_t>("communicationlink.function2");
        }

        void UDPConnection::tearDown() {
            cout << "Shutting down UDPConnection" << endl;
            if (!m_image.empty()) {
                m_image.deallocate();
            }
            // Stop receiving bytes and unregister our handler.
            udp_receiver->stop();
            udp_receiver->setStringListener(NULL);
        }

        // This method returns a boolean true if it gets an image from the shared image memory
        bool UDPConnection::readSharedImage(Container &c) {
            bool retVal = false;

            if (c.getDataType() == SharedImage::ID()) {
                SharedImage si = c.getData<SharedImage>();

                // Check if we have already attached to the shared memory.
                if (!m_hasAttachedToSharedImageMemory) {
                    m_sharedImageMemory = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(si.getName());
                    m_hasAttachedToSharedImageMemory = true;
                }

                // Check if we could successfully attach to the shared memory.
                if (m_sharedImageMemory->isValid()) {
                    // Lock the memory region to gain exclusive access using a scoped lock.
                    m_sharedImageMemory->lock();

                    if (m_image.empty()) { // If image is empty, create a new cv::Mat image
                        m_image.create(si.getHeight(), si.getWidth(),
                                       CV_8UC3); //From opencv documentation: CV_8UC3 = unsigned integer 8bit matrix/image wih 3 mats (typically RGB or BRG in opencv case)
                    } else { // Copying the image data
                        memcpy(m_image.data, m_sharedImageMemory->getSharedMemory(),
                               si.getWidth() * si.getHeight() * si.getBytesPerPixel());
                    }
                    m_sharedImageMemory->unlock(); // Release the memory region lock

                    retVal = true;
                }
            }
            return retVal;
        }

        double UDPConnection::Median(Mat mat) {
            double m = (mat.rows * mat.cols) / 2;
            int bin = 0;
            double med = -1.0;

            int histSize = 256;
            float range[] = {0, 256};
            const float *histRange = {range};
            bool uniform = true;
            bool accumulate = false;
            Mat hist;
            calcHist(&mat, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

            for (int i = 0; i < histSize && med < 0.0; ++i) {
                bin += cvRound(hist.at<float>(i));
                if (bin > m && med < 0.0)
                    med = i;
            }

            return med;
        }

        // Process Image
        void UDPConnection::processImage() {
            // New image
            m_image_mat = Mat(m_image.rows, m_image.cols, CV_8UC1);
            // Copy the original image to the new image as greyscale

            cvtColor(m_image, m_image_mat, COLOR_BGR2GRAY);

            GaussianBlur(m_image_mat, m_image_new, Size(5, 5), 0, 0);
            // calc median of pixel color
            double median;
            median = Median(m_image_new);

            m_threshold1 = max(static_cast<double>(0), ((1.0 - 0.33) * median));
            m_threshold2 = min(static_cast<double>(255), (1.0 + 0.33) * median);

            Canny(m_image_new, m_image_new, m_threshold1, m_threshold2,
                  3); // see header for algorithm and threshold explanation

            if ( ! m_image_new.isContinuous() )
            {
                m_image_new = m_image_new.clone();
            }

            const int size = m_image_new.total() * m_image_new.elemSize();
            unsigned char *bytes = new unsigned char[size];
            memcpy(bytes, m_image_new.data, size * sizeof(unsigned char));

            string s(reinterpret_cast<char const *>(bytes), size);

            cerr << "IMAGE SIZE : " << size << endl;

//            const string SEND_TO = "127.0.0.1";
//            const uint32_t _PORT = 1234;
//
//            // We are using OpenDaVINCI's std::shared_ptr to automatically
//            // release any acquired resources.
//            try {
//                std::shared_ptr<UDPSender> udpsender(UDPFactory::createUDPSender(SEND_TO, _PORT));
//
//                udpsender->send(s);
//            }
//            catch(string &exception) {
//                cerr << "Data could not be sent: " << exception << endl;
//            }

        }

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode UDPConnection::body() {

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
                    odcore::base::Thread::usleepFor(5 * ONE_SECOND);

                    // Stop receiving bytes and unregister our handler.
                    udp_receiver->stop();

                    udp_receiver->setStringListener(NULL);

                    bool has_next_frame = false;

                    // Get the most recent available container for a SharedImage.
                    Container image_container = getKeyValueDataStore().get(SharedImage::ID());

                    if (image_container.getDataType() == SharedImage::ID()) {
                        has_next_frame = readSharedImage(image_container);
                    }

                    // If we have an image from the previous call, it is then processed
                    if (has_next_frame) {
                        processImage();
                    }
                }
                catch (string &exception) {
                    cerr << "Error while creating UDP receiver: " << exception << endl;
                }
            }
            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    }
} // scaledcars::control

