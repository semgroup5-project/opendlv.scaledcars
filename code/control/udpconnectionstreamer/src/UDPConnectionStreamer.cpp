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

        string clientIp = "";

        UDPConnectionStreamer::UDPConnectionStreamer(const int &argc, char **argv)
                : DataTriggeredConferenceClientModule(argc, argv, "UDPConnectionStreamer"),
                  m_hasAttachedToSharedImageMemory(false),
                  m_image(),
                  m_image_mat(),
                  m_image_new(),
                  m_sharedImageMemory(),
                  m_sharedProcessedImageMemory(),
                  m_sharedProcessedImage(),
                  m_threshold1(50),
                  m_threshold2(200),
                  si() {}


        UDPConnectionStreamer::~UDPConnectionStreamer() {}

        void UDPConnectionStreamer::setUp() {
            cout << "Starting UDPConnectionStreamer" << endl;
            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();

            clientIp = kv.getValue<string>("udpconnectionstreamer.ip");
        }

        void UDPConnectionStreamer::tearDown() {
            cout << "Shutting down UDPConnectionStreamer" << endl;
        }

        // This method returns a boolean true if it gets an image from the shared image memory
        bool UDPConnectionStreamer::readSharedImage(Container &c) {
            bool retVal = false;

            if (c.getDataType() == SharedImage::ID()) {
                si = c.getData<SharedImage>();
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

        void UDPConnectionStreamer::nextContainer(Container &c) {
            bool has_next_frame = readSharedImage(c);

            // If we have an image from the previous call, it is then processed
            if (has_next_frame) {
                processImage();
            }
        }

        // Process Image
        void UDPConnectionStreamer::processImage() {
            // New image
            m_image_mat = Mat(m_image.rows, m_image.cols, CV_8UC1);
            // Copy the original image to the new image as greyscale

            cvtColor(m_image, m_image_mat, COLOR_BGR2GRAY);   //COLOR_BGR2RGB   for full colors, set the color and stop this method here, change the compressor to m_image_mat and change the width + height back to si.getHeight() si.getWidth()

            GaussianBlur(m_image_mat, m_image_new, Size(5, 5), 0, 0);
            // calc median of pixel color
            double median;
            median = Median(m_image_new);

            m_threshold1 = max(static_cast<double>(0), ((1.0 - 0.33) * median));
            m_threshold2 = min(static_cast<double>(255), (1.0 + 0.33) * median);

            Canny(m_image_new, m_image_new, m_threshold1, m_threshold2,
                  3); // see header for algorithm and threshold explanation
//
//                if (m_image.data != NULL) {
//                    imshow("Debug Image",
//                           m_image_new);  //m_image = image without canny || m_image_new = fully processed image
//                    waitKey(10);
//                }

            stream();
        }

        double UDPConnectionStreamer::Median(Mat mat) {
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

        void UDPConnectionStreamer::stream() {
            bool retVal = false;

            if ((1 == si.getBytesPerPixel()) ||
                (3 == si.getBytesPerPixel())) {

                int compressedSize = si.getWidth() * si.getHeight() * si.getBytesPerPixel();
                void *buffer = ::malloc(compressedSize);
                if (buffer != NULL) {
                    retVal = odcore::wrapper::jpg::JPG::compress(buffer, compressedSize, m_image_new.cols,
                                                                 m_image_new.rows,1,
                                                                 m_image_new.data,
                                                                 50);
                }
                // Check if the size of the compressed image fits in a UDP packet.
                const int32_t MAX_SIZE_UDP_PAYLOAD = 65000;
                cerr << "Compressed Size is: " << compressedSize << endl;
                if (retVal && (compressedSize < MAX_SIZE_UDP_PAYLOAD)) {
                    // Create the CompressedImage data structure.
                    odcore::data::image::CompressedImage ci(si.getName(), si.getWidth(), si.getHeight(),
                                                            si.getBytesPerPixel(), compressedSize);
                    ::memcpy(ci.getRawData(), buffer, compressedSize);

                    string s(reinterpret_cast<char const *>(buffer), compressedSize);

                    const string SEND_TO = clientIp;
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
    }
} // scaledcars::control

