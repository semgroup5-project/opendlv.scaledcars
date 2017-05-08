

#include <automotivedata/GeneratedHeaders_AutomotiveData.h>

#include <opendavinci/GeneratedHeaders_OpenDaVINCI.h>
#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>

#include "protocol.h"


using namespace std;

using namespace odcore;
using namespace odcore::base::module;
using namespace odcore::data;
using namespace odcore::wrapper;

namespace scaledcars {
    namespace control {

        class Filter {
        
        

        public:
            /**
             * Constructor.
             */
            Filter();

            
            //bool isSensorValue();

            void filter(protocol_data data);
            
            bool isOdometer(protocol_data data);
            
            map<uint32_t, double> normalize();
            
        		double handleOdometerValue(protocol_data data);

        private:
        		double medianValue(vector<double> array);
        		void average(double array[], int counter[]);
        		bool isIRUS(protocol_data data);
        		
	
            
        };
    }
}
