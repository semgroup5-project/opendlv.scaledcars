#include <opendavinci/odcore/io/StringListener.h>
#include "protocol.h"

namespace scaledcars {
	namespace control {
	
		// This class will handle the bytes received via a serial link.
		class DataHandler : 
    		
    		private:
        		/**
         	* "Forbidden" copy constructor. Goal: The compiler should warn
         	* already at compile time for unwanted bugs caused by any misuse
         	* of the copy constructor.
         	*
         	* @param obj Reference to an object of this class.
         	*/
        		DataHandler(const DataHandler &/*obj*/);

        		/**
         	* "Forbidden" assignment operator. Goal: The compiler should warn
         	* already at compile time for unwanted bugs caused by any misuse
         	* of the assignment operator.
         	*
         	* @param obj Reference to an object of this class.
         	* @return Reference to this instance.
         	*/
        		DataHandler& operator=(const DataHandler &/*obj*/);

    		public:
        /**
         * Constructor.
         *
         * @param argc Number of command line arguments.
         * @param argv Command line arguments.
         */
        		DataHandler(const int32_t &argc, char **argv);

        		virtual ~DataHandler();

        		virtual void filterProtocol(const protocol_data data);

    		private:
        		virtual void setUp();

        		virtual void tearDown();
        		
			public odcore::io::StringListener {
	
    			virtual void nextString(const std::string &s);
    
    		};
	}
}
