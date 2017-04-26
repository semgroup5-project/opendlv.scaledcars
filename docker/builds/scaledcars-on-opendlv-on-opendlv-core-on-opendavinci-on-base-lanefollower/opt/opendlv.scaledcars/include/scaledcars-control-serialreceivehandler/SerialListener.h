#include <opendavinci/odcore/io/StringListener.h>


namespace scaledcars {
	namespace control {

		// This class will handle the bytes received via a serial link.
		class SerialListener : public odcore::io::StringListener {

		    // Your class needs to implement the method void nextString(const std::string &s).
		    virtual void nextString(const std::string &s);
		};
	}
}
