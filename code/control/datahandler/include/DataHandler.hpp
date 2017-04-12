#include <opendavinci/odcore/io/StringListener.h>
#include "protocol.h"

// This class will handle the bytes received via a serial link.
class DataHandler : public odcore::io::StringListener {

    // Your class needs to implement the method void nextString(const std::string &s).
    virtual void nextString(const std::string &s);
    
    virtual void filterProtocol(const protocol_data data);
};
