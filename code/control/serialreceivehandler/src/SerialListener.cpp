#include "SerialListener.h"
#include "Filter.cpp"
#include "protocol.h"

namespace scaledcars {
	namespace control {

		using namespace std;
		
		void SerialListener::nextString(const string &s) {
    		
    		protocol_frame pf;
    		protocol_frame *ppf = &pf;
    		for(int i = 0; i < (int)s.length(); i++){
    			int j = i + 1;
    			if(i != '\0'){
    				ppf->a = s.at(i);
    				ppf->b = s.at(j);
    			
    				//Filter::.filterProtocol(protocol_decode(pf)); TODO Sebastian
    			}
    			i++;
    		}
		}
	}
}
