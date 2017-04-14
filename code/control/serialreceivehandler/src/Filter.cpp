#include "Filter.h"
#include "protocol.h"

namespace scaledcars {
	namespace control {
	
		using namespace std;
		
		void Filter::filterProtocol(protocol_data pd){
			protocol_data *ppd = &pd;
			
			//Container c;
			//SensorBoardData sbd;
			//VehicleData vd;
	
			//TODO SEBASTIAN filter stuff
	
	
			if(ppd->id == 1){ //Sensor data stuff
			//	sbd(/* PUT THE FILTERED DATA HERE*/ NULL);
			//	c(sbd);
			//	getConference().send(c);
   		} 
   
   		if (/*TODO SEBASTIAN if its other vehicle data stuff go here*/ 1){ 
   		//	vd(/* PUT THE FILTERED DATA HERE*/NULL);
   		// 	c(vd)
			//	getConference().send(c);
   		}
		}
	}
}
