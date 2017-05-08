

#include <iostream>
#include <memory>
#include <stdint.h>
#include <string>

#include "Filter.h"

namespace scaledcars {
    namespace control {

        using namespace std;
        using namespace odcore;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace odcore::wrapper;
        using namespace automotive;
        using namespace automotive::miniature;
        
        map<uint32_t, double> sensors;
        double sensorValueStore[5] = {};
            int sensorValueCounter[5] = {};
            double odometerCounter = 0;
            int odometerOldValue = 0;
            map<uint32_t, vector<double>> sensorValuesForMedian;
            bool isSensors = false;
        

		  Filter::Filter(){}        

         /**
        * Filters the data according to what sensor it represents and that sensors ranges.
        *
        * @param data to filter
        */
        void Filter::filter(protocol_data data){
        			if(isIRUS(data)){
        				sensorValueStore[data.id] += data.value;
        				sensorValuesForMedian[data.id].push_back(data.value);
        				sensorValueCounter[data.id] += 1;
					} else {
        				cout << "not a sensor value" << endl;
        			}
			}
			
			/**
			* @return true if a sensor value has been filtered
			*/
			//bool isSensorValue(){
			//	return isSensors;
			//}
			
			//US-SENSOR [ID 1] [ID 2] with value between 1 - 70
			//IR-SENSOR [ID 3] [ID 4] with value between 3 - 40
			bool Filter::isIRUS(protocol_data data){
				if(((data.id == 1 || data.id == 2) && data.value >= 1 && data.value <= 70) ||
					((data.id == 3 || data.id == 4 || data.id == 5) && data.value >= 3 && data.value <= 40)){
					
					isSensors = true;
				}
				return isSensors;
			}
			
			//ODOMETER [ID 6] with value between 0 - 255
			bool Filter::isOdometer(protocol_data data){
				if (data.id == 6 && data.value >= 0 && data.value <= 255)
					return true;
				return false;
			}
			
			/**
			* Increments the odometer counter with the amout differing between 
			* the new and the last odometer value received
			*
			* @return the updated odometer counter value
			*/
			double Filter::handleOdometerValue(protocol_data data){
				if((int)odometerOldValue > data.value){
					odometerCounter += (odometerOldValue - data.value);
				} else {
					odometerCounter += data.value;
				}
				odometerOldValue = data.value;
				return odometerCounter;
			}
			
			/**
			* Compares the average and the median of every sensor to distinguish 
			* and filter out noisy values. Add the average of the median and average
			* as the sensor value. If the difference between the average and the 
			* median is greater or smaller than 5 units the sensor value is set to 0
			*
			* @return a map of normalized sensor values
			*/
			map<uint32_t, double> Filter::normalize(){
				if(isSensors){
            	average(sensorValueStore, sensorValueCounter);
            	for(int i = 0; i < 5; i++){
               	protocol_data d;
                	d.id = i+1;
                  double median = medianValue(sensorValuesForMedian.at(i));
                  if(median + 5 > sensorValueStore[i] && median - 5 < sensorValueStore[i]){
                		d.value = (sensorValueStore[i] + median) / 2;	
                  } else {
                     d.value = 0;
                  }
                  sensors[d.id] = d.value;
                	cout << "ID: " << d.id << " VALUE: " << d.value << endl;
               }
            }
            return sensors;
			}
			
			/**
			* @return the median of the array
			*/
			double Filter::medianValue(vector<double> a){
				return (a[2] + a[3]) / 2;
			}
			
			/**
			* Average an array of doubles with an array of integers
			*/
			void Filter::average(double a[], int n[]){
				for(int i = 0; i < 5; i++)
					if(n[i] > 0)
						a[i] = (a[i] / n[i]);
			}

    }
}
