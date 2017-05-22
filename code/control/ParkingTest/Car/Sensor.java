package car;

public interface Sensor {
    /**
     Description:       The sensor gets the distance to nearest object, there are 3 main sensors involve
     					in the parking.
     Pre-condition:     The sensor exists and the car is on the street to provide its location.
     Post-condition:    The sensor has returned its value.
     Test-cases:        testMoveBackwardWhenParked(), testMoveBackward(), testMoveBackwardOverBounds(), testMoveBackwardUnderBounds()
                        testMoveBackwardEndOfStreet(), testMoveBackwardStartOfStreet().
     **/

    int getDistanceAtLocation(int location);
}
/*
package car;

public class Sensor {
	 int distance;

	    public Sensor(){
	    	//public static Sensor;
	    }

	    public int getDistance(){
	        return distance;
	    }

	    public void setDistance(int distance){
	        this.distance = distance;
	    }

	}

*/