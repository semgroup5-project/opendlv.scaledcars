package car;


	public interface Car {

	    public CarState moveForward();
	    /**
	     Description:     The car advances four centimetes at the time in a straight line and
	                      checks for parking.
	     Pre-condition:   The car is on the street and not parked.
	     Post-condition:  The car has changed the position and has gone forward four centimeters.
	     Test-cases:      testMoveForwardIsParked(), testMoveForward(), testMoveForwardStartOfStreet(),
	                      testMoveForwardEndOfStreet(),testMoveForwardUnderBounds(),testMoveForwardOverBounds()
	     */

	    public boolean isEmpty();
	    /**
	     Description:     Evaluates if a parking spot is available with the help from sensors.
	     Pre-condition:   The car is moving forward on the street. The sensors are working.
	     Post-condition:  If return value is true, the car parks. If its false, the car does not park.
	     Test-cases:      testIsEmptyTrue(), testIsEmptyFalse()
	     */

	    public CarState moveBackward();

	    /**
	     Description     The car reverses one meter in a straight line and checks for parking.
	     Pre-condition:  The car is at least one meter in on the street and not parked.
	     Post-condition: The car has changed the position and has gone backward one meter.
	     Test-cases:     testMoveBackwardWhenParked(), testMoveBackward(), testMoveBackwardOverBounds(), testMoveBackwardUnderBounds()
	                     testMoveBackwardEndOfStreet(), testMoveBackwardStartOfStreet()
	     */

	    public void park();

	    /**
	     Description: The car checks for a free space and parks into it.
	     Pre-condition: The car is on the road and not parked.
	     Post-condition: The car is parked.
	     Test-cases: testParkIsParked(), testPark(), testParkNoPlaceToPark()
	     */


	    public void unPark();
	     /*
	     Description: Unparks the car.
	     Pre-condition: Car is parked.
	     Post-condition: Car is not parked anymore.
	     Test-cases: testUnPark(), testUnParkIsNotParked()
	     */


	    public CarState whereIs();
	    /**
	     Description: Returns the status of the car.
	     Pre-condition: There is a car.
	     Post-condition: The status of the car remains the same.
	     Test-cases: testWhereIsParked(), testWhereIsNotParked()
	     */

	}

