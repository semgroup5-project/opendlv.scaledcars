package car;

public interface Motor
{

    public int moveForward(int centimeter);


        /**
         Description:     The car advances four centimeters in a straight line and checks for parking.
         Pre-condition:   The car is on the street and not parked.
         Post-condition:  The car has changed the position and has gone forward one meter.
         Test-cases:      testMoveForwardIsParked(), testMoveForward(), testMoveForwardStartOfStreet(),
         testMoveForwardEndOfStreet(),testMoveForwardUnderBounds(),testMoveForwardOverBounds()
         */

        public int moveBackward(int centimeter);

        /**
        Description:     The car goes backwards 4 centimeters in order to park.
        Pre-condition:   The car is on the street and not parked.
        Post-condition:  The car has changed the position and has gone backwards four centimeters at the time.
        Test-cases:      testMoveForwardIsParked(), testMoveForward(), testMoveForwardStartOfStreet(),
        testMoveForwardEndOfStreet(),testMoveForwardUnderBounds(),testMoveForwardOverBounds()
        */

    }

