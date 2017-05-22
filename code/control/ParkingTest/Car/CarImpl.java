
package car;

public class CarImpl implements Car{

    private CarState state;
    private Sensor sensorImpl1;
    private Sensor sensorImpl2;

    //private Motor motorImpl;


    public CarImpl(int location, boolean parked, boolean foundParking, Sensor sensor1, Sensor sensor2, Motor motor){
        state = new CarState(location, parked, foundParking);
        this.sensorImpl1 = sensor1;
        this.sensorImpl2 = sensor2;
        //this.motorImpl = motor;
    }

    @Override
    public CarState moveForward(){
        //Made for testPark()
        boolean parkingState = isEmpty();
        state.updateParking(parkingState);
        //testMoveForwardEndOfStreet()
        if (state.getLocation() >= Street.END) {
            return state;
        }
        //Made for testMoveForwardStartOfStreet()
        if (state.getLocation() < Street.START) {
            return state;
        }
        //Made for testMoveForwardIsParked()
        if (state.isParked()) {
            return state;
        } else {
            //Made for testMoveForward()
            state.moveForward(1);
            return state;
        }
    }

   // @Override
    public boolean isEmpty() {
        //Made for testIsEmptyTrue(), testIsEmptyFalse()
        int resSensor1 = sensorImpl1.getDistanceAtLocation(state.getLocation());
        int resSensor2 = sensorImpl2.getDistanceAtLocation(state.getLocation());

    	int minDistanceForEmpty = 85;

        if(resSensor1 == -1 && resSensor2 == -1){
            //if you don't know what's there, better not to park
            return false;
        }
        if (resSensor1 == -1) {
           // partial invalid data don't park is missing a parameter
            return false;
        }
        if (resSensor2 == -1) {
            //partial invalid data don't park, is missing a parameter
            return false;
        }
        if (resSensor1 >= minDistanceForEmpty && resSensor2 >= minDistanceForEmpty) {
            return true;
        }
        return false;
    }

    @Override
    public CarState moveBackward(){

        //Made for testMoveBackwardOutOfBounds()
        if (state.getLocation() > Street.END) {
            return state;
        }
        if (state.getLocation() <= Street.START) {
            return state;
        }
        //Made for testMoveBackwardWhenParked()
        if (state.isParked()) {
            return state;
        } else {
            //Made for testMoveBackward()
            state.moveBackward(1);
            return state;
        }
    }

    @Override
    public void park(){
        if (state.isParked()) {
            //Made for testParkIsParked()
            throw new IllegalStateException("Car is already parked.");
        }
        //Made for testPark() while the car has nor found a parking place
        while (!state.getFoundParking() && state.getLocation() < Street.END) {
            moveForward();
        }
        if (state.getFoundParking()) {
            state.park();
        } else {
            //Made for testParkNoPlaceToPark()
            throw new UnsupportedOperationException("Car did not find a parking place.");
        }
    }

    @Override
    public void unPark(){
        //Made for testUnParkIsNotParked()
        if (state.isParked()) {
            //Made for testUnPark()
            state.unPark();
        } else {
            throw new IllegalStateException("Car is not parked to unpark.");
        }
    }

    @Override
    public CarState whereIs(){
        //Made for testMoveForwardIsParked(), mtestMveBackwardWhenParked()
        return state;
    }

}