package car;

public class CarState{
    private int location;
    private boolean isParked;
    private boolean foundParking;
    private int spotsFound;

    //made for testPark()
    public CarState(int location, boolean parked, boolean foundParking){
        this.location = location;
        this.isParked = parked;
        this.foundParking = foundParking;
        spotsFound = 0;
    }

    public int getLocation() {
        //Made for testMoveForwardIsParked()
        return location;
    }

    public void setLocation(int location){
        this.location = location;
        System.out.println(">>>>>>>>>>>>>>>>>>>>>>>" + getLocation());
    }

    public boolean isParked() {
        //Made for testMoveForwardIsParked()
        return isParked;
    }

    public boolean getFoundParking(){
        //made for testPark()
        return foundParking;
    }

    public void updateParking(boolean freeSpace){
        //made for testPark()
        if(freeSpace){
            spotsFound++;
            if(spotsFound == 85){
                foundParking = true;
            }
        }
        else{
            spotsFound = 0;
        }
    }
    /* the odometer will be the one that will be calculating the distance
       the car is travelling, since the odometer calculates the number of times
       that the wheel has done turn completely*/
    public void moveForward(int centimeter){
        //Made for testMoveForward()
        location = location + centimeter;
    }

    public void moveBackward(int centimeter){
        //Made for testMoveBackward()
        location = location - centimeter;
    }

    public void park(){
        //Made for testMoveForwardIsParked()
        isParked = true;
        //made for testPark()
        location = location - 2; //update location since the car has parked
    }

    public void unPark(){
        //Made for testUnPark()
        isParked = false;
        location = location + 2; //update location since the car has unparked
        foundParking = false; //after the car has unparked the car cannot park there at the same spot anymore.
        spotsFound = 0;
    }
}