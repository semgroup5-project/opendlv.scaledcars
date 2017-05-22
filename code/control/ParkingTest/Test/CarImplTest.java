package tests;

import car.*;

import static org.junit.Assert.assertEquals;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.Mockito;
import org.mockito.MockitoAnnotations;

import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.Mockito.when;

public class CarImplTest{

    @Mock
    Sensor sensor1, sensor2;
    @Mock
    Motor motor;

    @Before
    public void setUp(){

        MockitoAnnotations.initMocks(this);

        //defaults to an empty street
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(200);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(200);
    }


    /**
     * When the car is parked the car should not move.
     */
    @Test
    public void testMoveForwardIsParked(){
        CarImpl car = new CarImpl(50, true, false, sensor1, sensor2, motor);
        int start = car.whereIs().getLocation();
        int afterMoving = car.moveForward().getLocation();
        Assert.assertEquals(start, afterMoving);
    }

    /**
     * When the car is on the street, it moves one meter forward .
     */
    @Test
    public void testMoveForward(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1, sensor2, motor);
        int start = car.whereIs().getLocation();
        int afterMoving  = car.moveForward().getLocation();
        Assert.assertEquals(start+1, afterMoving);
    }

    /**
     * When the car is on the start of the street, it moves one meter forward.
     */
    @Test
    public void testMoveForwardStartOfStreet(){
        CarImpl car = new CarImpl(Street.START,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveForward().getLocation();
        Assert.assertEquals(Street.START+1, afterMoving);
    }

    /**
     * When the car is on the end of the street, it should not move any further.
     */

    @Test
    public void testMoveForwardEndOfStreet(){
        CarImpl car = new CarImpl(Street.END,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveForward().getLocation();
        Assert.assertEquals(Street.END, afterMoving);
    }


    /**
     * When the car is not on the street.
     */

    @Test
    public void testMoveForwardUnderBounds(){
        CarImpl car = new CarImpl(Street.START-1,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveForward().getLocation();
        Assert.assertEquals(Street.START-1, afterMoving);
    }

    /**
     * When the car is not on the street.
     */
    @Test
    public void testMoveForwardOverBounds(){
        CarImpl car = new CarImpl(Street.END+1,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveForward().getLocation();
        Assert.assertEquals(Street.END+1, afterMoving);
    }


    /**
    * Test to see if the car is moving backwards 1 meter when prompted
    * Obs! The car needs to have move forward before this testcase executes
    */
    @Test
    public void testMoveBackward(){
        CarImpl car = new CarImpl(100, false, false, sensor1, sensor2, motor);
        int start = car.whereIs().getLocation();
        int afterMoving = car.moveBackward().getLocation();
        Assert.assertEquals(start-1, afterMoving);
    }


    /**
     * When the car is on the start of the street, it moves should not move backward.
     */
    @Test
    public void testMoveBackwardStartOfStreet(){
        CarImpl car = new CarImpl(Street.START,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveBackward().getLocation();
        Assert.assertEquals(Street.START, afterMoving);
    }

    /**
     * When the car is on the end of the street, it should reverse one meter.
     */

    @Test
    public void testMoveBackwardEndOfStreet(){
        CarImpl car = new CarImpl(Street.END,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveBackward().getLocation();
        Assert.assertEquals(Street.END -1, afterMoving);
    }

    /**
    * When the car is not in the street
    */
    @Test
    public void testMoveBackwardUnderBounds(){
        CarImpl car = new CarImpl(-1,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveBackward().getLocation();
        Assert.assertEquals(-1, afterMoving);
    }

    /**
     * When the car is not in the street
     */
    @Test
    public void testMoveBackwardOverBounds(){
        CarImpl car = new CarImpl(501,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveBackward().getLocation();
        Assert.assertEquals(501, afterMoving);
    }


   /**
     * When the car is parked the car should not move.
     * Obs! The car must be in a parkable condition for this testcase to execute!
     */
   @Test
    public void testMoveBackwardWhenParked(){
        CarImpl car = new CarImpl(300, true, false, sensor1, sensor2, motor);
        int start = car.whereIs().getLocation();
        int afterMoving = car.moveBackward().getLocation();
        Assert.assertEquals(start, afterMoving);
    }
    /**
     * "space is free"
     * Test for valid sensor data over limit
     * Any distance = 200 is emptyexpected result is empty 1
     */
    @Test
    public void testIsEmptyIsFree() {
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(200);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(200);

        assertEquals(true, car.isEmpty());
    }

    /**
    * Test for valid sensor data under limit
    * Any distance below 200 is not empty expected result is not empty 0
    */

    @Test
    public void testIsEmptyIsNotFreeSensor2Less200() {
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(200);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(150);

        assertEquals(false, car.isEmpty());
    }

    @Test
    public void testIsEmptyIsNotFreeSensor1Less200() {
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(150);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(200);

        assertEquals(false, car.isEmpty());
    }

    @Test
    public void testIsEmptyIsNotFreeSensor1and2Less200() {
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(150);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(150);

        assertEquals(false, car.isEmpty());
    }
    /*
    * Test for invalid sensor data for sensor2 expected result is not empty 0
    */
    @Test
    public void testisEmptyIsNotFreeDiscardSensor1IsMinus1(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(-1);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(150);

        assertEquals(false, car.isEmpty());
    }

    @Test
    public void testisEmptyIsNotFreeDiscardSensor1(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(-1);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(200);

        assertEquals(false, car.isEmpty());
    }
    /*
    * Test for invalid sensor data for sensor2 expected result is not empty 0
    */

    @Test
    public void testisEmptyIsNotFreeDiscardSensor2IsMinus1(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(200);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(-1);

        assertEquals(false, car.isEmpty());
    }

    @Test
    public void testisEmptyIsNotFreeDiscardSensor2(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(150);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(-1);

        assertEquals(false, car.isEmpty());
    }

    /**
    * Test for invalid sensor data for both sensors expected result is not empty 0
    */
    @Test
    public void testIsEmptyCannotMeasure(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(-1);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(-1);

        assertEquals(false, car.isEmpty());
    }

    /**
     * When the car is already parked.
     */
    @Test(expected = IllegalStateException.class)
    public void testParkIsParked(){
            CarImpl car = new CarImpl(0, true, false, sensor1, sensor2, motor);
            car.park();
    }

    /**
     * When the car tries to park and has found a parking place.
     */
    @Test
    public void testPark(){
        int startLocation = 100;
        CarImpl car = new CarImpl(startLocation, false, true, sensor1, sensor2, motor);
        car.park();
        boolean status = car.whereIs().isParked();
        int location = car.whereIs().getLocation();
        Assert.assertEquals(true, status);
        Assert.assertEquals(startLocation-2, location);
    }

    /**
     * When the car cannot find a park place.
     */
    @Test(expected = UnsupportedOperationException.class)
    public void testParkNoPlaceToPark(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1, sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(150);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(150);
        car.park();
    }

    /**
     * When the car tries to unpark.
     */
    @Test
    public void testUnPark(){
        int startLocation = 300;
        CarImpl car = new CarImpl(startLocation, true, false, sensor1, sensor2, motor);
        car.unPark();
        boolean status = car.whereIs().isParked();
        int location = car.whereIs().getLocation();
        Assert.assertEquals(startLocation+2, location);
        Assert.assertEquals(false, status);
    }

    /**
     * When the car is not parked and tries to unpark.
     */
    @Test(expected = IllegalStateException.class)
    public void testUnParkIsNotParked(){
        CarImpl car = new CarImpl(300, false, false, sensor1, sensor2, motor);
        car.unPark();
    }

    /**
     * When the car is parked.
     */
    @Test
    public void testWhereIsParked(){
        CarImpl car = new CarImpl(300, true, false, sensor1, sensor2, motor);
        CarStatus status = car.whereIs();
        Assert.assertEquals(300, status.getLocation());
        Assert.assertEquals(true, status.isParked());
    }

    /**
     * When the car is not parked.
     */
    @Test
    public void testWhereIsNotParked(){
        CarImpl car = new CarImpl(300, false, false, sensor1, sensor2, motor);
        CarStatus status = car.whereIs();
        Assert.assertEquals(300, status.getLocation());
        Assert.assertEquals(false, status.isParked());
    }
}

/*package tests;

import car.*;

import static org.junit.Assert.assertEquals;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.Mockito;
import org.mockito.MockitoAnnotations;

import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.Mockito.when;

public class CarImplTest{

    @Mock
    Sensor sensor1, sensor2;
    @Mock
    Motor motor;

    @Before
    public void setUp(){

        MockitoAnnotations.initMocks(this);

        //defaults to an empty street
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(200);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(200);
    }


    *//**
     * When the car is parked the car should not move.
     *//*
    @Test
    public void testMoveForwardIsParked(){
        CarImpl car = new CarImpl(50, true, false, sensor1, sensor2, motor);
        int start = car.whereIs().getLocation();
        int afterMoving = car.moveForward().getLocation();
        Assert.assertEquals(start, afterMoving);
    }

    *//**
     * When the car is on the street, it moves one meter forward .
     *//*
    @Test
    public void testMoveForward(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1, sensor2, motor);
        int start = car.whereIs().getLocation();
        int afterMoving  = car.moveForward().getLocation();
        Assert.assertEquals(start+1, afterMoving);
    }

    *//**
     * When the car is on the start of the street, it moves one meter forward.
     *//*
    @Test
    public void testMoveForwardStartOfStreet(){
        CarImpl car = new CarImpl(Street.START,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveForward().getLocation();
        Assert.assertEquals(Street.START+1, afterMoving);
    }

    *//**
     * When the car is on the end of the street, it should not move any further.
     *//*

    @Test
    public void testMoveForwardEndOfStreet(){
        CarImpl car = new CarImpl(Street.END,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveForward().getLocation();
        Assert.assertEquals(Street.END, afterMoving);
    }


    *//**
     * When the car is not on the street.
     *//*

    @Test
    public void testMoveForwardUnderBounds(){
        CarImpl car = new CarImpl(Street.START-1,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveForward().getLocation();
        Assert.assertEquals(Street.START-1, afterMoving);
    }

    *//**
     * When the car is not on the street.
     *//*
    @Test
    public void testMoveForwardOverBounds(){
        CarImpl car = new CarImpl(Street.END+1,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveForward().getLocation();
        Assert.assertEquals(Street.END+1, afterMoving);
    }


    *//**
    * Test to see if the car is moving backwards 1 meter when prompted
    * Obs! The car needs to have move forward before this testcase executes
    *//*
    @Test
    public void testMoveBackward(){
        CarImpl car = new CarImpl(100, false, false, sensor1, sensor2, motor);
        int start = car.whereIs().getLocation();
        int afterMoving = car.moveBackward().getLocation();
        Assert.assertEquals(start-1, afterMoving);
    }


    *//**
     * When the car is on the start of the street, it moves should not move backward.
     *//*
    @Test
    public void testMoveBackwardStartOfStreet(){
        CarImpl car = new CarImpl(Street.START,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveBackward().getLocation();
        Assert.assertEquals(Street.START, afterMoving);
    }

    *//**
     * When the car is on the end of the street, it should reverse one meter.
     *//*

    @Test
    public void testMoveBackwardEndOfStreet(){
        CarImpl car = new CarImpl(Street.END,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveBackward().getLocation();
        Assert.assertEquals(Street.END -1, afterMoving);
    }

    *//**
    * When the car is not in the street
    *//*
    @Test
    public void testMoveBackwardUnderBounds(){
        CarImpl car = new CarImpl(-1,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveBackward().getLocation();
        Assert.assertEquals(-1, afterMoving);
    }

    *//**
     * When the car is not in the street
     *//*
    @Test
    public void testMoveBackwardOverBounds(){
        CarImpl car = new CarImpl(501,false, false, sensor1, sensor2, motor);
        int afterMoving = car.moveBackward().getLocation();
        Assert.assertEquals(501, afterMoving);
    }


   *//**
     * When the car is parked the car should not move.
     * Obs! The car must be in a parkable condition for this testcase to execute!
     *//*
   @Test
    public void testMoveBackwardWhenParked(){
        CarImpl car = new CarImpl(300, true, false, sensor1, sensor2, motor);
        int start = car.whereIs().getLocation();
        int afterMoving = car.moveBackward().getLocation();
        Assert.assertEquals(start, afterMoving);
    }
    *//**
     * "space is free"
     * Test for valid sensor data over limit
     * Any distance = 200 is emptyexpected result is empty 1
     *//*
    @Test
    public void testIsEmptyIsFree() {
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(200);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(200);

        assertEquals(true, car.isEmpty());
    }

    *//**
    * Test for valid sensor data under limit
    * Any distance below 200 is not empty expected result is not empty 0
    *//*

    @Test
    public void testIsEmptyIsNotFreeSensor2Less200() {
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(200);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(150);

        assertEquals(false, car.isEmpty());
    }

    @Test
    public void testIsEmptyIsNotFreeSensor1Less200() {
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(150);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(200);

        assertEquals(false, car.isEmpty());
    }

    @Test
    public void testIsEmptyIsNotFreeSensor1and2Less200() {
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(150);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(150);

        assertEquals(false, car.isEmpty());
    }

    * Test for invalid sensor data for sensor2 expected result is not empty 0

    @Test
    public void testisEmptyIsNotFreeDiscardSensor1IsMinus1(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(-1);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(150);

        assertEquals(false, car.isEmpty());
    }

    @Test
    public void testisEmptyIsNotFreeDiscardSensor1(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(-1);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(200);

        assertEquals(false, car.isEmpty());
    }

    * Test for invalid sensor data for sensor2 expected result is not empty 0


    @Test
    public void testisEmptyIsNotFreeDiscardSensor2IsMinus1(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(200);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(-1);

        assertEquals(false, car.isEmpty());
    }

    @Test
    public void testisEmptyIsNotFreeDiscardSensor2(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(150);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(-1);

        assertEquals(false, car.isEmpty());
    }

    *//**
    * Test for invalid sensor data for both sensors expected result is not empty 0
    *//*
    @Test
    public void testIsEmptyCannotMeasure(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1,sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(-1);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(-1);

        assertEquals(false, car.isEmpty());
    }

    *//**
     * When the car is already parked.
     *//*
    @Test(expected = IllegalStateException.class)
    public void testParkIsParked(){
            CarImpl car = new CarImpl(0, true, false, sensor1, sensor2, motor);
            car.park();
    }

    *//**
     * When the car tries to park and has found a parking place.
     *//*
    @Test
    public void testPark(){
        int startLocation = 100;
        CarImpl car = new CarImpl(startLocation, false, true, sensor1, sensor2, motor);
        car.park();
        boolean status = car.whereIs().isParked();
        int location = car.whereIs().getLocation();
        Assert.assertEquals(true, status);
        Assert.assertEquals(startLocation-2, location);
    }

    *//**
     * When the car cannot find a park place.
     *//*
    @Test(expected = UnsupportedOperationException.class)
    public void testParkNoPlaceToPark(){
        CarImpl car = new CarImpl(Street.START, false, false, sensor1, sensor2, motor);
        when(sensor1.getDistanceAtLocation(anyInt())).thenReturn(150);
        when(sensor2.getDistanceAtLocation(anyInt())).thenReturn(150);
        car.park();
    }

    *//**
     * When the car tries to unpark.
     *//*
    @Test
    public void testUnPark(){
        int startLocation = 300;
        CarImpl car = new CarImpl(startLocation, true, false, sensor1, sensor2, motor);
        car.unPark();
        boolean status = car.whereIs().isParked();
        int location = car.whereIs().getLocation();
        Assert.assertEquals(startLocation+2, location);
        Assert.assertEquals(false, status);
    }

    *//**
     * When the car is not parked and tries to unpark.
     *//*
    @Test(expected = IllegalStateException.class)
    public void testUnParkIsNotParked(){
        CarImpl car = new CarImpl(300, false, false, sensor1, sensor2, motor);
        car.unPark();
    }

    *//**
     * When the car is parked.
     *//*
    @Test
    public void testWhereIsParked(){
        CarImpl car = new CarImpl(300, true, false, sensor1, sensor2, motor);
        CarStatus status = car.whereIs();
        Assert.assertEquals(300, status.getLocation());
        Assert.assertEquals(true, status.isParked());
    }

    *//**
     * When the car is not parked.
     *//*
    @Test
    public void testWhereIsNotParked(){
        CarImpl car = new CarImpl(300, false, false, sensor1, sensor2, motor);
        CarStatus status = car.whereIs();
        Assert.assertEquals(300, status.getLocation());
        Assert.assertEquals(false, status.isParked());
    }
}*/