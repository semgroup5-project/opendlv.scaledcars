#include "term4_car_lib.h"

Car myCar;

void setup() {
  myCar.setUp();
}

void loop() {
  myCar.provideSensorsData();
}