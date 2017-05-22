#include "term4_car_lib.h"

Car myCar;

void setup() {
  myCar.setUp();
}

void loop() {
  int count = 0;
  while(count++ < 5) {
  myCar.provideSensorsData();
  }
}