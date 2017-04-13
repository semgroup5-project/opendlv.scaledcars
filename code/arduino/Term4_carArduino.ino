#include "protocol.h"
#include <Smartcar.h>
#include <TimerOne.h>

#define US_C 0x73 //front ultrasonic pin
#define US_R 0x70 //front right ultrasonic
#define ENCODER_PIN_A 2 //wheel encoder channel A pin
#define ENCODER_PIN_B 4 //wheel encoder channel B pin
#define SERVO_PIN 3 //steering servo pin
#define ESC_PIN 9 //ESC motor pin
#define SIDE_FRONT_PIN A0 //side front infrared pin
#define SIDE_BACK_PIN A1 //side back infrared pin
#define BACK_PIN A2 //back infrared pin
#define CH_1 5 //RC receiver channel 1 pin
#define CH_2 6 //RC receiver channel 2 pin
#define redPin 22 //LED red pin
#define greenPin 24 //LED green pin
#define bluePin 26 //LED blue pin
#define COMMON_ANODE //LED common

//Signal Conditioning limits for RC controller
const int lo = 1000;
const int hi = 2000;
const int deadLo = 1430;
const int deadHi = 1570;
const int center = 1500;

//Conditions limits for writting to both servos(steering and motor)
const int centerVar = 90;
const int loVar = 0;
const int hiVar = 180;

//Ultrasonic front and side front
SRF08 us_center, us_right;

//Infrared side frony, side back, back
GP2D120 sideFrontIR, sideBackIR, backIR;

//Steering Servo and Motor
Servo steeringServo, motorServo;  

//Wheel encoder
Odometer encoder;

//Flag to signal of channel 2 is in the center
int notCenter = 0;

//Array to store values comming from the RC receiver
int ch[1];

//Variables to store throttle and steering values
int steeringVal, throttleVal = 90;

//Variables to store sensors values
int us_f, us_fr, ir_sf, ir_sb, ir_b;

//Flag that indicates the direction the car is travelling 0 for forwards 1 for backwards
int direction_int = 0;

//Flag to control which function we should be running RCcontroller or automatedDrive
int func_is_changed = -1;

//Variable to store encoder position
int encoderPos = 0;

//Variable to store driving distance from the requested point
int odometer = 0;

//Flag to start or stop encoder values sending
int odometerStart = 0;

/*
 * Attaching pins
 */
void setup()
{
  steeringServo.attach(SERVO_PIN);
  motorServo.attach(ESC_PIN);

  pinMode(CH_1, INPUT);
  pinMode(CH_2, INPUT);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  encoder.attach(ENCODER_PIN_A, ENCODER_PIN_B, true);
  encoder.begin();

  us_center.attach(US_C);
  us_right.attach(US_R);

  sideFrontIR.attach(SIDE_FRONT_PIN);
  sideBackIR.attach(SIDE_BACK_PIN);
  backIR.attach(BACK_PIN);

  Serial.begin(115200); //start the serial

  armServos(steeringServo, motorServo);

  //Timer interrupt that checks if RC controller is on to apply the emergencyBrake if necessary
  Timer1.initialize(1700000);
  Timer1.attachInterrupt(emergencyBrake, 1700000);
}

/*
 * In every interation we check if the RC controller is on and execute a function according also in every interation we send all sensors values
 */
void loop()
{
  if (!isRCControllerOn()) {
    automatedDrive();
  } else {
    rcControl();
  }
  provideSensorsData();
}

/*
 * ISR function called by the timer interrupt it checks if RC controller is on or off and apply a brake if necessary
 */
void emergencyBrake()
{
  if (isFunctionChanged() != -1)
  {
    int ch1 = pulseIn(CH_1, HIGH);
    if (ch1 && !isFunctionChanged() && (throttleVal < centerVar || throttleVal > centerVar))
    {
      brake();
    } else if (!ch1 && isFunctionChanged() && (throttleVal < centerVar || throttleVal > centerVar))
    {
      brake();
    }
  }
}

/**
 * Function responsible for arming steering servo and motor
 */
void armServos(Servo steering, Servo motor)
{
  steering.write(centerVar);
  delay(500);
  motor.write(centerVar);
  delay(500);
  setLEDColour(0, 255, 255);  // aqua
}

/*
 * boolean function to indicates if RC controller is on
 */
int isRCControllerOn()
{
  return readChannel1();
}

/*
 * boolean function to indicates which of the main functions we are running 0 for automated and 1 for rcController
 */
int isFunctionChanged()
{
  return func_is_changed;
}

/*
 * functions that reads the incoming signal from the receiver's channel 1
 */
int readChannel1()
{
  return pulseIn(CH_1, HIGH); // steer
}

/*
 * functions that reads the incoming signal from the receiver's channel 2
 */
int readChannel2()
{
  return pulseIn(CH_2, HIGH); // drive
}

/*
 * function responsible for receiving and executing commands from the code
 */
void automatedDrive()
{
  if (isFunctionChanged()) armServos(steeringServo, motorServo); //always arm the servos to make sure we have control over them

  func_is_changed = 0;

  setLEDColour(0, 0, 255);  // blue

  protocol_state state;

  if (Serial.available())
  {
    Serial.println("received");
    protocol_receive(&state, Serial.read());
    if (state.valid)
    {
      Serial.println("valid");
      int id = state.data.id;
      int value = state.data.value;
      if (id == ID_OUT_MOTOR) //incoming motor instructions
      {
        if (value > 180) brake(); //applying values greater than 180 will be our indicative to brake
        else
        {
          throttle(value);
        }
      }
      else if (id == ID_OUT_SERVO) //incoming steering instructions
      {
        steer(value);
      }
      else if (id == ID_OUT_ODOMETER) //incoming odometer instructions 
      {
        odometerStart = value ? 1 : 0; //change flag to to start sending the odometer data 1 to send 0 to to not send
        encoderPos = odometerStart ? encoder.getDistance() : 0; //if we are sending odometer data set the position for the actual position of the car
      }
    }
  }
}

/*
 * functions responsible for receiving and executing signal from the RC controller
 */
void rcControl()
{
  if (!isFunctionChanged()) armServos(steeringServo, motorServo); //always arm the servos to make sure we have control over them

  func_is_changed = 1;
  setLEDColour(0, 255, 0);  // green

  ch[0] = readChannel1(); // steer channel;
  ch[1] = readChannel2(); // drive channel;

  //  Serial.print("CH1 ");
  //  Serial.println(ch[0]);
  //
  //  Serial.print("CH2 ");
  //  Serial.println(ch[1]);

  //trim values
  filterValues(0);
  filterValues(1);

  //Steering Control Output
  steeringVal = ch[0];

  //apply steering value
  steer(steeringVal);

  //we only apply throttle when necessary if the value is always in the center we ignore it
  if (ch[1] < deadLo || ch[1] > deadHi)
  {
    throttle(ch[1]);
    notCenter = 1;
  } else if (notCenter)
  {
    throttle(ch[1]);
    notCenter = 0;
  }
}

/*
 * Function responsible for trimming the incomming RC controller signals
 */
void filterValues(int i)
{
  if (ch[i] == 0) //zero is also considered as the center value
  {
    ch[i] = center;
  } else if (ch[i] <= lo) //Trim Noise from bottom end
  {
    ch[i] = lo;
  } else if (ch[i] <= deadHi && ch[i] >= deadLo) //Create Dead-Band
  {
    ch[i] = center;
  } else if (ch[i] >= hi) //Trim Noise from top end
  {
    ch[i] = hi;
  }
}

/*
 * Function responsible for applying the steering values to the car
 */
void steer(int angle)
{
  if (angle > 180) //map rc controller values to the desired range
  {
    if (angle >= lo && angle <= deadLo)
    {
      angle = map(angle, lo, deadLo, 50, 90);
    } else if (angle == center)
    {
      angle = centerVar;
    } else if (angle >= deadHi && angle <= hi)
    {
      angle = map(angle, deadHi, hi, 90, 130);
    }
  } else //mapping values to avoid full turn which affects the ultrasonics
  {
    if (angle >= loVar && angle < centerVar)
    {
      angle = map(angle, loVar, 89, 50, 90);
    } else if (angle == centerVar)
    {
      angle = centerVar;
    } else if (angle > 90 && angle <= hiVar)
    {
      angle = map(angle, 91, hiVar, 90, 130);
    }
  }
  steeringServo.write(angle); //writting value to the servo
}

/*
 * Function responsible for applying throttle to the servo motor
 */
void throttle(int throttle)
{
  if (throttle > 180) //map rc controller values to the desired range
  {
    if (throttle >= lo && throttle <= deadLo)
    {
      throttle = map(throttle, lo, deadLo, loVar + 25, centerVar - 1);
    } else if (throttle == center)
    {
      throttle = centerVar;
    } else if (throttle >= deadHi && throttle <= hi)
    {
      throttle = map(throttle, deadHi, hi, centerVar + 1, hiVar - 30);
    }
  }

  //dependending on the direction the car is going the flag direction is flipped 1=backward or 0=forward
  if (throttle < 90) 
  {
    setLEDColour(160, 0, 160);  // purple
    direction_int = 1;
  } else if (throttle > 90)
  {
    setLEDColour(255, 255, 0);  // yellow
    direction_int = 0;
  }

  motorServo.write(throttle); //writting value to the motor
  throttleVal = throttle; //storing value to be used along the logic
}

/*
 * Function responsible for applying a brake effect in the car. The intention is to check the direction the car is going to and apply contrary rotation to the motor
 */
void brake()
{
  setLEDColour(255, 0, 0);  // red
  if (direction_int == 1)
  {
    throttle(centerVar + 15);
    delay(1000);
    throttle(centerVar);
  } else if (direction_int == 0)
  {
    throttle(centerVar - 55);
    delay(1000);
    throttle(centerVar);
  }
}

/*
 * Function responsible for setting different color in the LED. The parameters should be balanced just as in any RGB color scheme
 */
void setLEDColour(int red, int green, int blue)
{
#ifdef COMMON_ANODE
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;
#endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}

/*
 * Function responsible for calling functions that pack and write sensors values to the serial port
 */
void provideSensorsData()
{
  provideIRSideFront();
  provideIRSideBack();
  provideIRBack();
  provideUSCenter();
  provideUSSide();
  provideDistance();
}

/*
 * Auxiliary function responsible for using the protocol to pack the data and write it to the serial port. Note value should not be greater than 255
 */
void encodeAndWrite(int id, int value)
{
  protocol_data data =
  {
    .id = id,
    .value = value
  };
  protocol_frame frame = protocol_encode(data);
  Serial.write(frame.a);//send byte 1
  Serial.write(frame.b);//send byte 2
}

/*
 * Send infrared side front
 */
void provideIRSideFront()
{
  ir_sf = sideFrontIR.getDistance();;
  encodeAndWrite(ID_IN_INFRARED_SIDE_FRONT, ir_sf);
}

/*
 * Send infrared side back
 */
void provideIRSideBack()
{
  ir_sb = sideBackIR.getDistance();;
  encodeAndWrite(ID_IN_INFRARED_SIDE_BACK, ir_sb);
}

/*
 * Send infrared back
 */
void provideIRBack()
{
  ir_b = backIR.getDistance();;
  encodeAndWrite(ID_IN_INFRARED_BACK, ir_b);
}

/*
 * Send ultrasonic front center
 */
void provideUSCenter()
{
  us_f = us_center.getDistance();;
  encodeAndWrite(ID_IN_ULTRASONIC_CENTER, us_f);
}

/*
 * Send ultrasonic front right side
 */
void provideUSSide()
{
  us_fr = us_right.getDistance();;
  encodeAndWrite(ID_IN_ULTRASONIC_SIDE_FRONT, us_fr);
}

/*
 * Function responsible for sending the values captured by the wheel encoder. This function will only send data if the flag odometerStart is flipped. To flip it we need to write from
 * our code to the arduino sending the respective id for odometer with a value greater than 0. Once fliped it stores the actual postion the wheel encoder is at and every time before
 * writting to the serial port it checks how much the car travelled from the initial store postion until the current moment. As the protocol does not send values greater than 255
 * this function checks the value and if it is greater than 255 it resets the count. To stop the function just write to the arduino the respective odometer id with 0 value
 */
void provideDistance()
{
  odometer = encoder.getDistance() - encoderPos;
  if (odometer > 255) 
  {
    encoderPos = encoder.getDistance();
    odometer -= 255;
  }
  if (odometerStart)
  {
    encodeAndWrite(ID_IN_ENCODER, odometer);
  }
}

