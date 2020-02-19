#include <PS4Controller.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include <SPI.h>
#include "SparkFun_LIS331.h"
#include <Wire.h>

#define enablePin 22
#define PIN_IR 15
#define motor1 3
#define motor2 4
#define vBatt A13

LIS331 xl;

unsigned long lastReceived = 0;
unsigned long lastSent = 0;
#define second 1000000

//controls
bool flip = 0;
int16_t thumbX = 0;
int16_t thumbY = 0;
uint16_t throt = 0;
uint16_t head = 0;
bool en = 0;

// Watchdog
const int loopTimeCtl = 0;
hw_timer_t *timer = NULL;

//**********************//
// MELTYBRAIN VARIABLES //
//**********************//
int16_t angle = 0;//LSB is one degree. Our current heading

int16_t meltyAngle = 0;//the commanded bearing angle for meltybrain control
uint16_t meltyThrottle = 0;

#define BEACON_SENSING 0x01//if this is defined, we are angle sensing using only the infrared receiver
#define ACCEL_SENSING 0x02//if this is defined, we are angle sensing using only the accelerometer
#define HYBRID_SENSING 0x03//if this is defined, we are angle sensing using both the beacon and the accelerometer
uint8_t senseMode = HYBRID_SENSING;

//BEACON
boolean beacon = false;//this variable keeps track of the status of the beacon internally. If this variable and the digital read don't match, it's a rising or falling edge

unsigned long beaconEdgeTime[2];//this is the array of rising edge acquisition times. We keep some history for better extrapolation

bool beaconEnvelopeStarted = false;
unsigned long beaconHoldTime;
#define BEACON_DEBOUNCE_TIME 2000//in microseconds
uint8_t beaconEdgesRecorded = 0;//this keeps track of how many beacon pulses we've seen, up to APPROXIMATION_ORDER. It resets when a revolution takes too long
#define REV_TIMEOUT 2000 //this (in ms) is how long a revolution can take before we say the robot is spinning too slowly to control and reset the algorithm

//ACCELEROMETER
void configAccelerometer(void);

#define STATE_IDLE 1
#define STATE_TANK 2
#define STATE_SPIN 3

//states
uint8_t state = STATE_IDLE;

void receivePacket(void);
void controllerConnect(void);

void runMeltyBrain(void);

uint16_t getBatteryVoltage() { //returns voltage in millivolts
  return analogRead(vBatt)/480.0;
}

void setMotorSpeed(int motor, int spd) {
  spd = constrain(spd, -128, 127);//make sure our speed value is valid. This lets us be lazier elsewhere
  //apply a deadband
  if(spd < 5 && spd > -5) spd = 0;

  if(motor == motor1) spd *= -1;

  // Change to ESP32 compatable
  // analogWrite(motor, map(spd, -128, 127, 64, 128));
}

void goIdle() {
  state = STATE_IDLE;
  PS4.setLed(128, 128, 128);

  digitalWrite(enablePin, HIGH);
  setMotorSpeed(motor1, 0);
  setMotorSpeed(motor2, 0);
}

void goTank() {
  state = STATE_TANK;
  PS4.setLed(255, 255, 0);

  digitalWrite(enablePin, LOW);
}

void goSpin() {
  state = STATE_SPIN;
  PS4.setLed(255, 0, 0);

  digitalWrite(enablePin, LOW);

  beaconEdgesRecorded = 0;
}

void feedWatchdog() {
  timerWrite(timer, 0);
}

//this runs if the robot code hangs! cut off the motors
void IRAM_ATTR ESTOP(){
  goIdle();
}

void setup() {
  Serial.begin(9600);
  
  PS4.attach(receivePacket);
  PS4.attachOnConnect(controllerConnect);
  PS4.begin("03:03:03:03:03:03");
  
  SPI.begin();
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);

  pinMode(PIN_IR, INPUT);

  timer = timerBegin(0, 80, true); //timer 0, div 80
  timerAttachInterrupt(timer, &ESTOP, true);
  timerAlarmWrite(timer, 1000000, false); //set time in us, 1 second
  timerAlarmEnable(timer); //enable interrupt

  // Change to ESP32 compatable
  // analogWriteFrequency(3, 250);//this changes the frequency of both motor outputs

  configAccelerometer();

  goIdle();
}

void loop() {

  //Bark bark
  feedWatchdog();

  //make sure comms haven't timed out
  long td = micros() - lastReceived;
  if(td > second && state != STATE_IDLE) {
    en = 0x0;
    goIdle();
  }

  //check if battery voltage is below 3.2V/cell cutoff (2.5V/cell under load)
  uint16_t batteryReading  = getBatteryVoltage();
  if((throt == 0 && batteryReading < 5)) {
    //disable motors
    digitalWrite(enablePin, HIGH);
    // RED LEDs
    PS4.setLed(255,0,0);
    PS4.sendToController();
    //sit until power is removed
    while(true) feedWatchdog();
  }

  switch(state) {
    case STATE_IDLE:

      if(en) {
        goTank();
      }
      
      break;
    case STATE_TANK:

      setMotorSpeed(motor1, thumbY+thumbX/2);
      setMotorSpeed(motor2, thumbY-thumbX/2);
      
      if(throt > 2) {
        goSpin();
      }

      if(!en) {
        goIdle();
      }
      break;
    case STATE_SPIN:

      runMeltyBrain();//manage all of the sensors and predict our current heading
      
      if(throt < 2) {
        goTank();
      }

      if(!en) {
        goIdle();
      }
    default:
      break;
  }

  send2Controller();
}
