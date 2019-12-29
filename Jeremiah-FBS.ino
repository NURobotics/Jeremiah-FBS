#include <PS4Controller.h>
#include <SPI.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#define enablePin 22
#define PIN_IR 15
#define motor1 3
#define motor2 4
#define vBatt A0

//serial
#define SERIAL_WAIT 0
#define SERIAL_PACKETSTART 1
byte serialState = 0;
unsigned long packetTime = 0;
unsigned long packetTimeout = 200;
byte packet[10];
byte bytesRead = 0;

unsigned long lastReceived = 0;

//controls
byte flip = 0;
int16_t thumbX = 0;
int16_t thumbY = 0;
uint16_t throt = 0;
uint16_t head = 0;
byte en = 0;
//leds

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

//states
uint8_t state = 1;

#define STATE_IDLE 1
#define STATE_TANK 2
#define STATE_SPIN 3

void pollSerial(void);
void receivePacket(void);

void runMeltyBrain(void);

uint16_t getBatteryVoltage() { //returns voltage in millivolts
  return (analogRead(vBatt)*49)/3;
}


void setMotorSpeed(int motor, int spd) {
  spd = constrain(spd, -100, 100);//make sure our speed value is valid. This lets us be lazier elsewhere
  //apply a deadband
  if(spd < 5 && spd > -5) spd = 0;

  if(motor == motor1) spd *= -1;

  // Change to ESP32 compatable
  // analogWrite(motor, map(spd, -100, 100, 64, 128));
}

void goIdle() {
  state = STATE_IDLE;

  //digitalWrite(enablePin, HIGH);
  setMotorSpeed(motor1, 0);
  setMotorSpeed(motor2, 0);
}

void goTank() {
  state = STATE_TANK;

  digitalWrite(enablePin, LOW);
}

void goSpin() {
  state = STATE_SPIN;

  digitalWrite(enablePin, LOW);

  beaconEdgesRecorded = 0;
}

void feedWatchdog() {
  esp_task_wdt_reset();
}

//this runs if the robot code hangs! cut off the motors
void watchdog_isr() {
  digitalWrite(enablePin, HIGH);
}

void setup() {
  Serial.begin(57600);
  Serial1.begin(57600);
  SPI.begin();
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);

  pinMode(PIN_IR, INPUT);

  //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 1800000, I2C_OP_MODE_IMM);//1.8MHz clock rate
  //Wire.begin();

  esp_task_wdt_init(1, true);

  // Change to ESP32 compatable
  // analogWriteFrequency(3, 250);//this changes the frequency of both motor outputs

  configAccelerometer();

  goIdle();
}

void loop() {

  //Bark bark
  feedWatchdog();

  //check for incoming messages
  pollSerial();

  //make sure comms haven't timed out
  if(micros() - lastReceived > 1000*1000 && state != STATE_IDLE) {
    en = 0x0;
    goIdle();
  }

  /*
  //check if battery voltage is below 3.2V/cell cutoff (2.5V/cell under load)
  uint16_t batteryReading  = getBatteryVoltage();
  if((throt == 0 && batteryReading < 3200*4)) {
    //disable motors
    digitalWrite(enablePin, HIGH);
    //blank LEDs
    strip.clear();
    strip.show();
    //sit until power is removed
    while(true) feedWatchdog();
  }
  */

  switch(state) {
    case STATE_IDLE:

      if(en == 0xAA) {
        goTank();
      }
      
      break;
    case STATE_TANK:

      setMotorSpeed(motor1, thumbY+thumbX/2);
      setMotorSpeed(motor2, thumbY-thumbX/2);
      
      if(throt > 2) {
        goSpin();
      }

      if(en != 0xAA) {
        goIdle();
      }
      break;
    case STATE_SPIN:

      runMeltyBrain();//manage all of the sensors and predict our current heading
      
      if(throt < 2) {
        goTank();
      }

      if(en != 0xAA) {
        goIdle();
      }
    default:
      break;
  }
}
