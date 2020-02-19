bool readAccel(int16_t &x, int16_t &y, int16_t &z) {
  if (xl.newXData() || xl.newYData() || xl.newZData()) {
    xl.readAxes(x, y, z);
    return true;
  }
  return false;
}

void configAccelerometer() {
  Wire.begin();
  xl.setI2CAddr(0x19);
  xl.begin(LIS331::USE_I2C);
  xl.setFullScale(LIS331::HIGH_RANGE);
  xl.setODR(LIS331::DR_1000HZ);
}

//we read from the accelerometer much slower than the accelerometer's data rate to make sure we always get new data
//reading the same data twice could mess with the prediction algorithms
//a better method is to use the interrupt pin on the accelerometer that tells us every time new data is available
unsigned long measurementPeriod = 10000;//in microseconds. Represents 100Hz

uint16_t robotPeriod[2];//measured in microseconds per degree, with some memory for discrete integration

//this is the times we measured the accelerometer at. We keep some history for extrapolation
unsigned long accelMeasTime[2];

//this angle (degrees) is calculated only using the accelerometer. We keep it separate to keep our discrete integration algorithms operating smoothly
//the beacon sets our heading to 0, which would mess up the discrete integration if allowed to affect this variable directly
//instead we utilize a trim variable. In Accel control mode, the user controls trim with the encoder wheel. in hybrid mode, the beacon controls trim
uint16_t accelAngle = 0;

//in degrees, this angle is added to the accel angle as adjusted by the beacon or the driver
uint16_t accelTrim = 0;

uint16_t angleAtLastMeasurement;

//we make this global so that comms can send it out for calibration purposes
int16_t zAccel;

void runAccelerometer() {
  //this uses blocking I2C, which makes it relatively slow. But given that we run our I2C at 1.8MHz we will likely be okay
  if(micros() - accelMeasTime[0] > measurementPeriod) {
    //shift all of the old values down
    for(int i=1; i>0; i--) {
      accelMeasTime[i] = accelMeasTime[i-1];
    }
    //put in the new value
    accelMeasTime[0] = micros();
    
    int16_t xAccel;
    int16_t yAccel;
    int16_t zAccel;
    if (!readAccel(xAccel, yAccel, zAccel)) {
      senseMode = BEACON_SENSING; //if communication with the accelerometer ever fails, we revert to beacon-only mode
      Serial.println("accelerometer failed");
    }

    //shift all of the old values down
    for (int i = 1; i > 0; i--) {
      robotPeriod[i] = robotPeriod[i-1];
    }
    
    //put in the new value
    //this equation has been carefully calibrated for this bot. See here for explanation:
    //https://www.swallenhardware.io/battlebots/2018/8/12/halo-pt-9-accelerometer-calibration
    robotPeriod[0] = (uint32_t) (726 / sqrt((double) (zAccel-225)/522));

    //give up if the bot is moving too slowly
    if(zAccel < 400) return;

    //find the new angle
    //TRIANGULAR INTEGRATION
    uint32_t deltaT = accelMeasTime[0] - accelMeasTime[1];
    angleAtLastMeasurement = (angleAtLastMeasurement + (deltaT/robotPeriod[0] + deltaT/robotPeriod[1])/2) % 360;

    accelAngle = angleAtLastMeasurement;
    
  }
  else {
    //if it isn't time to check the accelerometer, predict our current heading
    //predict the current velocity by extrapolating old data
    uint32_t newTime = micros();
    uint32_t periodPredicted = robotPeriod[1] + (newTime - accelMeasTime[1]) * (robotPeriod[0] - robotPeriod[1]) / (accelMeasTime[0] - accelMeasTime[1]);

    //predict the current robot heading by triangular integration up to the extrapolated point
    uint32_t deltaT = newTime - accelMeasTime[0];
    accelAngle = (angleAtLastMeasurement + (deltaT/periodPredicted + deltaT/robotPeriod[0])/2) % 360;
  }
}
