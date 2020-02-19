void receivePacket() {
  lastReceived = micros();
  
  if(PS4.data.button.options && senseMode != BEACON_SENSING) {//If we are in beacon-only mode due to a fault or deliberate code change, the sense select switch does nothing
    if(senseMode == HYBRID_SENSING) {
      senseMode = ACCEL_SENSING;
    } else {
      senseMode = HYBRID_SENSING;
    }
  }

  if (PS4.data.button.l3) {
    flip = !flip;//indicates whether the bot is inverted. 1 is inverted, 0 is normal
  }

  //thumbstick X
  thumbX = PS4.data.analog.stick.lx;
  //thumbstick Y
  thumbY = PS4.data.analog.stick.ly;
  //throttle
  throt = PS4.data.analog.button.l2;

  //heading
  //head = ((uint16_t) packet[7]) << 8 | ((uint16_t) packet[8]);

  //enable
  en = PS4.data.button.r1 && PS4.data.button.cross;

  if (state == STATE_SPIN) {
    //calculate the commanded direction and speed
    meltyThrottle = sqrt(thumbX*thumbX + thumbY*thumbY)/2;
    int16_t calcAngle = (int16_t) (atan2((double) thumbY, (double) thumbX*(flip*2-1))*180.0/PI);
    if (calcAngle < 0) calcAngle += 360;
    meltyAngle = (uint16_t) calcAngle;
  }

  // Flash controller if it's battery is about to run out
  if (PS4.data.status.battery < 3 && !PS4.data.status.charging) {
    PS4.setFlashRate(500, 500);
  }
}

void controllerConnect() {
  Serial.println("Connected!.");
  PS4.setLed(255,255,255);
}

void send2Controller() {
  if (PS4.isConnected() && micros() - lastSent > 10000) {
    PS4.sendToController();
    lastSent = micros();
  }
}
