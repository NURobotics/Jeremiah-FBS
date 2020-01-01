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
  /*
  //now we build the return packet
  packet[0] = 0x7E;//start of packet byte
  
  //standard return packet
  uint16_t batteryVoltage = getBatteryVoltage();
  packet[1] = (byte) ((batteryVoltage & 0xFF00) >> 8);
  packet[2] = (byte) (batteryVoltage & 0x00FF);
  packet[3] = 0x00;

  Serial1.write(packet, 4);
  */
  //*/
  
  /*/this code is used in calibration testing
  packet[1] = (byte) (((beaconEdgeTime[0] - beaconEdgeTime[1]) & 0xFF000000) >> 24);
  packet[2] = (byte) (((beaconEdgeTime[0] - beaconEdgeTime[1]) & 0x00FF0000) >> 16);
  packet[3] = (byte) (((beaconEdgeTime[0] - beaconEdgeTime[1]) & 0x0000FF00) >> 8);
  packet[4] = (byte) ((beaconEdgeTime[0] - beaconEdgeTime[1]) & 0x000000FF);
  packet[5] = (byte) ((zAccel & 0xFF00) >> 8);
  packet[6] = (byte) (zAccel & 0x00FF);

  Serial1.write(packet, 7);
  //*/
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
