
// printReceived helps us see what data we are getting from the leader


void printReceived() {
// Print on serial console
Serial.print("Instruction received: ");
for (int i=0;i<msgLength;i++) {
Serial.print(String(instruction[i])+"\t");
}
Serial.println("");
}

// function called when an I2C interrupt event happens
void receive(int numBytes) {
  piOn = Wire.available();

  while(Wire.available()){
    for(int i = 0; i< numBytes; i++){
      instructionDistance[i] = Wire.read();
    }
    for(int i = 0; i< numBytes; i++){
      instructionAngle[i] = Wire.read();
    }
    //memcpy(&receivedDistance, instructionDistance, sizeof(float));
    //memcpy(&receivedAngle, instructionAngle, sizeof(float));
    //msgLength++;
    receivedDistance = instructionDistance[0] * 100 + instructionDistance[1] * 10 + instructionDistance[2] + instructionDistance[3] * 0.1;
    receivedAngle = instructionAngle[0] * 10 + instructionAngle[1] + instructionAngle[2] * 0.1 + instructionAngle[3]* 0.01;
    foundMarker = (receivedDistance > 5 ? true : false);
  }

}

