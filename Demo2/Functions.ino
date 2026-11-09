
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
  piOn = Wire.read();
  foundMarker = Wire.read();

  while(Wire.available()){
    for(int i = 0; i< numBytes; i++){
      instructionDistance[i] = Wire.read();
    }
    for(int i = 0; i< numBytes; i++){
      instructionAngle[i] = Wire.read();
    }
    memcpy(&receivedDistance, instructionDistance, sizeof(float));
    memcpy(&receivedAngle, instructionAngle, sizeof(float));
    msgLength++;
  }

}

