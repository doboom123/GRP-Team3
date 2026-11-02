
 #include <Wire.h>



 

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
  foundMarker = Wire.read();

  while(Wire.available()){
    for(int i = 0; i< numBytes; i++){
      memcpy(&receivedDistance, instructionDistance, sizeof(float));
      memcpy(&receivedAngle, instructionAngle, sizeof(float));
    }
    msgLength++;
  }

}

