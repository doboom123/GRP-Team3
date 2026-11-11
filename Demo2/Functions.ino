
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

  /**void receive(int numBytes) {
    receivedAngle = 0;
    receivedDistance = 0;

  // Only accept complete 9-byte messages
  if (numBytes != 9) {
    while (Wire.available()) {
      Wire.read();
    }
    return;
  }
  while (Wire.available()) {

  // Byte 0: marker detection flag
  uint8_t markerFlag = Wire.read();

  // Bytes 1-4: distance
  uint8_t distanceBytes[4];
  for (int i = 0; i < 4; i++) {
    distanceBytes[i] = Wire.read();
    Serial.print(distanceBytes[i]);
    receivedDistance += distanceBytes[i] * pow(10.0,(2-i));
  }

  // Bytes 5-8: angle
  uint8_t angleBytes[4];
  for (int i = 0; i < 4; i++) {
    angleBytes[i] = Wire.read();
    Serial.print(angleBytes[i]);

  }
  receivedAngle += angleBytes[1] * 10.0;
  receivedAngle += angleBytes[2];
  receivedAngle += angleBytes[3] / 10.0;
  if (!angleBytes[0]) {
  receivedAngle = receivedAngle * -1;
  }
    Serial.print("\n");
   Serial.print(receivedDistance);
    Serial.print(" ");
   Serial.print(receivedAngle);
    Serial.print(" ");
    Serial.print("\n");
  }}
**/
//     if(Wire.available()){}
//     for(int i = 0; i< 4; i++){
      
//       instructionDistance[i] = Wire.read();
//       Serial.print(i);
//       Serial.print(" ");
//       Serial.print(instructionDistance[i]);
//       Serial.print(" ");}
//     }
//     Serial.print(receivedDistance);
//     Serial.print("\n");

//     //memcpy(&receivedDistance, instructionDistance, sizeof(float));
//     //memcpy(&receivedAngle, instructionAngle, sizeof(float));
//     //msgLength++;
//     receivedDistance = instructionDistance[0] * 100 + instructionDistance[1] * 10 + instructionDistance[2] + instructionDistance[3] * 0.1;
//     receivedAngle = instructionAngle[0] * 10 + instructionAngle[1] + instructionAngle[2] * 0.1 + instructionAngle[3]* 0.01;
//     foundMarker = (receivedDistance > 5 ? true : false);
// }

