 

void printReceived() {
// Print on serial console
Serial.print("Offset received: ");
Serial.println(offset);
Serial.print("Message Length: ");
Serial.println(msgLength);
Serial.print("Instruction received: ");
for (int i=0;i<msgLength;i++) {
Serial.print(String(instruction[i])+"\t");
}
Serial.println("");
}

// function called when an I2C interrupt event happens
void receive() {
  msgLength = 0;
// Set the offset, this will always be the first byte.
// If there is information after the offset, it is telling us more about the command.
while (Wire.available()) {
instruction[msgLength] = Wire.read();
msgLength++;
}
}
