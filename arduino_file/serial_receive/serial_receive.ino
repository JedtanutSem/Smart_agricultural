#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX

boolean messageStarted = false;
byte dataBuffer[10];
byte dataBufferIndex = 0;

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600);
}
void loop() {
  //if we have serial data
  if (mySerial.available()) {
    // read one more data into the buffer
    dataBuffer[dataBufferIndex] = mySerial.read();
    // if no message start yet
    if (!messageStarted) {
      boolean dataBufferIndexIndrease = true;
      // if message is at lesat 2 byte long
      if (dataBufferIndex > 1) {
        if (dataBuffer[dataBufferIndex - 1] == 253 && dataBuffer[dataBufferIndex] == 255) {
          // we set the flag
          messageStarted = true;
          // reset the index
          dataBufferIndex = 0;
          dataBufferIndexIndrease = false;
        } //end if
      } //end if (dataBufferIndex > 1)

      // check if we need to increase the index
      if (dataBufferIndexIndrease)
        dataBufferIndex++;
    }
    // if message is started
    else
    {
      // if we got to the last byte of the message
      if (dataBufferIndex == 3)
      {
        // check the checksum
        if ((byte) (dataBuffer[0] +  dataBuffer[1]) == dataBuffer[2]) {
          int analogVal = dataBuffer[0] | dataBuffer[1] << 8;
          Serial.print("analogVal - ");
          Serial.println(analogVal);
        } else {
          Serial.println("Error reading data - fail on check sum");
        } //end if

        // reset the flag
        messageStarted = false;
      } //end if (dataBufferIndex == 3)

      // increase the buffer index
      dataBufferIndex++;
    } //end if (!messageStarted)

  } //end if mySerial.available()
}
