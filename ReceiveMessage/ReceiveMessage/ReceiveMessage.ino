// Copyright (c) 2018 Flight Dynamics and Control Lab

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


// NOTE: Before running this code, you must configure the IMU as
// defined in the README file. If you want to get different data,
// you must update the respective parameters while configuring the
// IMU. 
// Further, this assumes you are using an Arduino board which has
// more than one serial ports. Serial1 is connected to the IMU and
// Serial is connected to a computer through a USB cable. If you do
// not wish to visualize data in the serial monitor, you may use
// Serial as the IMU port.


// Function declarations
void read_imu_data(void);
void check_sync_byte(void);
unsigned short calculate_imu_crc(byte data[], unsigned int length);
unsigned char calculateChecksum(unsigned char data[], unsigned int length);


// Union functions for byte to float conversions
// IMU sends data as bytes, the union functions are used to convert
// these data into other data types

// Attitude data
union {float f; byte b[4];} yaw;
union {float f; byte b[4];} pitch;
union {float f; byte b[4];} roll;

// Angular rates
union {float f; byte b[4];} W_x;
union {float f; byte b[4];} W_y;
union {float f; byte b[4];} W_z;

// Acceleration
union {float f; byte b[4];} a_x;
union {float f; byte b[4];} a_y;
union {float f; byte b[4];} a_z;

// Checksum
union {unsigned short s; byte b[2];} checksum;


// Parameters
bool imu_sync_detected = false;  // check if the sync byte (0xFA) is detected
byte in[100];  // array to save data send from the IMU



void setup() {

  // Start Serial for printing data to the Serial Monitor
  Serial.begin(115200);

  // Start Serial1 for IMU communication
  Serial1.begin(9600);
}


void loop() {
  imu_sync_detected = false;

  // Check if new IMU data is available
  if (Serial1.available() > 4) check_sync_byte();

  // If sync byte is detected, read the rest of the data
  if (imu_sync_detected) read_imu_data();
  
  delay(1);
}



// Check for the sync byte (0xFA)
void check_sync_byte(void) {
  uint16_t count = 0;

  for (int i = 0; i < 6; i++) {
    Serial1.readBytes(in, 1);
    if (in[0] == 0xFA) {
      imu_sync_detected = true;
      break;
    }      
  } 
}


// Read the IMU bytes
void read_imu_data(void) {
  int len = 17;
  Serial1.readBytes(in, len);

  checksum.b[0] = in[16];
  checksum.b[1] = in[15];


  if ((calculate_imu_crc(in, 15) == checksum.s)) {
    for (int i = 0; i < 4; i++) {
      yaw.b[i] = in[3 + i];
      pitch.b[i] = in[7 + i];
      roll.b[i] = in[11 + i];
      //W_x.b[i] = in[15 + i];
      //W_y.b[i] = in[19 + i];
      //W_z.b[i] = in[23 + i];
      //a_x.b[i] = in[27 + i];
      //a_y.b[i] = in[31 + i];
      //a_z.b[i] = in[35 + i];
    }

    Serial.println(String(yaw.f) + "," + String(pitch.f) + "," + String(roll.f));
    //Serial.println(String(W_x.f) + "," + String(W_y.f) + "," + String(W_z.f));
    
  }
}


// Calculate the 16-bit CRC for the given ASCII or binary message.
unsigned short calculate_imu_crc(byte data[], unsigned int length)
{
  unsigned int i;
  unsigned short crc = 0;
  for(i=0; i<length; i++){
    crc = (byte)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (byte)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}

// Calculates the 8-bit checksum for the given byte sequence. 
unsigned char calculateChecksum(unsigned char data[], unsigned int length) 
{ 
    unsigned int i; 
    unsigned char cksum = 0; 
 
    for(i=0; i<length; i++){ 
        cksum ^= data[i]; 
    } 
 
    return cksum; 
} 