 /* Arduino Datalogger
  * Author: Erik Winkler
  * Date: 18.07.2023
  * Notes:
  * 
  * 
  * 
  */
  
  /*possible prescaler settings
   * CSn2   CSn1   CSn0     presc           
   *  0       0      1      no prescaling           
   *  0       1      0      8                 
   *  0       1      1      64                
   *  1       0      0      256               
   *  1       0      1      1024              
   *  t_ovf=2^16*presc/(16MHz)
   */ 


#include <avr/interrupt.h>
#include <SPI.h>
#include <SD.h>
#include "string.h"

#define SAMPLE_FREQ 10
#define CLK_FREQ 8000000
#define TIMEOUT 10000
#define cardSelect 4
#define BIT_STRING_LENGTH 18

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

// Time
union {uint32_t i; byte b[4];} t_start;


// Checksum
union {unsigned short s; byte b[2];} checksum;


// Parameters
bool imu_sync_detected = false;  // check if the sync byte (0xFA) is detected
byte in[50];  // array to save data send from the IMU


//private variables
//private variables
uint8_t debug = 0;
String msg = "";
uint8_t sample = 0;

File logfile;
int count = 0;
char filename[15];

void setInterrupt();
void logMessage();
void error(uint8_t errno);

int read_imu_data(void);
void check_sync_byte(void);
unsigned short calculate_imu_crc(byte data[], unsigned int length);
unsigned char calculateChecksum(unsigned char data[], unsigned int length);

void run();

int setMessage();
void logMsg();
int readData();


ISR(TIMER1_COMPA_vect)
{
  TCNT1=0;            //reset timer
  //readData();
  if(count<10)
  {
    //run();
    sample = 1;
    count++;
  }

}


void setup() {
  //set BAUDRATE for serial communication
  Serial.begin(9600);
  while(!Serial){/*wait for serial*/}
  Serial1.begin(57600);
  while(!Serial1){/*wait for serial*/}

  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);

  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
    error(2);
  }

  strcpy(filename, "log00.txt");
  for (uint8_t i = 0; i < 100; i++) {
    filename[3] = '0' + i/10;
    filename[4] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }


  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error(3);
  }
  
  Serial.print("Writing to "); 
  Serial.println(filename);

  //configurate timers
  delay(1000);

  logMessage();

  //setInterrupt();

}


void loop() 
{
if(count<20)
{
  run();
  count ++;
}
delay(20);

}

void run()
{

  float time_now = millis();
  uint8_t timeout = 0;
  int ret = 0;


  while(!timeout && !ret)
  {
    imu_sync_detected = false;
    if((millis() - time_now) > TIMEOUT) timeout = 1;
    
    // Check if new IMU data is available
    if (Serial1.available() > 4)
    {
      check_sync_byte();
    }

    // If sync byte is detected, read the rest of the data
    if (imu_sync_detected) 
    {
      ret = 1;
      ret = read_imu_data();     
    }
    delay(1);
  }
  if(timeout) Serial.print("Timeout");
  else logMessage();

  sample = 0;


}


void logMessage()
{
  digitalWrite(8, HIGH);
  File logfile = SD.open(filename, FILE_WRITE);
  
  logfile.print(millis(),3);
  logfile.print(",");
  if(debug)Serial.print(msg);
  logfile.println(msg);
  logfile.close();
  msg = "";
  digitalWrite(8, LOW);
}


void setInterrupt()
{
  //disable global interrupt
  cli();
  /*timer 1 settings */
  // timer 1 is set up in ctc mode
  // reset timer 1
  TCCR1A=0;
  TCCR1B=0;
  //set prescaler to 1024
  TCCR1B |= 1<<CS12 | 1<<CS10;
  //set ctc mode
  TCCR1B |= 1<<WGM12;
  // reset timer 1 counter
  TCNT1 = 0;          
  OCR1A = (int)((CLK_FREQ/1024)/SAMPLE_FREQ)-1;
  TIMSK1 |= 1<<OCIE1A; // Enable Timer 1 interrupt
  // re-enable interrupts
  sei(); 
}

void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
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
int read_imu_data(void) {
  int len = BIT_STRING_LENGTH - 1;
  Serial1.readBytes(in, len);

  checksum.b[0] = in[BIT_STRING_LENGTH - 2];
  checksum.b[1] = in[BIT_STRING_LENGTH - 3];

  Serial.print("Haja");
  if ((calculate_imu_crc(in, BIT_STRING_LENGTH - 3) == checksum.s)) 
  {
    for (int i = 0; i < 4; i++) 
    {
      yaw.b[i] = in[3 + i];
      pitch.b[i] = in[7 + i];
      roll.b[i] = in[11 + i];
      //t_start.b[i] = in[2+15 + i];
      //W_x.b[i] = in[15 + i];
      //W_y.b[i] = in[19 + i];
      //W_z.b[i] = in[23 + i];
      //a_x.b[i] = in[27 + i];
      //a_y.b[i] = in[31 + i];
      //a_z.b[i] = in[35 + i];
    }
    msg = String(yaw.f) + "," + String(pitch.f) + "," + String(roll.f);
    if(1)Serial.println(String(yaw.f) + "," + String(pitch.f) + "," + String(roll.f));
    return 1;
    //Serial.println(String(W_x.f) + "," + String(W_y.f) + "," + String(W_z.f));
  }
  return 0;
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

