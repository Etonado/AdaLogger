 /*Timer Interrupt
  * author: Erik Winkler
  * date:10.12.2020
  * Notes:
  * internal 16-bit timers of ATmega2560 TIM4 and TIM5 are used in Input Cature Mode
  * output signals are provided by pin 4 (for ICP4) and pin 5 (for ICP5)
  * 
  * serial read for COMP mode Timer 0
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

#define SAMPLE_FREQ 1
#define CLK_FREQ 8000000
#define TIMEOUT 10000
#define cardSelect 4
#define BUFFER_SIZE 1000


//private variables
float time = 0;
String msg = "";
File logfile;
int count = 0;
char filename[15];
char message[BUFFER_SIZE];
int charPos = 0;
int readData();
void setInterrupt();
void logMsg();
void error(uint8_t errno);




ISR(TIMER1_COMPA_vect)
{
  TCNT1=0;            //reset timer
  //readData();
  if(count<20)
  {
    logMsg();
    count++;
  }

}


void setup() {
  //set BAUDRATE for serial communication
  Serial.begin(115200);
  while(!Serial){/*wait for serial*/}
  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
  //configurate timers
  setInterrupt();

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

  Serial.println("Ready!");
}


void loop() 
{
  readData();
}

int readData()
{
  int time_now = millis();
  while (!Serial.available()) 
  {
    //wait for input
    if(millis()-time_now>TIMEOUT)return 0;
  }
  while (Serial.available() && charPos < BUFFER_SIZE ) 
  {
    message[charPos] = Serial.read();  //read until timeout
    charPos++;
  }
  Serial.println("on");

  
  return 1;
}

void logMsg()
{
  if(charPos>0)
  {

  digitalWrite(8, HIGH);
  File logfile = SD.open(filename, FILE_WRITE);
  
  logfile.print(millis()/1000,3);
  logfile.print(",");
  for(int i = 0; i< charPos; i++){
    Serial.print(message[i]);
    logfile.println(message[i]);
  }
  logfile.close();
  charPos=0;
  digitalWrite(8, LOW);
  }

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

