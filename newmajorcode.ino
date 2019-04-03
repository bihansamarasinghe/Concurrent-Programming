//Define gyroscope module

#include <Wire.h>                                 //Add Wire.h library
#define MPU_addr 0x68                            //Define MPU6050  Device Address
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,Tmp;            //Define direction objects for Wire library

//Define Upcounter switch

#define SW1_BIT       B01000000               //Define PD6 as a switch 1 bit
#define SW1_PRESSED   !(PIND & SW1_BIT)       /*PIND is the input register variable It 
                                              will read all of the digital input pins at the same time*/

//define Mode change Switch

#define SW2_BIT       B00100000               //Define PD6 as a Switch 2 bit
#define SW2_PRESSED   !(PIND & SW2_BIT)       //Define bit pattern after pressing the Sw2 

//Define Arduino ports for 74HC595 shift register

#define DATA          B10000000               //Define Input data pin to 74HC595 shift register
#define CLOCK         B00010000               //Define Clock pin to 74HC595 shift register

/* Meaning of Latch - A circuit which retains whatever 
output state results from a momentary input signal 
until reset by another signal.*/

#define LATCH         B00001000               //Define Latch pin to 74HC595 shift register           
#define DATA_ON       PORTD |= DATA           //Define Data on state bit pattern
#define DATA_OFF      PORTD &= ~DATA          //Define Data off state bit pattern
#define CLOCK_ON      PORTD |= CLOCK          //Define Clock on state bit pattern
#define CLOCK_OFF     PORTD &= ~CLOCK         //Define Clock off state bit pattern
#define LATCH_ON      PORTD |= LATCH          //Define Latch on state bit pattern
#define LATCH_OFF     PORTD &= ~LATCH         //Define Latch off state bit pattern


//Define Traffic light states

#define TRAFFIC_ON        B00111111
#define TRAFFIC_OFF       B00000000

//Define LED Control bit patterns

#define LED_OFF         B00000000
#define LED_RED1        B00000100
#define LED_RED2        B00001000
#define LED_GREEN1      B00000001
#define LED_GREEN2      B00100000
#define LED_YELLOW1     B00000010
#define LED_YELLOW2     B00010000

//Define Maximum number of Traffic light SET1 and SET2 changing states
#define MAX_STATES    10

//Set byte array including seven segment display numbers and leters in binary format

byte sevenSegArray[] = {
                    B00111111, // 0
                    B00000110, // 1
                    B01011011, // 2
                    B01001111, // 3
                    B01100110, // 4
                    B01101101, // 5
                    B01111101, // 6
                    B00000111, // 7
                    B01111111, // 8
                    B01101111, // 9
                    B01110111, // A
                    B01111100, // B
                    B00111001, // C
                    B01011110, // D
                    B01111001, // E
                    B01110001  // F
                 };

                    
unsigned long count = 0; //Array element counting variable
boolean beatOn = false;  //Define variable of HeartBeat state

//Display Gyroscope Axis data on Seven Segment

byte GyroArray[] = {
                    B01110001, // F
                    B00111110, // U
                    B00111000, // L
                    B01001000, // PR(r)
                    B00000110, // PL(l)
                  };
                  
  int GyroAxis=0;       //Return GyroArray function axis variable

  byte TrafficTArray[]={
                          B01111000, //Define character "t" on seven segment if traffic lights working
  };
  byte ModeArray[]    ={
                          B01111000, // Define Modes in Program
  };


byte sevenSegArray1[] = {
                          B01110001, //Display F in Seven Segment for FLAT
                          B00111110, //Display U in Seven Segment for UPSIDE DOWN
                          B00000110, //Display l in Seven Segment for LEFT
                          B01001000, //Display r in Seven Segment for RIGHT
                          };

static unsigned int  MODE = 0; 

//Define function of lights module

void LightsModule(){
  
  const byte states[MAX_STATES]={
                                  LED_OFF,
                                  LED_YELLOW1                 |LED_YELLOW2,
                                  LED_RED1                    |LED_RED2,
                                  LED_RED1    |LED_YELLOW1    |LED_RED2,
                                  LED_GREEN1                  |LED_RED2,
                                  LED_YELLOW1                 |LED_RED2,
                                  LED_RED1                    |LED_RED2,
                                  LED_RED1    |LED_RED2       |LED_YELLOW2,
                                  LED_RED1                    |LED_GREEN2,
                                  LED_RED1                    |LED_YELLOW2
                          
                                  };

  //Define delay times for Traffic Lights in Normal Condition
  
  const unsigned long timeEqual[MAX_STATES]={
    1000,
    5000,
    1000,
    2000,
    9000,
    2000,
    1000,
    2000,
    9000,
    2000,
    
  };
  
  //Define delay times for Traffic Lights in Set1 Condition
   
  const unsigned long timeSet1[MAX_STATES]={
    10,
    5000,
    1000,
    2000,
    12000,
    2000,
    1000,
    2000,
    6000,
    2000,
    
  };
  
   //Define delay times for Traffic Lights in Set2 Condition
   
  const unsigned long timeSet2[MAX_STATES]={
    10,
    5000,
    1000,
    2000,
    6000,
    2000,
    1000,
    2000,
    12000,
    2000,
    
  };

//Define delay time for traffic lights

  static unsigned long  requiredTime=0;
  static int            currentState=0;
  unsigned long         currentTime=millis();


  if((long)(currentTime-requiredTime)>=0){
 
    
    PORTB=states[currentState]|(B11000000&PINB);
   
     if (GyroAxis<=3){
    requiredTime=currentTime+timeEqual[currentState];
    }
   else if (GyroAxis=5){
    requiredTime=currentTime+timeSet1[currentState];
    }
   else if (GyroAxis=4){
    requiredTime=currentTime+timeSet2[currentState];
       
   }
   currentState=(currentState+1); //After Starting Normal condition ,state changes will change neglecting yellow and yellow state
   if(currentState==MAX_STATES){
    currentState=2;
    }
  }
}

//Define Groscope sensor

void GyroSensor(){

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  Serial.print("AcX = ") ; Serial.print(AcX);
  Serial.print("AcY = ") ; Serial.print(AcY);
  Serial.print("AcZ = ") ; Serial.print(AcZ);
  Serial.print("Temp = ") ; Serial.print(Tmp/340.00+36.53);
  Serial.print("GyX = ") ; Serial.print(GyX);
  Serial.print("GyY = ") ; Serial.print(GyY);
  Serial.print("GyZ = ") ; Serial.println(GyZ);

if(AcZ > 14000){

        Serial.println("F") ;
        GyroAxis=1;
        writeToShift(B01110001);
      }

else if(AcZ < -14000){
  
        Serial.println("U") ;
        writeToShift(B00111110); 
        GyroAxis=2;
      }
      
else if(AcY > 14000 && AcX >0){
  
        Serial.println("L") ; 
        writeToShift(B00000110);
        GyroAxis=3;
}

else if(AcX>14000 && AcY>0){
 
        Serial.println("PR") ; 
        writeToShift(B01001000);
        GyroAxis=4;
}

else if(AcX > 14000 && AcY < 0){
   
        Serial.print("PL") ; 
        writeToShift(B00000110);
        GyroAxis=5;
    }

}

/*=================
Switch 1 - Counter Function
/==================*/

void waitForSW1Press() {
  static unsigned long rredTime = 300;
  static unsigned long currentTime = millis();
  if (SW1_PRESSED) {
    if (( (long)(millis() - currentTime)) >= rredTime) {
        writeToShift(sevenSegArray[count++ % 16]);
        currentTime = millis();

        Serial.println("model sw2");
    } 
  } else {
      currentTime = millis();
    }
}

/*=================
Switch 2 - Mode Change Function
/==================*/

void waitForSW2Press() {
  static unsigned long reqTime = 300;
  static unsigned long currentTime = millis();
  if (SW2_PRESSED) {
    if (( (long)(millis() - currentTime)) >= reqTime) {
      Serial.println("model sw2");
      MODE=(MODE+1) % 5;
      currentTime = millis();
    } 
  } else {
      currentTime = millis();
    }
}

/*=================
Control Seven Segement Function
/==================*/

void writeToShift(byte val) {
  byte mask = B10000000;
  LATCH_ON;
  LATCH_OFF;
  CLOCK_ON;
  CLOCK_OFF;
  for(byte i=1; i<9; i++) {
    if((val & mask) != 0) {
      DATA_ON;
    } else {
      DATA_OFF;
    }
    CLOCK_ON;
    mask >>= 1;
    CLOCK_OFF;
  }
  LATCH_ON;
}

/*=================
Control Heartbeat Function
/==================*/

void heartBeat() {
  static unsigned long reedTime = 500;
  static unsigned long currentTime = millis();
    if (( (long)(millis() - currentTime)) >= reedTime) {
      if(MODE==1|MODE==3){
        writeToShift(sevenSegArray[count % 16] | (beatOn?B10000000:B00000000));
      
      }
      else if(MODE==2|MODE==4) {
        writeToShift(GyroArray[GyroAxis]| (beatOn?B10000000:B00000000));
      }
       else if (MODE==0){
        
       writeToShift(TrafficTArray[0]| (beatOn?B10000000:B00000000));
       }
      beatOn = !beatOn;
      currentTime = millis();
      }
    } 
/*=================
Define Setup functin
/==================*/
void setup() {
  
DDRB = B00111111;

  // for gyroscope
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
 
  
  // for switch and 7 segment 
  DDRD &= SW1_BIT;
  PORTD |= SW1_BIT;
  DDRD &= SW2_BIT;
  PORTD |= SW2_BIT;
  DDRD |= DATA;
  DDRD |= CLOCK;
  DDRD |= LATCH;

  //for testing
  Serial.begin(9600);
  Serial.println("setup");

writeToShift(ModeArray[0]);
}

void loop() {

waitForSW2Press();

Serial.println("model");
  if(MODE==0){

    writeToShift(TrafficTArray[0]);
    Serial.println("F1"); 
    LightsModule();
    heartBeat();
  }
 else if(MODE==1){
  
    Serial.println("F2"); 
    DDRB= TRAFFIC_OFF;
    waitForSW1Press();
    heartBeat();
 }
else if(MODE==2){
  
    DDRB= TRAFFIC_OFF;
    GyroSensor();
    heartBeat();
    Serial.println("F3");
    
}
else if(MODE==3){
  
    DDRB =TRAFFIC_ON;
    LightsModule();
    heartBeat();
    waitForSW1Press();
    Serial.println("F4");
}
else if(MODE==4){
  
    DDRB =TRAFFIC_ON;
    LightsModule();
    heartBeat();
    GyroSensor();
    Serial.println("F5");
}
else{
  Serial.println("erro");
}

}
