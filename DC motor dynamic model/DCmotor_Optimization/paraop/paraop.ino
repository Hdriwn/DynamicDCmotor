
long prevT = 0;
#define CON1 9  // Yellow
#define CON2 8  // Green
#define EN1 2
#define EN2 3
#define CS A1
#define pwm 10
double pos = 0;
// int pwmVal =130;
double pos1 = 0;
double output = 0;
double volts = 0.00;
volatile int encoderPos = 0;      // current encoder position
volatile int lastEncoderPos = 0;  // previous encoder position
unsigned long lastTime = 0;       // last time recorded
float rpm = 0.0;                  // calculated RPM value


#include <math.h>
const int ledPin =13;
const int rgb = 11;
int cnt=0;
int rotpin =5;
const byte encoderPinA = 2;//outputA digital pin2
const byte encoderPinB = 3;//outoutB digital pin3
volatile long int count = 0;
long int constCount = 0;
long int previousCount = 0;
#define CR 12//complete rotation
int i =0;
#define readA bitRead(PIND,2)//faster than digitalRead()
#define readB bitRead(PIND,3)

// Pin declares
int pwmPin = 6; // PWM output pin for motor 1
int dirPin1 = 8;
int dirPin2 = 9; 

// Pin declares
int fsrPin = A3; // input pin for FSR sensor

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
long int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 518;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;
double lastTimeAtSurface = 0;
// Kinematics variables
double xh = 0;           // position of the handle [m]
double lastXh = 0;     //last x position of the handle    
double lastts = 0;   
double vh = 0;         //velocity of the handle
double lastVh = 0;     //last velocity of the handle
double lastLastVh = 0; //last last velocity of the handle
double kkk=0;
// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
float xhp=0;
float xhh=0;
float xx_wall=0;
float x_wall=0;
float x=0;
void setup() {
  Serial.begin(115200);
  pinMode(CON1, OUTPUT);
  pinMode(CON2, OUTPUT);
  pinMode(pwm, OUTPUT);

  pinMode(EN1, INPUT_PULLUP);
  pinMode(EN2, INPUT_PULLUP);

    pinMode(EN1, INPUT_PULLUP);
  pinMode(EN2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EN1), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EN2), isrB, CHANGE);
  // myEnc.write(0);
  // Pins D9 and D10 - 122 Hz
TCCR1A = 0b00000001; // 8bit
TCCR1B = 0b00001001; // x1 fast pwm
}

void loop() {
  setMotor3();
  constCount = 0;
  rawPos = 0;
  count=0;

      pwmOut(0);
delay(2000);
      pos1 = rawPos;
      Serial.println("<<<<<<<<<<<.>>>>>>>>>>>>>>start2<<<<<<<<<<<.>>>>>>>>>>>>>>");
uint32_t period = 0.16667 * 60000L;       // 10 seconds corresponds to 0.16667, here 1 corresponds to 1 minute
for( uint32_t tStart = millis();  (millis()-tStart) < period;  ){
  pwmOut(125);
  delay(0);
      // readrpm();
        noInterrupts();
  constCount = count;
  rawPos = constCount; 
  interrupts();
  Serial.println(rawPos);

}
  Serial.println(rawPos);
  Serial.println("---------------last--------------------");

count=0;
pos1=0;
setMotor3();
delay(2000);
}

void pwmOut(int out) {
  if (out > 0) {
    analogWrite(pwm, out);
    setMotor1();
  } else if (out < 0) {
    analogWrite(pwm, abs(out));
    setMotor2();
  } else {
    analogWrite(pwm, abs(out));
    setMotor3();
  }
}
void setMotor1() {

  digitalWrite(CON1, HIGH);
  digitalWrite(CON2, LOW);
}
void setMotor2() {

  digitalWrite(CON1, LOW);
  digitalWrite(CON2, HIGH);
}
void setMotor3() {

  digitalWrite(CON1, LOW);
  digitalWrite(CON2, LOW);
}

void isrA() {
  if (readA == LOW) { 
    if (readB == LOW) {
      count++;
    } else {
      count--;
    }
  } else { 
    if (readB == LOW) {
      count--;
    } else {
      count++;
    }
  }
}

void isrB() {
  if (readB == LOW) {
    if (readA == LOW) {
      count--;
    } else {
      count++;
    }
  } else {
    if (readA == LOW) {
      count++;
    } else {
      count--;
    }
  }
}

