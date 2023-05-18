/* 2023 May 05
 *  Libraries used in this program
 *  1. github.com/GreyGnome/EnableInterrupt
 *  2. github.com/NicksonYap/digitalWriteFast
 *  3. github.com/JChristensen/Timer
 *  4. github.com/adafruit/Adafruit_MCP4725
 *  5. arduino.cc/en/reference/wire
 *   
 *  v = dL / t = 2 * r * PI / (N * t)
 *  r: radius \ N: resolution of encoder \ t: sampling period
 *  
 *  ω = (2 * π * n) / (N *t)
 *  ω: rad/s \ n: number of pulses \ t: sampling period \ N: resolution of encoder
 *  
 */
// Libraries
#include <EnableInterrupt.h>          //insufficient number of standard Interrupt pins on the Arduino Uno
#include <digitalWriteFast.h>
#include <Wire.h>
#include "Timer.h"
#include <Adafruit_MCP4725.h>
#include <Adafruit_BusIO_Register.h>
Adafruit_MCP4725 dac;

// Input Pins
#define encoderA    2
#define encoderB    3
#define lick        4
#define lever       5
#define hijack      6

// Output Pins
#define out_sync    7                          // gray
#define out_cue     8                          // brown
#define out_punish  9                          // red
#define out_lever   10                         // orange
//#define out_dir     11
#define out_module  11                         // yellow
#define out_reward  12                         // green
#define out_lick    13                         // blue
// output of speed by DAC is using SDA pin     // purple
                    

//fixed figures for encoder, these figures are unique to the encoder
byte encoderlimit = 10;                                   // Maximum permissible speed : 6000 r/min = 1 r/ 10 ms
const int N = 1000;                                       // resolution of encoder
const int r = 100;                                        // avg radius = 100 mm => circum = 2*PI*r;                               
//double dL =  2*PI*r / N;                                  // dL = cirum / N (mm)

//variables for motion
long Now;
long Prev;
long Diff;
long counter;                                             
unsigned long runT =0;
//int  Dir;                                                  // 0 = CW, 1 = CCW


//variables for MCP4725 12bit DAC
int   scale = 200;                                          // a magic scale make analogue signal beautiful
float outputV = 0;

//variables for trials, reward and punishment
unsigned long rule;                                       // reward rules, is modulated by a potentiometer as analog value
unsigned int reward_state = false;
unsigned int cue_state = false;
unsigned long count_reward;                               // decision to give reward
unsigned long Cue_time;
unsigned long punish_length;
unsigned long Trial_time= 13000;                          // set each trial time
long slowTime = -4200;                                    //
const unsigned long PERIOD = 500;                         // reward duration, depending on the water valve usage
// 25ml full syringe gives 0.2ml in 10 secs  ;  2ml only in 25ml syringe give 0.1ml in 10 secs

// variables for synchronisation 
const float PW = 2.0;                                         // PulseWidth to blink   (milliseconds)
const float interval = 20.5;                              // interval between pulse (milliseconds)
unsigned long syncT = 0;                                  // will store last time LED was updated
unsigned long last_sync=0;
unsigned long expTime = 300000;                           // synchronise 5 min (300 sec)
bool sync_status = false;

// variables for analog things
//const unsigned int analogIn0 = A0;                   // Analog input pin controlled by a potentiometer to change rule
//const unsigned int analogIn1 = A1;                   // Analog input pin controlled by a potentiometer to change punishment
unsigned int A0_Value = 0;                                    // value read from the pot
unsigned int A1_Value = 0;                                    // value output to the PWM (analog out)
const int analogOutPin = out_module;                 // Analog output pin that the LED is attached to

Timer t;                                                  // initiate the timer object

void setup() {
  Serial.begin(9600);                                     // change higher number if need higher sampling rate
  dac.begin(0x62);                                        // call DAC function
  
  pinModeFast(encoderA, INPUT);                           // faster than pinMode(encoderA, INPUT_PULLUP);
  pinModeFast(encoderB, INPUT);                           // faster than pinMode(encoderB, INPUT_PULLUP);
  pinMode(lick, INPUT_PULLUP);
  pinMode(lever, INPUT_PULLUP);
  pinMode(hijack, INPUT_PULLUP);

  pinMode(out_sync, OUTPUT);
  pinMode(out_cue, OUTPUT);
  pinMode(out_punish, OUTPUT);
  pinMode(out_lever, OUTPUT);
//  pinMode(out_dir,OUTPUT);
  pinMode(out_module,OUTPUT);
  pinMode(out_reward, OUTPUT);
  pinMode(out_lick, OUTPUT);

  digitalWrite(out_sync,LOW);
  digitalWrite(out_cue,LOW);
  digitalWrite(out_punish,LOW);
  digitalWrite(out_lever,LOW);
  digitalWrite(out_module,LOW);
  digitalWrite(out_reward,LOW);
  digitalWrite(out_lick,LOW);
  
  enableInterrupt(encoderA, motions, RISING);
  enableInterrupt(lever, lever_click, RISING);
  enableInterrupt(lick, lick_chk, CHANGE);
  enableInterrupt(hijack, hijack_click, RISING);
  
  Prev = 0;
}

void motions(){  
  if( (bool)digitalReadFast(encoderB) == HIGH){           //digitalReadFast() is faster than digitalRead()
    counter ++;
    count_reward ++;
  }else if ( (bool) digitalReadFast(encoderB) == LOW ){     // HONTKO encoders could not work without this line
    counter --;
  }
//  if (Diff > 0){
//    Dir = 1;
////    digitalWrite(out_dir,HIGH);
//  }else{
//    Dir = 0;                                               // if move forward
////    digitalWrite(out_dir,LOW);
//  }
}

void lick_chk(){
  if ( digitalRead(lick) == HIGH){
    digitalWrite(out_lick,HIGH);
  }else{
    digitalWrite(out_lick,LOW);
  }
}

void give_cue(){
  cue_state = true;
  Cue_time = millis();
  t.pulseImmediate(out_cue, PERIOD*2, HIGH);               // use Timer to turn on reward for specific period
}

void give_punish(){
  if (A1_Value != 0){
    t.pulseImmediate(out_punish, punish_length, HIGH);           // use Timer to turn on reward for specific period
  }
}

void give_reward(){
  t.pulseImmediate(out_reward,PERIOD,HIGH);       // use Timer to turn on reward for specific period
  reward_state = false;
}

void lever_click(){
  if ( digitalRead(lever) == HIGH){
    t.pulseImmediate(out_lever,PERIOD/10,HIGH);
    if (reward_state == true){
      give_reward();
      digitalWrite(out_lever,LOW);      
    }
  }   
}

void hijack_click(){
  // ***** manually give reward ***** //
  if (digitalRead(hijack) == HIGH){
    t.pulseImmediate(out_lever,PERIOD/10,HIGH);
    give_reward();
  }

  // ***** synchronise camera ***** //
  if (sync_status == false && syncT == 0){
    sync_status = true;
    last_sync = millis();
  }
}

void loop() {
// ***** Analog modules to determine the values for rule and punishment *****//
A0_Value = map(analogRead(A0), 0, 1023, 0, 5);                 // read the analog value and map it to the range of the analog out:
rule = N* A0_Value;

A1_Value = map(analogRead(A1), 0, 1023, 0, 5); 
punish_length = PERIOD* A1_Value;
//analogWrite(analogOutPin0, A0_Value);                         // change the analog out value:


// Synchronise the camera in 50 Hz with 2ms pulse width shutter
  if ( sync_status == true) {
    if ( (millis() - syncT) >= interval) {
      syncT = millis();
      t.pulseImmediate(out_sync,PW,HIGH);
    }

    if ( (millis()-last_sync) >= expTime) {
      sync_status = false;
      digitalWrite(out_sync,LOW);
      syncT =0;
    }
  }

/* Calculate motion
 *  IMPORTANT: the calculation of movement cannot be triggered by interrupt funct, bcz the device will stop recording  */
  if ( (millis() - runT) > encoderlimit*0.4) {
    Now = counter;
    Diff = Now - Prev;
  //  outputV = abs(Diff)*dL*scale;
    outputV = abs(Diff)*scale;
    dac.setVoltage(outputV, false);
    Prev = counter;
    runT = millis();
  }

/* Give Cue, reward or punish:
 * 1. if the animal slow down for more than 1 sec, give reward
 * 2. if the animal doesn't slow down, give punishment
 */
if ( cue_state == false) {
  if (count_reward > rule && Diff >= 2.6){                  // when the animal achieve reward rule
    give_cue();
    count_reward =0;
  }
}

if ( cue_state == true) {
  if (abs(Diff) >= 3){                                 // renew time point if the animal is still moving 
    slowTime = millis();
    reward_state = false;
  }
    if ( ( (millis() - Cue_time) >= 1000) && ( (millis() - Cue_time) <= 4000) ) {
      if ( (millis() - slowTime) < 500 ) {
        reward_state = false;
        give_punish();                                       // if the animal is still moving, give punishment
      }

      if ( (millis() - slowTime) >= 500){
        digitalWrite(out_punish,LOW);
        reward_state = true;
      }
    }
    if ( (millis() - Cue_time) > 12000){
      cue_state = false;
    }
}
  t.update();
}
