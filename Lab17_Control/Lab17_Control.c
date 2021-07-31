// Lab17_Control.c
// Runs on MSP432
// Implementation of the control system.
// Daniel and Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/


#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/UART0.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/ADC14.h"
#include "../inc/TimerA1.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/IRDistance.h"
#include "../inc/Nokia5110.h"
#include "../inc/LPF.h"
#include "../inc/SysTickInts.h"
#include "../inc/Tachometer.h"
#include "../inc/Reflectance.h"

#define P2_0 (*((volatile uint8_t *)(0x42098060)))
#define P1_0 (*((volatile uint8_t *)(0x42098040)))

//******************************************************
// Speed Measuring functions
uint16_t Period0;              // (1/SMCLK) units = 83.3 ns units
uint16_t First0;               // Timer A3 first edge, P10.4
int Done0;                     // set each rising
// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure0(uint16_t time){
  P2_0 = P2_0^0x01;            // thread profile, P2.0
  Period0 = (time - First0)&0xFFFF; // 16 bits, 83.3 ns resolution
  First0 = time;               // setup for next
  Done0 = 1;
}
uint16_t Period1;              // (1/SMCLK) units = 83.3 ns units
uint16_t First1;               // Timer A3 first edge, P10.5
int Done1;                     // set each rising
// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure1(uint16_t time){
  P1_0 = P1_0^0x01;            // thread profile, P1.0
  Period1 = (time - First1)&0xFFFF; // 16 bits, 83.3 ns resolution
  First1 = time;               // setup for next
  Done1 = 1;
}

//******************************************************
// Distance to wall proportional control
// Incremental speed control 
// Integral control, Line follower
//int32_t SpeedBufferL[1000];      // RPM
//int32_t SpeedBufferR[1000];      // RPM
uint32_t Time = 0; // in 0.01 sec = 10ms
int32_t XstarL, XprimeL, XstarR, XprimeR, ErrorL, ErrorR, UL, UR;
int32_t A = 15360;
void Controller(void){ //run at 100Hz (5000 period for timer A1): 10ms period, 100ms time constant of motor
// write this as part of Lab 17
    XprimeL = 2000000/Period1;  //take measurement from sensor
    XprimeR = 2000000/Period0;

    //SpeedBufferL[Time] = XprimeL;   //store in buffer for debugging
    //SpeedBufferR[Time] = XprimeR;
    Time++;

    ErrorL = XstarL - XprimeL;  //calculate error
    ErrorR = XstarR - XprimeR;
    UL = UL + (A*ErrorL)/1024; //adjust input
    UR = UR + (A*ErrorR)/1024;
    if(UL > 14998) UL = 14998;  //check input bounds
    else if(UL < 2) UL = 2;
    if(UR > 14998) UR = 14998;
    else if(UR < 2) UR = 2;

    Motor_Forward(UL,UR);       //drive motors at new input

    if((Time==1000)||Bump_Read()){
        Motor_Stop();      // 0%
        TimerA1_Stop();
    }
}

void main0(void){
  DisableInterrupts();
// initialization
// write this as part of Lab 17
  Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
  Bump_Init();
  Motor_Init();        // activate Lab 12 software
  TimerA3Capture_Init01(&PeriodMeasure0, &PeriodMeasure1); //right is 0, left is 1
  TimerA1_Init(&Controller, 5000);
  XstarL = 120;
  XstarR = 120;
  UL = 7500;
  UR = 7500;
  Motor_Forward(UL, UR);
  EnableInterrupts();

  while(1){
// write this as part of Lab 17
      WaitForInterrupt();
  }
}

