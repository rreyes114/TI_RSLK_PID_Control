/*
 * PIDController_main.c
 *
 *  Created on: Jul 26, 2021
 *      Author: Roehl
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
#include <driverlib.h>
#include <BSP.h>
#include "i2c_driver.h"
#include <math.h>

//******************************************************
// BME160 IMU two wheel balance PID controller
volatile struct bmi160_accel_t accelData;
//struct bmi160_gyro_t gyroData;
volatile int16_t gyroData_x;
volatile float pitch;
int32_t pitch_fp, gyroDeg_fp, compDeg_fp; //0.001 degrees fixed point
int32_t angleBuffer[1000];
uint32_t time = 0;
void MeasureAngle(void) {
     pitch = atan2f(accelData.y, accelData.z)*57.3;
     pitch_fp = (int32_t)(pitch*1000) ;
     gyroDeg_fp = (gyroData_x*10)/131;
     compDeg_fp = ((compDeg_fp + gyroDeg_fp)*99 + pitch)/100;
     angleBuffer[time] = compDeg_fp;
}

#define DEADZONE 500
void AdjustMotors(int32_t UL, int32_t UR) {
    //change sign if negative
    bool ForwardL = 1;
    bool ForwardR = 1;
    if(UL < 0) { UL = -UL; ForwardL = 0; }
    if(UR < 0) { UR = -UR; ForwardR = 0; }
    //check input bounds
    if(UL > 14998) { UL = 14998; }
    else if(UL < DEADZONE){ //UL = 2;
        Motor_Stop();
        return;}
    if(UR > 14998) { UR = 14998; }
    else if(UR < DEADZONE){ //UR = 2;
        Motor_Stop();
        return;}
    //drive motors at new input
    if(ForwardL==0 && ForwardR==0) Motor_Backward(UL,UR);
    else if(ForwardL==0 && ForwardR==1) Motor_Left(UL,UR);
    else if(ForwardL==1 && ForwardR==0) Motor_Right(UL,UR);
    else if(ForwardL==1 && ForwardR==1) Motor_Forward(UL,UR);
}

int32_t Error, Error_old, Up, Ui, Ud, Upid;
int32_t Kp = -2048;
int32_t Ki = -56;
int32_t Kd = 64;
int32_t dutyBuffer[1000];
#define RANGE_MIN 200
#define RANGE_MAX 1000
#define GAIN_MIN -8
#define GAIN_MAX -2048
void PIDController(void) {
    //P8->OUT |= 0x20;
    MeasureAngle();
    Error_old = Error;
    Error = 0 - compDeg_fp;
    Up = (Kp*Error)/1024;
    Ui = Ui + (Ki*Error)/1024;
    Ud = (Kd*(Error-Error_old))/1024;
    Upid = Up + Ui + Ud;
    AdjustMotors(Upid,Upid);
    dutyBuffer[time] = Upid;
    if(time!=999) time++;
    if(Bump_Read()){
        Motor_Stop();      // 0%
        TimerA1_Stop();
    }
    //P8->OUT &= ~0x20;
}

void main(void) {
    WDT_A_clearTimer();
    WDT_A_holdTimer();
    ClockSys_SetMaxFreq();
    WDT_A_initWatchdogTimer(WDT_A_CLOCKSOURCE_SMCLK,WDT_A_CLOCKITERATIONS_128M);
    //WDT_A_startTimer(); //used to reset if i2c bugs out
    initI2C();
    bmi160_initialize_sensor();
    DisableInterrupts();
    //Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
    P8->SEL0 &= ~0x60;
    P8->SEL1 &= ~0x60;
    P8->DIR |= 0x60;
    P8->OUT &= ~0x60;
    Bump_Init();
    Motor_Init();        // activate Lab 12 software
    TimerA1_Init(&PIDController, 5000); // 5000 = 100Hz
    EnableInterrupts();
    while(1){
        //P8->OUT |= 0x20;
        while(bmi160_read_accel_xyz(&accelData));   //when I2C gets stuck, its in here
        //P8->OUT &= ~0x20;
        for(uint32_t i = 0; i < 48; i++);   //48 = 10us delay
        //P8->OUT |= 0x40;
        while(bmi160_read_gyro_x(&gyroData_x));     //I2C gets stuck here too
        //P8->OUT &= ~0x40;
        for(uint32_t i = 0; i < 4800*3; i++);   //48 = 10us delay
    }
}

/*  //variable gain scheme
    if(Error > -RANGE_MIN && Error < RANGE_MIN) {
        Kp = GAIN_MIN;//(Error*Error*GAIN_MAX)/(RANGE*RANGE);
    }
    else if(Error > -RANGE_MAX && Error < RANGE_MAX) {
        Kp = ((GAIN_MAX-GAIN_MIN)*(ERROR-RANGE_MIN))/(RANGE_MAX-RANGE_MIN) + GAIN_MIN;
        if(Kp > 0)
            Kp = -Kp;
    }
    else {
        Kp = GAIN_MAX;
    }
*/
