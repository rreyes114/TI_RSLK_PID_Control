/*
 * RGBLeds.h
 *
 *  Created on: Jan 27, 2021
 *      Author: Roehl
 */
#include "msp.h"

#ifndef RGBLEDS_H_
#define RGBLEDS_H_

typedef enum device
{
    Blue = 0, //original val
    Green = 1,
    Red = 2
} unit_desig;

//static void LP3943_ColorSet(uint32_t unit, uint32_t PWM_DATA); //(optional)

void LP3943_LedModeSet(uint32_t unit, uint16_t LED_DATA);

void init_RGBLEDS();

#endif /* RGBLEDS_H_ */
