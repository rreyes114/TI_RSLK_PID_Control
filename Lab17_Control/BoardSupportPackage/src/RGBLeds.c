/*
 * RGBLeds.c
 *
 *  Created on: Jan 27, 2021
 *      Author: Roehl
 */

#include "RGBLeds.h"
#include "msp.h"

void LP3943_LedModeSet(uint32_t unit, uint16_t LED_DATA)
{
    /* LP3943 LEDS and Register Addresses
     * LS0 | LED0-3 Selector    0x06
     * LS1 | LED4-7             0x07
     * LS2 | LED8-11            0x08
     * LS3 | LED12-15           0x09
     *
     * Important Registers:
     * eUSCI_Bx I2C Slave Address Register
     * eUSCI_Bx Control Word Register 0
     * eUSCI_Bx Transmit Buffer Register UCB2TXBUF
     * eUSCI_Bx Interrupt Flag Register UCB2TFG
     *
     * Slave Addresses and unit
     * Red      0x60    2       //addr for r and b seem to be swapped...
     * Green    0x61    1
     * Blue     0x62    0
     * */

    //generate data to send
    /*format: start(0)|chip addr|w(0)|reg addr|data|stop(1)
     *        1bit    |7bit     |1bit|8bit    |8bit|1bit   */
    uint32_t x = LED_DATA;
    //interleaves 0 for the B1 bits in each LS register
    x = (x | (x << 8)) & 0x00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F;
    x = (x | (x << 2)) & 0x33333333;
    x = (x | (x << 1)) & 0x55555555;

    //set initial slave address
    UCB2I2CSA = (0x60 + unit);

    //generate start condition
    UCB2CTLW0 |= UCTXSTT;

    while(UCB2CTLW0 & UCTXSTT);
    //while(!(UCB2IFG & UCTXIFG0)); //while buffer not empty
    UCB2TXBUF = 0x16; // write to register addr and set to auto-inc

    //wait for buffer availability
    //LOOP: fill TXBUF with data for LP3943
    //wait for buffer availability. B LOOP
    for(int i = 0; i < 5; i++) {        //why 5? idk, but it works (4->5)
        while(!(UCB2IFG & UCTXIFG0)); //while buffer not empty
        UCB2TXBUF = x & 0xFF;
        x = x >> 8;
    }

    //generate STOP condition
    UCB2CTLW0 |= UCTXSTP;
}

void init_RGBLEDS()
{
    //pulse SCL line
    P3DIR |=  BIT7;
    P3DIR &= ~BIT7;

    uint16_t UNIT_OFF = 0x0000;

    //software reset enable
    UCB2CTLW0 = UCSWRST;

    //initialize I2C master
    // set as master, I2C mode, Clock sync, SMCLK source, Transmitter
    UCB2CTLW0 |= UCMST|UCMODE_3|UCSYNC|UCSSEL_3|UCTR;

    //set clock frequency as 400hz
    UCB2BRW = 30;

    //set pins to I2C mode
    // (Table found on 160 of SLAS826E)
    // Set P3.6 as UCB2_SDA and 3.7 as UCB2_SLC
    P3SEL0 |= BIT6 | BIT7;
    P3SEL1 &= ~BIT6 & ~BIT7;

    //turn off software reset
    UCB2CTLW0 &= ~UCSWRST;

    LP3943_LedModeSet(Red, UNIT_OFF);
    LP3943_LedModeSet(Green, UNIT_OFF);
    LP3943_LedModeSet(Blue, UNIT_OFF);
}
