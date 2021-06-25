/*
 * F28377S_Interface_Config.c
 *
 *  Created on: 4 ??? 2021
 *      Author: WB
 */

#include "spiconfig.h"

/*-----------------------------------------------------------------------*/
/* spi pin configuration                                */
/*-----------------------------------------------------------------------*/
void spi_pin_config (void)
{
    EALLOW;
    //Set up  MUX & DIR
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;    //Leave as GPIO for manual CS control
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;    //Leave as GPIO for manual CS control
    //
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;    // debug led 1
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;    // debug led 1
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;    // debug led 1 on syncworks board
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;    // debug led 1 on syncworks board
    //
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;

    //Set up GPIO Pull-up disables/enables
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0; //Needs to be normally pulled high
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0; //Needs to be normally pulled high
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;
    //Set up GPIOs in asynch mode
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Asynch input
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;
    GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;
    EDIS;
    /* Deassert */
    CS_HIGH;
    /* Configure the SPI A port */
    //Set reset bit low
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    // set clock phase
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
    // set clock polarity
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;
    //Master mode
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
    // SPI Baud Rate = LSPCLK /( SPIBRR + 1) , LSPCLK = SYSCLK/4 --> default, For SPIBRR = 3 to 127
    //bitrate setting, 0x007C =400Khz,0x0031 =1MHz, 0x0018 = 2MHz, 0x0009 = 5MHz, 0x0006=7MHz
    SpiaRegs.SPIBRR.all = 0x00018;
    //Set char length to 8 bits
    SpiaRegs.SPICCR.bit.SPICHAR = 0x7;
    SpiaRegs.SPICTL.bit.TALK = 1;
    //Release SPI from reset
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
    SpiaRegs.SPIPRI.bit.FREE = 1;
    SpiaRegs.SPIPRI.bit.SOFT = 1;
}

void SELECT(void)
{
    //
    // Select the sensor.
    //
    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
}

//
void DESELECT(void)
{
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
}

/*-----------------------------------------------------------------------*/
/* Transmit a byte to sensor via SPI                   */
/*-----------------------------------------------------------------------*/

void xmit_spi(BYTE dat)
{
    volatile DWORD rcvdat;   //included file like integer.h for DWORD definition

    /* Write the data to the tx fifo */
    while (SpiaRegs.SPISTS.bit.BUFFULL_FLAG);    //Wait for room to write data
    SpiaRegs.SPITXBUF = ((DWORD) dat) << 8;                        //Write data

    /* flush data read during the write */
    while (SpiaRegs.SPISTS.bit.INT_FLAG != 1);        //May be possible to switch to '!SpicRegs.SPISTS.bit.INT_FLAG'
    rcvdat = (SpiaRegs.SPIRXBUF && 0xFF);                 //Clear Receive Buffer
}

/*-----------------------------------------------------------------------*/
/* Receive a byte from sensor via SPI                */
/*-----------------------------------------------------------------------*/

BYTE rcvr_spi(void)
{
    volatile DWORD rcvdat;

    //Disable transmission channel
    //SpicRegs.SPICTL.bit.TALK = 0;

    /* write dummy data */
    while (SpiaRegs.SPISTS.bit.BUFFULL_FLAG);    //Wait for space to write
    SpiaRegs.SPITXBUF = 0xFF00;                     //Write dummy data

    /* read data from RX fifo */
    while (SpiaRegs.SPISTS.bit.INT_FLAG != 1);        //May be possible to switch to '!SpicRegs.SPISTS.bit.INT_FLAG'
    rcvdat = (SpiaRegs.SPIRXBUF & 0xFF);                 //Read Transferred data

    return (BYTE) rcvdat;
}

void rcvr_spi_m(BYTE *dst)
{
    *dst = rcvr_spi();
}
