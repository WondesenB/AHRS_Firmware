/*
 * sciconfig.c
 *
 *  Created on: 15 ጁን 2021
 *      Author: WB
 */
#include "sciconfig.h"

//sci
void sci_pin_config(void)
{
    EALLOW ;
    //mux
    //==== SCIA =====
    GpioCtrlRegs.GPCGMUX2.bit.GPIO84 = 1;
    GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 1;
    GpioCtrlRegs.GPCGMUX2.bit.GPIO85 = 1;
    GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 1;
    //===SCIB====
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 2;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 2;

    EDIS;
    //    GpioCtrlRegs.GPCPUD.bit.GPIO85 =0;
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //
    //==== SCIA =====
    SciaRegs.SCICCR.all = 0x0007;  // 1 stop bit,  No loopback
    // No parity,8 char bits,
    // async mode, idle-line protocol
    SciaRegs.SCICTL1.all = 0x0003; // enable TX, RX, internal SCICLK,
    // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.all = 0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
    // BRR = (SCIHBAUD << 8) + (SCILBAUD)
    // SCI Asynchronous Baud = LSPCLK / ((BRR + 1) *8) , where BRR is decimal number and LSPCLK = 50MHz
    // BRR = LSPCLK / (SCI Asynchronous Baud * 8) - 1   , where BRR is decimal number
    SciaRegs.SCIHBAUD.all = 0x0000;
    SciaRegs.SCILBAUD.all = 0x0037; // SCIHBAUD= 0x0,SCILBAUD =0x0035(~0x0037) --> 115200 ,SCIHBAUD= 0x02,SCILBAUD =0x8A --> 9600
    SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
    //===SCIB====
    ScibRegs.SCICCR.all = 0x0007;  // 1 stop bit,  No loopback
    // No parity,8 char bits,
    // async mode, idle-line protocol
    ScibRegs.SCICTL1.all = 0x0003; // enable TX, RX, internal SCICLK,
    // Disable RX ERR, SLEEP, TXWAKE
    ScibRegs.SCICTL2.all = 0x0003;
    ScibRegs.SCICTL2.bit.TXINTENA = 1;
    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;
    // BRR = (SCIHBAUD << 8) + (SCILBAUD)
    // SCI Asynchronous Baud = LSPCLK / ((BRR + 1) *8) , where BRR is decimal number and LSPCLK = 50MHz
    // BRR = LSPCLK / (SCI Asynchronous Baud * 8) - 1   , where BRR is decimal number
    ScibRegs.SCIHBAUD.all = 0x0000;
    ScibRegs.SCILBAUD.all = 0x0037; // SCIHBAUD= 0x0,SCILBAUD =0x0035(~0x0037) --> 115200 ,SCIHBAUD= 0x02,SCILBAUD =0x8A --> 9600
    ScibRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
}

// scia_xmit - Transmit a character from the SCI
//
void scia_xmit(int16_t a)
{
    if(SerialPort == 1)
    {
        while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
        SciaRegs.SCITXBUF.all =a;
    }
    if(SerialPort == 2)
    {
        while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}
        ScibRegs.SCITXBUF.all =a;
    }
    DELAY_US(100);
}
