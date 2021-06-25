/*
 * timerconfig.c
 *
 *  Created on: 15 ጁን 2021
 *      Author: WB
 */
#include "timerconfig.h"
#include "ICM20948.h"
void timer_config(void)
{
    // ISR functions found within this file.
    //
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    InitCpuTimers();   // only initialize the Cpu Timers
    //
    // Configure CPU-Timer 0 to __interrupt every 1 microsecond:
    // 200MHz CPU Freq,  (in uSeconds)
    //
    ConfigCpuTimer(&CpuTimer0, 200, 1);
    // Configure CPU-Timer 1 to __interrupt every 1 millisecond:
    ConfigCpuTimer(&CpuTimer1, 200, 1000);
    //
    // To ensure precise timing, use write-only instructions to write to the entire
    // register. Therefore, if any of the configuration bits are changed in
    // ConfigCpuTimer and InitCpuTimers (in F2837xS_cputimervars.h), the below
    // settings must also be updated.
    //
    CpuTimer0Regs.TCR.all = 0x4001;
    CpuTimer1Regs.TCR.all = 0x4000;
    //
    // Enable CPU INT1 which is connected to CPU-Timer 0:
    //
    IER |= M_INT1;
    IER |= M_INT13;
    //
    // Enable TINT0 in the PIE: Group 1 __interrupt 7
    //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    EINT;   // Enable Global __interrupt INTM
    ERTM;   // Enable Global realtime __interrupt DBGM
}

//
// cpu_timer0_isr - CPU Timer0 ISR that toggles GPIO32 once per 500ms
//
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    tm.Now++;

    //
    // Acknowledge this __interrupt to receive more __interrupts from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// cpu_timer1_isr - CPU Timer1 ISR
//ll
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;
    tm.time_millis++;
}



