/*
 * timerconfig.c
 *
 *  Created on: 15 ጁን 2021
 *      Author: WB
 */
#include "timerconfig.h"
#include "ICM20948.h"
#include "sciconfig.h"
void timer_config(void)
{
    // ISR functions found within this file.
    //
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
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
    ConfigCpuTimer(&CpuTimer2, 200, 1000000);
    // To ensure precise timing, use write-only instructions to write to the entire
    // register. Therefore, if any of the configuration bits are changed in
    // ConfigCpuTimer and InitCpuTimers (in F2837xS_cputimervars.h), the below
    // settings must also be updated.
    //
    CpuTimer0Regs.TCR.all = 0x4001;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;
    //
    // Enable CPU INT1 which is connected to CPU-Timer 0:
    //
    IER |= M_INT1;
    IER |= M_INT13;
    IER |= M_INT14;
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

//
// cpu_timer2_isr CPU Timer2 ISR
//
__interrupt void cpu_timer2_isr(void)
{
    int i;
   CpuTimer2.InterruptCount++;
   /* Mavlink heartbeat send */
/*   static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                                  uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status);*/
   mavlink_msg_heartbeat_pack(1, 1, &message,1, 0, 128, 8, 0);
   // Translate message to buffer
   unsigned len = mavlink_msg_to_send_buffer((Uint8_t*) buff, &message);
   for (i = 0; i < len; i++)
   {
       scia_xmit(buff[i]);
   }

}

