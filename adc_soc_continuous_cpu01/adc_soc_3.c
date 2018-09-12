//###########################################################################
//
// FILE:   adc_soc_continuous_cpu01.c
//
// TITLE:  ADC continuous self-triggering for F2837xS.
//
//! \addtogroup cpu01_example_list
//! <h1> ADC Continuous Triggering (adc_soc_continuous)</h1>
//!
//! This example sets up the ADC to convert continuously, achieving maximum
//! sampling rate.\n
//!
//! After the program runs, the memory will contain:
//!
//! - \b AdcaResults \b: A sequence of analog-to-digital conversion samples
//! from pin A0. The time between samples is the minimum possible based on the
//! ADC speed.
//
//###########################################################################
// $TI Release: F2837xS Support Library v210 $
// $Release Date: Tue Nov  1 15:35:23 CDT 2016 $
// $Copyright: Copyright (C) 2014-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Included Files
//
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <file.h>

#include "F28x_Project.h"
//
// Function Prototypes
//
void ConfigureADC(void);
void SetupADCContinuous(void); //SetupADCContinuous(Uint16 channel);

//
// Defines
//


__interrupt void cpu_timer0_isr(void); //CANAL A

#define RESULTS_BUFFER_SIZE 256

//
// Globals
//

Uint16 ADCA0;
Uint16 ADCA1;
Uint16 ADCA2;

Uint16 ADCB0;
Uint16 ADCB1;
Uint16 ADCB2;

void main(void)
{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xS_SysCtrl.c file.
//
    InitSysCtrl();

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xS_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
    InitGpio(); // Skipped for this example

//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xS_PieCtrl.c file.
//
    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xS_DefaultIsr.c.
// This function is found in F2837xS_PieVect.c.
//
    InitPieVectTable();

//TODO FUNÇÕES ISR
// Map ISR Functions
//
    EALLOW;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    //PieVectTable.XINT3_INT = &xint3_isr;
    //PieVectTable.ADCB1_INT = &adcal_isr_b;
    EDIS;

    InitCpuTimers();

    ConfigCpuTimer(&CpuTimer0, 60, 500000);

    CpuTimer0Regs.TCR.all = 0x4001;
//
// Configure the ADC and power it up
/*
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

   // ConfigureEpwm();


    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
*/

    IER |= M_INT1;

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

//
// Enable global Interrupts and higher priority real-time debug events:
//
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    ConfigureADC();
    SetupADCContinuous();

    for(;;){
           asm ("          NOP");
       }
// TODO Enable PIE interrupts


}//END MAIN

// TODO INTERRUPT FUNCTIONS
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    AdcaRegs.ADCSOCFRC1.all = 0x0007; //SOC0, SOC1 and SOC2

    AdcbRegs.ADCSOCFRC1.all = 0x0007; //SOC0, SOC1 and SOC2

    while(AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0);
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    while(AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0);
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    ADCA0 = AdcaResultRegs.ADCRESULT0;
    ADCA1 = AdcaResultRegs.ADCRESULT1;
    ADCA2 = AdcaResultRegs.ADCRESULT2;

    ADCB0 = AdcbResultRegs.ADCRESULT0;
    ADCB1 = AdcbResultRegs.ADCRESULT1;
    ADCB2 = AdcbResultRegs.ADCRESULT2;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

void ConfigureADC(void)
{
    EALLOW;
    //
    //write configurations ADCA
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //
    //write configurations ADCB
    //
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    DELAY_US(1000);

    EDIS;

}

//void SetupADCContinuous(Uint16 channel)
void SetupADCContinuous()
{
    Uint16 acqps;

    //
    //determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    EALLOW;
    //setup ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL  = 0;  //SOC will convert on channel 0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS  = acqps;    //sample window is acqps + 1 SYSCLK cycles

    AdcaRegs.ADCSOC1CTL.bit.CHSEL  = 1;  //SOC will convert on channel 0
    AdcaRegs.ADCSOC1CTL.bit.ACQPS  = acqps;    //sample window is acqps + 1 SYSCLK cycles

    AdcaRegs.ADCSOC2CTL.bit.CHSEL  = 2;  //SOC will convert on channel 0
    AdcaRegs.ADCSOC2CTL.bit.ACQPS  = acqps;    //sample window is acqps + 1 SYSCLK cycles

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;  //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //0 = disable / 1 = enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //setup ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL  = 0;  //SOC will convert on channel 0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS  = acqps;    //sample window is acqps + 1 SYSCLK cycles

    AdcbRegs.ADCSOC1CTL.bit.CHSEL  = 1;  //SOC will convert on channel 0
    AdcbRegs.ADCSOC1CTL.bit.ACQPS  = acqps;    //sample window is acqps + 1 SYSCLK cycles

    AdcbRegs.ADCSOC2CTL.bit.CHSEL  = 2;  //SOC will convert on channel 0
    AdcbRegs.ADCSOC2CTL.bit.ACQPS  = acqps;    //sample window is acqps + 1 SYSCLK cycles

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1;  //end of SOC0 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //0 = disable / 1 = enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

}

//
// End of file
//


