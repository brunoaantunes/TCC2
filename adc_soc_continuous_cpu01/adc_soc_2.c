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
void SetupADCContinuous(Uint16 channel);
//void ConfigureEpwm(void);

//
// Defines
//

//#define testeADCa AdcaResultRegs.ADCRESULT0;
//#define testeADCc AdcbResultRegs.ADCRESULT0;

//#define ia_PPB   ((signed int)AdcbResultRegs.ADCPPB1RESULT.all)
//#define ic_PPB   ((signed int)AdcbResultRegs.ADCPPB1RESULT.all)
//#define ADC_PU_PPB_SCALE_FACTOR    0.000488281250     //1/2^11

//inline void sensordecorrente(void);

interrupt void adcal_isr(void); //CANAL A
//interrupt void xint3_isr(void);
//__interrupt void adcal_isr_b(void); //CANAL b

#define RESULTS_BUFFER_SIZE 256

//
// Globals
//
Uint16 resultsIndex;
Uint16 AdcaResults[RESULTS_BUFFER_SIZE];

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
    PieVectTable.ADCA1_INT = &adcal_isr;
    //PieVectTable.XINT3_INT = &xint3_isr;
    //PieVectTable.ADCB1_INT = &adcal_isr_b;
    EDIS;
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
/*
    for(resultsIndex = 0; resultsIndex < RESULTS_BUFFER_SIZE; resultsIndex++)
    {
        AdcaResults[resultsIndex] = 0;
    }
    resultsIndex = 0;
*/
    ConfigureADC();

    SetupADCContinuous(0);

    for(;;){
           asm ("          NOP");
       }
// TODO Enable PIE interrupts


}//END MAIN

// TODO INTERRUPT FUNCTIONS
interrupt void adcal_isr(void)
{
    //AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1;
    AdcaRegs.ADCSOCFRC1.all = 0x0007; //SOC0, SOC1 and SOC2

    while(AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0);
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
/*
    AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT0;
    if(RESULTS_BUFFER_SIZE<=resultsIndex)
    {
       resultsIndex = 0;
    }

*/
    AdcaResults[resultsIndex] = AdcaResultRegs.ADCRESULT0;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}
/*
interrupt void xint3_isr(void)
{
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}
*/
//
// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
//
void ConfigureADC(void)
{
    EALLOW;
    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    DELAY_US(1000);

    EDIS;
    /*
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;


    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //

    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //

    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);
*/

}
/*
void ConfigureEpwm()
{
    EALLOW;
        // Assumes ePWM clock is already enabled

    EPwm1Regs.ETSEL.bit.SOCAEN  = 0;            // Disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;            // Select SOC on up-count
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;             // Generate pulse on 1st event
    EPwm1Regs.CMPA.bit.CMPA = 311;
    EPwm1Regs.TBPRD = 624;
    EPwm1Regs.TBCTL.bit.CTRMODE = 3;            // freeze counter
    EDIS;
}
*/
//
// SetupADCContinuous - setup the ADC to continuously convert on one channel
//
void SetupADCContinuous(Uint16 channel)
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
    AdcaRegs.ADCSOC0CTL.bit.CHSEL  = channel;  //SOC will convert on channel 0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS  = acqps;    //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;  //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //0 = disable / 1 = enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1;

    //
    //AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

    //AdcaRegs.ADCINTSOCSEL1.bit.SOC0 = 2;

    EDIS;
    /*
    AdcbRegs.ADCSOC0CTL.bit.CHSEL  = 0;

    AdcaRegs.ADCSOC4CTL.bit.CHSEL  = 1;  //SOC will convert on channel
    AdcbRegs.ADCSOC2CTL.bit.CHSEL  = 1;

    AdcaRegs.ADCSOC6CTL.bit.CHSEL  = 2;  //SOC will convert on channel
    AdcbRegs.ADCSOC6CTL.bit.CHSEL  = 2;


    AdcbRegs.ADCSOC0CTL.bit.ACQPS  = 14;

    AdcaRegs.ADCSOC4CTL.bit.ACQPS  = 14;    //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.ACQPS  = 14;

    AdcaRegs.ADCSOC6CTL.bit.ACQPS  = 14;    //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC6CTL.bit.ACQPS  = 14;



    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 0;


    AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 1;


    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0;
*/

}
// TODO FUNCTION CURRENT SENSE
/*inline void sensordecorrente()
{
    float ia, ic;

    ia = (float)testeADCa;// ADC_PU_PPB_SCALE_FACTOR;
    ic = (float)testeADCc;// ADC_PU_PPB_SCALE_FACTOR;

}
*/

/*
__interrupt void adcal_isr_b(void)
{
    float ic;

    EALLOW;
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //pin 28
    EDIS;
    AdcbRegs.ADCSOCFRC1.bit.SOC0 = 1;
    EALLOW;
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 0;
    EDIS;

    ic = (float)ic_PPB* ADC_PU_PPB_SCALE_FACTOR;
}
*/
//
// End of file
//
