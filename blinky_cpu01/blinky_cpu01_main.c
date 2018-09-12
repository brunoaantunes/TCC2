//###########################################################################
//
// FILE:   blinky_cpu01.c
//
// TITLE:  LED Blink Example for F2837xS.
//
//! \addtogroup cpu01_example_list
//! <h1> Blinky </h1>
//!
//! This example blinks LED X
//!
//! \note If using a Launchpad, use the Launchpad build configurations.
//!
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
#include "main_includes.h"
#include "main_settings.h"

//
// Function Prototypes
//

#if BUILDLEVEL  != LEVEL1
inline void retCurrentSense(void);
inline void retVoltageSense(void);
inline void retVoltageOutputSense(void);
inline void sensordecorrente(void);
#endif

__interrupt void adcal_isr_a(void); //CANAL A
__interrupt void adcal_isr_b(void); //CANAL b
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
__interrupt void RectifierControlISR(void); // change the build level in main_settings.h

//
// Defines
//
#define BLINKY_LED_GPIO      12
#define BLINKY_LED_GPIO_2    13

#define SIGNAL_LENGTH 512 // sine gen

#define testeADCa AdcaResultRegs.ADCRESULT0;
#define testeADCc AdcbResultRegs.ADCRESULT0;

#define ia_PPB   ((signed int)AdcbResultRegs.ADCPPB1RESULT.all)
#define ic_PPB   ((signed int)AdcbResultRegs.ADCPPB1RESULT.all)
#define ADC_PU_PPB_SCALE_FACTOR    0.000488281250     //1/2^11

//
// Globals
//

RECTIFIER_VARS1 ret1;

#pragma DATA_SECTION(ipcb1, "ipcb1");
#pragma DATA_SECTION(ipcb2, "ipcb2");
#pragma DATA_SECTION(ipcb3, "ipcb3");
float ipcb1[SIGNAL_LENGTH];
float ipcb2[SIGNAL_LENGTH];
float ipcb3[SIGNAL_LENGTH];

float a1, b1, c1;
float angle;

//float K1 = 0.998;          // Offset filter coefficient K1: 0.05/(T+0.05);
//float K2 = 0.001999;       // Offset filter coefficient K2: T/(T+0.05);


#if BUILDLEVEL != LEVEL1
// ******************************************************************************
// CURRENT SENSOR SUITE
// - Reads rectifier currents from inverter bottom leg SHUNTs
// ******************************************************************************
inline void retCurrentSense()
{
    ret1.currentAs = (float)IFB_A1_PPB* ADC_PU_PPB_SCALE_FACTOR;
    ret1.currentCs = (float)IFB_C1_PPB* ADC_PU_PPB_SCALE_FACTOR;
    ret1.currentBs = -ret1.currentAs - ret1.currentCs;

    return;
}

inline void retVoltageSense()
{
    ret1.voltageAs = (float)VFB_A1;
    ret1.voltageCs = (float)VFB_C1;
    ret1.voltageBs = -ret1.voltageAs - ret1.VoltageCs;
}

inline void retVoltageOutputSense()
{
    ret1.voltageDC1 = (float)VDC1FB_A1;
    ret1.voltageDC2 = (float)VDC2FB_B1;
}

#endif

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
    InitGpio();
    GPIO_SetupPinMux(BLINKY_LED_GPIO, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(BLINKY_LED_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(BLINKY_LED_GPIO_2, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(BLINKY_LED_GPIO_2, GPIO_OUTPUT, GPIO_PUSHPULL);

//
// Enable PWM6, PWM7 and PWM8
//
    CpuSysRegs.PCLKCR2.bit.EPWM6=1;
    CpuSysRegs.PCLKCR2.bit.EPWM7=1;
    CpuSysRegs.PCLKCR2.bit.EPWM8=1;

//
// For this case just init GPIO pins for ePWM6, ePWM7, ePWM8
// These functions are in the F2837xS_EPwm.c file

    InitEPwm6Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();

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

//
//TODO Map ISR Functions
//
    EALLOW; // This is needed to write to EALLOW protected registers

    PieVectTable.ADCA1_INT = &adcal_isr_a; //&RectifierControlISR;
    PieVectTable.ADCB1_INT = &adcal_isr_b;

    PieVectTable.EPWM6_INT = &epwm1_isr;
    PieVectTable.EPWM7_INT = &epwm2_isr;
    PieVectTable.EPWM8_INT = &epwm3_isr;

    EDIS; // This is needed to disable write to EALLOW protected registers

    //
    // Configure the ADC and power it up
    //
    ConfigureADC();

//
// Setup the ADC for continuous conversions
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    SetupADCContinuous();
    //sensordecorrente();

    InitEPwm1Example();
    InitEPwm2Example();
    InitEPwm3Example();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

// TODO Enable PIE interrupts

    // PIE interrupts for ADC
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;

    // PIE interrupts for ePWM
    PieCtrlRegs.PIEIER2.bit.INTx6 = 1;
    PieCtrlRegs.PIEIER2.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER2.bit.INTx8 = 1;

// ****************************************************************************
// ****************************************************************************
//TODO Parameter Initialisation
// ****************************************************************************
// ****************************************************************************

    // Initialize the PI module for voltage
    ret1.pi_voltage1.Kp = 0.051;
    ret1.pi_voltage1.Ki = 13.837;
    ret1.pi_voltage1.Umax = 1.0;
    ret1.pi_voltage1.Umin = -1.0;

    // Init PI module for ID loop
    ret1.pi_id1.Kp   = -0.26;
    ret1.pi_id1.Ki   = -4613.21;
    ret1.pi_id1.Umax = 0.5;
    ret1.pi_id1.Umin = -0.5;

    // Init PI module for IQ loop
    ret1.pi_iq1.Kp   = -0.26;
    ret1.pi_iq1.Ki   = -4613.210;
    ret1.pi_iq1.Umax = 0.8;
    ret1.pi_iq1.Umin = -0.8;

    // Set references for Voltage, Id and Iq loops
    ret1.VoltageRef = 400.0;
    ret1.IdRef = 1.0;
    ret1.IqRef = 0.0;
    ret1.Kqd = 0.00377;

//
// Enable global Interrupts and higher priority real-time debug events:
//

    IER |= M_INT3; // enable group 3 interrupts - ADC
    IER |= M_INT1; // enable group 1 interrupts - ADC

    IER |= M_INT6; // enable group 6 interrupts - ePWM
    IER |= M_INT7; // enable group 7 interrupts - ePWM

    EALLOW;
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
    EDIS;

//
// Step 6. IDLE loop. Just sit and loop forever (optional):
//

    for(;;);

} //END MAIN

#if BUILDLEVEL == LEVEL1
// ****************************************************************************
// ****************************************************************************
//TODO Rectifier Control ISR - Build level 1
//    Checks target independent modules, duty cycle waveforms and PWM update
//    Generate a sine wave for testing SVPWM
// ****************************************************************************
// ****************************************************************************
inline void BuildLevel1(RECTIFIER_VARS1 * ret) //svpwm generation test
{
    unsigned long i;
// ------------------------------------------------------------------------------
//  Sins generation
// ------------------------------------------------------------------------------
    sgen.offset = 0;
    sgen.gain = 0x7fff; // gain = 1 in Q15
    sgen.freq = 5369; // freq = (req_freq/max_freq)*2^15 >> (50/305.17)*2^15
    sgen.step_max = 1000; // max_freq = (step_max*sampling_freq)/65536 >> (1000*20k)/65536
    //sgen.alpha = 0; // phase = (req_phase/180) in Q15 >> (+90/180) in Q15 = 4000h

    for(i=0;i<SIGNAL_LENGTH;i++)
    {
        ipcb1[i]=0;
        ipcb2[i]=0;
        ipcb3[i]=0;
    }

    for(i=0;i<SIGNAL_LENGTH;i++)
    {
        sgen.calc(&sgen);
        a1 = sgen.out1;
        b1 = sgen.out2;
        c1 = sgen.out3;
// ------------------------------------------------------------------------------
//  Connect inputs of the CLARKE module and call the clarke macro
// ------------------------------------------------------------------------------
        ret1.clarke1.As = a1;
        ret1.clarke1.Bs = b1;
        CLARKE_CLA_MACRO(ret1.clarke1);
// ------------------------------------------------------------------------------
//  Generate the Angle for the park transform through atan2() and call the macro
// ------------------------------------------------------------------------------
        angle = atan2f(ret1.clarke1.alpha, ret1.clarke1.beta); //angle float
        ret1.park1.alpha = ret1.clarke1.alpha;
        ret1.park1.beta = ret1.clarke1.beta;
        ret1.park1.theta = angle;
        PARK_CLA_MACRO(ret1.park1);
// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
        ret1.ipark1.d = ret1.park1.d;
        ret1.ipark1.q = ret1.park1.q;
        ret1.ipark1.sine = ret1.park1.sine;
        ret1.ipark1.cossine = ret1.park1.cossine;
        iPARK_CLA_MACRO(ret1.ipark1);
// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
        ret1.svgen1.Ualpha = ret1.ipark1.alpha;
        ret1.svgen1.Ubeta = ret1.ipark1.beta;
        SVGEN_CLA_MACRO(ret1.svgen1)

// ------------------------------------------------------------------------------
//  Connect inputs to display data
// ------------------------------------------------------------------------------
        ipcb1[i] = ret1.ipark1.alpha;
        ipcb2[i] = ret1.svgen1.Ta;
        ipcb3[i] = ret1.svgen1.Tb;
    }
// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
        EPwm6Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret1.svgen1.Ta)+INV_PWM_HALF_TBPRD;
        EPwm7Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret1.svgen1.Tb)+INV_PWM_HALF_TBPRD;
        EPwm8Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret1.svgen1.Tc)+INV_PWM_HALF_TBPRD;

    return;
}
#endif


#if BUILDLEVEL == LEVEL2
// ****************************************************************************
// ****************************************************************************
//TODO Recitfier Control ISR - - Build level 2
//    Level 2 verifies the ADC reads and offset calibration
// ****************************************************************************
// ****************************************************************************
inline void BuildLevel2(RECTIFIER_VARS1 * ret)
{
// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//  Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
    ret1.clarke1.As = currentAs;
    ret1.clarke1.Bs = currentBs;
    CLARKE_CLA_MACRO(ret1.clarke1);
// ------------------------------------------------------------------------------
//  Generate the Angle for the park transform through atan2() and call the macro
// ------------------------------------------------------------------------------
    angle = atan2f(ret1.clarke1.alpha, ret1.clarke1.beta); //angle float
    ret1.park1.alpha = ret1.clarke1.alpha;
    ret1.park1.beta = ret1.clarke1.beta;
    ret1.park1.theta = angle;
    PARK_CLA_MACRO(ret1.park1);
// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ret1.ipark1.d = ret1.IdTesting;
    ret1.ipark1.q = ret1.IqTesting;
    ret1.ipark1.sine = ret1.park1.sine;
    ret1.ipark1.cossine = ret1.park1.cossine;
    iPARK_CLA_MACRO(ret1.ipark1);
// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    ret1.svgen1.Ualpha = ret1.ipark1.alpha;
    ret1.svgen1.Ubeta = ret1.ipark1.beta;
    SVGEN_CLA_MACRO(ret1.svgen1)
// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
    EPwm6Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret1.svgen1.Ta)+INV_PWM_HALF_TBPRD;
    EPwm7Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret1.svgen1.Tb)+INV_PWM_HALF_TBPRD;
    EPwm8Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret1.svgen1.Tc)+INV_PWM_HALF_TBPRD;

    return;
}
#endif


#if BUILDLEVEL == LEVEL3
// ****************************************************************************
// ****************************************************************************
//TODO Rectifier Control ISR - - Build level 3
//  Level 3 verifies the id and iq open loop control
// ****************************************************************************
// ****************************************************************************
inline void BuildLevel3(RECTIFIER_VARS1 * ret)
{
// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//  Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
    ret1.clarke1.As = currentAs;
    ret1.clarke1.Bs = currentBs;
    CLARKE_CLA_MACRO(ret1.clarke1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    angle = atan2f(ret1.clarke1.alpha, ret1.clarke1.beta); //angle float
    ret1.park1.alpha = ret1.clarke1.alpha;
    ret1.park1.beta = ret1.clarke1.beta;
    ret1.park1.theta = angle;
    PARK_CLA_MACRO(ret1.park1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI Id controller macro
// ------------------------------------------------------------------------------
    ret1.pi_id1.Ref = ret1.IdRef;
    ret1.pi_id1.Fbk = ret1.park1.d;
    PI_CLA(ret1.pi_iq1)
// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI Iq controller macro
// ------------------------------------------------------------------------------
    ret1.pi_iq1.Ref = ret1.IqRef;
    ret1.pi_iq1.Fbk = ret1.park1.q;
    PI_CLA(ret1.pi_iq1)
// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ret1.ipark1.d = ret1.pi_id1.Out + ret1.park1.d*Kdq;   // decoupling Iq
    ret->ipark1.q = ret1.pi_iq1.Out + ret1.park1.q*(-Kdq);  // decoupling Id
    ret1.ipark1.sine = ret1.park1.sine;
    ret1.ipark1.cossine = ret1.park1.cossine;
    iPARK_CLA_MACRO(ret1.ipark1);
// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    ret1.svgen1.Ualpha = ret1.ipark1.alpha;
    ret1.svgen1.Ubeta = ret1.ipark1.beta;
    SVGEN_CLA_MACRO(ret1.svgen1)
// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
    EPwm6Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret1.svgen1.Ta)+INV_PWM_HALF_TBPRD;
    EPwm7Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret1.svgen1.Tb)+INV_PWM_HALF_TBPRD;
    EPwm8Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret1.svgen1.Tc)+INV_PWM_HALF_TBPRD;

    return;
}
#endif

#if BUILDLEVEL == LEVEL4
// ****************************************************************************
// ****************************************************************************
//TODO Rectifier Control ISR - - Build level 4
//  Level 4 verifies the closed loop control
// ****************************************************************************
// ****************************************************************************
inline void BuildLevel4(RECTIFIER_VARS1 * ret)
{
// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//  Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
    ret1.clarke1.As = currentAs;
    ret1.clarke1.Bs = currentBs;
    CLARKE_CLA_MACRO(ret1.clarke1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    angle = atan2f(ret1.clarke1.alpha, ret1.clarke1.beta); //angle float
    ret1.park1.alpha = ret1.clarke1.alpha;
    ret1.park1.beta = ret1.clarke1.beta;
    ret1.park1.theta = angle;
    PARK_CLA_MACRO(ret1.park1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI Vdc controller macro
// ------------------------------------------------------------------------------
    ret1.pi_voltage1.Ref =  ret1.VoltageRef;
    ret1.pi_voltage1.Fbk = ; //leitura do sensor de tensão de saída
    PI_CLA(ret1.pi_voltage1)
// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI Id controller macro
// ------------------------------------------------------------------------------
    ret1.pi_id1.Ref = ret1.pi_voltage1.Out
    ret1.pi_id1.Fbk = ret1.park1.d;
    PI_CLA(ret1.pi_iq1)
// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI Iq controller macro
// ------------------------------------------------------------------------------
    ret1.pi_iq1.Ref = ret1.IqRef;
    ret1.pi_iq1.Fbk = ret1.park1.q;
    PI_CLA(ret1.pi_iq1)
// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ret1.ipark1.d = ret1.pi_id1.Out + ret1.park1.d*Kdq;   // decoupling Iq
    ret->ipark1.q = ret1.pi_iq1.Out + ret1.park1.q*(-Kdq);  // decoupling Id
    ret1.ipark1.sine = ret1.park1.sine;
    ret1.ipark1.cossine = ret1.park1.cossine;
    iPARK_CLA_MACRO(ret1.ipark1);

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    ret1.svgen1.Ualpha = ret1.ipark1.alpha;
    ret1.svgen1.Ubeta = ret1.ipark1.beta;
    SVGEN_CLA_MACRO(ret1.svgen1)
// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
    EPwm6Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret1.svgen1.Ta)+INV_PWM_HALF_TBPRD;
    EPwm7Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret1.svgen1.Tb)+INV_PWM_HALF_TBPRD;
    EPwm8Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret1.svgen1.Tc)+INV_PWM_HALF_TBPRD;

    return;
}
#endif

// TODO FUNCTION CURRENT SENSOR
/*inline void sensordecorrente()
{
    float ia, ic;

    ia = (float)testeADCa;// ADC_PU_PPB_SCALE_FACTOR;
    ic = (float)testeADCc;// ADC_PU_PPB_SCALE_FACTOR;

}
*/

//
// TODO INTERRUPTION FUNCTIONS
//

__interrupt void adcal_isr_a(void)
{
    float ia;

    EALLOW;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //pin 27
    EDIS;
    AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1;
    EALLOW;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 0;
    EDIS;

    ia = testeADCa;
}

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

//
// epwm1_isr - EPWM1 ISR
//
__interrupt void epwm1_isr(void)
{
    //
    // Clear INT flag for this timer
    //
    EPwm6Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// epwm2_isr - EPWM2 ISR
//
__interrupt void epwm2_isr(void)
{
    //
    // Clear INT flag for this timer
    //
    EPwm7Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// epwm3_isr - EPWM3 ISR
//
__interrupt void epwm3_isr(void)
{
    //
    // Clear INT flag for this timer
    //
    EPwm8Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
//TODO Main interruption function
//
__interrupt void RectifierControlISR(void)
{

#if (BUILDLEVEL == LEVEL1)

    BuildLevel1(&ret1);

#else
// ------------------------------------------------------------------------------
//  Measure phase currents
// ------------------------------------------------------------------------------
    retCurrentSense();    //  Measure normalised phase currents (-1,+1)
    retVoltageSense();
    retVoltageOutputSense();

#if (BUILDLEVEL==LEVEL2)

    BuildLevel2(&ret1);

#elif (BUILDLEVEL==LEVEL3)

    BuildLevel3(&ret1);

#elif (BUILDLEVEL==LEVEL4)

    BuildLevel4(&ret1);

#endif

#endif

    //clear ADCINT1 INT and ack PIE INT
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1=1;
    PieCtrlRegs.PIEACK.all=PIEACK_GROUP1;

}// MainISR Ends Here

//
// End of file
//



