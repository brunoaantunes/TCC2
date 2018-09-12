/* ============================================================================
System Name:    Boost Rectifier Control using F28377s-XL

File Name:      Boost_main.c

Target:         F28377s Launch Pad

Author:         Lucas Plentz based on C2000 Systems Lab, 30th September 2015

Description:    Rectifier ISR
                Coded within ADCB1INT ISR @ 10Khz,
                  --> triggered by ADCB SOC6,
                      --> set up by EPWM6_SOCA tied to EPWM6 PRD

//----------------------------------------------------------------------------------
//  Copyright Texas Instruments © 2004-2015
//----------------------------------------------------------------------------------
//  Revision History: Lucas Plentz
//----------------------------------------------------------------------------------
//  Date      | Description / Status
//----------------------------------------------------------------------------------
// 6 May 2018 - Field Oriented Control of Bidirectional Boost Rectifier using F28377s-XL
//----------------------------------------------------------------------------------
 *
 *
Peripheral Assignments:
   Rectifier 1:
         - EPWMs ==>> EPWM6, EPWM7,  EPWM8  ---> A, B, C

         Analog signals - Rectifier 1
         Vdc+  ADC A2
         Vdc- ADC B2
         Va   ADC A1
         Vb   ADC
         Vc   ADC B1
         Ia   ADC A0
         Ib   ADC
         Ic   ADC B0

      DAC-C  ---> General purpose display (??)

===========================================================================  */

// Include header files used in the main function
// define float maths and then include IQmath library

#include "Boost.h"
#include "Boost-Settings.h"
#include "sgen.h"

// **********************************************************
// Prototypes for local functions within this file
// **********************************************************

// INTERRUPT FUNCTIONS
// ---------------------
#ifdef _FLASH
#pragma CODE_SECTION(RectifierControlISR, "ramfuncs"); //conferir essa função
#endif

#pragma INTERRUPT (RectifierControlISR, HPI)

// Prototype statements for functions found within this file.
interrupt void RectifierControlISR(void);

// Core Motor Control Functions
// ------------------------------
#if BUILDLEVEL  != LEVEL1
inline void retCurrentSense(void);
//inline void retVoltageSense(void);
//inline void retVoltageOutputSense(void);
#endif

void PwmTripConfig(volatile struct EPWM_REGS * PwmRegs, Uint16 TripNum);
//void DMC1_Protection(void);
//void DMC2_Protection(void);

// Miscellaneous functions
// -------------------------
void GPIO_TogglePin(Uint16 pin);

// State Machine function prototypes
//------------------------------------

// ****************************************************************************
// Variables for CPU control
// ****************************************************************************
// adc static cal
int *adc_cal;
//*********************** USER Variables *************************************


//****************************************************************************
// Global variables used in this system
//****************************************************************************

// ****************************************************************************
// Flag variables
// ****************************************************************************
volatile Uint16 EnableFlag = FALSE;

Uint32 IsrTicker = 0;

Uint16 BackTicker = 0;

int    LedCnt = 500;

int16 OffsetCalCounter;

_iq K1 = _IQ(0.998),          // Offset filter coefficient K1: 0.05/(T+0.05);
    K2 = _IQ(0.001999);       // Offset filter coefficient K2: T/(T+0.05);

RECTIFIER_VARS ret1 = RECTIFIER_DEFAULTS;

// ****************************************************************************
// 3-Phase Sine generation
// ****************************************************************************
SGENT_3 sgen = SGENT_3_DEFAULTS;

int a1, b1, c1;

// ****************************************************************************
// Miscellaneous Variables
// ****************************************************************************
_iq  IdRef_start = _IQ(0.1),
     IdRef_run   = _IQ(0.0);

// ****************************************************************************
// Variables for Datalog module
// ****************************************************************************
float DBUFF_4CH1[200],
      DBUFF_4CH2[200],
      DBUFF_4CH3[200],
      DBUFF_4CH4[200],
      DlogCh1,
      DlogCh2,
      DlogCh3,
      DlogCh4;

// Create an instance of DATALOG Module
DLOG_4CH_F dlog_4ch1;

//*******************************************************************************

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
/*
inline void retVoltageSense()
{
    ret1.voltageAs = (float)VFB_A1;
    //ret1.voltageBs =
    ret1.voltageCs = (float)VFB_C1;
}

inline void retVoltageOutputSense()
{
    ret1.voltageDC1 = (float)VDC1FB_A1;
    ret1.voltageDC2 = (float)VDC2FB_B1;
}
*/
#endif


// ****************************************************************************
// ****************************************************************************
//TODO  DMC Protection Against Over Current Protection
// ****************************************************************************
// ****************************************************************************

// ****************************************************************************
// ****************************************************************************
//TODO GENERAL PURPOSE UTILITY FUNCTIONS
// ****************************************************************************
// ****************************************************************************

// slew programmable ramper
/*_iq ramper(_iq in, _iq out, _iq rampDelta)
{
    _iq err;

    err = in - out;
    if (err > rampDelta)
        return(out + rampDelta);
    else if (err < -rampDelta)
        return(out - rampDelta);
    else
        return(in);
}
*/
//*****************************************************************************
// Ramp Controller for speed reference (Not currently used)
/*_iq ramper_speed(_iq in, _iq out, _iq rampDelta)
{
    _iq err;

    err = in - out;
    if (err > rampDelta)
    {
        if((out+rampDelta)>1.0)
            return(1.0);
        else
            return (out+rampDelta);
    }
    else if (err < -rampDelta)
    {
        if(out-rampDelta<=0.0)
            return(0.0);
        else
            return(out - rampDelta);
    }
    else
        return(in);
}
*/
//*****************************************************************************
// Reference Position Generator for position loop
/*_iq refPosGen(_iq out)
{
    _iq in = posArray[ptr1];

    out = ramper(in, out, posSlewRate);

    if (in == out)
    if (++cntr1 > 1000)
    {
        cntr1 = 0;
        if (++ptr1 >= ptrMax)
            ptr1 = 0;
    }
    return (out);
}
*/
//*****************************************************************************
//Toggle a GPIO pin
void GPIO_TogglePin(Uint16 pin)
{
    volatile Uint32 *gpioDataReg;
    Uint32 pinMask;

    gpioDataReg = (volatile Uint32 *)&GpioDataRegs + (pin/32)*GPY_DATA_OFFSET;
    pinMask = 1UL << (pin % 32);

    gpioDataReg[GPYTOGGLE] = pinMask;

    return;
}

//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************

void main(void)
{

    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This function derived from the one found in F2837x_SysCtrl.c file
    InitSysCtrl1();

    //CPU_enableDebugInt();

    // Waiting for enable flag set
    while (EnableFlag == FALSE)
    {
      BackTicker++;
    }

    // Clear all interrupts and initialize PIE vector table:

    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F28M3Xx_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F28M3Xx_DefaultIsr.c.
    // This function is found in F28M3Xx_PieVect.c.
    InitPieVectTable();

    // Configure a temp output pin for flagging (GPIO78)
    GPIO_SetupPinOptions(TEMP_GPIO, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(TEMP_GPIO, 0, TEMP_MUX);

    GPIO_SetupPinOptions(BLUE_LED_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(BLUE_LED_GPIO, GPIO_MUX_CPU1, BLUE_LED_MUX);

//    for (;;)
//    {
//      GPIO_TogglePin(BLUE_LED_GPIO);
//      DELAY_US(500000);
//    }

// Timing sync for background loops
// Timer period definitions found in device specific PeripheralHeaderIncludes.h
    CpuTimer0Regs.PRD.all =  10000;     // A tasks
    CpuTimer1Regs.PRD.all =  20000;     // B tasks
    CpuTimer2Regs.PRD.all =  30000;     // C tasks

// Tasks State-machine init

// ****************************************************************************
// ****************************************************************************
// Set up peripheral assignments for rectifier control
// ****************************************************************************
// ****************************************************************************
    ret1.PwmARegs = &EPwm6Regs;      // set up EPWM for rectifier branch A
    ret1.PwmBRegs = &EPwm7Regs;      // set up EPWM for rectifier branch B
    ret1.PwmCRegs = &EPwm8Regs;      // set up EPWM for rectifier branch C
// ****************************************************************************
// ****************************************************************************
// Initialize EPWM modules for inverter PWM generation
// ****************************************************************************
// ****************************************************************************

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    // *****************************************
    // PWM configuration for rectifier
    // ****************************************

    PWM_1ch_UpDwnCnt_CNF_noDB(ret1.PwmARegs, INV_PWM_TICKS);
    PWM_1ch_UpDwnCnt_CNF_noDB(ret1.PwmBRegs, INV_PWM_TICKS);
    PWM_1ch_UpDwnCnt_CNF_noDB(ret1.PwmCRegs, INV_PWM_TICKS);

    // configure Epwms 7 and 8 as slaves
    (ret1.PwmBRegs)->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    (ret1.PwmBRegs)->TBCTL.bit.PHSEN    = TB_ENABLE;
    (ret1.PwmBRegs)->TBPHS.bit.TBPHS    = 2;
    (ret1.PwmBRegs)->TBCTL.bit.PHSDIR   = TB_UP;

    (ret1.PwmCRegs)->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    (ret1.PwmCRegs)->TBCTL.bit.PHSEN    = TB_ENABLE;
    (ret1.PwmCRegs)->TBPHS.bit.TBPHS    = 2;
    (ret1.PwmCRegs)->TBCTL.bit.PHSDIR   = TB_UP;

    InitRet1EPwmGpio();  // Set up GPIOs for EPWMA of 6,7,8

//---------------------------------------------------------------------------------------

    // Setting up link from EPWM to ADC (EPwm6 is chosen)
    EPwm6Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD; // Select SOC from counter at ctr = PRD
    EPwm6Regs.ETPS.bit.SOCAPRD  = ET_1ST;     // Generate pulse on 1st even
    EPwm6Regs.ETSEL.bit.SOCAEN  = 1;          // Enable SOC on A group

// ****************************************************************************
// ****************************************************************************
//TODO ADC Configuration
// ****************************************************************************
// ****************************************************************************
    //Configure the ADC and power it up
    ConfigureADC();

    //Select the channels to convert and end of conversion flag

    EALLOW;

    // Analog signals - Rectifier 1
    // Vdc+  ADC A2
    // Vdc- ADC B2
    // Va   ADC A1
    // Vb   ADC
    // Vc   ADC B1
    // Ia   ADC A0
    // Ib   ADC
    // Ic   ADC B0

    // Configure SOCx on ADCs A and B (C and D not used)

    // Rectifier 1: Ia  @ A0
    // ********************************
    AdcaRegs.ADCSOC0CTL.bit.CHSEL     =  0;                    // SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS     = 14;                    // sample window in SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL   = ADCTRIG15_EPWM6SOCA;   // trigger on ePWM7 SOCA/C
    // Configure the post processing block (PPB) to eliminate subtraction related calculation
    AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;                     // PPB is associated with SOC0
    AdcaRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;                     // Write zero to this for now till offset ISR is run

    /*
    // Rectifier 1: Ib  @
    // ********************************
    AdcbRegs.ADCSOC0CTL.bit.CHSEL     =  0;                    // SOC0 will convert pin B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS     = 30;                    // sample window in SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL   = ADCTRIG17_EPWM7SOCA;   // trigger on ePWM2 SOCA/C
    // Configure the post processing block (PPB) to eliminate subtraction related calculation
    AdcbRegs.ADCPPB1CONFIG.bit.CONFIG = 0;                     // PPB is associated with SOC0
    AdcbRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;                     // Write zero to this for now till offset ISR is run
    */

    // Rectifier 1: Ic  @ B0
    // ********************************
    AdcbRegs.ADCSOC0CTL.bit.CHSEL     =  0;                    // SOC0 will convert pin B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS     = 14;                    // sample window in SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL   = ADCTRIG15_EPWM6SOCA;   // trigger on ePWM7 SOCA/C
    // Configure the post processing block (PPB) to eliminate subtraction related calculation
    AdcbRegs.ADCPPB1CONFIG.bit.CONFIG = 0;                     // PPB is associated with SOC0
    AdcbRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;                     // Write zero to this for now till offset ISR is run

    // Rectifier 1: Va  @ A1
    // ********************************
    AdcaRegs.ADCSOC4CTL.bit.CHSEL     =  1;                    // SOC4 will convert pin A1
    AdcaRegs.ADCSOC4CTL.bit.ACQPS     = 14;                    // sample window in SYSCLK cycles
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL   = ADCTRIG15_EPWM6SOCA;   // trigger on ePWM6 SOCA/C

    /*
    // Rectifier 1: Vb  @
    // ********************************
    AdcbRegs.ADCSOC3CTL.bit.CHSEL     =  4;                    // SOC3 will convert pin B4
    AdcbRegs.ADCSOC3CTL.bit.ACQPS     = 30;                    // sample window in SYSCLK cycles
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL   = ADCTRIG17_EPWM7SOCA;   // trigger on ePWM2 SOCA/C
    */

    // Rectifier 1: Vc  @ B1
    // ********************************
    AdcbRegs.ADCSOC2CTL.bit.CHSEL     =  1;                    // SOC2 will convert pin B1
    AdcbRegs.ADCSOC2CTL.bit.ACQPS     = 14;                    // sample window in SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL   = ADCTRIG15_EPWM6SOCA;   // trigger on ePWM6 SOCA/C

    // Recitifier 1: Vdc+  @ A2
    // ********************************
    AdcaRegs.ADCSOC6CTL.bit.CHSEL     = 2;                     // SOC6 will convert pin A2
    AdcaRegs.ADCSOC6CTL.bit.ACQPS     = 14;                    // sample window in SYSCLK cycles
    AdcaRegs.ADCSOC6CTL.bit.TRIGSEL   = ADCTRIG15_EPWM6SOCA;   // trigger on ePWM6 SOCA/C

    // Recitifier 1: Vdc-  @ B2
    // ********************************
    AdcbRegs.ADCSOC6CTL.bit.CHSEL     = 2;                    // SOC6 will convert pin B2
    AdcbRegs.ADCSOC6CTL.bit.ACQPS     = 14;                    // sample window in SYSCLK cycles
    AdcbRegs.ADCSOC6CTL.bit.TRIGSEL   = ADCTRIG15_EPWM6SOCA;   // trigger on ePWM6 SOCA/C

// ****************************************************************************
// ****************************************************************************
//TODO ISR Mapping
// ****************************************************************************
// ****************************************************************************
    // ADC B EOC of SOC6 is used to trigger Rectifier control Interrupt
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL  = 6;
    AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
    AdcbRegs.ADCINTSEL1N2.bit.INT1E    = 1;

    PieVectTable.ADCB1_INT         = &RectifierControlISR;
    PieCtrlRegs.PIEIER1.bit.INTx2  = 1;  // Enable ADCB1INT in PIE group 1

    IER |= M_INT1;                       // Enable group 1 interrupts

    // SETUP DAC-C (DACs A, B and C are already used up)

    EDIS;

// ****************************************************************************
// ****************************************************************************
//TODO Paramaeter Initialisation
// ****************************************************************************
// ****************************************************************************

    // Initialize the PI module for voltage
    ret1.pi_voltage.Kp = _IQ(0.051);
    ret1.pi_voltage.Ki = _IQ(13.837);
    ret1.pi_voltage.Umax = _IQ(1.0);
    ret1.pi_voltage.Umin = _IQ(-1.0);

    // Init PI module for ID loop
    ret1.pi_id.Kp   = _IQ(-0.260);
    ret1.pi_id.Ki   = _IQ(-4613.210);
    ret1.pi_id.Umax = _IQ(0.5);
    ret1.pi_id.Umin = _IQ(-0.5);

    // Init PI module for IQ loop
    ret1.pi_iq.Kp   = _IQ(-0.260);
    ret1.pi_iq.Ki   = _IQ(-4613.210);
    ret1.pi_iq.Umax = _IQ(0.8);
    ret1.pi_iq.Umin = _IQ(-0.8);

    // Set mock REFERENCES for Voltage and Iq loops
    ret1.VoltageRef = _IQ(400.0);

    ret1.IqRef = _IQ(0.0);

    ret1.Kqd = _IQ(0.00377);

    // Init FLAGS
    //motor1.RunMotor = 1;

//  Note that the vectorial sum of d-q PI outputs should be less than 1.0 which
//  refers to maximum duty cycle for SVGEN. Another duty cycle limiting factor
//  is current sense through shunt resistors which depends on hardware/software
//  implementation. Depending on the application requirements 3,2 or a single
//  shunt resistor can be used for current waveform reconstruction. The higher
//  number of shunt resistors allow the higher duty cycle operation and better
//  dc bus utilization. The users should adjust the PI saturation levels
//  carefully during open loop tests (i.e pi_id.Umax, pi_iq.Umax and Umins) as
//  in project manuals. Violation of this procedure yields distorted  current
// waveforms and unstable closed loop operations which may damage the inverter.

// ****************************************************
// Initialize DATALOG module
// ****************************************************
    DLOG_4CH_F_init(&dlog_4ch1);
    dlog_4ch1.input_ptr1 = &DlogCh1;    //data value
    dlog_4ch1.input_ptr2 = &DlogCh2;
    dlog_4ch1.input_ptr3 = &DlogCh3;
    dlog_4ch1.input_ptr4 = &DlogCh4;
    dlog_4ch1.output_ptr1 = &DBUFF_4CH1[0];
    dlog_4ch1.output_ptr2 = &DBUFF_4CH2[0];
    dlog_4ch1.output_ptr3 = &DBUFF_4CH3[0];
    dlog_4ch1.output_ptr4 = &DBUFF_4CH4[0];
    dlog_4ch1.size = 200;
    dlog_4ch1.pre_scalar = 5;
    dlog_4ch1.trig_value = 0.01;
    dlog_4ch1.status = 2;

// ****************************************************************************
// ****************************************************************************
// Call DMC Protection function
// ****************************************************************************
// ****************************************************************************
//    DMC1_Protection();

// TODO
// ****************************************************************************
// ****************************************************************************
// Feedbacks OFFSET Calibration Routine
// ****************************************************************************
// ****************************************************************************
    EALLOW;
      CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    //conferir essa parte do código

    ret1.offset_shntA = 0;
    ret1.offset_shntB = 0;
    ret1.offset_shntC = 0;

    for (OffsetCalCounter=0; OffsetCalCounter<20000; )
    {
        if(EPwm6Regs.ETFLG.bit.SOCA==1)
        {
            if(OffsetCalCounter>1000)
            {
                ret1.offset_shntA = K1*ret1.offset_shntA + K2*(IFB_A1)*ADC_PU_SCALE_FACTOR;     //Mtr1 : Phase A offset
                //ret1.offset_shntB = K1*ret1.offset_shntB + K2*(IFB_B1)*ADC_PU_SCALE_FACTOR;     //Mtr1 : Phase B offset
                ret1.offset_shntC = K1*ret1.offset_shntC + K2*(IFB_C1)*ADC_PU_SCALE_FACTOR;     //Mtr1 : Phase C offset
            }
            EPwm6Regs.ETCLR.bit.SOCA=1;
            OffsetCalCounter++;
        }
    }

    // ********************************************
    // Init OFFSET regs with identified values
    // ********************************************
    EALLOW;

    AdcaRegs.ADCPPB1OFFREF = ret1.offset_shntA*4096.0;      // set shunt Iu1 offset
    //AdcbRegs.ADCPPB1OFFREF = ret1.offset_shntB*4096.0;      // set shunt Iv1 offset
    AdcbRegs.ADCPPB1OFFREF = ret1.offset_shntC*4096.0;      // set shunt Iw1 offset

    EDIS;

// ****************************************************************************
// ****************************************************************************
//TODO Enable Interrupts
// ****************************************************************************
// ****************************************************************************
    EALLOW;
    EINT;          // Enable Global interrupt INTM
    ERTM;          // Enable Global realtime interrupt DBGM
    EDIS;

// ***************************************************************************
//  Initialisations COMPLETE
//  - IDLE loop. Just loop forever
// ***************************************************************************
    /*for(;;)  //infinite loop
    {
        // State machine entry & exit point
        //===========================================================
        (*Alpha_State_Ptr)();   // jump to an Alpha state (A0,B0,...)
        //===========================================================
    }*/
} //END MAIN CODE

/* ****************************************************************************
 * ****************************************************************************
 * ****************************************************************************
 * ****************************************************************************
 */
//=================================================================================
//  STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
//If needed, insert state machine functions here. See motorVars.h for reference

#if BUILDLEVEL == LEVEL1
// ****************************************************************************
// ****************************************************************************
//TODO Rectifier Control ISR - Build level 1
//    Checks target independent modules, duty cycle waveforms and PWM update
//    Generate a sine wave for testing SVPWM
// ****************************************************************************
// ****************************************************************************
inline void BuildLevel1(RECTIFIER_VARS * ret) //svpwm generation test
{
// ------------------------------------------------------------------------------
//  Sins generation
// ------------------------------------------------------------------------------
    sgen.offset = 0;
    sgen.gain = 0x7fff; // gain = 1 in Q15
    sgen.freq = 5369; // freq = (req_freq/max_freq)*2^15 >> (50/305.17)*2^15
    sgen.step_max = 1000; // max_freq = (step_max*sampling_freq)/65536 >> (1000*20k)/65536
    //sgen.alpha = 0; // phase = (req_phase/180) in Q15 >> (+90/180) in Q15 = 4000h

    sgen.calc(&sgen);
    a1 = sgen.out1;
    b1 = sgen.out2;
    c1 = sgen.out3;
// ------------------------------------------------------------------------------
//  Connect inputs of the CLARKE module and call the clarke macro
// ------------------------------------------------------------------------------

    ret->clarke.As = a1; // ret->currentAs;
    ret->clarke.Bs = b1; // ret->currentBs;
    ret->clarke.Cs = c1; // ret->currentCs;
    CLARKE_MACRO(ret->clarke)

// ------------------------------------------------------------------------------
//  Generate de Angle for the park transform though atan2()
// ------------------------------------------------------------------------------
    ret->park.Alpha = ret->clarke.Alpha;
    ret->park.Beta  = ret->clarke.Beta;
    ret->park.Angle = atan2(ret->clarke.Alpha, ret->clarke.Beta);
// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
//  There are two option for trigonometric functions:
//  IQ sin/cos look-up table provides 512 discrete sin and cos points in Q30 format
//  IQsin/cos PU functions interpolate the data in the lookup table yielding higher resolution.
// ------------------------------------------------------------------------------

    ret->park.Sine    = __sinpuf32(ret->park.Angle);
    ret->park.Cosine  = __cospuf32(ret->park.Angle);

    ret->ipark.Ds     = ret->IdTesting; //ver esses valores de Vd e Vq testing
    ret->ipark.Qs     = ret->IqTesting;
    ret->ipark.Sine   = ret->park.Sine;
    ret->ipark.Cosine = ret->park.Cosine;
    IPARK_MACRO(ret->ipark)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    ret->svgen.Ualpha = ret->ipark.Alpha;
    ret->svgen.Ubeta  = ret->ipark.Beta;
    SVGENDQ_MACRO(ret->svgen)

// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
    (ret->PwmARegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret->svgen.Ta)+INV_PWM_HALF_TBPRD;
    (ret->PwmBRegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret->svgen.Tb)+INV_PWM_HALF_TBPRD;
    (ret->PwmCRegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret->svgen.Tc)+INV_PWM_HALF_TBPRD;

// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = a1; // ret->park.Angle;
    DlogCh2 = b1; // ret->svgen.Ta;
    //DlogCh3 = ret->svgen.Tb;
    //DlogCh4 = ret->svgen.Tc;

    return;
}
#endif


#if BUILDLEVEL == LEVEL2
// ****************************************************************************
// ****************************************************************************
//TODO Recitfier Control ISR - - Build level 2
//    Level 2 verifies
//       - current sense schems
//       - analog-to-digital conversion - shunt current
//       - clarke/park transformations (CLARKE/PARK)
//
// ****************************************************************************
// ****************************************************************************
inline void BuildLevel2(RECTIFIER_VARS * ret)
{

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//  Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
    ret->clarke.As = ret->currentAs; // Phase A curr.
    ret->clarke.Bs = ret->currentBs; // Phase B curr.
    ret->clarke.Cs = ret->currentCs; // Phase C curr.
    CLARKE_MACRO(ret->clarke)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    ret->park.Alpha = ret->clarke.Alpha;
    ret->park.Beta  = ret->clarke.Beta;
    ret->park.Angle = atan2(ret->clarke.Alpha, ret->clarke.Beta);
    ret->park.Sine   = __sinpuf32(ret->park.Angle);
    ret->park.Cosine = __cospuf32(ret->park.Angle);
    PARK_MACRO(ret->park)

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ret->ipark.Ds = ret->IdTesting; //change to Iq and Id testing
    ret->ipark.Qs = ret->IqTesting;

    ret->ipark.Sine   = ret->park.Sine;
    ret->ipark.Cosine = ret->park.Cosine;
    IPARK_MACRO(ret->ipark)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    ret->svgen.Ualpha = ret->ipark.Alpha;
    ret->svgen.Ubeta  = ret->ipark.Beta;
    SVGENDQ_MACRO(ret->svgen)

// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
    (ret->PwmARegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret->svgen.Ta)+INV_PWM_HALF_TBPRD;
    (ret->PwmBRegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret->svgen.Tb)+INV_PWM_HALF_TBPRD;
    (ret->PwmCRegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret->svgen.Tc)+INV_PWM_HALF_TBPRD;

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    //DlogCh1 = ret->
    //DlogCh2 = ret->   //  motor->svgen.Ta;      //
    DlogCh3 = ret->clarke.As;
    DlogCh4 = ret->clarke.Bs;

    return;
}
#endif


#if BUILDLEVEL == LEVEL3
// ****************************************************************************
// ****************************************************************************
//TODO Rectifier Control ISR - - Build level 3
//  Level 3 verifies the dq-axis current regulation performed by PID and voltage
//  measurement modules
//  lsw=0: lock the rotor of the motor
//  lsw=1: close the current loop

// ****************************************************************************
// ****************************************************************************
inline void BuildLevel3(RECTIFIER_VARS * ret)
{
// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//  Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
    ret->clarke.As = ret->currentAs; // Phase A curr.
    ret->clarke.Bs = ret->currentBs; // Phase B curr.
    ret->clarke.Cs = ret->currentCs;
    CLARKE_MACRO(ret->clarke)
// ------------------------------------------------------------------------------
//  Connect inputs of the Vdc module and call the output voltage measurement
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    ret->park.Alpha = ret->clarke.Alpha;
    ret->park.Beta  = ret->clarke.Beta;

    ret->park.Sine   = __sinpuf32(ret->park.Angle);
    ret->park.Cosine = __cospuf32(ret->park.Angle);
    PARK_MACRO(ret->park)

// ------------------------------------------------------------------------------
//  Connect inputs of the PID_REG3 module and call the PID Vdc controller macro
// ------------------------------------------------------------------------------
    ret->pi_voltage.Ref =  ret->VoltageRef;
    ret->pi_voltage.Fbk = ; //leitura do sensor de tensão de saída
    PI_MACRO(ret->pi_voltage)
// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PID ID controller macro
// ------------------------------------------------------------------------------
    ret->pi_id.Ref = ret->pi_voltage.Out
    ret->pi_id.Fbk = ret->park.Ds;
    PI_MACRO(ret->pi_iq)
// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PID IQ controller macro
// ------------------------------------------------------------------------------
    ret->pi_iq.Ref = ret->IqRef;
    ret->pi_iq.Fbk = motor->park.Qs;
    PI_MACRO(ret->pi_id)
// ------------------------------------------------------------------------------
//  Desacoplamento
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ret->ipark.Ds = ret->pi_id.Out + ret->park.Qs*Kdq;   // decoupling Iq
    ret->ipark.Qs = ret->pi_iq.Out + ret->park.Ds*(-Kdq);  // decoupling Id
    ret->ipark.Sine   = ret->park.Sine;
    ret->ipark.Cosine = ret->park.Cosine;
    IPARK_MACRO(ret->ipark)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    ret->svgen.Ualpha = ret->ipark.Alpha;
    ret->svgen.Ubeta  = ret->ipark.Beta;
    SVGENDQ_MACRO(ret->svgen)

// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
    (ret->PwmARegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret->svgen.Ta)+INV_PWM_HALF_TBPRD;
    (ret->PwmBRegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret->svgen.Tb)+INV_PWM_HALF_TBPRD;
    (ret->PwmCRegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*ret->svgen.Tc)+INV_PWM_HALF_TBPRD;

// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    //DlogCh2 = ret->;     // output voltage
    //DlogCh1 = ret->rg.Out;
    DlogCh3 = ret->clarke.As;  // motor->current.Bs;
    DlogCh4 = ret->clarke.Bs;  // motor->current.Cs;

    return;
}
#endif

// ****************************************************************************
// ****************************************************************************
//TODO Rectifier Control ISR
// ****************************************************************************
// ****************************************************************************
interrupt void RectifierControlISR(void) //Analisar essa função
{

    // Verifying the ISR
    IsrTicker++;

//    GPIO_TogglePin(TEMP_GPIO, IsrTicker%2);

#if (BUILDLEVEL == LEVEL1)

    BuildLevel1(&ret1);

#else
// ------------------------------------------------------------------------------
//  Measure phase currents
// ------------------------------------------------------------------------------
    retCurrentSense();    //  Measure normalised phase currents (-1,+1)
    //retVoltageSense();
    //retVoltageOutputSense();

#if (BUILDLEVEL==LEVEL2)

    BuildLevel2(&ret1);

#elif (BUILDLEVEL==LEVEL3)

    BuildLevel3(&ret1);

#endif

#endif

// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
    DLOG_4CH_F_FUNC(&dlog_4ch1);

    //clear ADCINT1 INT and ack PIE INT
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1=1;
    PieCtrlRegs.PIEACK.all=PIEACK_GROUP1;

}// MainISR Ends Here


/****************************************************************************
 * End of Code *
 * ***************************************************************************
 */
