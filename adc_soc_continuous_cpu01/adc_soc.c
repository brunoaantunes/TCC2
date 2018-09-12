//
//TODO ADC configure vide pmsm control
// ****************************************************************************
// Variables for CPU control
// ****************************************************************************
// adc static cal
int *adc_cal;

RET_VARS ret1 = RET_DEFAULTS;

/*
// ******************************************************************************
// CURRENT SENSOR SUITE
// - Reads motor currents from inverter bottom leg SHUNTs
// ******************************************************************************
inline void motorCurrentSense()
{
    motor1.currentAs = (float)IFB_A1_PPB* ADC_PU_PPB_SCALE_FACTOR;
    motor1.currentBs = (float)IFB_B1_PPB* ADC_PU_PPB_SCALE_FACTOR;
    motor1.currentCs = -motor1.currentAs - motor1.currentBs;

    return;
}
*/

void main(void)
{

    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This function derived from the one found in F2837x_SysCtrl.c file
    InitSysCtrl();

    InitGpio();

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
    //GPIO_SetupPinOptions(TEMP_GPIO, GPIO_OUTPUT, GPIO_ASYNC);
    //GPIO_SetupPinMux(TEMP_GPIO, 0, TEMP_MUX);

    //GPIO_SetupPinOptions(BLUE_LED_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);
    //GPIO_SetupPinMux(BLUE_LED_GPIO, GPIO_MUX_CPU1, BLUE_LED_MUX);

    // ****************************************************************************
    // ****************************************************************************
    //TODO ADC Configuration
    // ****************************************************************************
    // ****************************************************************************
        //Configure the ADC and power it up
        ConfigureADC();

        //Select the channels to convert and end of conversion flag

        EALLOW;

        // Analog signals - Motor 1
        // Vdc  ADC (B)14
        // Va   ADC B1
        // Vb   ADC B4
        // Vc   ADC B2
        // Ia   ADC A0
        // Ib   ADC B0
        // Ic   ADC A1

        // On piccolo 133ns for ACQPS
        // hencce ACQPS on soprano is 133/5~30

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

        for(;;);

}//END MAIN CODE
