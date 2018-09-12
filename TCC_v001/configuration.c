//----------------------------------------------------------------------------------
//  FILE:           configuration.c
//
//  Description:    Contains all peripheral init / config functions
//
//  Version:        1.0
//
//  Target:         TMS320F28377S
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments © 2004-2015
//----------------------------------------------------------------------------------
//  Revision History: Lucas Plentz
//----------------------------------------------------------------------------------
//  Date      | Description / Status
//----------------------------------------------------------------------------------
// 6 May 2018 - Peripheral config modules
//----------------------------------------------------------------------------------


/*-------------------------------------------------------------------------------
Include project specific include files.
-------------------------------------------------------------------------------*/
#include "F28x_Project.h"
#include "F28377S_IO_assignment.h"
#include "Boost-Settings.h"


// ****************************************************************************
// ****************************************************************************
//TODO EPWM GPIO Configuration
// ****************************************************************************
// ****************************************************************************
void InitRet1EPwmGpio(void)
{
    // set up EPWM-A
    GPIO_SetupPinOptions(RET1_EPWM_A_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(RET1_EPWM_A_GPIO, 0, RET1_EPWM_A_MUX);

    // set up EPWM-B
    GPIO_SetupPinOptions(RET1_EPWM_B_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(RET1_EPWM_B_GPIO, 0, RET1_EPWM_B_MUX);

    // set up EPWM-C
    GPIO_SetupPinOptions(RET1_EPWM_C_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(RET1_EPWM_C_GPIO, 0, RET1_EPWM_C_MUX);

    return;
}

// ****************************************************************************
// ****************************************************************************
//TODO CPU SYSTEM CONTROL Configuration
// ****************************************************************************
// ****************************************************************************
#pragma CODE_SECTION(InitFlash_Bank0, "ramfuncs");
#pragma CODE_SECTION(InitFlash_Bank1, "ramfuncs");

void InitSysCtrl1(void)
{
    // Disable the watchdog
    DisableDog();

#ifdef _FLASH
// Copy time critical code and Flash setup code to RAM
// This includes the following functions:  InitFlash();
// The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
// symbols are created by the linker. Refer to the device .cmd file.
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
    InitFlash_Bank0();
#endif

    // *IMPORTANT*
    // The Device_cal function, which copies the ADC & oscillator calibration values
    // from TI reserved OTP into the appropriate trim registers, occurs automatically
    // in the Boot ROM. If the boot ROM code is bypassed during the debug process, the
    // following function MUST be called for the ADC and oscillators to function according
    // to specification. The clocks to the ADC MUST be enabled before calling this
    // function.
    // See the device data manual and/or the ADC Reference
    // Manual for more information.

    EALLOW;

    //enable pull-ups on unbonded IOs as soon as possible to reduce power consumption.
    GPIO_EnableUnbondedIOPullups();

    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_D = 1;

    //check if device is trimmed
    if(*((Uint16 *)0x5D1B6) == 0x0000){
        //device is not trimmed, apply static calibration values
        AnalogSubsysRegs.ANAREFTRIMA.all = 31709;
        AnalogSubsysRegs.ANAREFTRIMB.all = 31709;
        AnalogSubsysRegs.ANAREFTRIMC.all = 31709;
        AnalogSubsysRegs.ANAREFTRIMD.all = 31709;
    }

    CpuSysRegs.PCLKCR13.bit.ADC_A = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_C = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_D = 0;
    EDIS;

    // Initialize the PLL control: PLLCR and CLKINDIV
    // F28_PLLCR and F28_CLKINDIV are defined in F2837xS_Examples.h
    // Note: The internal oscillator CANNOT be used as the PLL source if the
    // PLLSYSCLK is configured to frequencies above 194 MHz.
    InitSysPll(XTAL_OSC,IMULT_40,FMULT_0,PLLCLK_BY_2);      //PLLSYSCLK = 10Mhz(OSCCLK) * 40 (IMULT) * 1 (FMULT) /  2 (PLLCLK_BY_2)

    //Turn on all peripherals
    InitPeripheralClocks();

}

// ****************************************************************************
// ****************************************************************************
//TODO ADC Configuration
// ****************************************************************************
// ****************************************************************************
//Write ADC configurations and power up the ADC for both ADC A and ADC B
void ConfigureADC(void)
{
    EALLOW;

    //write configurations
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //power up the ADC
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);

    EDIS;
}

// ****************************************************************************
// ****************************************************************************
//TODO PWM Configuration
// ****************************************************************************
// ****************************************************************************
void PWM_1ch_UpDwnCnt_CNF(volatile struct EPWM_REGS * ePWM, Uint16 period, int16 db)
{

    EALLOW;
    // Time Base SubModule Registers
    ePWM->TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load
    ePWM->TBPRD = period / 2; // PWM frequency = 1 / period
    ePWM->TBPHS.bit.TBPHS = 0;
    ePWM->TBCTR = 0;
    ePWM->TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;
    ePWM->TBCTL.bit.HSPCLKDIV = TB_DIV1;
    ePWM->TBCTL.bit.CLKDIV    = TB_DIV1;

    ePWM->TBCTL.bit.PHSEN    = TB_DISABLE;
    ePWM->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"

    // Counter Compare Submodule Registers
    ePWM->CMPA.bit.CMPA = 0; // set duty 0% initially
    ePWM->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    ePWM->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

    // Action Qualifier SubModule Registers
    ePWM->AQCTLA.bit.CAU = AQ_CLEAR;
    ePWM->AQCTLA.bit.CAD = AQ_SET;

    // Active high complementary PWMs - Set up the deadband
    ePWM->DBCTL.bit.IN_MODE  = DBA_ALL;
    ePWM->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    ePWM->DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    ePWM->DBRED.all = db;
    ePWM->DBFED.all = db;
    EDIS;
}

// ****************************************************************************
// ****************************************************************************
//TODO PWM Configuration
// ****************************************************************************
// ****************************************************************************
void PWM_1ch_UpDwnCnt_CNF_noDB(volatile struct EPWM_REGS * ePWM, Uint16 period)
{
    EALLOW;
    // Time Base SubModule Registers
    ePWM->TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load
    ePWM->TBPRD = period / 2; // PWM frequency = 1 / period
    ePWM->TBPHS.bit.TBPHS = 0;
    ePWM->TBCTR = 0;
    ePWM->TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;
    ePWM->TBCTL.bit.HSPCLKDIV = TB_DIV1;
    ePWM->TBCTL.bit.CLKDIV    = TB_DIV1;

    ePWM->TBCTL.bit.PHSEN    = TB_DISABLE;
    ePWM->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"

    // Counter Compare Submodule Registers
    ePWM->CMPA.bit.CMPA = 0; // set duty 0% initially
    ePWM->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    ePWM->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

    // Action Qualifier SubModule Registers
    ePWM->AQCTLA.bit.CAU = AQ_CLEAR;
    ePWM->AQCTLA.bit.CAD = AQ_SET;

    // Active high complementary PWMs - Disable deadband
    ePWM->DBCTL.bit.OUT_MODE = DB_DISABLE;
    EDIS;
}
/*****************************************************************************/
// EOF
/*****************************************************************************/
