#include "F28x_Project.h"
#include "F28377S_IO_assignment.h"
#include "ret-Settings.h"


// ****************************************************************************
// ****************************************************************************
//TODO EPWM GPIO Configuration
// ****************************************************************************
// ****************************************************************************
void Initret1EPwmGpio(void)
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



