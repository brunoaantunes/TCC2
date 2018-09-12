#include "F28x_Project.h"
//#include "F28377S_IO_assignment.h"
#include "main_settings.h"

//
//TODO ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B

void ConfigureADC(void)
{
    EALLOW;
    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

//
//TODO SetupADCContinuous - setup the ADC to continuously convert on one channel
//

void SetupADCContinuous()
{
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL  = 0;  //SOC will convert on channel
    AdcbRegs.ADCSOC0CTL.bit.CHSEL  = 0;

    AdcaRegs.ADCSOC4CTL.bit.CHSEL  = 1;  //SOC will convert on channel
    AdcbRegs.ADCSOC2CTL.bit.CHSEL  = 1;

    AdcaRegs.ADCSOC6CTL.bit.CHSEL  = 2;  //SOC will convert on channel
    AdcbRegs.ADCSOC6CTL.bit.CHSEL  = 2;

    AdcaRegs.ADCSOC0CTL.bit.ACQPS  = 14;    //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.ACQPS  = 14;

    AdcaRegs.ADCSOC4CTL.bit.ACQPS  = 14;    //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.ACQPS  = 14;

    AdcaRegs.ADCSOC6CTL.bit.ACQPS  = 14;    //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC6CTL.bit.ACQPS  = 14;


    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 0; //disable INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 0;

    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
    AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 1;

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  //end of SOC6 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0;

    EDIS;
}

//TODO ePWM configuration Functions

//
// InitEPwm1Example - Initialize EPWM1 configuration
//

void InitEPwm1Example()
{
    EPwm6Regs.TBPRD = /*6250/2;*/ 6000;                       // Set timer period
    EPwm6Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm6Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4; //TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV4;    //TB_DIV4;
    //EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    //EPwm6Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;

    //EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    //EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    //EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    //EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Setup compare
    //
    EPwm6Regs.CMPA.bit.CMPA = 0; //3000;

    //
    // Set actions
    //
    EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on Zero
    EPwm6Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm6Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM1A on Zero
    EPwm6Regs.AQCTLB.bit.CAD = AQ_SET;

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm6Regs.DBRED.bit.DBRED = 0; //EPWM6_MIN_DB;
    EPwm6Regs.DBFED.bit.DBFED = 0; //EPWM6_MIN_DB;
    //EPwm6_DB_Direction = DB_UP;

    //
    // Interrupt where we will change the Deadband
    //
    EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm6Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm6Regs.ETPS.bit.INTPRD = ET_1ST; //ET_3RD;          // Generate INT on 3rd event
}

//
// InitEPwm2Example - Initialize EPWM2 configuration
//
void InitEPwm2Example()
{
    EPwm7Regs.TBPRD = 6000;                       // Set timer period
    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm7Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV4;          // Slow just to observe on
                                                   // the scope
    //
    // Setup compare
    //
    EPwm7Regs.CMPA.bit.CMPA = 0; //3000;

    //
    // Set actions
    //
    EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on Zero
    EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm7Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM2A on Zero
    EPwm7Regs.AQCTLB.bit.CAD = AQ_SET;

    //
    // Active Low complementary PWMs - setup the deadband
    //
    EPwm7Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm7Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm7Regs.DBRED.bit.DBRED = 0; //EPWM7_MIN_DB;
    EPwm7Regs.DBFED.bit.DBFED = 0; //EPWM7_MIN_DB;
    //EPwm7_DB_Direction = DB_UP;

    //
    // Interrupt where we will modify the deadband
    //
    EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm7Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm7Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event
}

//
// InitEPwm3Example - Initialize EPWM3 configuration
//
void InitEPwm3Example()
{
    EPwm8Regs.TBPRD = 6000;                        // Set timer period
    EPwm8Regs.TBPHS.bit.TBPHS = 0x0000;            // Phase is 0
    EPwm8Regs.TBCTR = 0x0000;                      // Clear counter

    //
    // Setup TBCLK
    //
    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV4;          // Slow so we can observe on
                                                   // the scope
    //
    // Setup compare
    //
    EPwm8Regs.CMPA.bit.CMPA = 0; //3000;

    //
    // Set actions
    //
    EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM3A on Zero
    EPwm8Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm8Regs.AQCTLB.bit.CAU = AQ_CLEAR;           // Set PWM3A on Zero
    EPwm8Regs.AQCTLB.bit.CAD = AQ_SET;

    //
    // Active high complementary PWMs - Setup the deadband
    //
    EPwm8Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm8Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm8Regs.DBRED.bit.DBRED = 0; //EPWM8_MIN_DB;
    EPwm8Regs.DBFED.bit.DBFED = 0; //EPWM8_MIN_DB;
    //EPwm8_DB_Direction = DB_UP;

    //
    // Interrupt where we will change the deadband
    //
    EPwm8Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
    EPwm8Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
    EPwm8Regs.ETPS.bit.INTPRD = ET_1ST;            // Generate INT on 3rd event

}

// END OF FILE



