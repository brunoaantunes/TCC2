//###########################################################################
//
// FILE:    epwm_deadband_c28.c
//
// TITLE:   Teste de carregamento do capacitor de bootstrap
//
//! \addtogroup cpu01_example_list
//! <h1> EPWM dead band control (epwm_deadband)</h1>
//!
//! During the test, monitor ePWM1, ePWM2, and/or ePWM3 outputs
//! on a scope.
//!
//! - ePWM6A is on GPIO10
//! - ePWM6B is on GPIO11
//! - ePWM7A is on GPIO12
//! - ePWM7B is on GPIO13
//! - ePWM8A is on GPIO14
//! - ePWM8B is on GPIO15
//!
//! This example configures ePWM1, ePWM2 and ePWM3 for:
//! - Count up/down
//! - Deadband
//!
//! 3 Examples are included:
//! - ePWM6: Active low PWMs
//! - ePWM7: Active low complementary PWMs
//! - ePWM8: Active high complementary PWMs
//!
//! Each ePWM is configured to interrupt on the 3rd zero event.
//! When this happens the deadband is modified such that
//! 0 <= DB <= DB_MAX.  That is, the deadband will move up and
//! down between 0 and the maximum value.
//!
//! View the EPWM6A/B, EPWM7A/B and EPWM8A/B waveforms
//! via an oscilloscope
//
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
#include "F28x_Project.h"
#include <stdint.h>
#include "defines.h"
#include "structs.h"
#include "math.h"
#include "epwm.h"
//RET_VARS ret1 = RET_DEFAULTS;
//
// Defines
/*
#define EPWM6_MAX_DB   0x03FF
#define EPWM7_MAX_DB   0x03FF
#define EPWM8_MAX_DB   0x03FF
#define EPWM6_MIN_DB   0
#define EPWM7_MIN_DB   0
#define EPWM8_MIN_DB   0
#define DB_UP          1
#define DB_DOWN        0

//
// Globals
//
Uint32 EPwm1TimerIntCount;
Uint32 EPwm2TimerIntCount;
Uint32 EPwm6TimerIntCount;
Uint32 EPwm7TimerIntCount;
Uint32 EPwm8TimerIntCount;
Uint16 EPwm1_DB_Direction;
Uint16 EPwm2_DB_Direction;
Uint16 EPwm6_DB_Direction;
Uint16 EPwm7_DB_Direction;
Uint16 EPwm8_DB_Direction;
*/
//
// Function Prototypes
//
void InitEPwm2Example(void);
void InitEPwm1Example(void);
void InitEPwm6Example(void);
void InitEPwm7Example(void);
void InitEPwm8Example(void);
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm6_isr(void);
__interrupt void epwm7_isr(void);
__interrupt void epwm8_isr(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);


extern void ConfigureADC(void);
extern void SetupADCSoftware(void);
void contador(void);
//void BootstrapPwm6(void);
//void BootstrapPwm7(void);
//void BootstrapPwm8(void);

void initvars(void);
extern void stage0(void); // Verificação inicial
extern void stage1(void);
extern void stage2(void);
extern void stage3(void);

void sincosf(float x, float *sinx, float *cosx);
void MODULATION(struct RETIFICADOR *conv);

struct RETIFICADOR conv;
struct CONTROLE ctrl;
struct TRIGONOMETRICO trig;
struct PLL pll;
struct PI Cvz,CizID,CizIQ;

Uint16 count = 0;

//void InitEPwmGpio_DB(void);
//
// Main
//
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
// illustrates how to set the GPIO to its default state.
//
    //InitGpio();
    //InitEPwmGpio_DB()
//
// enable PWM1, PWM2 and PWM3
//
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;
    CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    CpuSysRegs.PCLKCR2.bit.EPWM6=1;
    CpuSysRegs.PCLKCR2.bit.EPWM7=1;
    CpuSysRegs.PCLKCR2.bit.EPWM8=1;

//
// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the F2837xS_EPwm.c file
//
    InitEPwm1Gpio();
    InitEPwm2Gpio();
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

    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &epwm1_isr;
    PieVectTable.EPWM2_INT = &epwm2_isr;
    PieVectTable.EPWM6_INT = &epwm6_isr;
    PieVectTable.EPWM7_INT = &epwm7_isr;
    PieVectTable.EPWM8_INT = &epwm8_isr;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;

    EDIS;   // This is needed to disable write to EALLOW protected registers
 //-------------------------------
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 50);
    ConfigCpuTimer(&CpuTimer1, 200, 50000);
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;

 //
 // Step 5. User specific code, enable __interrupts:
 // Configure GPIO34 as a GPIO output pin
 //
    EALLOW;
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1; // LED D9
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1; // LED D10
    // Set LED data
    GpioDataRegs.GPADAT.bit.GPIO12 = 1; // LED D9
    GpioDataRegs.GPADAT.bit.GPIO13 = 1; // LED D10
    EDIS;


  //-------------------------------
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;
    EDIS;

    InitEPwm1Example();
    InitEPwm2Example();
    InitEPwm6Example();
    InitEPwm7Example();
    InitEPwm8Example();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1;
    EDIS;

        IER |= M_INT1;
        IER |= M_INT6;
        IER |= M_INT7;
        IER |= M_INT13;

        PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
        PieCtrlRegs.PIEIER2.bit.INTx1 = 1;
        PieCtrlRegs.PIEIER2.bit.INTx2 = 1;
        PieCtrlRegs.PIEIER2.bit.INTx6 = 1;
        PieCtrlRegs.PIEIER2.bit.INTx7 = 1;
        PieCtrlRegs.PIEIER2.bit.INTx8 = 1;

        EINT;
        ERTM;


        //Configure the ADCs and power them up
         ConfigureADC();
        //Setup the ADCs for software conversions
         SetupADCSoftware();

         initvars(); // Inicializa variáveis

        for(;;);


} // END MAIN

__interrupt void cpu_timer0_isr(void)
{

   CpuTimer0.InterruptCount++;
   GpioDataRegs.GPATOGGLE.bit.GPIO12 = 1; // LED D9
   GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1; // LED D10

   count++;

   if(count<20){
       EPwm6Regs.CMPA.bit.CMPA = epwm_duty*2;
       EPwm7Regs.CMPA.bit.CMPA = epwm_duty*2;
       EPwm8Regs.CMPA.bit.CMPA = epwm_duty*2;
   }


   if(count>20){

       // Maquina de estados
       switch(ctrl.STATE){
       case 0: // Estagio 0 - Verificação e teste iniciais
               stage0();
       break;
       case 1: // Estagio 1 - Retificador
               stage1();
       break;
       case 2: // Estagio 2 - Inversor - testes inicias
               stage2();
       break;
       case 3: // Estagio 3 - Erro, Proteção ou desligamento
       default:
               stage3();
               ctrl.ENABLE = 0;
       break;
       }


       MODULATION(&conv);

       if((conv.da+conv.d0)<0)conv.da=0;
       if((conv.db+conv.d0)<0)conv.db=0;
       if((conv.dc+conv.d0)<0)conv.dc=0;

       //-------------------------------------------------------//
       // Atualização das razões cíclicas - Conversor
       //-------------------------------------------------------//

       EPwm6Regs.CMPA.bit.CMPA=(unsigned int) ((conv.da+conv.d0)*5000);
       EPwm7Regs.CMPA.bit.CMPA=(unsigned int) ((conv.db+conv.d0)*5000);
       EPwm8Regs.CMPA.bit.CMPA=(unsigned int) ((conv.dc+conv.d0)*5000);
       }
//
// Acknowledge this __interrupt to receive more __interrupts from group 1
//
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void cpu_timer1_isr(void)
{

    CpuTimer1.InterruptCount++;
   //convert, wait for completion, and store results
   //start conversions immediately via software, ADCA
   //
   AdcaRegs.ADCSOCFRC1.all = 0x0007; //SOC0, SOC1 and SOC2

   //
   //start conversions immediately via software, ADCB
   //
   AdcbRegs.ADCSOCFRC1.all = 0x0007; //SOC0, SOC1 and SOC2

   //
   //wait for ADCA to complete, then acknowledge flag
   //
   while(AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0);
   AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

   //
   //wait for ADCB to complete, then acknowledge flag
   //
   while(AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0);
   AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

       //  ADCA0 -> ia
       //  ADCB0 -> ic
       //  ADCA1 -> vab
       //  ADCB1 -> vcb
       //  ADCA2 -> vccP
       //  ADCB2 -> vccN

      //store results
       conv.ADCA0 = AdcaResultRegs.ADCRESULT0;
       conv.ADCA1 = AdcaResultRegs.ADCRESULT1;
       conv.ADCA2 = AdcaResultRegs.ADCRESULT2;

       conv.ADCB0 = AdcbResultRegs.ADCRESULT0;
       conv.ADCB1 = AdcbResultRegs.ADCRESULT1;
       conv.ADCB2 = AdcbResultRegs.ADCRESULT2;

       // Maquina de estados
       switch(ctrl.STATE){
       case 0: // Estagio 0 - Verificação e teste iniciais
               stage0();
       break;
       case 1: // Estagio 1 - Retificador
               stage1();
       break;
       case 2: // Estagio 2 - Inversor - testes inicias
               stage2();
       break;
       case 3: // Estagio 3 - Erro, Proteção ou desligamento
       default:
               stage3();
               ctrl.ENABLE = 0;
       break;
       }
}

// epwm1_isr - EPWM1 ISR
__interrupt void epwm1_isr(void)
{

    // Clear INT flag for this timer
    EPwm1Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// epwm2_isr - EPWM2 ISR
__interrupt void epwm2_isr(void)
{
    // Clear INT flag for this timer
    EPwm2Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// epwm6_isr - EPWM6 ISR
__interrupt void epwm6_isr(void)
{


    // Clear INT flag for this timer
    EPwm6Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// epwm7_isr - EPWM7 ISR
__interrupt void epwm7_isr(void)
{

    // Clear INT flag for this timer
    EPwm7Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// epwm8_isr - EPWM8 ISR
__interrupt void epwm8_isr(void)
{

    // Clear INT flag for this timer
    EPwm8Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// InitEPwm1Example - Initialize EPWM1 configuration
//TODO pwm examples

void InitEPwm1Example()
{
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;        // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 1;        // Select SOC from from CPMA on upcount
    EPwm1Regs.ETPS.bit.SOCAPRD  = 1;        // Generate pulse on 1st event

    EPwm1Regs.TBPRD = epwm_timer;//81                 // Set timer period
    EPwm1Regs.TBPHS.bit.TBPHS = 0;           // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading (Master)
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; //TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;    //TB_DIV4;

    //adicionado agora
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Setup compare
    EPwm1Regs.CMPA.bit.CMPA = epwm_duty;

    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM1A on Zero
    EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBRED.bit.DBRED = 0; //EPWM6_MIN_DB;
    EPwm1Regs.DBFED.bit.DBFED = 0; //EPWM6_MIN_DB;
    //EPwm6_DB_Direction = DB_UP;

    //
    // Interrupt where we will change the Deadband
    //
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST; //ET_3RD;          // Generate INT on 3rd event
}

void InitEPwm2Example()
{
    EPwm2Regs.TBPRD = epwm_timer;                // Set timer period
    EPwm2Regs.TBPHS.bit.TBPHS = 0;           // Phase
    EPwm2Regs.TBCTL.bit.PHSDIR = 0;               // Direction
    EPwm2Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading (Slave)
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; //TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;    //TB_DIV4;

    //adicionado agora
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Setup compare
    EPwm2Regs.CMPA.bit.CMPA = epwm_duty;

    //
    // Set actions
    //
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on Zero
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM1A on Zero
    EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED.bit.DBRED = 0; //EPWM6_MIN_DB;
    EPwm2Regs.DBFED.bit.DBFED = 0; //EPWM6_MIN_DB;
    //EPwm6_DB_Direction = DB_UP;

    //
    // Interrupt where we will change the Deadband
    //
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST; //ET_3RD;          // Generate INT on 3rd event
}

void InitEPwm6Example()
{


    EPwm6Regs.TBPRD = epwm_timer;                 // Set timer period
    EPwm6Regs.TBPHS.bit.TBPHS = 0;           // Phase
    EPwm6Regs.TBCTL.bit.PHSDIR = 1;
    EPwm6Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading (Slave)
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; //TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;    //TB_DIV4;

    //adicionado agora
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //EPwm6Regs.CMPA.bit.CMPA =2500 ;

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
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm6Regs.DBRED.bit.DBRED = 200; //EPWM6_MIN_DB;
    EPwm6Regs.DBFED.bit.DBFED = 200; //EPWM6_MIN_DB;
    //EPwm6_DB_Direction = DB_UP;

    //
    // Interrupt where we will change the Deadband
    //
    EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm6Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm6Regs.ETPS.bit.INTPRD = ET_1ST; //ET_3RD;          // Generate INT on 3rd event
}

void InitEPwm7Example()
{
    EPwm7Regs.TBPRD = epwm_timer;                 // Set timer period
    EPwm7Regs.TBPHS.bit.TBPHS = 0;//10000/3
    EPwm7Regs.TBCTL.bit.PHSDIR = 0;
    EPwm7Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm7Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading (Slave)
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; //TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;    //TB_DIV4;

    //adicionado agora
    EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;//TB_CTR_ZERO;
    EPwm7Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //EPwm7Regs.CMPA.bit.CMPA =2500 ;

    //
    // Set actions
    //
    EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on Zero
    EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm7Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM1A on Zero
    EPwm7Regs.AQCTLB.bit.CAD = AQ_SET;

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm7Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm7Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm7Regs.DBRED.bit.DBRED = 200; //EPWM6_MIN_DB;
    EPwm7Regs.DBFED.bit.DBFED = 200; //EPWM6_MIN_DB;
    //EPwm6_DB_Direction = DB_UP;

    //
    // Interrupt where we will change the Deadband
    //
    EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm7Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm7Regs.ETPS.bit.INTPRD = ET_1ST; //ET_3RD;          // Generate INT on 3rd event
}

void InitEPwm8Example()
{
    EPwm8Regs.TBPRD = epwm_timer;    // Set timer period
    EPwm8Regs.TBPHS.bit.TBPHS = 0;//(2*10000)/3
    EPwm8Regs.TBCTL.bit.PHSDIR = 0;
    EPwm8Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm8Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading (Slave)
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; //TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;    //TB_DIV4;

    //adicionado agora
    EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm8Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //EPwm8Regs.CMPA.bit.CMPA = 2500;

    //
    // Set actions
    //
    EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on Zero
    EPwm8Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm8Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM1A on Zero
    EPwm8Regs.AQCTLB.bit.CAD = AQ_SET;

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm8Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm8Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm8Regs.DBRED.bit.DBRED = 200; //EPWM6_MIN_DB;
    EPwm8Regs.DBFED.bit.DBFED = 200; //EPWM6_MIN_DB;
    //EPwm6_DB_Direction = DB_UP;

    //
    // Interrupt where we will change the Deadband
    //
    EPwm8Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm8Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm8Regs.ETPS.bit.INTPRD = ET_1ST; //ET_3RD;          // Generate INT on 3rd event
}



void MODULATION(struct RETIFICADOR *conv)
{
    float min, max;

     if((conv->da > conv->db) && (conv->da > conv->dc) ) max=conv->da;
else if((conv->db > conv->da) && (conv->db > conv->dc) ) max=conv->db;
else  max=conv->dc;

     if((conv->da < conv->db) && (conv->da < conv->dc) ) min=conv->da;
else if((conv->db < conv->da) && (conv->db < conv->dc) ) min=conv->db;
else  min=conv->dc;

    conv->d0=(-0.5*(max+min-1.0));

    return;
}

void initvars(void){
   Cvz.e0 = 0;  // Erro atual
   Cvz.u0= 0; // Calcula saída atual
   Cvz.u1= 0; // Atualiza saída anterior
   Cvz.e1= 0; // Atualiza erro anterior

   CizID.e0 = 0;  // Erro atual
   CizID.u0=0; // Calcula saída atual
   CizID.u1=0; // Atualiza saída anterior
   CizID.e1=0; // Atualiza erro anterior

   CizIQ.e0 =0;  // Erro atual
   CizIQ.u0= 0; // Calcula saída atual
   CizIQ.u1=0; // Atualiza saída anterior
   CizIQ.e1=0; // Atualiza erro anterior

   ctrl.STATE =0;
   ctrl.ENABLE = 0;
    return;
}


// Retorna seno e cosseno de x
void sincosf(float x, float *sinx, float *cosx)
{

*sinx=sin(x);
*cosx=cos(x);
  return;
}
/*
void InitEPwmGpio_DB(void)
{
    EALLOW;


    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;    // Disable pull-up on GPIO10 (EPWM6A)
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;    // Disable pull-up on GPIO10 (EPWM7A)
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Disable pull-up on GPIO10 (EPWM8A)

    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;   // Configure GPIO12 as EPWM7A
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;   // Configure GPIO14 as EPWM8A

    EDIS;
}
*/


//
// End of file
//










