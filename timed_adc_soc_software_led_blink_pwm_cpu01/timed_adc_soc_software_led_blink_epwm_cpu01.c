
//###########################################################################
// $TI Release: F2837xS Support Library v3.04.00.00 $
// $Release Date: Sun Mar 25 13:27:27 CDT 2018 $
// $Copyright:
// Copyright (C) 2014-2018 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include <stdint.h>
#include "F28x_Project.h"
#include "ePWM.h"
#include "ADC.h"
#include "Structs.h"
#include "Defines.h"
#include "math.h"

//
// Function Prototypes
//
__interrupt void cpu_timer0_isr(void);
void ConfigureADC(void);
void SetupADCSoftware(void);

void InitEPwmGpio(void);
void InitEPwm(void);
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
void update_compare(EPWM_INFO*);

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

//
// Globals
//

EPWM_INFO epwm1_info;
EPWM_INFO epwm2_info;
EPWM_INFO epwm3_info;


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
// InitGpio();  // Skipped for this example

//
// enable PWM1, PWM2 and PWM3
//
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;
    CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    CpuSysRegs.PCLKCR2.bit.EPWM3=1;
   //
   // For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
   // These functions are in the F2837xS_EPwm.c file
   //
    InitEPwmGpio();
//
// Step 3. Clear all __interrupts and initialize PIE vector table:
//
   DINT;

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE __interrupts disabled and flags
// are cleared.
// This function is found in the F2837xS_PieCtrl.c file.
//
   InitPieCtrl();

//
// Disable CPU __interrupts and clear all CPU __interrupt flags:
//
   IER = 0x0000;
   IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the __interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xS_DefaultIsr.c.
// This function is found in F2837xS_PieVect.c.
//
   InitPieVectTable();

//
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
//
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.TIMER0_INT = &cpu_timer0_isr;
//   PieVectTable.EPWM1_INT = &epwm1_isr;
//   PieVectTable.EPWM2_INT = &epwm2_isr;
 //  PieVectTable.EPWM3_INT = &epwm3_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

//
// Step 4. Initialize the Device Peripheral. This function can be
//         found in F2837xS_CpuTimers.c
//
   InitCpuTimers();   // For this example, only initialize the Cpu Timers

//
// Configure CPU-Timer 0 to __interrupt every 500 milliseconds:
// 60MHz CPU Freq, 50 millisecond Period (in uSeconds)
//
   ConfigCpuTimer(&CpuTimer0, 60, 500000);

//
// To ensure precise timing, use write-only instructions to write to the entire
// register. Therefore, if any of the configuration bits are changed in
// ConfigCpuTimer and InitCpuTimers (in F2837xS_cputimervars.h), the below
// settings must also be updated.
//
   CpuTimer0Regs.TCR.all = 0x4001;

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

   //
   // For this example, only initialize the ePWM
   //
   EALLOW;
   CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   EDIS;
   InitEPwm();
 //  InitEPwm1Example();
 //  InitEPwm2Example();
 //  InitEPwm3Example();

   EALLOW;
   CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;

//
// Enable CPU INT1 which is connected to CPU-Timer 0:
//
   IER |= M_INT1;

//
// Enable CPU INT3 which is connected to EPWM1-3 INT:
//
   IER |= M_INT3;

//
// Enable TINT0 in the PIE: Group 1 __interrupt 7
//
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
//
// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
//
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;

//
// Enable global Interrupts and higher priority real-time debug events:
//
   EINT;   // Enable Global __interrupt INTM
   ERTM;   // Enable Global realtime __interrupt DBGM


   //Configure the ADCs and power them up
    ConfigureADC();
   //Setup the ADCs for software conversions
    SetupADCSoftware();

    initvars(); // Inicializa variáveis

//
// Step 6. IDLE loop. Just sit and loop forever (optional):
//
    for(;;){
           asm ("          NOP");
       }
}



//
// cpu_timer0_isr - CPU Timer0 ISR that toggles GPIO32 once per 500ms
//
__interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;
   GpioDataRegs.GPATOGGLE.bit.GPIO12 = 1; // LED D9
   GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1; // LED D10


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
    conv.ADCA2 = AdcaResultRegs.ADCRESULT2; // Precisa ser configurado

    conv.ADCB0 = AdcbResultRegs.ADCRESULT0;
    conv.ADCB1 = AdcbResultRegs.ADCRESULT1;
    conv.ADCB2 = AdcbResultRegs.ADCRESULT2; // Precisa ser configurado


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

    EPwm6Regs.CMPA.bit.CMPA=(unsigned int) ((conv.da+conv.d0)*TIMER_PWM);
    EPwm4Regs.CMPA.bit.CMPA=(unsigned int) ((conv.db+conv.d0)*TIMER_PWM);
    EPwm2Regs.CMPA.bit.CMPA=(unsigned int) ((conv.dc+conv.d0)*TIMER_PWM);


   //
   // Acknowledge this __interrupt to receive more __interrupts from group 1
   //
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


// Inicialização de variáveis
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


//
// End of file
//
