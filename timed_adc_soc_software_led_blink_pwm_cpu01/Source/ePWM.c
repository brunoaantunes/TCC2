/*
 * ePWM.c
 */


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

extern EPWM_INFO epwm1_info;
extern EPWM_INFO epwm2_info;
extern EPWM_INFO epwm3_info;



void InitEPwm(void)
{

//EALLOW;
//SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=0;    // for carriers sinc
//EDIS;

/*******************************************************************************/
/* INVVERSOR Delta : ePWM1,3,5                                                 */
/*******************************************************************************/


//-------------------------------------------------------------------------------
// EPWM Module 1
//-------------------------------------------------------------------------------

EPwm1Regs.TBCTL.bit.HSPCLKDIV=0;                    // Frequencia do clock/1
EPwm1Regs.TBCTL.bit.CLKDIV=0;                       // pré escala 1/1

EPwm1Regs.TBPRD = TIMER_PWM;                        // Period
EPwm1Regs.TBPHS.bit.TBPHS = PHASE_PWM1;            // Set Phase register to zero

EPwm1Regs.TBCTL.bit.CTRMODE = 2;                    // Symmetrical mode (up down)
EPwm1Regs.TBCTL.bit.PHSEN = 0;                      // Master module
EPwm1Regs.TBCTL.bit.PRDLD = 0;                      // Period timer is load at zero
EPwm1Regs.TBCTL.bit.SYNCOSEL = 1;                   // Sync EPWMxsync when timer is 1
EPwm1Regs.TBCTL.bit.SWFSYNC =1;                     // Force a Sync pulse
EPwm1Regs.TBCTL.bit.FREE_SOFT=2;                    // runs free

EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;                 // Shadow mode atualização não imediata
EPwm1Regs.CMPCTL.bit.SHDWBMODE = 0;                 // Shadow mode atualização não imediata
EPwm1Regs.CMPCTL.bit.LOADAMODE = 2;                 // load on zero or per. (2), zero(0), per.(1)
EPwm1Regs.CMPCTL.bit.LOADBMODE = 2;                 // load on zero or per. (2), zero(0), per.(1)

EPwm1Regs.AQCTLA.bit.CAU = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm1Regs.AQCTLA.bit.CAD = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm1Regs.AQCTLA.bit.CBU = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm1Regs.AQCTLA.bit.CBD = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm1Regs.AQCTLA.bit.PRD = 1;                       // 0 Não faz nada no modo opdown
EPwm1Regs.AQCTLA.bit.ZRO = 1;                       // 0 Não faz nada no modo opdown

EPwm1Regs.AQCTLB.bit.CAU = 0;                       // actions for EPWM1B -> 1 clear, 2 set, 3 toggle
EPwm1Regs.AQCTLB.bit.CAD = 0;                       // actions for EPWM1B -> 1 clear, 2 set, 3 toggle
EPwm1Regs.AQCTLB.bit.CBU = 1;                       // actions for EPWM1B -> 1 clear, 2 set, 3 toggle
EPwm1Regs.AQCTLB.bit.CBD = 2;                       // actions for EPWM1B -> 1 clear, 2 set, 3 toggle
EPwm1Regs.AQCTLB.bit.PRD = 0;                       // 0 Não faz nada no modo opdown
EPwm1Regs.AQCTLB.bit.ZRO = 0;                       // 0 Não faz nada no modo opdown

EPwm1Regs.DBCTL.bit.OUT_MODE =0;                    // Bypass Dead-band module

EPwm1Regs.ETSEL.bit.SOCAEN=0;                       //  habilita geraçaõ de eventos de SOCA para AD (no caso para SEQ1)
EPwm1Regs.ETSEL.bit.SOCASEL=1;                      // SOCA gerado qdo timer =0  (periodo =2)
EPwm1Regs.ETPS.bit.SOCAPRD=1;                       // gera ção em todos os eventos


EALLOW;
EPwm1Regs.TZSEL.bit.OSHT1=1;                        // en
EPwm1Regs.TZCTL.bit.TZA  =2;                        // Force PWMxA to low  state (2)
EPwm1Regs.TZCTL.bit.TZB  =2;                        // Force PWMxB to low state (2)
EPwm1Regs.TZFRC.bit.OST  =1;                        // Forced trip in Rectifier=1
EDIS;


//-------------------------------------------------------------------------------
// EPWM Module 3
//-------------------------------------------------------------------------------

EPwm3Regs.TBCTL.bit.HSPCLKDIV=0;                    // Frequencia do clock/1
EPwm3Regs.TBCTL.bit.CLKDIV=0;                       // pré escala 1/1

EPwm3Regs.TBPRD = TIMER_PWM;                        // Period
EPwm3Regs.TBPHS.bit.TBPHS = PHASE_PWM3;            // Set Phase register
EPwm3Regs.TBCTL.bit.CTRMODE = 2;                    // Symmetrical mode (up down)
EPwm3Regs.TBCTL.bit.PHSEN = 1;                      // Slave module (EPwm1 is Master)
EPwm3Regs.TBCTL.bit.PHSDIR =1;                      // Count up after the Sync event
EPwm3Regs.TBCTL.bit.PRDLD = 0;                      // Period timer is load at zero
EPwm3Regs.TBCTL.bit.SYNCOSEL = 0;                   // Sync when timer is 0
EPwm3Regs.TBCTL.bit.SWFSYNC =0;                     // Do not force a Sync pulse
EPwm3Regs.TBCTL.bit.FREE_SOFT=2;                    // runs free

EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0;                 // Shadow mode atualização não imediata
EPwm3Regs.CMPCTL.bit.SHDWBMODE = 0;                 // Shadow mode atualização não imediata
EPwm3Regs.CMPCTL.bit.LOADAMODE = 2;                 // load on zero or per. (2), zero(0), per.(1)
EPwm3Regs.CMPCTL.bit.LOADBMODE = 2;                 // load on zero or per. (2), zero(0), per.(1)

EPwm3Regs.AQCTLA.bit.CAU = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm3Regs.AQCTLA.bit.CAD = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm3Regs.AQCTLA.bit.CBU = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm3Regs.AQCTLA.bit.CBD = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm3Regs.AQCTLA.bit.PRD = 1;                       // 0 Não faz nada no modo opdown
EPwm3Regs.AQCTLA.bit.ZRO = 1;                       // 0 Não faz nada no modo opdown


EPwm3Regs.AQCTLB.bit.CAU = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm3Regs.AQCTLB.bit.CAD = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm3Regs.AQCTLB.bit.CBU = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm3Regs.AQCTLB.bit.CBD = 2;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm3Regs.AQCTLB.bit.PRD = 0;                       // 0 Não faz nada no modo opdown
EPwm3Regs.AQCTLB.bit.ZRO = 0;                       // 0 Não faz nada no modo opdown

EPwm3Regs.DBCTL.bit.OUT_MODE =0;                    // Bypass Dead-band module


EPwm3Regs.ETSEL.bit.SOCBEN=0;                       // Habilita SOCB para SEQ2
EPwm3Regs.ETSEL.bit.SOCBSEL=2;                      // Gera no período
EPwm3Regs.ETPS.bit.SOCBPRD=1;                       // gera soc em todos os eventos


EALLOW;
EPwm3Regs.TZSEL.bit.OSHT1=1;                        //  disable
EPwm3Regs.TZCTL.bit.TZA  =2;                        // Force PWMxA to low  state (2)
EPwm3Regs.TZCTL.bit.TZB  =2;                        // Force PWMxB to low state (2)
EPwm3Regs.TZFRC.bit.OST  =1;                        // Forced trip in rectifier=1
EDIS;


//-------------------------------------------------------------------------------
// EPWM Module 5
//-------------------------------------------------------------------------------

EPwm5Regs.TBCTL.bit.HSPCLKDIV=0;                    // Frequencia do clock/1
EPwm5Regs.TBCTL.bit.CLKDIV=0;                       // pré escala 1/1

EPwm5Regs.TBPRD = TIMER_PWM;                        // Period
EPwm5Regs.TBPHS.bit.TBPHS = PHASE_PWM5;            // Set Phase register
EPwm5Regs.TBCTL.bit.CTRMODE = 2;                    // Symmetrical mode (up down)
EPwm5Regs.TBCTL.bit.PHSEN = 1;                      // Slave module (EPwm1 is Master)
EPwm5Regs.TBCTL.bit.PHSDIR =1;                      // Count up after the Sync event
EPwm5Regs.TBCTL.bit.PRDLD = 0;                      // Period timer is load at zero
EPwm5Regs.TBCTL.bit.SYNCOSEL = 0;                   // Sync when timer is 0
EPwm5Regs.TBCTL.bit.SWFSYNC =0;                     // Do not force a Sync pulse
EPwm5Regs.TBCTL.bit.FREE_SOFT=2;                    // runs free

EPwm5Regs.CMPCTL.bit.SHDWAMODE = 0;                 // Shadow mode atualização não imediata
EPwm5Regs.CMPCTL.bit.SHDWBMODE = 0;                 // Shadow mode atualização não imediata
EPwm5Regs.CMPCTL.bit.LOADAMODE = 2;                 // load on zero or per. (2), zero(0), per.(1)
EPwm5Regs.CMPCTL.bit.LOADBMODE = 2;                 // load on zero or per. (2), zero(0), per.(1)

EPwm5Regs.AQCTLA.bit.CAU = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm5Regs.AQCTLA.bit.CAD = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm5Regs.AQCTLA.bit.CBU = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm5Regs.AQCTLA.bit.CBD = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm5Regs.AQCTLA.bit.PRD = 1;                       // 0 Não faz nada no modo opdown
EPwm5Regs.AQCTLA.bit.ZRO = 1;                       // 0 Não faz nada no modo opdown


EPwm5Regs.AQCTLB.bit.CAU = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm5Regs.AQCTLB.bit.CAD = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm5Regs.AQCTLB.bit.CBU = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm5Regs.AQCTLB.bit.CBD = 2;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm5Regs.AQCTLB.bit.PRD = 0;                       // 0 Não faz nada no modo opdown
EPwm5Regs.AQCTLB.bit.ZRO = 0;                       // 0 Não faz nada no modo opdown

EPwm5Regs.DBCTL.bit.OUT_MODE =0;                    // Bypass Dead-band module


EPwm5Regs.ETSEL.bit.SOCBEN=0;                       // Habilita SOCB para SEQ2
EPwm5Regs.ETSEL.bit.SOCBSEL=2;                      // Gera no período
EPwm5Regs.ETPS.bit.SOCBPRD=1;                       // gera soc em todos os eventos


EALLOW;
EPwm5Regs.TZSEL.bit.OSHT1=1;                        //  disable
EPwm5Regs.TZCTL.bit.TZA  =2;                        // Force PWMxA to low  state (2)
EPwm5Regs.TZCTL.bit.TZB  =2;                        // Force PWMxB to low state (2)
EPwm5Regs.TZFRC.bit.OST  =1;                        // Forced trip in rectifier=1
EDIS;

/*******************************************************************************/
/* INVVERSOR VSI-2L : ePWM2,4,6                                                 */
/*******************************************************************************/

//-------------------------------------------------------------------------------
// EPWM Module 2
//-------------------------------------------------------------------------------

EPwm2Regs.TBCTL.bit.HSPCLKDIV=0;                    // Frequencia do clock/1
EPwm2Regs.TBCTL.bit.CLKDIV=0;                       // pré escala 1/1

EPwm2Regs.TBPRD = TIMER_PWM;                        // Period
EPwm2Regs.TBPHS.bit.TBPHS = PHASE_PWM2;            // Set Phase register

EPwm2Regs.TBCTL.bit.CTRMODE = 2;                    // Symmetrical mode (up down)
EPwm2Regs.TBCTL.bit.PHSEN = 1;                      // Slave module (EPwm1 is Master)
EPwm2Regs.TBCTL.bit.PHSDIR =0;                      // Count down after the Sync event
EPwm2Regs.TBCTL.bit.PRDLD = 0;                      // Period timer is load at zero
EPwm2Regs.TBCTL.bit.SYNCOSEL = 0;                   // Sync when timer is 0
EPwm2Regs.TBCTL.bit.SWFSYNC =0;                     // Do not force a Sync pulse
EPwm2Regs.TBCTL.bit.FREE_SOFT=2;                    // runs free

EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;                 // Shadow mode atualização não imediata
EPwm2Regs.CMPCTL.bit.SHDWBMODE = 0;                 // Shadow mode atualização não imediata
EPwm2Regs.CMPCTL.bit.LOADAMODE = 2;                 // load on zero or per. (2), zero(0), per.(1)
EPwm2Regs.CMPCTL.bit.LOADBMODE = 2;                 // load on zero or per. (2), zero(0), per.(1)

EPwm2Regs.AQCTLA.bit.CAU = 2;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm2Regs.AQCTLA.bit.CAD = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm2Regs.AQCTLA.bit.CBU = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm2Regs.AQCTLA.bit.CBD = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm2Regs.AQCTLA.bit.PRD = 0;                       // 0 Não faz nada no modo opdown
EPwm2Regs.AQCTLA.bit.ZRO = 0;                       // 0 Não faz nada no modo opdown


EPwm2Regs.AQCTLB.bit.CAU = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm2Regs.AQCTLB.bit.CAD = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm2Regs.AQCTLB.bit.CBU = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm2Regs.AQCTLB.bit.CBD = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm2Regs.AQCTLB.bit.PRD = 0;                       // 0 Não faz nada no modo opdown
EPwm2Regs.AQCTLB.bit.ZRO = 0;                       // 0 Não faz nada no modo opdown

EPwm2Regs.DBCTL.bit.IN_MODE=0;                      //  Dead-band module
EPwm2Regs.DBCTL.bit.POLSEL=2;
EPwm2Regs.DBCTL.bit.OUT_MODE =DEAD_BAND_OUT;
EPwm2Regs.DBRED.bit.DBRED=DEAD_TIME;
EPwm2Regs.DBFED.bit.DBFED=DEAD_TIME;

EPwm2Regs.ETSEL.bit.SOCAEN=1;                       // Habilita SOCB para SEQ2
EPwm2Regs.ETSEL.bit.SOCASEL=1;                      // Gera no zero
EPwm2Regs.ETPS.bit.SOCAPRD=1;                       // gera soc em todos os eventos


EALLOW;
EPwm2Regs.TZSEL.bit.OSHT1=1;                        //  en
EPwm2Regs.TZCTL.bit.TZA  =2;                        // Force PWMxA to low  state (2)
EPwm2Regs.TZCTL.bit.TZB  =2;                        // Force PWMxB to low state (2)
EPwm2Regs.TZFRC.bit.OST  =1;                        // Forced trip in rectifier=1
EDIS;


//-------------------------------------------------------------------------------
// EPWM Module 4
//-------------------------------------------------------------------------------

EPwm4Regs.TBCTL.bit.HSPCLKDIV=0;                    // Frequencia do clock/1
EPwm4Regs.TBCTL.bit.CLKDIV=0;                       // pré escala 1/1

EPwm4Regs.TBPRD = TIMER_PWM;                        // Period
EPwm4Regs.TBPHS.bit.TBPHS = PHASE_PWM4;            // Set Phase register
EPwm4Regs.TBCTL.bit.CTRMODE = 2;                    // Symmetrical mode (up down)
EPwm4Regs.TBCTL.bit.PHSEN = 1;                      // Slave module (EPwm1 is Master)
EPwm4Regs.TBCTL.bit.PHSDIR =1;                      // Count up after the Sync event
EPwm4Regs.TBCTL.bit.PRDLD = 0;                      // Period timer is load at zero
EPwm4Regs.TBCTL.bit.SYNCOSEL = 0;                   // Sync when timer is 0
EPwm4Regs.TBCTL.bit.SWFSYNC =0;                     // Do not force a Sync pulse
EPwm4Regs.TBCTL.bit.FREE_SOFT=2;                    // runs free

EPwm4Regs.CMPCTL.bit.SHDWAMODE = 0;                 // Shadow mode atualização não imediata
EPwm4Regs.CMPCTL.bit.SHDWBMODE = 0;                 // Shadow mode atualização não imediata
EPwm4Regs.CMPCTL.bit.LOADAMODE = 2;                 // load on zero or per. (2), zero(0), per.(1)
EPwm4Regs.CMPCTL.bit.LOADBMODE = 2;                 // load on zero or per. (2), zero(0), per.(1)

EPwm4Regs.AQCTLA.bit.CAU = 2;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm4Regs.AQCTLA.bit.CAD = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm4Regs.AQCTLA.bit.CBU = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm4Regs.AQCTLA.bit.CBD = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm4Regs.AQCTLA.bit.PRD = 0;                       // 0 Não faz nada no modo opdown
EPwm4Regs.AQCTLA.bit.ZRO = 0;                       // 0 Não faz nada no modo opdown


EPwm4Regs.AQCTLB.bit.CAU = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm4Regs.AQCTLB.bit.CAD = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm4Regs.AQCTLB.bit.CBU = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm4Regs.AQCTLB.bit.CBD = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm4Regs.AQCTLB.bit.PRD = 0;                       // 0 Não faz nada no modo opdown
EPwm4Regs.AQCTLB.bit.ZRO = 0;                       // 0 Não faz nada no modo opdown

EPwm4Regs.DBCTL.bit.IN_MODE=0;                      //  Dead-band module
EPwm4Regs.DBCTL.bit.POLSEL=2;
EPwm4Regs.DBCTL.bit.OUT_MODE =DEAD_BAND_OUT;
EPwm4Regs.DBRED.bit.DBRED=DEAD_TIME;
EPwm4Regs.DBFED.bit.DBFED=DEAD_TIME;

EPwm4Regs.ETSEL.bit.SOCAEN=1;                       // Habilita SOCB para SEQ2
EPwm4Regs.ETSEL.bit.SOCASEL=2;                      // Gera no periodo
EPwm4Regs.ETPS.bit.SOCAPRD=1;                       // gera soc em todos os evento

EALLOW;
EPwm4Regs.TZSEL.bit.OSHT1=1;                        //  disable
EPwm4Regs.TZCTL.bit.TZA  =2;                        // Force PWMxA to low  state (2)
EPwm4Regs.TZCTL.bit.TZB  =2;                        // Force PWMxB to low state (2)
EPwm4Regs.TZFRC.bit.OST  =1;                        // Forced trip in rectifier=1
EDIS;

//-------------------------------------------------------------------------------
// EPWM Module 6
//-------------------------------------------------------------------------------

EPwm6Regs.TBCTL.bit.HSPCLKDIV=0;                    // Frequencia do clock/1
EPwm6Regs.TBCTL.bit.CLKDIV=0;                       // pré escala 1/1

EPwm6Regs.TBPRD = TIMER_PWM;                        // Period
EPwm6Regs.TBPHS.bit.TBPHS = PHASE_PWM6;             // Set Phase register
EPwm6Regs.TBCTL.bit.CTRMODE = 2;                    // Symmetrical mode (up down)
EPwm6Regs.TBCTL.bit.PHSEN = 1;                      // Slave module (EPwm1 is Master)
EPwm6Regs.TBCTL.bit.PHSDIR =1;                      // Count up after the Sync event
EPwm6Regs.TBCTL.bit.PRDLD = 0;                      // Period timer is load at zero
EPwm6Regs.TBCTL.bit.SYNCOSEL = 0;                   // Sync when timer is 0
EPwm6Regs.TBCTL.bit.SWFSYNC =0;                     // Do not force a Sync pulse
EPwm6Regs.TBCTL.bit.FREE_SOFT=2;                    // runs free

EPwm6Regs.CMPCTL.bit.SHDWAMODE = 0;                 // Shadow mode atualização não imediata
EPwm6Regs.CMPCTL.bit.SHDWBMODE = 0;                 // Shadow mode atualização não imediata
EPwm6Regs.CMPCTL.bit.LOADAMODE = 2;                 // load on zero or per. (2), zero(0), per.(1)
EPwm6Regs.CMPCTL.bit.LOADBMODE = 2;                 // load on zero or per. (2), zero(0), per.(1)

EPwm6Regs.AQCTLA.bit.CAU = 2;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm6Regs.AQCTLA.bit.CAD = 1;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm6Regs.AQCTLA.bit.CBU = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm6Regs.AQCTLA.bit.CBD = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm6Regs.AQCTLA.bit.PRD = 0;                       // 0 Não faz nada no modo opdown
EPwm6Regs.AQCTLA.bit.ZRO = 0;                       // 0 Não faz nada no modo opdown


EPwm6Regs.AQCTLB.bit.CAU = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm6Regs.AQCTLB.bit.CAD = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm6Regs.AQCTLB.bit.CBU = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm6Regs.AQCTLB.bit.CBD = 0;                       // actions for EPWM1A -> 1 clear, 2 set, 3 toggle
EPwm6Regs.AQCTLB.bit.PRD = 0;                       // 0 Não faz nada no modo opdown
EPwm6Regs.AQCTLB.bit.ZRO = 0;                       // 0 Não faz nada no modo opdown

EPwm6Regs.DBCTL.bit.IN_MODE=0;                      //  Dead-band module
EPwm6Regs.DBCTL.bit.POLSEL=2;
EPwm6Regs.DBCTL.bit.OUT_MODE =DEAD_BAND_OUT;
EPwm6Regs.DBRED.bit.DBRED=DEAD_TIME;
EPwm6Regs.DBFED.bit.DBFED=DEAD_TIME;

EPwm6Regs.ETSEL.bit.SOCBEN=0;                       // Habilita SOCB para SEQ2
EPwm6Regs.ETSEL.bit.SOCBSEL=2;                      // Gera no período
EPwm6Regs.ETPS.bit.SOCBPRD=1;                       // gera soc em todos os eventos


EALLOW;
EPwm6Regs.TZSEL.bit.OSHT1=1;                        //  disable
EPwm6Regs.TZCTL.bit.TZA  =2;                        // Force PWMxA to low  state (2)
EPwm6Regs.TZCTL.bit.TZB  =2;                        // Force PWMxB to low state (2)
EPwm6Regs.TZFRC.bit.OST  =1;                        // Forced trip in rectifier=1
EDIS;

//-------------------------------------------------------------------------------
// EPWM Sync
//-------------------------------------------------------------------------------

//EALLOW;
//SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=1;    // for carriers sinc
//EDIS;
}


//
// InitEPwmGpio - Initialize EPWM GPIOs
//
void InitEPwmGpio(void)
{
    EALLOW;

    //
    // Disable internal pull-up for the selected output pins
    //   for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)

    //
    // Configure EPwm-3 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM3 functional pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B

    EDIS;
}



//
// InitEPwm1Example - Initialize EPWM1 configuration
//
void InitEPwm1Example()
{
    //
    // Setup TBCLK
    //
    EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;       // Set timer period 801 TBCLKs
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter

    //
    // Set Compare values
    //
    EPwm1Regs.CMPA.bit.CMPA = EPWM1_MIN_CMPA;    // Set compare A value
    EPwm1Regs.CMPB.bit.CMPB = EPWM1_MAX_CMPB;    // Set Compare B value

    //
    // Setup counter mode
    //
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup shadowing
    //
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on event A, up
                                                  // count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM1A on event A,
                                                  // down count

    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;            // Set PWM1B on event B, up
                                                  // count
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // Clear PWM1B on event B,
                                                  // down count

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

    //
    // Information this example uses to keep track
    // of the direction the CMPA/CMPB values are
    // moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwm1_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA
    epwm1_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // & decreasing CMPB
    epwm1_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
    epwm1_info.EPwmRegHandle = &EPwm1Regs;          // Set the pointer to the
                                                    // ePWM module
    epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;        // Setup min/max CMPA/CMPB
                                                    // values
    epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
    epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
    epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
}

//
// InitEPwm2Example - Initialize EPWM2 configuration
//
void InitEPwm2Example()
{
    //
    // Setup TBCLK
    //
    EPwm2Regs.TBPRD = EPWM2_TIMER_TBPRD;         // Set timer period 801 TBCLKs
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                    // Clear counter

    //
    // Set Compare values
    //
    EPwm2Regs.CMPA.bit.CMPA = EPWM2_MIN_CMPA;    // Set compare A value
    EPwm2Regs.CMPB.bit.CMPB = EPWM2_MIN_CMPB;    // Set Compare B value

    //
    // Setup counter mode
    //
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup shadowing
    //
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set actions
    //
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;         // Set PWM2A on event A, up
                                               // count
    EPwm2Regs.AQCTLA.bit.CBD = AQ_CLEAR;       // Clear PWM2A on event B, down
                                               // count

    EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;       // Clear PWM2B on zero
    EPwm2Regs.AQCTLB.bit.PRD = AQ_SET;         // Set PWM2B on period

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;          // Generate INT on 3rd event

    //
    // Information this example uses to keep track
    // of the direction the CMPA/CMPB values are
    // moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP;  // Start by increasing CMPA
    epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_UP;  // & increasing CMPB
    epwm2_info.EPwmTimerIntCount = 0;              // Zero the interrupt counter
    epwm2_info.EPwmRegHandle = &EPwm2Regs;         // Set the pointer to the
                                                   // ePWM module
    epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA;       // Setup min/max CMPA/CMPB
                                                   // values
    epwm2_info.EPwmMinCMPA = EPWM2_MIN_CMPA;
    epwm2_info.EPwmMaxCMPB = EPWM2_MAX_CMPB;
    epwm2_info.EPwmMinCMPB = EPWM2_MIN_CMPB;
}

//
// InitEPwm3Example - Initialize EPWM3 configuration
//
void InitEPwm3Example(void)
{
    //
    // Setup TBCLK
    //
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up/down and down
    EPwm3Regs.TBPRD = EPWM3_TIMER_TBPRD;           // Set timer period
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;            // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                      // Clear counter
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup shadow register load on ZERO
    //
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set Compare values
    //
    EPwm3Regs.CMPA.bit.CMPA = EPWM3_MIN_CMPA;   // Set compare A value
    EPwm3Regs.CMPB.bit.CMPB = EPWM3_MAX_CMPB;   // Set Compare B value

    //
    // Set Actions
    //
    EPwm3Regs.AQCTLA.bit.PRD = AQ_SET;         // Set PWM3A on period
    EPwm3Regs.AQCTLA.bit.CBD = AQ_CLEAR;       // Clear PWM3A on event B, down
                                               // count

    EPwm3Regs.AQCTLB.bit.PRD = AQ_CLEAR;       // Clear PWM3A on period
    EPwm3Regs.AQCTLB.bit.CAU = AQ_SET;         // Set PWM3A on event A, up
                                               // count

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm3Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;          // Generate INT on 3rd event

    //
    // Information this example uses to keep track
    // of the direction the CMPA/CMPB values are
    // moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwm3_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA
    epwm3_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // & decreasing CMPB
    epwm3_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
    epwm3_info.EPwmRegHandle = &EPwm3Regs;          // Set the pointer to the
                                                    // ePWM module
    epwm3_info.EPwmMaxCMPA = EPWM3_MAX_CMPA;        // Setup min/max CMPA/CMPB
                                                    // values
    epwm3_info.EPwmMinCMPA = EPWM3_MIN_CMPA;
    epwm3_info.EPwmMaxCMPB = EPWM3_MAX_CMPB;
    epwm3_info.EPwmMinCMPB = EPWM3_MIN_CMPB;
}

//
// update_compare - Update the PWM compare values
//
void update_compare(EPWM_INFO *epwm_info)
{
    //
    // Every 10'th interrupt, change the CMPA/CMPB values
    //
    if(epwm_info->EPwmTimerIntCount == 10)
    {
        epwm_info->EPwmTimerIntCount = 0;

        //
        // If we were increasing CMPA, check to see if
        // we reached the max value.  If not, increase CMPA
        // else, change directions and decrease CMPA
        //
        if(epwm_info->EPwm_CMPA_Direction == EPWM_CMP_UP)
        {
            if(epwm_info->EPwmRegHandle->CMPA.bit.CMPA <
               epwm_info->EPwmMaxCMPA)
            {
                epwm_info->EPwmRegHandle->CMPA.bit.CMPA++;
            }
            else
            {
                epwm_info->EPwm_CMPA_Direction = EPWM_CMP_DOWN;
                epwm_info->EPwmRegHandle->CMPA.bit.CMPA--;
            }
        }

        //
        // If we were decreasing CMPA, check to see if
        // we reached the min value.  If not, decrease CMPA
        // else, change directions and increase CMPA
        //
        else
        {
            if(epwm_info->EPwmRegHandle->CMPA.bit.CMPA ==
               epwm_info->EPwmMinCMPA)
            {
                epwm_info->EPwm_CMPA_Direction = EPWM_CMP_UP;
                epwm_info->EPwmRegHandle->CMPA.bit.CMPA++;
            }
            else
            {
                epwm_info->EPwmRegHandle->CMPA.bit.CMPA--;
            }
        }

        //
        // If we were increasing CMPB, check to see if
        // we reached the max value.  If not, increase CMPB
        // else, change directions and decrease CMPB
        //
        if(epwm_info->EPwm_CMPB_Direction == EPWM_CMP_UP)
        {
            if(epwm_info->EPwmRegHandle->CMPB.bit.CMPB < epwm_info->EPwmMaxCMPB)
            {
                epwm_info->EPwmRegHandle->CMPB.bit.CMPB++;
            }
            else
            {
                epwm_info->EPwm_CMPB_Direction = EPWM_CMP_DOWN;
                epwm_info->EPwmRegHandle->CMPB.bit.CMPB--;
            }
        }

        //
        // If we were decreasing CMPB, check to see if
        // we reached the min value.  If not, decrease CMPB
        // else, change directions and increase CMPB
        //
        else
        {
            if(epwm_info->EPwmRegHandle->CMPB.bit.CMPB == epwm_info->EPwmMinCMPB)
            {
                epwm_info->EPwm_CMPB_Direction = EPWM_CMP_UP;
                epwm_info->EPwmRegHandle->CMPB.bit.CMPB++;
            }
            else
            {
                epwm_info->EPwmRegHandle->CMPB.bit.CMPB--;
            }
        }
    }
    else
    {
        epwm_info->EPwmTimerIntCount++;
    }

    return;
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





