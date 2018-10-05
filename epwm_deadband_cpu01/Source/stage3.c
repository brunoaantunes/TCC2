/*
 * stage3.c
 *
 */

//
// Included Files
//
#include "F28x_Project.h"
//#include "ePWM.h"
//#include "ADC.h"
#include "Structs.h"
#include "Defines.h"

extern struct RETIFICADOR conv;
extern struct CONTROLE ctrl;
extern struct TRIGONOMETRICO trig;
extern struct PLL pll;
extern struct PI Cvz,CizID,CizIQ;

void stage3(void){
    EALLOW;
                EPwm2Regs.TZFRC.bit.OST = 1;            // Force one shot triger via software
                EPwm4Regs.TZFRC.bit.OST = 1;            // Force one shot triger via software
                EPwm6Regs.TZFRC.bit.OST = 1;            // Force one shot triger via software
                EDIS;
    ctrl.STATE =0;
    asm ("          NOP"); // Para fins de teste
    return;
}


