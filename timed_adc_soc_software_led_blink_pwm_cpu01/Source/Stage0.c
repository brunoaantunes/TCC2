/*
 * Stage0.c
 */

//
// Included Files
//
#include "F28x_Project.h"
#include "ePWM.h"
#include "ADC.h"
#include "Structs.h"
#include "Defines.h"

extern struct RETIFICADOR conv;
extern struct CONTROLE ctrl;
extern struct TRIGONOMETRICO trig;
extern struct PLL pll;
extern struct PI Cvz,CizID,CizIQ;


void stage0(void){



    ctrl.STATE =2; // Modo inversor
    return;
}


