/*
 * Stage2.c
 *
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

extern void sincosf(float x, float *sinx, float *cosx);

void stage2(void){
//    ctrl.STATE =3;


    // Geração do wt
    trig.w = 377;
    trig.wt+=trig.w*TS_PLL;
    if(trig.wt>PIX1) trig.wt = trig.wt - PIX2;

    sincosf(trig.wt, &(trig.senoA), &(trig.coseA));

    conv.dd = 0.5; // Obtido por simulação
    conv.dq = 0;

    // Modulação
    conv.da = trig.senoA*conv.dd + trig.coseA*conv.dq;
    conv.db = -(cte3*trig.coseA + trig.senoA*0.5)*conv.dd + (-trig.coseA*0.5 + cte3*trig.senoA)*conv.dq;
    conv.dc = (cte3*trig.coseA - trig.senoA*0.5)*conv.dd - (trig.coseA*0.5 + cte3*trig.senoA)*conv.dq;


    return;
}


