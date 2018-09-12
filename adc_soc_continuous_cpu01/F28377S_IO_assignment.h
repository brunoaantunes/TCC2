/*
 * F28377S_IO_assignment.h
 *
 *  Created on: 25 de mai de 2018
 *      Author: ESPM
 */

#ifndef F28377S_IO_ASSIGNMENT_H_
#define F28377S_IO_ASSIGNMENT_H_

/*
// ret 1 EPWM selections
// ========================
#define  RET1_EPWM_A_GPIO    10
#define  RET1_EPWM_A_MUX      1

#define  RET1_EPWM_B_GPIO    12
#define  RET1_EPWM_B_MUX      1

#define  RET1_EPWM_C_GPIO    14
#define  RET1_EPWM_C_MUX      1
*/
// *************************************************
// ************ ADC pin assignments ***************
// *************************************************

#define IFB_A1       AdcaResultRegs.ADCRESULT0  //sensor corrente Ia em A0
//#define IFB_B1       AdcbResultRegs.ADCRESULT0    //sensor corrente Ib n�o utilizado
#define IFB_C1       AdcbResultRegs.ADCRESULT0  //sensor corrente Ic em B0
/*
#define IFB_A1_PPB   ((signed int)AdcaResultRegs.ADCPPB1RESULT.all)
//#define IFB_B1_PPB   ((signed int)AdcbResultRegs.ADCPPB1RESULT.all)
#define IFB_C1_PPB   ((signed int)AdcbResultRegs.ADCPPB1RESULT.all)
*/
#define VFB_A1       AdcaResultRegs.ADCRESULT2  //sensor tens�o Va em A1
//#define VFB_B1       AdcbResultRegs.ADCRESULT2    //sensor tens�o Vb n�o utilizado
#define VFB_C1       AdcbResultRegs.ADCRESULT2  //sensor tens�o Vc em B1
/*
#define VFB_A1_PPB   ((signed int)AdcaResultRegs.ADCPPB2RESULT.all)
//#define IFB_B1_PPB   ((signed int)AdcbResultRegs.ADCPPB1RESULT.all)
#define VFB_C1_PPB   ((signed int)AdcbResultRegs.ADCPPB2RESULT.all)
*/
#define VDC1FB_A1     AdcaResultRegs.ADCRESULT4 //sensor tens�o Vout + em A2
#define VDC2FB_B1     AdcbResultRegs.ADCRESULT4 //sensor tens�o Vout - em B2
/*
#define VDC1FB_A1_PPB   ((signed int)AdcaResultRegs.ADCPPB3RESULT.all)
#define VDC1FB_B1_PPB   ((signed int)AdcbResultRegs.ADCPPB3RESULT.all)
*/
#define ADC_PU_SCALE_FACTOR        0.000244140625     //1/2^12
#define ADC_PU_PPB_SCALE_FACTOR    0.000488281250     //1/2^11

// *****************************************************************************
// *****************************************************************************

#endif /* F28377S_IO_ASSIGNMENT_H_ */
