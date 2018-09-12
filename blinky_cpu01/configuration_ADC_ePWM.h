/*
 * configuration.h
 *
 *  Created on: 2 de jun de 2018
 *      Author: ESPM
 */
/*
         Analog signals - Rectifier 1
         Vdc+  ADC A2
         Vdc- ADC B2
         Va   ADC A1
         Vb   ADC
         Vc   ADC B1
         Ia   ADC A0
         Ib   ADC
         Ic   ADC B0
*/

#ifndef CONFIGURATION_ADC_EPWM_H_
#define CONFIGURATION_ADC_EPWM_H_

void ConfigureADC(void);
void SetupADCContinuous(void);

void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);

//==============================
// ADC trigger selection table
//==============================
#define  ADCTRIG0_SW_ONLY        0
#define  ADCTRIG1_CPU1_T0        1
#define  ADCTRIG2_CPU1_T1        2
#define  ADCTRIG3_CPU1_T2        3
#define  ADCTRIG4_GPIOEXTSOC     4

#define  ADCTRIG5_EPWM1SOCA      5
#define  ADCTRIG6_EPWM1SOCB      6

#define  ADCTRIG7_EPWM2SOCA      7
#define  ADCTRIG8_EPWM2SOCB      8

#define  ADCTRIG9_EPWM3SOCA      9
#define  ADCTRIG10_EPWM3SOCB    10

#define  ADCTRIG11_EPWM4SOCA    11
#define  ADCTRIG12_EPWM4SOCB    12

#define  ADCTRIG13_EPWM5SOCA    13
#define  ADCTRIG14_EPWM5SOCB    14

#define  ADCTRIG15_EPWM6SOCA    15
#define  ADCTRIG16_EPWM6SOCB    16

#define  ADCTRIG17_EPWM7SOCA    17
#define  ADCTRIG18_EPWM7SOCB    18

#define  ADCTRIG19_EPWM8SOCA    19
#define  ADCTRIG20_EPWM8SOCB    20

#define  ADCTRIG21_EPWM9SOCA    21
#define  ADCTRIG22_EPWM9SOCB    22

#define  ADCTRIG23_EPWM10SOCA   23
#define  ADCTRIG24_EPWM10SOCB   24

#define  ADCTRIG25_EPWM11SOCA   25
#define  ADCTRIG26_EPWM11SOCB   26

#define  ADCTRIG27_EPWM12SOCA   27
#define  ADCTRIG28_EPWM12SOCB   28


#endif /* CONFIGURATION_ADC_EPWM_H_ */