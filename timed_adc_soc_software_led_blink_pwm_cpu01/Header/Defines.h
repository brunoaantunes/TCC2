/*
 * Defines.h
 *
 */

#ifndef HEADER_DEFINES_H_
#define HEADER_DEFINES_H_



#define PIX1      (float)3.1415926535897932384626433832795
#define PIX2      (float)3.1415926535897932384626433832795*2
#define PIDiv2    (float)(3.1415926535897932384626433832795/2)
#define cte1      (float) 0.577350269189626   // sqrt(3)/3
#define cte2      (float) 1.154700538379251   // 2*sqrt(3)/3
#define sqrt3     (float) 1.732050807568877   // sqrt(3)
#define cte3      (float) 0.866025403784439   // sqrt(3)/2

//#define cte1        (float) sqrt(3)/3   // sqrt(3)/3
//#define cte2        (float) 2*sqrt(3)/3 // 2*sqrt(3)/3
//#define sqrt3       (float) sqrt(3)     // sqrt(3)
//#define cte3        (float) sqrt(3)/2   // sqrt(3)/2


#define B0_PLL  61.8034088702226
#define B1_PLL  -61.7218092663225
#define CI_PLL  376.991118430775
#define TS_PLL  2.5025025025025e-005

#define Cib0z  -6.58653214e-01
#define Cib1z  4.13338686e-01

#define Cvb0z 5.11815551e-02
#define Cvb1z -5.08216203e-02

#define kdq 7.75447734e-03

#define Hvca 0.004232013940752
#define HvcaInv 2.362941176470588e+02

#define Hvcc 7.59373517e-03
#define HvccInv 131.6875

#define Hica 0.325600000000000
#define HicaInv 3.071253071253071

#define GADCinv 7.324218750000000e-04 // 3/4096
#define ADCoffset 2048



// TODO: Verificar estes valores
#define FCLK        200000000L
#define FS_CONV      19980L
#define TIMER_PWM   (unsigned int)(FCLK/(2*FS_CONV))

#define PHASE_PWM1  0
#define PHASE_PWM2  0
#define PHASE_PWM3  0
#define PHASE_PWM4  0
#define PHASE_PWM5  0//(unsigned int)((float)TIMER_RET*0.6666666666666) // in falling edge PHSDIR=0
#define PHASE_PWM6  0//(unsigned int)((float)TIMER_RET*0.6666666666666) // in rising edge  PHSDIR=1

#define DEAD_TIMEs  0.000001    // entrar com o tempo morto desejado em segundos
#define DEAD_TIME   (unsigned int) ((float)DEAD_TIMEs*FCLK)
#define DEAD_BAND_OUT 3 // 0 é o bypass do dead band; 3 é o full dead band






#endif /* HEADER_DEFINES_H_ */
