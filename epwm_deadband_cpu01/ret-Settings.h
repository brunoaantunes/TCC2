/*
 * ret-Settings.h
 *
 *  Created on: 25 de mai de 2018
 *      Author: ESPM
 */

#ifndef RET_SETTINGS_H_
#define RET_SETTINGS_H_

///*-------------------------------------------------------------------------------
//Include project specific include files.
//-------------------------------------------------------------------------------*/
// define math type as float(1)
#define   MATH_TYPE      1
#include "F28x_Project.h"

#define PI 3.14159265358979

#define SYSTEM_FREQUENCY 200 //(MHz)


// Timer definitions based on System Clock
// 150 MHz devices
#define     mSec0_5     0.5*SYSTEM_FREQUENCY*1000       // 0.5 mS
#define     mSec1       1*SYSTEM_FREQUENCY*1000     // 1.0 mS
#define     mSec2       2.0*SYSTEM_FREQUENCY*1000       // 2.0 mS
#define     mSec5       5*SYSTEM_FREQUENCY*1000     // 5.0 mS
#define     mSec7_5     7.5*SYSTEM_FREQUENCY*1000       // 7.5 mS
#define     mSec10      10*SYSTEM_FREQUENCY*1000        // 10 mS
#define     mSec20      20*SYSTEM_FREQUENCY*1000        // 20 mS
#define     mSec50      50*SYSTEM_FREQUENCY*1000        // 50 mS
#define     mSec100     100*SYSTEM_FREQUENCY*1000       // 100 mS
#define     mSec500     500*SYSTEM_FREQUENCY*1000   // 500 mS
#define     mSec1000    1000*SYSTEM_FREQUENCY*1000  // 1000 mS

// Define the ISR frequency (kHz)
#define  PWM_FREQUENCY        19.2
#define  ISR_FREQUENCY        PWM_FREQUENCY
#define  INV_PWM_TICKS        ((SYSTEM_FREQUENCY/2.0)/PWM_FREQUENCY)*1000
#define  INV_PWM_TBPRD        INV_PWM_TICKS/2
#define  INV_PWM_HALF_TBPRD   INV_PWM_TICKS/4


#endif /* RET_SETTINGS_H_ */
