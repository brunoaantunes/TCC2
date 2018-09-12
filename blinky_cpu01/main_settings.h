/*
 * main_settings.h
 *
 *  Created on: 10 de jun de 2018
 *      Author: ESPM
 */

#ifndef MAIN_SETTINGS_H_
#define MAIN_SETTINGS_H_

#include "F28x_Project.h"

/*------------------------------------------------------------------------------
Following is the list of the Build Level choices.
------------------------------------------------------------------------------*/
#define LEVEL1  1           // Module check out
#define LEVEL2  2           // Verify ADC, park/clarke, calibrate the offset and voltage measurement
#define LEVEL3  3           // Verify open current loop and its PIs
#define LEVEL4  4           // Verify closed control loop

/*------------------------------------------------------------------------------
This line sets the BUILDLEVEL to one of the available choices.
------------------------------------------------------------------------------*/
#define   BUILDLEVEL LEVEL1

#ifndef BUILDLEVEL
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL
//------------------------------------------------------------------------------

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define SYSTEM_FREQUENCY 200 //(MHz)

// Define the ISR frequency (kHz)
#define  PWM_FREQUENCY        19.2
#define  ISR_FREQUENCY        PWM_FREQUENCY
#define  TS                   0.001/ISR_FREQUENCY // sampling time
#define  INV_PWM_TICKS        ((SYSTEM_FREQUENCY/2.0)/PWM_FREQUENCY)*1000
#define  INV_PWM_TBPRD        INV_PWM_TICKS/2
#define  INV_PWM_HALF_TBPRD   INV_PWM_TICKS/4

#endif /* MAIN_SETTINGS_H_ */
