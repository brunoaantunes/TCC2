/*
 * retVars.h
 *
 *  Created on: 25 de mai de 2018
 *      Author: ESPM
 */

#ifndef RETVARS_H_
#define RETVARS_H_

#define   MATH_TYPE      1

#include "F28x_Project.h"
#include "ret-Settings.h"

// ****************************************************************************
// ret variables - EPWM
// ****************************************************************************
typedef struct {
    volatile struct EPWM_REGS * PwmARegs,
                              * PwmBRegs,
                              * PwmCRegs;
}RET_VARS;

// ****************************************************************************
// Default values for motor variables with DRV8301 and DRV8305
// ****************************************************************************
#define RET_DEFAULTS  {                         \
            &EPwm1Regs, /*  PwmARegs  - change in main */ \
            &EPwm1Regs, /*  PwmBRegs  - change in main */ \
            &EPwm1Regs /*  PwmCRegs  - change in main */ \
}

#endif /* RETVARS_H_ */
