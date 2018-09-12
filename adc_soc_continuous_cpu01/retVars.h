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
    float32  offset_shntA,   // shunt current feedback A - offset @ 0A
             offset_shntB,   // shunt current feedback B - offset @ 0A
             offset_shntC,   // shunt current feedback C - offset @ 0A

             currentAs,      // phase A
             currentBs,      // phase B
             currentCs;      // phase C
}RET_VARS;

// ****************************************************************************
// Default values for motor variables with DRV8301 and DRV8305
// ****************************************************************************
#define RET_DEFAULTS  {                                   \
            0,          /*  offset_shntA          */      \
            0,          /*  offset_shntB          */      \
            0,          /*  offset_shntC          */      \
                                                          \
            0,                                            \
            0,                                            \
            0      /*  currents A, B, C      */           \
}

#endif /* RETVARS_H_ */
