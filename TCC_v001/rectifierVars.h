//----------------------------------------------------------------------------------
//  FILE:           rectfierVars.h
//
//  Description:    Variable structure definition for Rectfier Control
//
//  Version:        1.0
//
//  Target:         TMS320F28377S
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments © 2004-2015
//----------------------------------------------------------------------------------
//  Revision History: Lucas Plentz
//----------------------------------------------------------------------------------
//  Date      | Description / Status
//----------------------------------------------------------------------------------
// 6 May 2018 - Variable structure defintion for Rectifier Control
//----------------------------------------------------------------------------------

#ifndef RECTIFIERVARS_H_
#define RECTIFIERVARS_H_

#define   MATH_TYPE      1
#include "IQmathLib.h"
#include "F28x_Project.h"
#include "Boost-Settings.h"

#include "park.h"               // Include header for the PARK object
#include "ipark.h"              // Include header for the IPARK object
#include "pi.h"                 // Include header for the PIDREG3 object
#include "clarke.h"             // Include header for the CLARKE object
#include "svgen.h"              // Include header for the SVGENDQ object
#include "rampgen.h"            // Include header for the RAMPGEN object
#include "rmp_cntl.h"           // Include header for the RMPCNTL object
#include "volt_calc.h"          // Include header for the PHASEVOLTAGE object
//#include "pid_grando.h"         // Include header for the PID_GRANDO object
//#include "pid_reg3.h"           // Include header for the PID_REG3 object

// ****************************************************************************
// Rectifier variables - for Field Oriented Control
// ****************************************************************************
typedef struct {
    volatile struct EPWM_REGS * PwmARegs,
                              * PwmBRegs,
                              * PwmCRegs;

    _iq  offset_shntA,   // shunt current feedback A - offset @ 0A
         offset_shntB,   // shunt current feedback B - offset @ 0A
         offset_shntC,   // shunt current feedback C - offset @ 0A

         IdTesting,         // Vd reference (pu)
         IqTesting,         // Vq reference (pu)
         IdRef,             // Id reference (pu)
         IqRef,             // Iq reference (pu)
         VoltageRef,        // voltage ref (pu)
         //v_out,             // output voltage (pu)
         Kqd;               // decoupling gain (pu)

    float32 currentAs,      // current branch A
            currentBs,      // current branch B
            currentCs;      // current branch C

    float32 voltageAs,      // phase A
            voltageBs,      // phase B
            voltageCs;      // phase C

    float32 voltadeDC1,     // +Vdc output
            voltageDC2;     // -Vdc output

    float32  T;               // sampling time

    // Transform variables
    CLARKE clarke;            // clarke transform
    PARK   park;              // park transform
    IPARK  ipark;             // inv park transform

    // Controller variables
    PI_CONTROLLER   pi_voltage;
    PI_CONTROLLER   pi_id;
    PI_CONTROLLER   pi_iq;

    SVGEN svgen;               // SVPWM variable

    //PHASEVOLTAGE volt;         // phase voltage variable

} RECTIFIER_VARS;

// ****************************************************************************
// Default values for rectifier variables
// ****************************************************************************
//
#define RECTIFIER_DEFAULTS  {                             \
            &EPwm1Regs,  /*  PwmARegs  - change in main*/ \
            &EPwm1Regs,  /*  PwmBRegs  - change in main*/ \
            &EPwm1Regs,  /*  PwmCRegs  - change in main*/ \
                                                          \
            0,          /*  offset_shntA  */              \
            0,          /*  offset_shntB  */              \
            0,          /*  offset_shntC  */              \
                                                          \
            0,          /*  IdTesting             */      \
            _IQ(0.07),  /*  IqTesting             */      \
            0,          /*  IdRef                 */      \
            0,          /*  IqRef                 */      \
            _IQ(400.0), /*  VoltagedRef           */      \
/*          0,        */  /*  v_out                 */    \
            0,          /*  Kqd                   */      \
                                                          \
            0,0,0,      /*  currents A, B, C      */      \
                                                          \
            0,0,0,      /*  voltages A, B, C      */      \
                                                          \
            0,0,        /*  voltages Vdc+, Vdc-   */      \
                                                          \
            0.001/ISR_FREQUENCY,          /*  T   */      \
                                                          \
            CLARKE_DEFAULTS,   /*  clarke  */             \
            PARK_DEFAULTS,     /*  park    */             \
            IPARK_DEFAULTS,    /*  ipark   */             \
                                                          \
            PI_CONTROLLER_DEFAULTS,   /*  pi_voltage */   \
            PI_CONTROLLER_DEFAULTS,   /*  pi_id      */   \
            PI_CONTROLLER_DEFAULTS,   /*  pi_iq      */   \
                                                          \
            SVGEN_DEFAULTS          /* svgen    */        \
            /*PHASEVOLTAGE_DEFAULTS */   /* volt     */   \
}


#endif /* RECTIFIERVARS_H_ */
