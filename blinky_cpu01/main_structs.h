/*
 * main_structs.h
 *
 *  Created on: 10 de jun de 2018
 *      Author: ESPM
 */

#ifndef MAIN_STRUCTS_H_
#define MAIN_STRUCTS_H_

#include "F28x_Project.h"
#include "main_settings.h"

#include "sgen.h"               //Bibliotecas para teste da geração dos senos
#include "SVGEN_CLA.h"          // Include header for svgen float lib
#include "PI_CLA.h"             // Include header for PI controllers float lib
#include "CLARKE_CLA.h"         // Include header for clarke transform float lib
#include "PARK_CLA.h"           // Include header for park transform float lib
#include "iPARK_CLA.h"          // Include header for ipark transform float lib

// Sine generation variables
SGENT_3 sgen = SGENT_3_DEFAULTS;

/*
// Transforms and SVGEN Variables
SVGEN_CLA svgen1; // space vector generation float
CLARKE_CLA clarke1;
PARK_CLA park1;
iPARK_CLA ipark1;

// Controller variables
PI_CLA   pi_voltage1; // output voltage controller
PI_CLA   pi_id1;      //
PI_CLA   pi_iq1;
*/

typedef struct {
    float32 offset_shntA;   // shunt current feedback A - offset @ 0A
    float32 offset_shntB;   // shunt current feedback B - offset @ 0A
    float32 offset_shntC;   // shunt current feedback C - offset @ 0A
    float32 IdTesting;      // Id Testing
    float32 IqTesting;      // Iq Testing
    float32 IdRef;          // Id Reference
    float32 IqRef;          // Iq Reference
    float32 VoltageRef;     // Voltage Reference
    float32 Kqd;            // Decoupling gain
    float32 currentAs;      // Current branch A
    float32 currentBs;      // Current branch B
    float32 currentCs;      // Current branch C
    float32 voltageAs;      // Phase A
    float32 voltageBs;      // Phase B
    float32 voltageCs;      // Phase C
    float32 voltadeDC1;     // +Vdc output
    float32 voltageDC2;     // -Vdc output
    float32 T;              // Sampling time
    SVGEN_CLA  svgen1;      // svpwm macro
    CLARKE_CLA clarke1;     // clarke transform macro
    PARK_CLA   park1;       // park transform macro
    iPARK_CLA  ipark1;      // ipark transform macro
    PI_CLA     pi_voltage1; // output voltage controller
    PI_CLA     pi_id1;      // output id current controller
    PI_CLA     pi_iq1;      // output iq current controller
} RECTIFIER_VARS1;

// ****************************************************************************
// Default values for rectifier variables
// ****************************************************************************

#endif /* MAIN_STRUCTS_H_ */
