//----------------------------------------------------------------------------------
//  FILE:           Boost.h
//
//  Description:    Header file for boost rectifier control with 37xS launch pad
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
// 6 May 2018 - Boost include file list
//----------------------------------------------------------------------------------

#ifndef BOOST_H_
#define BOOST_H_

#include "F28377S_IO_assignment.h"
#include "Boost-Settings.h"
/*-------------------------------------------------------------------------------
Include project specific include files.
-------------------------------------------------------------------------------*/
// define math type as float(1)
#define   MATH_TYPE      1
#include "IQmathLib.h"
#include "F28x_Project.h"
#include "rectifierVars.h"
#include "configuration.h"
#include <math.h>

#include "DLOG_4CH_F.h"



#endif /* BOOST_H_ */
