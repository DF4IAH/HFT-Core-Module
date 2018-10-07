/*
 * task_Si5338_RegMap_TCXO.h
 *
 *  Created on: 22.09.2018
 *      Author: DF4IAH
 */

#ifndef SI5338_TCXO_H_
#define SI5338_TCXO_H_

#include "task_Si5338.h"


//Register map for use with AN428 (JumpStart)
//http://www.silabs.com/clocks 
//Copyright 2012 Silicon Laboratories
//#BEGIN_HEADER
//Date = Sunday, September 02, 2018 8:11 PM
//File version = 3
//Software Name = ClockBuilder Desktop
//Software version = 6.5
//Software date = June 4, 2015
//Chip = Si533x
//Part Number = Si533x
//#END_HEADER
//Input Frequency (MHz) = 20.000000000
//Input Type = LVDS_LVPECL_HCSL
//P1 = 1
//Input Mux = FbClk
//FDBK Input Frequency (MHz) = 20.000000000
//FDBK Input Type = LVDS_LVPECL_HCSL
//P2 = 1
//FDBK Mux = NoClk
//PFD Input Frequency (MHz) = 20.000000000
//VCO Frequency (GHz) = 2.500000
//N = 125  (125.0000)
//Internal feedback enabled
//Output Clock 0
// Output is off
//Output Clock 1
// Output is off
//Output Clock 2
// Output is off
//Output Clock 3
// Output Frequency (MHz) = 10.000000000
// Mux Selection = IDn
// MultiSynth = 250  (250.0000)
// R = 1
//Driver 0
// Disabled
// Powered off
// Output voltage = 3.30
// Output type = 3.3V LVDS
// Output state when disabled = StopLow
//Driver 1
// Disabled
// Powered off
// Output voltage = 3.30
// Output type = 3.3V LVDS
// Output state when disabled = StopLow
//Driver 2
// Disabled
// Powered off
// Output voltage = 3.30
// Output type = 3.3V LVDS
// Output state when disabled = StopLow
//Driver 3
// Enabled
// Powered on
// Output voltage = 3.30
// Output type = 3.3V CMOS on A and B
// Output state when disabled = StopLow
//Clock 0 phase inc/dec step size (ns) = 0.000
//Clock 1 phase inc/dec step size (ns) = 0.000
//Clock 2 phase inc/dec step size (ns) = 0.000
//Clock 3 phase inc/dec step size (ns) = 0.000
//Phase increment and decrement pin control is off
//Frequency increment and decrement pin control is off
//Frequency increment and decrement is disabled
//Initial phase offset 0 (ns) = 0.000
//Initial phase offset 1 (ns) = 0.000
//Initial phase offset 2 (ns) = 0.000
//Initial phase offset 3 (ns) = 0.000
//SSC is disabled

#define SI5338_TCXO_NUM_REGS_MAX 350


#endif /* SI5338_TCXO_H_ */
