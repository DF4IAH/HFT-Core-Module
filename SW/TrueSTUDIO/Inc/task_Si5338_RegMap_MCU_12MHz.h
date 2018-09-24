/*
 * task_Si5338_RegMap_MCU_12MHz.h
 *
 *  Created on: 22.09.2018
 *      Author: DF4IAH
 */

#ifndef SI5338_MCO_12MHZ_H_
#define SI5338_MCO_12MHZ_H_

#include "task_Si5338.h"


//Register map for use with AN428 (JumpStart)
//http://www.silabs.com/clocks 
//Copyright 2012 Silicon Laboratories
//#BEGIN_HEADER
//Date = Saturday, September 22, 2018 11:47 PM
//File version = 3
//Software Name = ClockBuilder Desktop
//Software version = 6.5
//Software date = June 4, 2015
//Chip = Si533x
//Part Number = Si533x
//#END_HEADER
//Input Frequency (MHz) = 12.000000000
//Input Type = CMOS_SSTL_HSTL
//P1 = 1
//Input Mux = RefClk
//FDBK Input Frequency (MHz) = 12.000000000
//FDBK Input Type = OFF
//P2 = 1
//FDBK Mux = NoClk
//PFD Input Frequency (MHz) = 12.000000000
//VCO Frequency (GHz) = 2.500000
//N = 208  1/3  (208.3333)
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

#define SI5338_MCU_12MHZ_NUM_REGS_MAX 350


#endif /* SI5338_MCO_12MHZ_H_ */
