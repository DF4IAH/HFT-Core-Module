/*
 * tcxo_20MHz.h
 *
 *  Created on: 30.08.2018
 *      Author: DF4IAH
 */

#ifndef TCXO_20MHZ_H_
#define TCXO_20MHZ_H_

#define TCXO_DAC_16BIT_3V3_PULL_DEFAULT_VALUE                 36200U


void tcxo20MhzTaskInit(void);
void tcxo20MhzTaskLoop(void);

#endif /* TCXO_20MHZ_H_ */
