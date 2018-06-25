#ifndef _ANTENNA_H_
#define _ANTENNA_H_

//===============================================================

#include <msp430.h> 			// prozessor spezific header

//===============================================================

// V1 ---> P2.2
// V2 ---> P2.3
#define ANTENNA_SELECT_PORT_SET   P2DIR |= (BIT2 + BIT3);
#define ANTENNA_SELECT_ANT1       P2OUT &= ~(BIT2 + BIT3); // 0 + 0
#define ANTENNA_SELECT_ANT2       P2OUT &= ~BIT2; P2OUT |= BIT3; // 0 + 1
#define ANTENNA_SELECT_ANT3       P2OUT |= BIT2; P2OUT &= ~BIT3; // 1 + 0
#define ANTENNA_SELECT_ANT4       P2OUT |= BIT2; P2OUT |= BIT3; // 1 + 1

#endif

