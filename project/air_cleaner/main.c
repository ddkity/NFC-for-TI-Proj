/*
 * File Name: main.c
 *
 * Description: The TRF7970A is an integrated analog front end and
 * data framing system for a 13.56 MHz RFID reader system.
 * Built-in programming options make it suitable for a wide range
 * of applications both in proximity and vicinity RFID systems.
 * The reader is configured by selecting the desired protocol in
 * the control registers. Direct access to all control registers
 * allows fine tuning of various reader parameters as needed.
 *
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*
* DESCRIPTION:
* This example detects ISO15693, Type 2, Type 3, Type 4A, Type 4B
* NFC/RFID tags. It then indicates the Tag type through LED's on the
* TRF7970A Booster pack. Information such as tag UID's and block data is
* sent out via a UART at 115200 Baud and can be read on a Computer.
*
* The TRF7970A is an integrated analog front end and
* data framing system for a 13.56 MHz RFID reader system.
* Built-in programming options make it suitable for a wide range
* of applications both in proximity and vicinity RFID systems.
* The reader is configured by selecting the desired protocol in
* the control registers. Direct access to all control registers
* allows fine tuning of various reader parameters as needed.
*
* The TRF7970A is interfaced to a MSP430G2553 through a SPI (serial)
* interface using a hardware USCI. The MCU is the master device and
* initiates all communication with the reader.
*
* The anti-collision procedures (as described in the ISO
* standards 14443A/B and 15693) are implemented in the MCU
* firmware to help the reader detect and communicate with one
* PICC/VICC among several PICCs/VICCs.
*
* AUTHORS:   	Josh Wyatt
* 				Ralph Jacobi
*
* BUILT WITH:
* Code Composer Studio Core Edition Version: 6.0.1.00040
* (c) Copyright Texas Instruments, 2014. All rights reserved.
*****************************************************************/

//===============================================================
// Program with hardware I2C and SPI communication	            ;
// interface with TRF7963A reader chip.                         ;
//                                                              ;
// PORT1.0 - SLAVE SELECT                                       ;
// PORT1.1 - SPI MISO                                           ;
// PORT1.2 - SPI MOSI                                           ;
// PORT1.3 - MOD                                                ;
// PORT1.4 - SPI DATACLK                                        ;
// PORT1.6 - I2C SCL                                            ;
// PORT1.7 - I2C SDA                                            ;
//                                                              ;
// PORT2.0 - IRQ (INTERUPT REQUEST from TRF7963A) (XOUT on LP)  ;
// PORT2.1 - ASK/OOK                                            ;
// PORT2.2 - V1 (ant select)                                    ;
// PORT2.3 - V2 (ant select)                                    ;
// PORT2.4 - LED1                                               ;
// PORT2.6 - SYS_CLK                                            ;
// PORT2.7 - EN_RF                                              ;
//===============================================================

#include "nfc_app.h"
#include "trf79xxa.h"
#include "mcu.h"
#include "comm.h"
#include "antenna.h"
#include "tag_read.h"

//===============================================================

typedef union __TAG_POLL_RES {
	uint16_t result;
	struct {
		uint16_t tag1:4;
		uint16_t tag2:4;
		uint16_t tag3:4;
		uint16_t tag4:4;
	}RES_T;
}TAG_POLL_RES_T;

//===============================================================

static uint8_t s_ant_select = 0;
static uint8_t s_poll_count = 0;
static uint8_t g_mcu_reset_flag = 1;

//uint8_t g_can_sleep = 0;
uint8_t g_nfc_running = 1;
TAG_POLL_RES_T g_poll_result = {0};

//===============================================================
void softMcuResetIfNeeded(uint8_t site)
{
	static uint16_t count=0;
	if(++count > 500) 
	{    
		count = 0;    
		if(g_mcu_reset_flag==0)    
		{
			WDTCTL = 0;
		}    
		g_mcu_reset_flag = 0;  
	}
}

void main(void)
{
#ifdef TIMER_SOURCE_USER_VLO
	uint8_t ui8VLOCalibCount;
#endif
	uint8_t ret;
	LED_PORT_SET;
	LED_14443A_ON;
        LED_14443A_OFF;
 	WDTCTL = WDT_MRST_32;
	// Select DCO to be 8 MHz
 	MCU_initClock();
	// Stop the Watchdog timer
 	WDTCTL = WDTPW + WDTHOLD; 
	MCU_delayMillisecond(10);

 	// Calibrate VLO
#ifdef TIMER_SOURCE_USER_VLO
 	MCU_calculateVLOFreq();
#endif

	// Set the SPI SS high
	SLAVE_SELECT_PORT_SET;
	SLAVE_SELECT_HIGH;

	// Four millisecond delay between bringing SS high and then EN high per TRF7970A Datasheet
	MCU_delayMillisecond(4);

	// Set TRF Enable Pin high
	TRF_ENABLE_SET;
	TRF_ENABLE;

	// Wait until TRF system clock started
	MCU_delayMillisecond(5);

	// Set up TRF initial settings
	TRF79xxA_initialSettings();
	TRF79xxA_setTrfPowerSetting(TRF79xxA_5V_FULL_POWER);

#ifdef ENABLE_HOST
	// Set up UART
	UART_setup();
#endif

	// Initialize all enabled technology layers
	NFC_init();

	// Enable global interrupts
	__bis_SR_register(GIE);

	// Enable IRQ Pin
	IRQ_ON;

#ifdef ENABLE_HOST
	UART_putIntroReaderMsg(RFID_READER_FW_VERSION, RFID_READER_FW_DATE);
#endif

	// for i2c comm
	i2c_comm_init();
        
	ANTENNA_SELECT_PORT_SET;
	while(1)
	{
		g_mcu_reset_flag = 1;    
		if((g_poll_result.result == 0) && (CurrentTestMode == 0))
	    {
	      clear_tag_info(0);
	      clear_tag_info(1);
	      clear_tag_info(2);
	      clear_tag_info(3);    
	    }
		
		// switch ANT select
		set_cur_tag_id(s_ant_select);
		switch(s_ant_select) {
		case 0:
			ANTENNA_SELECT_ANT1;
			break;
		case 1:
			ANTENNA_SELECT_ANT2;
			break;
		case 2:
			ANTENNA_SELECT_ANT3;
			break;
		case 3:
			ANTENNA_SELECT_ANT4;
			break;
		}
        
		// Poll for NFC tags
		ret = NFC_findTag();

		if (ret != 0) {
			g_poll_result.result |= ((1 << s_ant_select*4) << s_poll_count);
		}

		s_ant_select++;

		if (s_ant_select >= 4)
		{
			s_ant_select = 0;
			s_poll_count++;

			if (s_poll_count >= 4) {
				s_poll_count = 0;
				//TRF_DISABLE;
				// Enter LPM0 w/ interrupts
				// Remain in LPM0 until receive I2C UCSTTIFG
				//while(!g_can_sleep);
				if (g_poll_result.RES_T.tag1 == 0) {
					clear_tag_info(0);
				}
				if (g_poll_result.RES_T.tag2 == 0) {
					clear_tag_info(1);
				}
				if (g_poll_result.RES_T.tag3 == 0) {
					clear_tag_info(2);
				}
				if (g_poll_result.RES_T.tag4 == 0) {
					clear_tag_info(3);
				}
				g_poll_result.result = 0;
				
				g_nfc_running = 0;
				if(CurrentTestMode == 0){	//非电流测试模式下才会进入低功耗模式
					__bis_SR_register(CPUOFF + GIE);
					g_nfc_running = 1;		//g_nfc_running等于1表示I2C处于忙状态，其它的I2C主设备无法读取
				}
				//进入电流测试模式
				else{
					CurrentTestMode++;
					if(CurrentTestMode >= 3)	//大概1s到4秒之间，无卡就是1秒，4张卡就是4s
					{
						CurrentTestMode = 0;	//退出电流测试模式
					}
				}
				//TRF_ENABLE;                         // EN pin on the TRF796x

			}
		}

#ifdef TIMER_SOURCE_USER_VLO
		// VLO drifts with temperature and over time, so it must be periodically recalibrated
		// Calibrate the VLO every 25 passes of the NFC polling routine
		ui8VLOCalibCount++;
		if (ui8VLOCalibCount == 25)
		{
			// Calibrate VLO
			MCU_calculateVLOFreq();
			// Reset Calibration Counter
		 	ui8VLOCalibCount = 0;
		}
#endif
	}
}

