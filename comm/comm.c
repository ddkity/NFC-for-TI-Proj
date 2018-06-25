#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "tag_read.h"
#include "comm.h"
#include "mcu.h"


#define COMM_SIMPLE_CIPHER

//高低八位组成0x03EA，转成十进制就是1002，上位机读出来的版本号就是V1.0.02
#define VERSION_MASTER 0x03		//高八位
#define VERSION_SLAVE 0xEA		//低八位
#define VERSION_SLAVE_NEW 0xEC		//低八位

//extern uint8_t g_can_sleep;
extern uint8_t g_nfc_running;

uint8_t send_buf[I2C_SEND_BUFF_LEN];
uint8_t send_idx = 0;
uint8_t recv_buf[I2C_RECV_BUFF_LEN];
uint8_t recv_idx = 0;

volatile uint8_t is_send = 0;
volatile uint8_t is_recv = 0;
//volatile uint8_t is_send_finish = 1;

static RECV_STATE_E s_recv_state = RECV_STATE_INIT;
static uint8_t s_data_len = 0;
static uint8_t s_cur_data_len = 0;

static FRAME_INFO_T s_frame_info = {ENCRYPTION_TYPE_NONE, 0, 0};

int parse_frame(FRAME_INFO_T *pinfo, const uint8_t *pframe, uint8_t len)
{
    uint8_t data_len;
    FRAME_CMD_T cmd;
    uint8_t cipher;
    uint8_t crpt_type;
    uint8_t cc = 0x00;

    if (len == 0) {
        return -1;
    }

    while (*pframe != I2C_FRAME_HEAD && len > 0) {
        pframe++;
        len--;
    }

    if (len > 0) {
        pframe++;
        data_len = *pframe;
        cc ^= *pframe++;
        len--;
    } else {
        return -1;
    }

    if (len >= data_len) {
        cmd.cmd = *pframe;
        cc ^= *pframe++;
        cipher = *pframe;
        cc ^= *pframe++;
        crpt_type = *pframe;
        cc ^= *pframe++;
    } else {
        return -1;
    }

    if (cc == *pframe) {
        pinfo->crpt_type = (ENCRYPTION_TYPE_E)crpt_type;
        pinfo->cipher = cipher;
        pinfo->tag_idx = cmd.cmd_t.tag_idx;
        return 0;

    } else {
        return -2;
    }

}

int form_frame(FRAME_INFO_T *pinfo, uint8_t *pframe)
{
    FRAME_CMD_T cmd = {0};
    uint8_t i;
    uint8_t index;
    uint8_t *temp = pframe;
    uint8_t cc = 0;
    uint8_t cipher;
    const TAG_INFO_T *ptaginfo;
    uint16_t year;

    *temp++ = I2C_FRAME_HEAD;
    *temp = TAG_INFO_DATA_LEN;
    cc ^= *temp++;
    cmd.cmd_t.direction = 1;            //NFC模组发出的应答信息
    cmd.cmd_t.tag_idx = pinfo->tag_idx;
    *temp = cmd.cmd;
    cc ^= *temp++;

    index = cmd.cmd_t.tag_idx - 1;      //单片机发下来的标签从1开始，本地取得的标签从0开始数

    //如果发现是一个版本查询命令，则只发回版本信息
	if(index == 4)
	{
		*temp = VERSION_MASTER;
		cc ^= *temp++;

		*temp = VERSION_SLAVE;
		cc ^= *temp++;

		*temp = 0x05;
		cc ^= *temp++;
                

		for(i = 0; i < 14; i++)
		{
			*temp = 0x00;
        	cc ^= *temp++;
		}

		*temp = cc;
		return 0;
	}

	//电流测试命令，I2C主设备发送电流测试命令，则模块退出低功耗模式进入测试模式，进入连续的寻卡模式，此时i2c主设备已经开始读取电流信息了，2s后模块自动退出测试模式
	if(index == 6)
	{
		CurrentTestMode = 1;	//标志着进入了测试模式，在定时中断中退出低功耗模式，并进入计时，计2s，在此期间主程序不进入低功耗模式，一直寻卡，最长4s后退出，最短1秒后退出

		*temp = 0x07;
		cc ^= *temp++;
		
		//buffer清零
		for(i = 0; i < 16; i++)
		{
			*temp = 0x00;
        	cc ^= *temp++;
		}
		*temp = cc;
		
		return 0;
	}

	if((index == 7) && (pinfo->cipher == 0xE0))
	{
		*temp = VERSION_MASTER;
		cc ^= *temp++;

		*temp = VERSION_SLAVE_NEW;
		cc ^= *temp++;

		*temp = 0x08;
		cc ^= *temp++;
		        

		for(i = 0; i < 14; i++)
		{
			*temp = 0x00;
			cc ^= *temp++;
		}

		*temp = cc;
		return 0;
	}
	
    ptaginfo = get_tag_info(index);

    if (ptaginfo == NULL) {
        return -1;
    }

    for (i = 0; i < 8; i++) {                   //UID
        *temp = ptaginfo->tag_uid[i];
        cc ^= *temp++;
    }

    for (i = 0; i < 4; i++) {                   //滤网序列号
        *temp = ptaginfo->filter_uid[i];
        cc ^= *temp++;
    }

    // year
    year = (uint16_t)(ptaginfo->year % 1000);           //取后三个数字
    *temp = (uint8_t)year;
    cc ^= *temp++;

    // month
    *temp = ptaginfo->month;
    cc ^= *temp++;

    // day
    *temp = ptaginfo->day;
    cc ^= *temp++;

    // manufacturer
    *temp = ptaginfo->manufacturer;
    cc ^= *temp++;

    // filter type
    *temp = ptaginfo->filter_type;
    cc ^= *temp++;

#ifdef COMM_SIMPLE_CIPHER
    cipher = pinfo->cipher;
#else
    switch (pinfo->crpt_type) {
    case ENCRYPTION_TYPE_XOR:
        cipher = pinfo->cipher ^ cc;
        break;
    case ENCRYPTION_TYPE_AND:
        cipher = pinfo->cipher & cc;
        break;
    case ENCRYPTION_TYPE_OR:
        cipher = pinfo->cipher | cc;
        break;
    default:
        cipher = pinfo->cipher;
        break;
    }
#endif

    temp = pframe + 2;

    // encrypt cmd & data domain, length not encrypt
    for (i = 0; i < TAG_INFO_DATA_LEN-1; i++) {
        *temp++ ^= cipher;
    }
    *temp = cc;

    return (TAG_INFO_DATA_LEN + 2);

}

void i2c_comm_init(void)
{
    //__disable_interrupt();
    IE2 &= ~UCB0TXIE;                         // Disable TX interrupt
    IE2 &= ~UCB0RXIE;                         // Disable RX interrupt

    P1SEL |= BIT6 + BIT7;					  // Assign I2C pins to USCI_B0
    P1SEL2 |= BIT6 + BIT7;					  // Assign I2C pins to USCI_B0

    UCB0CTL1 |= UCSWRST;					  // Enable SW reset
    UCB0CTL0 = UCMODE_3 + UCSYNC;			  // I2C Slave, synchronous mode
    UCB0I2COA = I2C_SLAVE_ADDR;				  // Own Address
    UCB0CTL1 &= ~UCSWRST;					  // Clear SW reset, resume operation
    UCB0I2CIE |= UCSTPIE + UCSTTIE;           // Enable STT and STP interrupt

    send_idx = 0;
    recv_idx = 0;

    IE2 |= UCB0TXIE + UCB0RXIE;               // Enable TX & RX interrupt
}

// USCI_B0 Data ISR
// TX & RX interrupt both in this vector
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
    if (IFG2 & UCB0TXIFG) {
        if (is_recv == 1) {
            //i2c_comm_init();
        }

        if(is_send == 0) {
            send_idx = 0;
            is_send = 1;
        }
        
        UCB0TXBUF = send_buf[send_idx++];      // Transmit data at address send_buf
        if (send_idx >= MAX_SEND_FRAME_LEN) {
            send_idx = 0;
            is_send = 0;
        }
    }
    else if(IFG2 & UCB0RXIFG) {

        if (is_send == 1) {
            //i2c_comm_init();
        }

        if (is_recv == 0) {
            recv_idx = 0;
            is_recv = 1;
        }
        
        recv_buf[recv_idx] = (uint8_t)UCB0RXBUF;    // Move RX data to address recv_buf
        if (g_nfc_running) {
            recv_idx = 0;
            is_recv = 0;
            send_idx = 0;
            is_send = 0;
			s_recv_state = RECV_STATE_INIT;
            UCB0CTL1 |= UCTXNACK; // if NFC running, send a NACK to notify master, then UCTXNACK auto reset.
			return;
		}
        switch(s_recv_state) {
        case RECV_STATE_INIT:
            if (recv_buf[recv_idx] == I2C_FRAME_HEAD) {
                s_recv_state = RECV_STATE_HEAD;
            }
            break;
        case RECV_STATE_HEAD:
            s_data_len = recv_buf[recv_idx];
            s_recv_state = RECV_STATE_LENGTH;
            break;
        case RECV_STATE_LENGTH:
            s_cur_data_len++;
            if (s_cur_data_len >= s_data_len) {
                s_recv_state = RECV_STATE_COMPLETE;
            }
            break;
        }

        recv_idx++;
        if (recv_idx >= I2C_RECV_BUFF_LEN) {
            recv_idx = 0;
        }

        // Received enough bytes to switch to TX?
        if (recv_idx >= MAX_RECV_FRAME_LEN || s_recv_state == RECV_STATE_COMPLETE) {
            s_recv_state = RECV_STATE_INIT;
            s_cur_data_len = 0;
            if (parse_frame(&s_frame_info, recv_buf, recv_idx) == 0) {
                form_frame(&s_frame_info, send_buf);
            } else {
                memset(send_buf, 0xF0, sizeof(send_buf));
            }
            
            recv_idx = 0;
            is_recv = 0;
        }

        
    }
}

// USCI_B0 State ISR
// start & stop condition
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
    if (UCB0STAT & UCSTTIFG) {
        /* USCI_B0 I2C Start Condition Interrupt Handler */
        //USCIAB0RX_ISR();
        //g_can_sleep = 0;
        //__bic_SR_register_on_exit(CPUOFF);
        UCB0STAT &= ~UCSTTIFG;
        send_idx = 0;
        recv_idx = 0;
        //is_send_finish = 0;

        s_cur_data_len = 0;

        /* No change in operating mode on exit */
    }
    else if (UCB0STAT & UCSTPIFG) {
        /* USCI_B0 I2C Stop Condition Interrupt Handler */
        //USCIAB0RX_ISR();
        //__bic_SR_register_on_exit(CPUOFF);
        UCB0STAT &= ~UCSTPIFG;
        //if (is_send_finish == 1) {
            //g_can_sleep = 1;
        //}
        //i2c_comm_init();

        //is_send = 0;
        //is_recv = 0;

        /* No change in operating mode on exit */
    }

}

