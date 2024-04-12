/*
 * NRF_Private.h
 *
 *  Created on: Mar 19, 2024
 *      Author: Mohammed Gaafar
 */

#ifndef ECUAL_NRF24_NRF_PRIVATE_H_
#define ECUAL_NRF24_NRF_PRIVATE_H_

typedef enum{
	REG_CONFIG 		  = 0x00,
	REG_EN_AA         = 0x01	,
	REG_EN_RXADDR	  = 0x02 ,
	REG_SETUP_AW      = 0x03 ,
	REG_SETUP_RETR    = 0x04 ,
	REG_RF_CH         = 0x05 ,
	REG_RF_SETUP      = 0x06 ,
	REG_STATUS        = 0x07 ,
	REG_OBSERVE_TX    = 0x08 ,
	REG_CD            = 0x09 ,
	REG_RX_ADDR_P0    = 0x0A ,
	REG_RX_ADDR_P1    = 0x0B ,
	REG_RX_ADDR_P2    = 0x0C ,
	REG_RX_ADDR_P3    = 0x0D ,
	REG_RX_ADDR_P4    = 0x0E ,
	REG_RX_ADDR_P5    = 0x0F ,
	REG_TX_ADDR       = 0x10 ,
	REG_RX_PW_P0      = 0x11 ,
	REG_RX_PW_P1      = 0x12 ,
	REG_RX_PW_P2      = 0x13 ,
	REG_RX_PW_P3      = 0x14 ,
	REG_RX_PW_P4      = 0x15 ,
	REG_RX_PW_P5      = 0x16 ,
	REG_FIFO_STATUS   = 0x17 ,
	REG_DYNPD         = 0x1C ,
	REG_FEATURE       = 0x1D
}NRF24L01_Registers_t;

/* Bit Mnemonics */
typedef enum {
    MASK_RX_DR = 6,
    MASK_TX_DS = 5,
    MASK_MAX_RT = 4,
    BIT_EN_CRC = 3,
    BIT_CRCO = 2,
    BIT_PWR_UP = 1,
    BIT_PRIM_RX = 0,
    BIT_ENAA_P5 = 5,
    BIT_ENAA_P4 = 4,
    BIT_ENAA_P3 = 3,
    BIT_ENAA_P2 = 2,
    BIT_ENAA_P1 = 1,
    BIT_ENAA_P0 = 0,
    BIT_ERX_P5 = 5,
    BIT_ERX_P4 = 4,
    BIT_ERX_P3 = 3,
    BIT_ERX_P2 = 2,
    BIT_ERX_P1 = 1,
    BIT_ERX_P0 = 0,
    BIT_AW = 0,
    BIT_ARD = 4,
    BIT_ARC = 0,
    BIT_PLL_LOCK = 4,
    BIT_RF_DR = 3,
    BIT_RF_PWR = 6,
    BIT_RX_DR = 6,
    BIT_TX_DS = 5,
    BIT_MAX_RT = 4,
    BIT_RX_P_NO = 1,
    BIT_TX_FULL = 0,
    BIT_PLOS_CNT = 4,
    BIT_ARC_CNT = 0,
    BIT_TX_REUSE = 6,
    BIT_FIFO_FULL = 5,
    BIT_TX_EMPTY = 4,
    BIT_RX_FULL = 1,
    BIT_RX_EMPTY = 0,
    BIT_DPL_P5 = 5,
    BIT_DPL_P4 = 4,
    BIT_DPL_P3 = 3,
    BIT_DPL_P2 = 2,
    BIT_DPL_P1 = 1,
    BIT_DPL_P0 = 0,
    BIT_EN_DPL = 2,
    BIT_EN_ACK_PAY = 1,
    BIT_EN_DYN_ACK = 0
} NRF24L01_Bit_t;

typedef enum {
    CMD_R_REGISTER = 0x00,
    CMD_W_REGISTER = 0x20,
    CMD_REGISTER_MASK = 0x1F,
    CMD_ACTIVATE = 0x50,
    CMD_R_RX_PL_WID = 0x60,
    CMD_R_RX_PAYLOAD = 0x61,
    CMD_W_TX_PAYLOAD = 0xA0,
    CMD_W_ACK_PAYLOAD = 0xA8,
    CMD_FLUSH_TX = 0xE1,
    CMD_FLUSH_RX = 0xE2,
    CMD_REUSE_TX_PL = 0xE3,
    CMD_NOP = 0xFF
}NRF24L01_Command_t;


/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define REG_RPD         0x09

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2


#define NRF_CSN_LOW      0
#define NRF_CSN_HIGH     1

//1. Pinout Ports and Pin
#define NRF_PORT	GPIOA
#define NRF_CSN_PIN_Pin GPIO_PIN_3
#define NRF_CE_PIN_Pin GPIO_PIN_4



#endif /* ECUAL_NRF24_NRF_PRIVATE_H_ */
