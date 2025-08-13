/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a.h"

uint8_t ST25R3911B_14A_Anticoll(ST25R *pInstance, T4A_INFOS *infos)
{
	uint8_t ret;

	ret = ST25R3911B_14A3_Anticoll(pInstance, &infos->t3a);
	if (ret == ST25R_STATUS_NO_ERROR)
	{
		if (infos->t3a.SAK & 0x20)
		{
			infos->CurrentBitrate = ST25R_BITRATE_106;
			ret = ST25R3911B_14A4_Rats(pInstance, 0x50, infos); // 64 b, CID 0
		}
	}

	return ret;
}

void ST25R3911B_14A_Initiator(ST25R *pInstance)
{
	ST25R3911B_Mask_IRQ(pInstance, /*ST25R3911B_IRQ_MASK_TXE | */ST25R3911B_IRQ_MASK_RXS, ST25R_IRQ_MASK_OP_ADD);
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_MASK_RX_TIMER, 0x0e);

	ST25R3911B_Write_Registers2_sep(pInstance, ST25R3911B_REG_ANT_CAL_CONTROL,
			0x00,
			0x80
	);
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_FIELD_THRESHOLD, ST25R3911B_REG_FIELD_THRESHOLD_trg_75mV | ST25R3911B_REG_FIELD_THRESHOLD_rfe_75mV);

	ST25R3911B_Write_Registers4_sep(pInstance, ST25R3911B_REG_RX_CONF1,
			0,
			ST25R3911B_REG_RX_CONF2_sqm_dyn | ST25R3911B_REG_RX_CONF2_agc_en | ST25R3911B_REG_RX_CONF2_agc_m,
			(0 << ST25R3911B_REG_RX_CONF3_shift_rg1_am) | (6 << ST25R3911B_REG_RX_CONF3_shift_rg1_pm),
			(2 << ST25R3911B_REG_RX_CONF4_shift_rg2_am) | (1 << ST25R3911B_REG_RX_CONF4_shift_rg2_pm)
	);

	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_MODE, ST25R3911B_REG_MODE_targ_init | ST25R3911B_REG_MODE_om_iso14443a);
	ST25R3911B_14A_TxRx106(pInstance);
}

void ST25R3911B_14A_TxRx106(ST25R *pInstance)
{
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_BIT_RATE, ST25R3911B_REG_BIT_RATE_txrate_106 | ST25R3911B_REG_BIT_RATE_rxrate_106);
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_AUX, ST25R3911B_REG_AUX_rx_tol | ST25R3911B_REG_AUX_crc_2_fifo);
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_RX_CONF1, 0);
}

void ST25R3911B_14A_TxRx212(ST25R *pInstance)
{
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_BIT_RATE, ST25R3911B_REG_BIT_RATE_txrate_212 | ST25R3911B_REG_BIT_RATE_rxrate_212);
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_AUX, ST25R3911B_REG_AUX_rx_tol | ST25R3911B_REG_AUX_crc_2_fifo);
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_RX_CONF1, ST25R3911B_REG_RX_CONF1_h200);
}

void ST25R3911B_14A_TxRx424(ST25R *pInstance)
{
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_BIT_RATE, ST25R3911B_REG_BIT_RATE_txrate_424 | ST25R3911B_REG_BIT_RATE_rxrate_424);
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_AUX, ST25R3911B_REG_AUX_rx_tol | ST25R3911B_REG_AUX_crc_2_fifo);
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_RX_CONF1, ST25R3911B_REG_RX_CONF1_h80 | ST25R3911B_REG_RX_CONF1_lp_2000khz);
}

void ST25R3911B_14A_TxRx848(ST25R *pInstance)
{
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_BIT_RATE, ST25R3911B_REG_BIT_RATE_txrate_848 | ST25R3911B_REG_BIT_RATE_rxrate_848);
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_AUX, ST25R3911B_REG_AUX_rx_tol | ST25R3911B_REG_AUX_crc_2_fifo | ST25R3911B_REG_AUX_tr_am);
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_RX_CONF1, ST25R3911B_REG_RX_CONF1_h80 | ST25R3911B_REG_RX_CONF1_lp_2000khz);

	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_AM_MOD_DEPTH_CONTROL, ST25R3911B_REG_AM_MOD_DEPTH_CONTROL_am_s);
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_RFO_AM_ON_LEVEL, 0xf0);
}
