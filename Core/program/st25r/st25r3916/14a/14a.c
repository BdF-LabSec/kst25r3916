/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a.h"
#include <string.h>

uint8_t ST25R3916_14A_Anticoll(ST25R *pInstance, T4A_INFOS *infos)
{
	uint8_t ret;

	ret = ST25R3916_14A3_Anticoll(pInstance, &infos->t3a);
	if (ret == ST25R_STATUS_NO_ERROR)
	{
		if (infos->t3a.SAK & 0x20)
		{
			infos->CurrentBitrate = ST25R_BITRATE_106;
			ret = ST25R3916_14A4_Rats(pInstance, 0x50); // 64 b, CID 0
			if (ret == ST25R_STATUS_NO_ERROR)
			{
				if (pInstance->cbData <= sizeof(infos->ATS))
				{
					infos->cbATS = (uint8_t) pInstance->cbData;
					memcpy(infos->ATS, pInstance->pbData, pInstance->cbData);
					ST25R_14A4_AdjustMaxBitRate(infos);
				}
				else
				{
					ret = ST25T_STATUS_APPLICATION;
				}
			}
		}
	}

	return ret;
}

void ST25R3916_14A_Initiator(ST25R *pInstance)
{
	ST25R3916_Mask_IRQ(pInstance, /*ST25R3916_IRQ_MASK_TXE | */ST25R3916_IRQ_MASK_RXS, ST25R_IRQ_MASK_OP_ADD);
	ST25R3916_Write_SingleRegister(pInstance, ST25R3916_REG_MASK_RX_TIMER, 0x0e);
	//ST25R3916_Write_SingleRegister(pInstance, ST25R3916_REG_NO_RESPONSE_TIMER2, 0x23); //
	ST25R3916_Write_SingleRegister(pInstance, ST25R3916_REG_TIMER_EMV_CONTROL, /*ST25R3916_REG_TIMER_EMV_CONTROL_gptc1 | ST25R3916_REG_TIMER_EMV_CONTROL_gptc0| */ST25R3916_REG_TIMER_EMV_CONTROL_nrt_step_64fc);
	ST25R3916_Write_Registers2_sep(pInstance, ST25R3916_REG_ANT_TUNE_A,
			0x82,
			0x82
	);
	ST25R3916_Write_Registers2_sep(pInstance, ST25R3916_REG_FIELD_THRESHOLD_ACTV,
			ST25R3916_REG_FIELD_THRESHOLD_ACTV_trg_105mV | ST25R3916_REG_FIELD_THRESHOLD_ACTV_rfe_105mV,
			ST25R3916_REG_FIELD_THRESHOLD_DEACTV_trg_75mV | ST25R3916_REG_FIELD_THRESHOLD_DEACTV_rfe_75mV
	);
	ST25R3916_Write_SingleRegisterB(pInstance, ST25R3916_REG_B_FIELD_ON_GT, 0x40);//0x06); // TODO bi techno

	ST25R3916_14A4_TxRx106(pInstance);
}

//    /* Mode Name: POLL_A_ANTICOL, Mode ID: 0x0103 */
//        ST25R3916_REG_B_CORR_CONF1,0x40,0x00  /* User Defined ; Set collision detection level different from data */
void ST25R3916_14A4_TxRx106(ST25R *pInstance)
{
	ST25R3916_Write_Registers2_sep(pInstance, ST25R3916_REG_MODE,
			ST25R3916_REG_MODE_om_iso14443a | ST25R3916_REG_MODE_tr_am_ook,
			ST25R3916_REG_BIT_RATE_txrate_106 | ST25R3916_REG_BIT_RATE_rxrate_106
	);

	ST25R3916_Write_SingleRegister(pInstance, ST25R3916_REG_TX_DRIVER, ST25R3916_REG_TX_DRIVER_am_mod_12percent | 0);

	ST25R3916_Write_RegistersB4_sep(pInstance, ST25R3916_REG_B_OVERSHOOT_CONF1, \
			ST25R3916_REG_OVERSHOOT_CONF1_ov_tx_mode0,
			ST25R3916_REG_OVERSHOOT_CONF2_ov_pattern1 | ST25R3916_REG_OVERSHOOT_CONF2_ov_pattern0,
			ST25R3916_REG_UNDERSHOOT_CONF1_un_tx_mode0,
			ST25R3916_REG_UNDERSHOOT_CONF2_un_pattern1 | ST25R3916_REG_UNDERSHOOT_CONF2_un_pattern0
	);

	ST25R3916_Write_Registers4_sep(pInstance, ST25R3916_REG_RX_CONF1,
			ST25R3916_REG_RX_CONF1_hz_600_400khz,
			ST25R3916_REG_RX_CONF2_sqm_dyn | ST25R3916_REG_RX_CONF2_agc_en | ST25R3916_REG_RX_CONF2_agc_m | ST25R3916_REG_RX_CONF2_agc6_3,
			0,
			0
	);

	ST25R3916_Write_RegistersB2_sep(pInstance, ST25R3916_REG_B_CORR_CONF1, \
			ST25R3916_REG_CORR_CONF1_corr_s6 | ST25R3916_REG_CORR_CONF1_corr_s4 | ST25R3916_REG_CORR_CONF1_corr_s0,
			0
	);
}

void ST25R3916_14A4_TxRx212(ST25R *pInstance)
{
	ST25R3916_Write_Registers2_sep(pInstance, ST25R3916_REG_MODE,
			ST25R3916_REG_MODE_om_iso14443a | ST25R3916_REG_MODE_tr_am_am,
			ST25R3916_REG_BIT_RATE_txrate_212 | ST25R3916_REG_BIT_RATE_rxrate_212
	);

	ST25R3916_Write_SingleRegister(pInstance, ST25R3916_REG_TX_DRIVER, ST25R3916_REG_TX_DRIVER_am_mod_12percent | 0);
	ST25R3916_Write_SingleRegisterB(pInstance, ST25R3916_REG_B_AUX_MOD, ST25R3916_REG_AUX_MOD_dis_reg_am | ST25R3916_REG_AUX_MOD_lm_dri | ST25R3916_REG_AUX_MOD_res_am | 0);
	ST25R3916_Write_SingleRegisterB(pInstance, ST25R3916_REG_B_RES_AM_MOD, ST25R3916_REG_RES_AM_MOD_fa3_f | 0x7f);

	ST25R3916_Write_RegistersB4_sep(pInstance, ST25R3916_REG_B_OVERSHOOT_CONF1, \
			ST25R3916_REG_OVERSHOOT_CONF1_ov_tx_mode0,
			ST25R3916_REG_OVERSHOOT_CONF2_ov_pattern1 | ST25R3916_REG_OVERSHOOT_CONF2_ov_pattern0,
			ST25R3916_REG_UNDERSHOOT_CONF1_un_tx_mode0,
			ST25R3916_REG_UNDERSHOOT_CONF2_un_pattern1 | ST25R3916_REG_UNDERSHOOT_CONF2_un_pattern0
	);

	ST25R3916_Write_Registers4_sep(pInstance, ST25R3916_REG_RX_CONF1,
			ST25R3916_REG_RX_CONF1_hz_40_80khz,
			ST25R3916_REG_RX_CONF2_sqm_dyn | ST25R3916_REG_RX_CONF2_pulz_61 | ST25R3916_REG_RX_CONF2_agc_en | ST25R3916_REG_RX_CONF2_agc_m | ST25R3916_REG_RX_CONF2_agc6_3,
			0,
			0
	);

	ST25R3916_Write_RegistersB2_sep(pInstance, ST25R3916_REG_B_CORR_CONF1, \
			ST25R3916_REG_CORR_CONF1_corr_s4 | ST25R3916_REG_CORR_CONF1_corr_s2,
			0
	);
}

void ST25R3916_14A4_TxRx424(ST25R *pInstance)
{
	ST25R3916_Write_Registers2_sep(pInstance, ST25R3916_REG_MODE,
			ST25R3916_REG_MODE_om_iso14443a | ST25R3916_REG_MODE_tr_am_am,
			ST25R3916_REG_BIT_RATE_txrate_424 | ST25R3916_REG_BIT_RATE_rxrate_424
	);

	ST25R3916_Write_SingleRegister(pInstance, ST25R3916_REG_TX_DRIVER, ST25R3916_REG_TX_DRIVER_am_mod_12percent | 0);
	ST25R3916_Write_SingleRegisterB(pInstance, ST25R3916_REG_B_AUX_MOD, ST25R3916_REG_AUX_MOD_dis_reg_am | ST25R3916_REG_AUX_MOD_lm_dri | ST25R3916_REG_AUX_MOD_res_am | 0);
	ST25R3916_Write_SingleRegisterB(pInstance, ST25R3916_REG_B_RES_AM_MOD, ST25R3916_REG_RES_AM_MOD_fa3_f | 0x7f);

	ST25R3916_Write_RegistersB4_sep(pInstance, ST25R3916_REG_B_OVERSHOOT_CONF1, \
			ST25R3916_REG_OVERSHOOT_CONF1_ov_tx_mode0,
			ST25R3916_REG_OVERSHOOT_CONF2_ov_pattern1 | ST25R3916_REG_OVERSHOOT_CONF2_ov_pattern0,
			ST25R3916_REG_UNDERSHOOT_CONF1_un_tx_mode0,
			ST25R3916_REG_UNDERSHOOT_CONF2_un_pattern1 | ST25R3916_REG_UNDERSHOOT_CONF2_un_pattern0
	);

	ST25R3916_Write_Registers4_sep(pInstance, ST25R3916_REG_RX_CONF1,
			ST25R3916_REG_RX_CONF1_lp_2000khz | ST25R3916_REG_RX_CONF1_hz_40_80khz,
			ST25R3916_REG_RX_CONF2_sqm_dyn | ST25R3916_REG_RX_CONF2_pulz_61 | ST25R3916_REG_RX_CONF2_agc_en | ST25R3916_REG_RX_CONF2_agc_m | ST25R3916_REG_RX_CONF2_agc6_3,
			0,
			0
	);

	ST25R3916_Write_RegistersB2_sep(pInstance, ST25R3916_REG_B_CORR_CONF1, \
			ST25R3916_REG_CORR_CONF1_corr_s7 | ST25R3916_REG_CORR_CONF1_corr_s4 | ST25R3916_REG_CORR_CONF1_corr_s2,
			0
	);
}

void ST25R3916_14A4_TxRx848(ST25R *pInstance)
{
	ST25R3916_Write_Registers2_sep(pInstance, ST25R3916_REG_MODE,
			ST25R3916_REG_MODE_om_iso14443a | ST25R3916_REG_MODE_tr_am_am,
			ST25R3916_REG_BIT_RATE_txrate_848 | ST25R3916_REG_BIT_RATE_rxrate_848
	);

	ST25R3916_Write_SingleRegister(pInstance, ST25R3916_REG_TX_DRIVER, ST25R3916_REG_TX_DRIVER_am_mod_40percent | 0);
	ST25R3916_Write_SingleRegisterB(pInstance, ST25R3916_REG_B_AUX_MOD, ST25R3916_REG_AUX_MOD_lm_dri | 0);

	ST25R3916_Write_RegistersB4_sep(pInstance, ST25R3916_REG_B_OVERSHOOT_CONF1, \
			0,
			0,
			0,
			0
	);

	ST25R3916_Write_Registers4_sep(pInstance, ST25R3916_REG_RX_CONF1,
			ST25R3916_REG_RX_CONF1_lp_2000khz | ST25R3916_REG_RX_CONF1_hz_40_80khz,
			ST25R3916_REG_RX_CONF2_sqm_dyn | ST25R3916_REG_RX_CONF2_pulz_61 | ST25R3916_REG_RX_CONF2_agc_en | ST25R3916_REG_RX_CONF2_agc_m | ST25R3916_REG_RX_CONF2_agc6_3,
			0,
			0
	);

	ST25R3916_Write_RegistersB2_sep(pInstance, ST25R3916_REG_B_CORR_CONF1, \
			ST25R3916_REG_CORR_CONF1_corr_s6 | ST25R3916_REG_CORR_CONF1_corr_s2,
			0
	);
}
