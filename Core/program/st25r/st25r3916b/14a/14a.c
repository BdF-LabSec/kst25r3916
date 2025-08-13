/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a.h"

uint8_t ST25R3916B_14A_Anticoll(ST25R *pInstance, T4A_INFOS *infos)
{
	uint8_t ret;

	ret = ST25R3916B_14A3_Anticoll(pInstance, &infos->t3a);
	if (ret == ST25R_STATUS_NO_ERROR)
	{
		if (infos->t3a.SAK & 0x20)
		{
			infos->CurrentBitrate = ST25R_BITRATE_106;
			ret = ST25R3916B_14A4_Rats(pInstance, 0x50, infos); // 64 b, CID 0
		}
	}

	return ret;
}

void ST25R3916B_14A_Initiator(ST25R *pInstance)
{
	ST25R3916B_Mask_IRQ(pInstance, /*ST25R3916B_IRQ_MASK_TXE | */ST25R3916B_IRQ_MASK_RXS, ST25R_IRQ_MASK_OP_ADD);
	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_MASK_RX_TIMER, 0x0e);

	ST25R3916B_Write_Registers2_sep(pInstance, ST25R3916B_REG_ANT_TUNE_A,
			0x40,
			0x58
	);
	ST25R3916B_Write_Registers2_sep(pInstance, ST25R3916B_REG_FIELD_THRESHOLD_ACTV,
			ST25R3916B_REG_FIELD_THRESHOLD_ACTV_trg_105mV | ST25R3916B_REG_FIELD_THRESHOLD_ACTV_rfe_105mV,
			ST25R3916B_REG_FIELD_THRESHOLD_DEACTV_trg_75mV | ST25R3916B_REG_FIELD_THRESHOLD_DEACTV_rfe_75mV
	);
	ST25R3916B_Write_SingleRegisterB(pInstance, ST25R3916B_REG_B_FIELD_ON_GT, 0x40);//0x06); // TODO bi techno

	ST25R3916B_Write_SingleRegisterB(pInstance, ST25R3916B_REG_B_AUX_MOD, ST25R3916B_REG_AUX_MOD_dis_reg_am | ST25R3916B_REG_AUX_MOD_lm_dri | ST25R3916B_REG_AUX_MOD_rgs_am | 0);

	ST25R3916B_14A4_TxRx106(pInstance);
}

void ST25R3916B_14A4_TxRx106(ST25R *pInstance)
{
	ST25R3916B_Write_Registers2_sep(pInstance, ST25R3916B_REG_MODE,
			ST25R3916B_REG_MODE_om_iso14443a | ST25R3916B_REG_MODE_tr_am_ook,
			ST25R3916B_REG_BIT_RATE_txrate_106 | ST25R3916B_REG_BIT_RATE_rxrate_106
	);

	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_TX_DRIVER, ST25R3916B_REG_TX_DRIVER_am_mod_82percent | 0);

	ST25R3916B_Write_RegistersB2_sep(pInstance, ST25R3916B_REG_B_AWS_CONF1, \
			ST25R3916B_REG_AWS_CONF1_rgs_txonoff | ST25R3916B_REG_AWS_CONF1_vddrf_cont,
			ST25R3916B_REG_AWS_CONF2_en_modsink | (8 << ST25R3916B_REG_AWS_CONF2_am_filt_shift)
	);
	ST25R3916B_Write_RegistersB4_sep(pInstance, ST25R3916B_REG_B_AWS_TIME1, \
			(0 << ST25R3916B_REG_AWS_TIME1_tmoddx1_shift) | (1 << ST25R3916B_REG_AWS_TIME1_tmodsw1_shift),
			(0 << ST25R3916B_REG_AWS_TIME2_tammod1_shift) | (0 << ST25R3916B_REG_AWS_TIME2_tdres1_shift),
			(7 << ST25R3916B_REG_AWS_TIME3_tentx1_shift)  | (9 << ST25R3916B_REG_AWS_TIME3_tmods2_shift),
			(0 << ST25R3916B_REG_AWS_TIME4_tholdx2_shift) | (7 << ST25R3916B_REG_AWS_TIME4_tmodsw2_shift)
			// 5 and 6 not touched
	);

	ST25R3916B_Write_Registers4_sep(pInstance, ST25R3916B_REG_RX_CONF1,
			ST25R3916B_REG_RX_CONF1_hz_600_400khz,
			ST25R3916B_REG_RX_CONF2_demod_mode | ST25R3916B_REG_RX_CONF2_amd_sel_mixer | ST25R3916B_REG_RX_CONF2_sqm_dyn | ST25R3916B_REG_RX_CONF2_agc_en | ST25R3916B_REG_RX_CONF2_agc_m | ST25R3916B_REG_RX_CONF2_agc6_3,
			0,
			0
	);

	ST25R3916B_Write_RegistersB2_sep(pInstance, ST25R3916B_REG_B_CORR_CONF1, \
			ST25R3916B_REG_CORR_CONF1_corr_s6 | ST25R3916B_REG_CORR_CONF1_corr_s4 | ST25R3916B_REG_CORR_CONF1_corr_s0,
			0
	);
}

void ST25R3916B_14A4_TxRx212(ST25R *pInstance)
{
	ST25R3916B_Write_Registers2_sep(pInstance, ST25R3916B_REG_MODE,
			ST25R3916B_REG_MODE_om_iso14443a | ST25R3916B_REG_MODE_tr_am_am, // TODO check
			ST25R3916B_REG_BIT_RATE_txrate_212 | ST25R3916B_REG_BIT_RATE_rxrate_212
	);

	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_TX_DRIVER, ST25R3916B_REG_TX_DRIVER_am_mod_82percent | 0);

	ST25R3916B_Write_RegistersB2_sep(pInstance, ST25R3916B_REG_B_AWS_CONF1, \
			ST25R3916B_REG_AWS_CONF1_rgs_txonoff | ST25R3916B_REG_AWS_CONF1_vddrf_cont,
			ST25R3916B_REG_AWS_CONF2_en_modsink | (8 << ST25R3916B_REG_AWS_CONF2_am_filt_shift)
	);
	ST25R3916B_Write_RegistersB4_sep(pInstance, ST25R3916B_REG_B_AWS_TIME1, \
			(0 << ST25R3916B_REG_AWS_TIME1_tmoddx1_shift) | (1 << ST25R3916B_REG_AWS_TIME1_tmodsw1_shift),
			(0 << ST25R3916B_REG_AWS_TIME2_tammod1_shift) | (0 << ST25R3916B_REG_AWS_TIME2_tdres1_shift),
			(7 << ST25R3916B_REG_AWS_TIME3_tentx1_shift)  | (9 << ST25R3916B_REG_AWS_TIME3_tmods2_shift),
			(0 << ST25R3916B_REG_AWS_TIME4_tholdx2_shift) | (7 << ST25R3916B_REG_AWS_TIME4_tmodsw2_shift)
			// 5 and 6 not touched
	);

	ST25R3916B_Write_Registers4_sep(pInstance, ST25R3916B_REG_RX_CONF1,
			ST25R3916B_REG_RX_CONF1_hz_40_80khz,
			ST25R3916B_REG_RX_CONF2_demod_mode | ST25R3916B_REG_RX_CONF2_amd_sel_mixer | ST25R3916B_REG_RX_CONF2_sqm_dyn | ST25R3916B_REG_RX_CONF2_pulz_61 | ST25R3916B_REG_RX_CONF2_agc_en | ST25R3916B_REG_RX_CONF2_agc_m | ST25R3916B_REG_RX_CONF2_agc6_3,
			0,
			0
	);

	ST25R3916B_Write_RegistersB2_sep(pInstance, ST25R3916B_REG_B_CORR_CONF1, \
			ST25R3916B_REG_CORR_CONF1_corr_s7 | ST25R3916B_REG_CORR_CONF1_corr_s4 | ST25R3916B_REG_CORR_CONF1_corr_s2 | ST25R3916B_REG_CORR_CONF1_corr_s1 | ST25R3916B_REG_CORR_CONF1_corr_s0,
			0
	);
}

void ST25R3916B_14A4_TxRx424(ST25R *pInstance)
{
	ST25R3916B_Write_Registers2_sep(pInstance, ST25R3916B_REG_MODE,
			ST25R3916B_REG_MODE_om_iso14443a | ST25R3916B_REG_MODE_tr_am_am, // TODO check
			ST25R3916B_REG_BIT_RATE_txrate_424 | ST25R3916B_REG_BIT_RATE_rxrate_424
	);

	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_TX_DRIVER, ST25R3916B_REG_TX_DRIVER_am_mod_82percent | 0);

	ST25R3916B_Write_RegistersB2_sep(pInstance, ST25R3916B_REG_B_AWS_CONF1, \
			ST25R3916B_REG_AWS_CONF1_rgs_txonoff | ST25R3916B_REG_AWS_CONF1_vddrf_cont,
			ST25R3916B_REG_AWS_CONF2_en_modsink | (8 << ST25R3916B_REG_AWS_CONF2_am_filt_shift)
	);
	ST25R3916B_Write_RegistersB4_sep(pInstance, ST25R3916B_REG_B_AWS_TIME1, \
			(0 << ST25R3916B_REG_AWS_TIME1_tmoddx1_shift) | (1 << ST25R3916B_REG_AWS_TIME1_tmodsw1_shift),
			(0 << ST25R3916B_REG_AWS_TIME2_tammod1_shift) | (0 << ST25R3916B_REG_AWS_TIME2_tdres1_shift),
			(7 << ST25R3916B_REG_AWS_TIME3_tentx1_shift)  | (9 << ST25R3916B_REG_AWS_TIME3_tmods2_shift),
			(0 << ST25R3916B_REG_AWS_TIME4_tholdx2_shift) | (7 << ST25R3916B_REG_AWS_TIME4_tmodsw2_shift)
			// 5 and 6 not touched
	);

	ST25R3916B_Write_Registers4_sep(pInstance, ST25R3916B_REG_RX_CONF1,
			ST25R3916B_REG_RX_CONF1_lp_2000khz | ST25R3916B_REG_RX_CONF1_hz_40_80khz,
			ST25R3916B_REG_RX_CONF2_demod_mode | ST25R3916B_REG_RX_CONF2_amd_sel_mixer | ST25R3916B_REG_RX_CONF2_sqm_dyn | ST25R3916B_REG_RX_CONF2_pulz_61 | ST25R3916B_REG_RX_CONF2_agc_en | ST25R3916B_REG_RX_CONF2_agc_m | ST25R3916B_REG_RX_CONF2_agc6_3,
			0,
			0
	);

	ST25R3916B_Write_RegistersB2_sep(pInstance, ST25R3916B_REG_B_CORR_CONF1, \
			ST25R3916B_REG_CORR_CONF1_corr_s7 | ST25R3916B_REG_CORR_CONF1_corr_s6 | ST25R3916B_REG_CORR_CONF1_corr_s4 | ST25R3916B_REG_CORR_CONF1_corr_s2 | ST25R3916B_REG_CORR_CONF1_corr_s1 | ST25R3916B_REG_CORR_CONF1_corr_s0,
			0
	);
}

void ST25R3916B_14A4_TxRx848(ST25R *pInstance)
{
	ST25R3916B_Write_Registers2_sep(pInstance, ST25R3916B_REG_MODE,
			ST25R3916B_REG_MODE_om_iso14443a | ST25R3916B_REG_MODE_tr_am_am,
			ST25R3916B_REG_BIT_RATE_txrate_848 | ST25R3916B_REG_BIT_RATE_rxrate_848
	);

	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_TX_DRIVER, ST25R3916B_REG_TX_DRIVER_am_mod_60percent | 0);

	ST25R3916B_Write_RegistersB2_sep(pInstance, ST25R3916B_REG_B_AWS_CONF1, \
			ST25R3916B_REG_AWS_CONF1_rgs_txonoff | ST25R3916B_REG_AWS_CONF1_vddrf_cont,
			ST25R3916B_REG_AWS_CONF2_en_modsink | (0 << ST25R3916B_REG_AWS_CONF2_am_filt_shift)
	);
	ST25R3916B_Write_RegistersB4_sep(pInstance, ST25R3916B_REG_B_AWS_TIME1, \
			(0 << ST25R3916B_REG_AWS_TIME1_tmoddx1_shift) | (1 << ST25R3916B_REG_AWS_TIME1_tmodsw1_shift),
			(0 << ST25R3916B_REG_AWS_TIME2_tammod1_shift) | (0 << ST25R3916B_REG_AWS_TIME2_tdres1_shift),
			(3 << ST25R3916B_REG_AWS_TIME3_tentx1_shift)  | (0 << ST25R3916B_REG_AWS_TIME3_tmods2_shift),
			(0 << ST25R3916B_REG_AWS_TIME4_tholdx2_shift) | (2 << ST25R3916B_REG_AWS_TIME4_tmodsw2_shift)
			// 5 and 6 not touched
	);

	ST25R3916B_Write_Registers4_sep(pInstance, ST25R3916B_REG_RX_CONF1,
			ST25R3916B_REG_RX_CONF1_lp_2000khz | ST25R3916B_REG_RX_CONF1_hz_40_80khz,
			ST25R3916B_REG_RX_CONF2_demod_mode | ST25R3916B_REG_RX_CONF2_amd_sel_mixer | ST25R3916B_REG_RX_CONF2_sqm_dyn | ST25R3916B_REG_RX_CONF2_pulz_61 | ST25R3916B_REG_RX_CONF2_agc_en | ST25R3916B_REG_RX_CONF2_agc_m | ST25R3916B_REG_RX_CONF2_agc6_3,
			0,
			0
	);

	ST25R3916B_Write_RegistersB2_sep(pInstance, ST25R3916B_REG_B_CORR_CONF1, \
			ST25R3916B_REG_CORR_CONF1_corr_s6 | ST25R3916B_REG_CORR_CONF1_corr_s2 | ST25R3916B_REG_CORR_CONF1_corr_s1 | ST25R3916B_REG_CORR_CONF1_corr_s0,
			0
	);
}
