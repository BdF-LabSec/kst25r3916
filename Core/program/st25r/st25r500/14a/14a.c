/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a.h"

uint8_t ST25R500_14A_Anticoll(ST25R *pInstance, T4A_INFOS *infos)
{
	uint8_t ret;

	ret = ST25R500_14A3_Anticoll(pInstance, &infos->t3a);
	if (ret == ST25R_STATUS_NO_ERROR)
	{
		if (infos->t3a.SAK & 0x20)
		{
			infos->CurrentBitrate = ST25R_BITRATE_106;
			ret = ST25R500_14A4_Rats(pInstance, 0x50, infos); // 64 b, CID 0
		}
	}

	return ret;
}

void ST25R500_14A_Initiator(ST25R *pInstance)
{
	ST25R500_Mask_IRQ(pInstance, /*ST25R500_IRQ_MASK_TXE | */ST25R500_IRQ_MASK_SUBC_START | ST25R500_IRQ_MASK_RXS, ST25R_IRQ_MASK_OP_ADD);
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_EFD_THRESHOLD, (8 << ST25R500_REG_EFD_THRESHOLD_efd_dt_shift) | (9 << ST25R500_REG_EFD_THRESHOLD_efd_at_shift));

	ST25R500_Write_GeneralPurposeTimer(pInstance, 16949); // for ~10 ms - initial Field ON Guard time

	ST25R500_14A4_TxRx106(pInstance);
}

const uint8_t ST25R500_14A4_TxRx106_data_0[] = {
	/* ST25R500_REG_RX_ANA1 */		(0x7 << ST25R500_REG_RX_ANA1_dig_clk_dly_shift) | (0x3 << ST25R500_REG_RX_ANA1_hpf_ctrl_shift),
	/* ST25R500_REG_RX_ANA2 */		(0x2 << ST25R500_REG_RX_ANA2_afe_gain_rw_shift) | (0x2 << ST25R500_REG_RX_ANA2_afe_gain_td_shift),
	/* ST25R500_REG_RX_ANA3 */		(0x8 << ST25R500_REG_RX_ANA3_afe_gain_ce_shift) | (1 << ST25R500_REG_RX_ANA3_ook_thr_hi_shift) | (1 << ST25R500_REG_RX_ANA3_ook_thr_lo_shift),
	/* ST25R500_REG_RX_ANA4 */		0,
	/* ST25R500_REG_RX_DIG */		ST25R500_REG_RX_DIG_agc_en | (6 << ST25R500_REG_RX_DIG_lpf_coef_shift) | (2 << ST25R500_REG_RX_DIG_hpf_coef_shift),
	/* ST25R500_REG_CORR1 */		(0xf << ST25R500_REG_CORR1_iir_coef2_shift) | (0x8 << ST25R500_REG_CORR1_iir_coef1_shift),
	/* ST25R500_REG_CORR2 */		(0x2 << ST25R500_REG_CORR2_agc_thr_squelch_shift) | (0xe << ST25R500_REG_CORR2_agc_thr_shift),
	/* ST25R500_REG_CORR3 */		0 | (0x0f << ST25R500_REG_CORR3_start_wait_shift),
	/* ST25R500_REG_CORR4 */		(0x8 << ST25R500_REG_CORR4_coll_lvl_shift) | (0x8 << ST25R500_REG_CORR4_data_lvl_shift), // coll_lvl 7 during anticoll
	/* ST25R500_REG_CORR5 */		ST25R500_REG_CORR5_dis_soft_sq | ST25R500_REG_CORR5_dis_agc_noise_meas | (2 << ST25R500_REG_CORR5_dec_f_shift),
	/* ST25R500_REG_CORR6 */		(0x2 << ST25R500_REG_CORR6_init_noise_lvl_shift) | (0x0 << ST25R500_REG_CORR6_agc_freeze_cnt_shift),
	/* ST25R500_REG_PROTOCOL */		ST25R500_REG_PROTOCOL_rx_rate_106_26 | ST25R500_REG_PROTOCOL_tx_rate_106 | ST25R500_REG_PROTOCOL_om_iso14443a,
	/* ST25R500_REG_PROTOCOL_TX1 */	ST25R500_REG_PROTOCOL_TX1_a_tx_par_on | ST25R500_REG_PROTOCOL_TX1_tx_crc_on | (0 << ST25R500_REG_PROTOCOL_TX1_p_len_shift), // OOK
	/* ST25R500_REG_PROTOCOL_TX2 */	ST25R500_REG_PROTOCOL_TX2_f_tx_len | ST25R500_REG_PROTOCOL_TX2_b_tx_half,
};
const uint8_t ST25R500_14A4_TxRx106_data_1[] = {
	/* ST25R500_REG_AWS_CONFIG1 */	ST25R500_REG_AWS_CONFIG1_dyn_ilim_aws | ST25R500_REG_AWS_CONFIG1_dyn_sink_offset | ST25R500_REG_AWS_CONFIG1_sc_prot_en | ST25R500_REG_AWS_CONFIG1_sink_offset_en | ST25R500_REG_AWS_CONFIG1_act_sink_en,
	/* ST25R500_REG_AWS_CONFIG2 */	(0x3 << ST25R500_REG_AWS_CONFIG2_am_fall_shift) | (0x3 << ST25R500_REG_AWS_CONFIG2_am_rise_shift),
	/* ST25R500_REG_AWS_TIME1 */	(0xf << ST25R500_REG_AWS_TIME1_tentx1_shift) | (0xf << ST25R500_REG_AWS_TIME1_tdres1_shift),
	/* ST25R500_REG_AWS_TIME2 */	(0x0 << ST25R500_REG_AWS_TIME2_tpasssinkx1_shift) | (0x0 << ST25R500_REG_AWS_TIME2_tsinkoff1_shift),
	/* ST25R500_REG_AWS_TIME3 */	(0xf << ST25R500_REG_AWS_TIME3_tilim2_shift) | (0x0 << ST25R500_REG_AWS_TIME3_tdres2_shift),
	/* ST25R500_REG_AWS_TIME4 */	(0x0 << ST25R500_REG_AWS_TIME4_tpasssinkx2_shift) | (0x0 << ST25R500_REG_AWS_TIME4_tsinkoff2_shift),
};
void ST25R500_14A4_TxRx106(ST25R *pInstance)
{
	ST25R500_Write_Registers2_sep(pInstance, ST25R500_REG_TX_MOD1,
			ST25R500_REG_TX_MOD1_am_mod_97percent | ST25R500_REG_TX_MOD1_res_am | ST25R500_REG_TX_MOD1_rgs_am,
			0x7f // High-Z
	);
	ST25R500_Write_Registers2_sep(pInstance, ST25R500_REG_MRT1,
			(0 << ST25R500_REG_MRT1_sq_del_shift) | ST25R500_REG_MRT1_mrt_step_16fc | ST25R500_REG_MRT1_sq_en,
			0x35
	);
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_SQT, 0x1c);
	ST25R_SPI_Write_Multiple_internal(pInstance, ST25R500_MK_WRITE(ST25R500_REG_RX_ANA1), ST25R500_14A4_TxRx106_data_0, sizeof(ST25R500_14A4_TxRx106_data_0));
	ST25R_SPI_Write_Multiple_internal(pInstance, ST25R500_MK_WRITE(ST25R500_REG_AWS_CONFIG1), ST25R500_14A4_TxRx106_data_1, sizeof(ST25R500_14A4_TxRx106_data_1));
}

const uint8_t ST25R500_14A4_TxRx212_data_0[] = {
	/* ST25R500_REG_RX_ANA1 */		(0x7 << ST25R500_REG_RX_ANA1_dig_clk_dly_shift) | (0x1 << ST25R500_REG_RX_ANA1_hpf_ctrl_shift),
	/* ST25R500_REG_RX_ANA2 */		(0x0 << ST25R500_REG_RX_ANA2_afe_gain_rw_shift) | (0x2 << ST25R500_REG_RX_ANA2_afe_gain_td_shift),
	/* ST25R500_REG_RX_ANA3 */		(0x8 << ST25R500_REG_RX_ANA3_afe_gain_ce_shift) | (1 << ST25R500_REG_RX_ANA3_ook_thr_hi_shift) | (1 << ST25R500_REG_RX_ANA3_ook_thr_lo_shift),
	/* ST25R500_REG_RX_ANA4 */		ST25R500_REG_RX_ANA4_en_rect_cor,
	/* ST25R500_REG_RX_DIG */		ST25R500_REG_RX_DIG_agc_en | (4 << ST25R500_REG_RX_DIG_lpf_coef_shift) | (2 << ST25R500_REG_RX_DIG_hpf_coef_shift),
	/* ST25R500_REG_CORR1 */		(0xd << ST25R500_REG_CORR1_iir_coef2_shift) | (0x4 << ST25R500_REG_CORR1_iir_coef1_shift),
	/* ST25R500_REG_CORR2 */		(0x2 << ST25R500_REG_CORR2_agc_thr_squelch_shift) | (0xd << ST25R500_REG_CORR2_agc_thr_shift),
	/* ST25R500_REG_CORR3 */		ST25R500_REG_CORR3_en_subc_end | (0x0f << ST25R500_REG_CORR3_start_wait_shift),
	/* ST25R500_REG_CORR4 */		(0xa << ST25R500_REG_CORR4_coll_lvl_shift) | (0x3 << ST25R500_REG_CORR4_data_lvl_shift),
	/* ST25R500_REG_CORR5 */		ST25R500_REG_CORR5_dis_soft_sq | ST25R500_REG_CORR5_dis_agc_noise_meas | (2 << ST25R500_REG_CORR5_dec_f_shift),
	/* ST25R500_REG_CORR6 */		(0x0 << ST25R500_REG_CORR6_init_noise_lvl_shift) | (0x1 << ST25R500_REG_CORR6_agc_freeze_cnt_shift),
	/* ST25R500_REG_PROTOCOL */		ST25R500_REG_PROTOCOL_rx_rate_212_53 | ST25R500_REG_PROTOCOL_tx_rate_212 | ST25R500_REG_PROTOCOL_om_iso14443a,
	/* ST25R500_REG_PROTOCOL_TX1 */	ST25R500_REG_PROTOCOL_TX1_a_tx_par_on | ST25R500_REG_PROTOCOL_TX1_tx_crc_on | ST25R500_REG_PROTOCOL_TX1_tr_am | (1 << ST25R500_REG_PROTOCOL_TX1_p_len_shift), // AM
	/* ST25R500_REG_PROTOCOL_TX2 */	ST25R500_REG_PROTOCOL_TX2_f_tx_len | ST25R500_REG_PROTOCOL_TX2_b_tx_half,
};
const uint8_t ST25R500_14A4_TxRx212_data_1[] = {
	/* ST25R500_REG_AWS_CONFIG1 */	ST25R500_REG_AWS_CONFIG1_dyn_ilim_aws | ST25R500_REG_AWS_CONFIG1_dyn_sink_offset | ST25R500_REG_AWS_CONFIG1_sc_prot_en | ST25R500_REG_AWS_CONFIG1_sink_offset_en | ST25R500_REG_AWS_CONFIG1_act_sink_en,
	/* ST25R500_REG_AWS_CONFIG2 */	(0x7 << ST25R500_REG_AWS_CONFIG2_am_fall_shift) | (0x0 << ST25R500_REG_AWS_CONFIG2_am_rise_shift),
	/* ST25R500_REG_AWS_TIME1 */	(0xf << ST25R500_REG_AWS_TIME1_tentx1_shift) | (0xf << ST25R500_REG_AWS_TIME1_tdres1_shift),
	/* ST25R500_REG_AWS_TIME2 */	(0x0 << ST25R500_REG_AWS_TIME2_tpasssinkx1_shift) | (0x0 << ST25R500_REG_AWS_TIME2_tsinkoff1_shift),
	/* ST25R500_REG_AWS_TIME3 */	(0xf << ST25R500_REG_AWS_TIME3_tilim2_shift) | (0x0 << ST25R500_REG_AWS_TIME3_tdres2_shift),
	/* ST25R500_REG_AWS_TIME4 */	(0x0 << ST25R500_REG_AWS_TIME4_tpasssinkx2_shift) | (0x0 << ST25R500_REG_AWS_TIME4_tsinkoff2_shift),
};
void ST25R500_14A4_TxRx212(ST25R *pInstance)
{
	ST25R500_Write_Registers2_sep(pInstance, ST25R500_REG_TX_MOD1,
			ST25R500_REG_TX_MOD1_am_mod_80percent | ST25R500_REG_TX_MOD1_rgs_am,
			0x7f // High-Z
	);
	ST25R500_Write_Registers2_sep(pInstance, ST25R500_REG_MRT1,
			(0 << ST25R500_REG_MRT1_sq_del_shift) | ST25R500_REG_MRT1_mrt_step_16fc | ST25R500_REG_MRT1_sq_en,
			0x35
	);
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_SQT, 0xff);
	ST25R_SPI_Write_Multiple_internal(pInstance, ST25R500_MK_WRITE(ST25R500_REG_RX_ANA1), ST25R500_14A4_TxRx212_data_0, sizeof(ST25R500_14A4_TxRx212_data_0));
	ST25R_SPI_Write_Multiple_internal(pInstance, ST25R500_MK_WRITE(ST25R500_REG_AWS_CONFIG1), ST25R500_14A4_TxRx212_data_1, sizeof(ST25R500_14A4_TxRx212_data_1));
}

const uint8_t ST25R500_14A4_TxRx424_data_0[] = {
	/* ST25R500_REG_RX_ANA1 */		(0x7 << ST25R500_REG_RX_ANA1_dig_clk_dly_shift) | (0x1 << ST25R500_REG_RX_ANA1_hpf_ctrl_shift),
	/* ST25R500_REG_RX_ANA2 */		(0x0 << ST25R500_REG_RX_ANA2_afe_gain_rw_shift) | (0x2 << ST25R500_REG_RX_ANA2_afe_gain_td_shift),
	/* ST25R500_REG_RX_ANA3 */		(0x8 << ST25R500_REG_RX_ANA3_afe_gain_ce_shift) | (1 << ST25R500_REG_RX_ANA3_ook_thr_hi_shift) | (1 << ST25R500_REG_RX_ANA3_ook_thr_lo_shift),
	/* ST25R500_REG_RX_ANA4 */		ST25R500_REG_RX_ANA4_en_rect_cor,
	/* ST25R500_REG_RX_DIG */		ST25R500_REG_RX_DIG_agc_en | (5 << ST25R500_REG_RX_DIG_lpf_coef_shift) | (2 << ST25R500_REG_RX_DIG_hpf_coef_shift),
	/* ST25R500_REG_CORR1 */		(0xd << ST25R500_REG_CORR1_iir_coef2_shift) | (0x6 << ST25R500_REG_CORR1_iir_coef1_shift),
	/* ST25R500_REG_CORR2 */		(0x2 << ST25R500_REG_CORR2_agc_thr_squelch_shift) | (0xd << ST25R500_REG_CORR2_agc_thr_shift),
	/* ST25R500_REG_CORR3 */		ST25R500_REG_CORR3_en_subc_end | (0x1f << ST25R500_REG_CORR3_start_wait_shift),
	/* ST25R500_REG_CORR4 */		(0xa << ST25R500_REG_CORR4_coll_lvl_shift) | (0x3 << ST25R500_REG_CORR4_data_lvl_shift),
	/* ST25R500_REG_CORR5 */		ST25R500_REG_CORR5_dis_soft_sq | ST25R500_REG_CORR5_dis_agc_noise_meas | ST25R500_REG_CORR5_no_phase | (1 << ST25R500_REG_CORR5_dec_f_shift),
	/* ST25R500_REG_CORR6 */		(0x0 << ST25R500_REG_CORR6_init_noise_lvl_shift) | (0x4 << ST25R500_REG_CORR6_agc_freeze_cnt_shift),
	/* ST25R500_REG_PROTOCOL */		ST25R500_REG_PROTOCOL_rx_rate_424 | ST25R500_REG_PROTOCOL_tx_rate_424 | ST25R500_REG_PROTOCOL_om_iso14443a,
	/* ST25R500_REG_PROTOCOL_TX1 */	ST25R500_REG_PROTOCOL_TX1_a_tx_par_on | ST25R500_REG_PROTOCOL_TX1_tx_crc_on | ST25R500_REG_PROTOCOL_TX1_tr_am | (0 << ST25R500_REG_PROTOCOL_TX1_p_len_shift), // AM
	/* ST25R500_REG_PROTOCOL_TX2 */	ST25R500_REG_PROTOCOL_TX2_f_tx_len | ST25R500_REG_PROTOCOL_TX2_b_tx_half,
};
const uint8_t ST25R500_14A4_TxRx424_data_1[] = {
	/* ST25R500_REG_AWS_CONFIG1 */	ST25R500_REG_AWS_CONFIG1_dyn_ilim_aws | ST25R500_REG_AWS_CONFIG1_dyn_sink_offset | ST25R500_REG_AWS_CONFIG1_sc_prot_en | ST25R500_REG_AWS_CONFIG1_sink_offset_en | ST25R500_REG_AWS_CONFIG1_act_sink_en,
	/* ST25R500_REG_AWS_CONFIG2 */	(0x0 << ST25R500_REG_AWS_CONFIG2_am_fall_shift) | (0x0 << ST25R500_REG_AWS_CONFIG2_am_rise_shift),
	/* ST25R500_REG_AWS_TIME1 */	(0xf << ST25R500_REG_AWS_TIME1_tentx1_shift) | (0xf << ST25R500_REG_AWS_TIME1_tdres1_shift),
	/* ST25R500_REG_AWS_TIME2 */	(0x0 << ST25R500_REG_AWS_TIME2_tpasssinkx1_shift) | (0x0 << ST25R500_REG_AWS_TIME2_tsinkoff1_shift),
	/* ST25R500_REG_AWS_TIME3 */	(0xf << ST25R500_REG_AWS_TIME3_tilim2_shift) | (0x0 << ST25R500_REG_AWS_TIME3_tdres2_shift),
	/* ST25R500_REG_AWS_TIME4 */	(0x0 << ST25R500_REG_AWS_TIME4_tpasssinkx2_shift) | (0x0 << ST25R500_REG_AWS_TIME4_tsinkoff2_shift),
};
void ST25R500_14A4_TxRx424(ST25R *pInstance)
{
	ST25R500_Write_Registers2_sep(pInstance, ST25R500_REG_TX_MOD1,
			ST25R500_REG_TX_MOD1_am_mod_60percent | ST25R500_REG_TX_MOD1_rgs_am,
			0x7f // High-Z
	);
	ST25R500_Write_Registers2_sep(pInstance, ST25R500_REG_MRT1,
			(0 << ST25R500_REG_MRT1_sq_del_shift) | ST25R500_REG_MRT1_mrt_step_16fc | ST25R500_REG_MRT1_sq_en,
			0x35
	);
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_SQT, 0xff);
	ST25R_SPI_Write_Multiple_internal(pInstance, ST25R500_MK_WRITE(ST25R500_REG_RX_ANA1), ST25R500_14A4_TxRx424_data_0, sizeof(ST25R500_14A4_TxRx424_data_0));
	ST25R_SPI_Write_Multiple_internal(pInstance, ST25R500_MK_WRITE(ST25R500_REG_AWS_CONFIG1), ST25R500_14A4_TxRx424_data_1, sizeof(ST25R500_14A4_TxRx424_data_1));
}

const uint8_t ST25R500_14A4_TxRx848_data_0[] = {
	/* ST25R500_REG_RX_ANA1 */		(0x7 << ST25R500_REG_RX_ANA1_dig_clk_dly_shift) | (0x1 << ST25R500_REG_RX_ANA1_hpf_ctrl_shift),
	/* ST25R500_REG_RX_ANA2 */		(0x0 << ST25R500_REG_RX_ANA2_afe_gain_rw_shift) | (0x2 << ST25R500_REG_RX_ANA2_afe_gain_td_shift),
	/* ST25R500_REG_RX_ANA3 */		(0x8 << ST25R500_REG_RX_ANA3_afe_gain_ce_shift) | (1 << ST25R500_REG_RX_ANA3_ook_thr_hi_shift) | (1 << ST25R500_REG_RX_ANA3_ook_thr_lo_shift),
	/* ST25R500_REG_RX_ANA4 */		ST25R500_REG_RX_ANA4_en_rect_cor,
	/* ST25R500_REG_RX_DIG */		ST25R500_REG_RX_DIG_agc_en | (6 << ST25R500_REG_RX_DIG_lpf_coef_shift) | (1 << ST25R500_REG_RX_DIG_hpf_coef_shift),
	/* ST25R500_REG_CORR1 */		(0xa << ST25R500_REG_CORR1_iir_coef2_shift) | (0xa << ST25R500_REG_CORR1_iir_coef1_shift),
	/* ST25R500_REG_CORR2 */		(0x2 << ST25R500_REG_CORR2_agc_thr_squelch_shift) | (0xb << ST25R500_REG_CORR2_agc_thr_shift),
	/* ST25R500_REG_CORR3 */		ST25R500_REG_CORR3_en_subc_end | (0x3f << ST25R500_REG_CORR3_start_wait_shift),
	/* ST25R500_REG_CORR4 */		(0xa << ST25R500_REG_CORR4_coll_lvl_shift) | (0x6 << ST25R500_REG_CORR4_data_lvl_shift),
	/* ST25R500_REG_CORR5 */		ST25R500_REG_CORR5_dis_soft_sq | ST25R500_REG_CORR5_dis_agc_noise_meas | ST25R500_REG_CORR5_no_phase | (0 << ST25R500_REG_CORR5_dec_f_shift),
	/* ST25R500_REG_CORR6 */		(0x1 << ST25R500_REG_CORR6_init_noise_lvl_shift) | (0x9 << ST25R500_REG_CORR6_agc_freeze_cnt_shift),
	/* ST25R500_REG_PROTOCOL */		ST25R500_REG_PROTOCOL_rx_rate_848 | ST25R500_REG_PROTOCOL_tx_rate_848 | ST25R500_REG_PROTOCOL_om_iso14443a,
	/* ST25R500_REG_PROTOCOL_TX1 */	ST25R500_REG_PROTOCOL_TX1_a_tx_par_on | ST25R500_REG_PROTOCOL_TX1_tx_crc_on | ST25R500_REG_PROTOCOL_TX1_tr_am | (0xe << ST25R500_REG_PROTOCOL_TX1_p_len_shift), // AM
	/* ST25R500_REG_PROTOCOL_TX2 */	ST25R500_REG_PROTOCOL_TX2_f_tx_len | ST25R500_REG_PROTOCOL_TX2_b_tx_half,
};
const uint8_t ST25R500_14A4_TxRx848_data_1[] = {
	/* ST25R500_REG_AWS_CONFIG1 */	ST25R500_REG_AWS_CONFIG1_dyn_ilim_aws | ST25R500_REG_AWS_CONFIG1_dyn_sink_offset | ST25R500_REG_AWS_CONFIG1_sc_prot_en | ST25R500_REG_AWS_CONFIG1_sink_offset_en | ST25R500_REG_AWS_CONFIG1_act_sink_en,
	/* ST25R500_REG_AWS_CONFIG2 */	(0x0 << ST25R500_REG_AWS_CONFIG2_am_fall_shift) | (0x0 << ST25R500_REG_AWS_CONFIG2_am_rise_shift),
	/* ST25R500_REG_AWS_TIME1 */	(0xf << ST25R500_REG_AWS_TIME1_tentx1_shift) | (0xf << ST25R500_REG_AWS_TIME1_tdres1_shift),
	/* ST25R500_REG_AWS_TIME2 */	(0x5 << ST25R500_REG_AWS_TIME2_tpasssinkx1_shift) | (0x0 << ST25R500_REG_AWS_TIME2_tsinkoff1_shift),
	/* ST25R500_REG_AWS_TIME3 */	(0xf << ST25R500_REG_AWS_TIME3_tilim2_shift) | (0x0 << ST25R500_REG_AWS_TIME3_tdres2_shift),
	/* ST25R500_REG_AWS_TIME4 */	(0x0 << ST25R500_REG_AWS_TIME4_tpasssinkx2_shift) | (0x0 << ST25R500_REG_AWS_TIME4_tsinkoff2_shift),
};
void ST25R500_14A4_TxRx848(ST25R *pInstance)
{
	ST25R500_Write_Registers2_sep(pInstance, ST25R500_REG_TX_MOD1,
			ST25R500_REG_TX_MOD1_am_mod_97percent | ST25R500_REG_TX_MOD1_rgs_am,
			0x7f // High-Z
	);
	ST25R500_Write_Registers2_sep(pInstance, ST25R500_REG_MRT1,
			(1 << ST25R500_REG_MRT1_sq_del_shift) | ST25R500_REG_MRT1_mrt_step_16fc | ST25R500_REG_MRT1_sq_en,
			0x35
	);
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_SQT, 0xff);
	ST25R_SPI_Write_Multiple_internal(pInstance, ST25R500_MK_WRITE(ST25R500_REG_RX_ANA1), ST25R500_14A4_TxRx848_data_0, sizeof(ST25R500_14A4_TxRx848_data_0));
	ST25R_SPI_Write_Multiple_internal(pInstance, ST25R500_MK_WRITE(ST25R500_REG_AWS_CONFIG1), ST25R500_14A4_TxRx848_data_1, sizeof(ST25R500_14A4_TxRx848_data_1));
}
