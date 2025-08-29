/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a3_target.h"
#include <string.h>

void ST25R500_14A_Target(ST25R *pInstance)
{
	ST25R500_Mask_IRQ(pInstance, /*ST25R500_IRQ_MASK_TXE | */ST25R500_IRQ_MASK_RXS, ST25R_IRQ_MASK_OP_ADD);

	ST25R500_Write_Registers4_sep(pInstance, ST25R500_REG_TX_MOD1,
			ST25R500_REG_TX_MOD1_am_mod_20percent | ST25R500_REG_TX_MOD1_res_am | ST25R500_REG_TX_MOD1_rgs_am,
			0x7f, // High-Z
			(0xf << ST25R500_REG_CE_TX_MOD1_cem_res_shift) | (0 << ST25R500_REG_CE_TX_MOD1_ce_res_shift),
			ST25R500_REG_CE_TX_MOD2_lm_dri
	);
	ST25R500_Write_Registers4_sep(pInstance, ST25R500_REG_RX_ANA1,
			(0x7 << ST25R500_REG_RX_ANA1_dig_clk_dly_shift) | (0x3 << ST25R500_REG_RX_ANA1_hpf_ctrl_shift),
			(0x0 << ST25R500_REG_RX_ANA2_afe_gain_rw_shift) | (0x2 << ST25R500_REG_RX_ANA2_afe_gain_td_shift),
			(0x8 << ST25R500_REG_RX_ANA3_afe_gain_ce_shift) | (1 << ST25R500_REG_RX_ANA3_ook_thr_hi_shift) | (1 << ST25R500_REG_RX_ANA3_ook_thr_lo_shift),
			ST25R500_REG_RX_ANA4_en_phase_deadzone // may be not necessary on 14A only
	);
	ST25R500_Write_Registers2_sep(pInstance, ST25R500_REG_MRT1,
			(0 << ST25R500_REG_MRT1_sq_del_shift) | ST25R500_REG_MRT1_mrt_step_512fc | ST25R500_REG_MRT1_sq_en,
			0x4f
	);
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_EFD_THRESHOLD, (8 << ST25R500_REG_EFD_THRESHOLD_efd_dt_shift) | (9 << ST25R500_REG_EFD_THRESHOLD_efd_at_shift));
}

void ST25R500_14A3_Target_Prepare_AC_Buffer(ST25R *pInstance, const T3A_INFOS *pInfos)
{
	uint8_t PT_A[31] = {0}, AUX_Flag;

	memcpy(PT_A, pInfos->UID, MIN(pInfos->cbUID, 7));
	*(uint16_t * ) (PT_A + 7) = pInfos->ATQA;
	PT_A[9] = pInfos->SAK;

	if(pInfos->cbUID > 4)
	{
		PT_A[9] |= 0x04;
		PT_A[10] = pInfos->SAK;

		AUX_Flag = (0xc << ST25R500_REG_CE_CONFIG2_fdel_shift) | ST25R500_REG_CE_CONFIG2_nfc_id_7bytes;
	}
	else
	{
		AUX_Flag = (0xc << ST25R500_REG_CE_CONFIG2_fdel_shift) | ST25R500_REG_CE_CONFIG2_nfc_id_4bytes;
	}

	// no ATS handled here

	ST25R500_PT_A_Config_Load(pInstance, PT_A, sizeof(PT_A));
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_CE_CONFIG2, AUX_Flag);
}

void ST25R500_14A4_Target_Prepare_AC_Buffer(ST25R *pInstance, const T4A_INFOS *pInfos)
{
	uint8_t PT_A[31] = {0}, AUX_Flag;

	memcpy(PT_A, pInfos->t3a.UID, MIN(pInfos->t3a.cbUID, 7));
	*(uint16_t * ) (PT_A + 7) = pInfos->t3a.ATQA;
	PT_A[9] = pInfos->t3a.SAK;

	if(pInfos->t3a.cbUID > 4)
	{
		PT_A[9] |= 0x04;
		PT_A[10] = pInfos->t3a.SAK;

		AUX_Flag = (0xc << ST25R500_REG_CE_CONFIG2_fdel_shift) | ST25R500_REG_CE_CONFIG2_nfc_id_7bytes;
	}
	else
	{
		AUX_Flag = (0xc << ST25R500_REG_CE_CONFIG2_fdel_shift) | ST25R500_REG_CE_CONFIG2_nfc_id_4bytes;
	}

	if(pInfos->cbATS)
	{
		memcpy(PT_A + 11, pInfos->ATS, MIN(pInfos->cbATS, 20));
	}

	ST25R500_PT_A_Config_Load(pInstance, PT_A, sizeof(PT_A));
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_CE_CONFIG2, AUX_Flag);
}
