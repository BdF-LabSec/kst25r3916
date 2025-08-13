/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a3_target.h"
#include <string.h>

void ST25R3916B_14A_Target(ST25R *pInstance)
{
	ST25R3916B_Mask_IRQ(pInstance, /*ST25R3916B_IRQ_MASK_TXE | */ST25R3916B_IRQ_MASK_RXS, ST25R_IRQ_MASK_OP_ADD);
	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_MASK_RX_TIMER, 0x02);
	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_TIMER_EMV_CONTROL, ST25R3916B_REG_TIMER_EMV_CONTROL_nrt_step_64fc);
	ST25R3916B_Write_Registers2_sep(pInstance, ST25R3916B_REG_ANT_TUNE_A,
			0x00,
			0xff
	);
	ST25R3916B_Write_Registers2_sep(pInstance, ST25R3916B_REG_FIELD_THRESHOLD_ACTV,
			ST25R3916B_REG_FIELD_THRESHOLD_ACTV_trg_105mV | ST25R3916B_REG_FIELD_THRESHOLD_ACTV_rfe_105mV,
			ST25R3916B_REG_FIELD_THRESHOLD_DEACTV_trg_75mV | ST25R3916B_REG_FIELD_THRESHOLD_DEACTV_rfe_75mV
	);

	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_PASSIVE_TARGET, 5 << ST25R3916B_REG_PASSIVE_TARGET_fdel_shift);
	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_PT_MOD, (2 << ST25R3916B_REG_PT_MOD_ptm_res_shift) | (15 << ST25R3916B_REG_PT_MOD_pt_res_shift));

	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_TX_DRIVER, ST25R3916B_REG_TX_DRIVER_am_mod_0percent | 0); // TODO ?
}

void ST25R3916B_14A3_Target_Prepare_AC_Buffer(ST25R *pInstance, const T3A_INFOS *pInfos)
{
	uint8_t PT_A[15] = {0}, AUX_Flag;

	memcpy(PT_A, pInfos->UID, MIN(pInfos->cbUID, 10));
	*(uint16_t * ) (PT_A + 10) = pInfos->ATQA;
	PT_A[12] = pInfos->SAK;

	if(pInfos->cbUID > 4)
	{
		PT_A[12] |= 0x04;
		PT_A[13] = pInfos->SAK;
		if(pInfos->cbUID > 7) // invalid on ST25R3916B !
		{
			PT_A[13] |= 0x04;
			PT_A[14] = pInfos->SAK;

			AUX_Flag = ST25R3916B_REG_AUX_nfc_id1;
		}
		else
		{
			AUX_Flag = ST25R3916B_REG_AUX_nfc_id_7bytes;
		}
	}
	else
	{
		AUX_Flag = ST25R3916B_REG_AUX_nfc_id_4bytes;
	}

	ST25R3916B_PT_A_Config_Load(pInstance, PT_A, sizeof(PT_A));
	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_AUX, AUX_Flag);
}
