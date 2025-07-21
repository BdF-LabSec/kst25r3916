/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a3_target.h"
#include <string.h>

void K14A3_TG_Prepare_AC_Buffer(ST25R *pInstance, const T3A_INFOS *pInfos)
{
	uint8_t PT_A[15] = {0}, AUX_Flag;

	memcpy(PT_A, pInfos->UID, pInfos->cbUID);
	*(uint16_t * ) (PT_A + 10) = pInfos->ATQA;
	PT_A[12] = pInfos->SAK;

    if(pInfos->cbUID > 4)
    {
    	PT_A[12] |= 0x04;
        PT_A[13] = pInfos->SAK;
        if(pInfos->cbUID > 7) // invalid on ST25R3916 !
        {
        	PT_A[13] |= 0x04;
        	PT_A[14] = pInfos->SAK;

           	AUX_Flag = ST25R3916_REG_AUX_nfc_id1;
        }
        else
        {
        	AUX_Flag = ST25R3916_REG_AUX_nfc_id_7bytes;
        }
    }
    else
    {
    	AUX_Flag = ST25R3916_REG_AUX_nfc_id_4bytes;
    }

    ST25R3916_PT_A_Config_Load(pInstance, PT_A, sizeof(PT_A));
    ST25R3916_Write_SingleRegister(pInstance, ST25R3916_REG_AUX, AUX_Flag);
}
