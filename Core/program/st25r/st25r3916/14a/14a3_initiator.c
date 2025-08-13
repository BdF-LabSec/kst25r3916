/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a3_initiator.h"
#include <string.h>

uint8_t ST25R3916_14A3_Anticoll_CL(ST25R *pInstance, uint8_t cl, uint8_t UID[10], uint8_t *pSAK, uint8_t *pUIDIdx)
{
	uint8_t ret, buffer[7], recIdx;

	buffer[0] = cl;
	buffer[1] = 0x02 << 4;

	ST25R3916_Write_SingleRegister(pInstance, ST25R3916_REG_AUX, ST25R3916_REG_AUX_no_crc_rx);
	ret = ST25R3916_Transmit_then_Receive(pInstance, buffer, 2, 0);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
	if(pInstance->cbData == 5)
		{
			if((pInstance->pbData[0] ^ pInstance->pbData[1] ^ pInstance->pbData[2] ^ pInstance->pbData[3]) == pInstance->pbData[4])
			{
				recIdx = (pInstance->pbData[0] == K14A_CASCADE_TAG);
				memcpy(UID + *pUIDIdx, pInstance->pbData + recIdx, 4 - recIdx);
				*pUIDIdx += 4 - recIdx;

				buffer[1] = 0x07 << 4;
				memcpy(buffer + 2, pInstance->pbData, 5);
				ST25R3916_Write_SingleRegister(pInstance, ST25R3916_REG_AUX, 0);
				ret = ST25R3916_Transmit_then_Receive(pInstance, buffer, 7, 1);
				if(ret == ST25R_STATUS_NO_ERROR)
				{
					if(pInstance->cbData == 1)
					{
						*pSAK = pInstance->pbData[0];
						if(((*pSAK >> 2) & 1) ^ recIdx)
						{
							ret = ST25T_STATUS_APPLICATION;
						}
					}
					else
					{
						ret = ST25T_STATUS_APPLICATION;
					}
				}
			}
			else
			{
				ret = ST25T_STATUS_APPLICATION;
			}
		}
		else
		{
			ret = ST25T_STATUS_APPLICATION;
		}
	}

	return ret;
}

uint8_t ST25R3916_14A3_Anticoll(ST25R *pInstance, T3A_INFOS *pInfos)
{
	uint8_t ret;

	pInfos->cbUID = 0;

	ST25R3916_Write_SingleRegister(pInstance, ST25R3916_REG_AUX, ST25R3916_REG_AUX_no_crc_rx);
	ST25R3916_Write_SingleRegister(pInstance, ST25R3916_REG_TIMER_EMV_CONTROL, ST25R3916_REG_TIMER_EMV_CONTROL_nrt_step_64fc);
	ST25R3916_Write_NoResponseTimer(pInstance, 0x0080);

	ST25R3916_DirectCommand(pInstance, ST25R3916_CMD_TRANSMIT_WUPA);
	ret = ST25R3916_Transmit_then_Receive(pInstance, NULL, 0, 0);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		if(pInstance->cbData == sizeof(pInfos->ATQA))
		{
			pInfos->ATQA = *(uint16_t *) pInstance->pbData;
			if(pInfos->ATQA & 0x1f) // anticoll
			{
				ret = ST25R3916_14A3_Anticoll_CL(pInstance, K14A_SDD_REQ_CL1, pInfos->UID, &pInfos->SAK, &pInfos->cbUID);
				if ((ret == ST25R_STATUS_NO_ERROR) && (pInfos->SAK & 0x04))
				{
					ret = ST25R3916_14A3_Anticoll_CL(pInstance, K14A_SDD_REQ_CL2, pInfos->UID, &pInfos->SAK, &pInfos->cbUID);
					if ((ret == ST25R_STATUS_NO_ERROR) && (pInfos->SAK & 0x04))
					{
						ret = ST25R3916_14A3_Anticoll_CL(pInstance, K14A_SDD_REQ_CL3, pInfos->UID, &pInfos->SAK, &pInfos->cbUID);
						if ((ret == ST25R_STATUS_NO_ERROR) && (pInfos->SAK & 0x04))
						{
							ret = ST25T_STATUS_APPLICATION; // not valid at this point
						}
					}
				}
			}
			else
			{
				ret = ST25T_STATUS_APPLICATION;
			}
		}
		else
		{
			ret = ST25T_STATUS_APPLICATION;
		}
	}

	ST25R3916_Write_SingleRegister(pInstance, ST25R3916_REG_AUX, 0);
	HAL_Delay(1 + ((1 << 2) / 3)); // TODO better

	return ret;
}

uint8_t ST25R3916_14A3_HLTA(ST25R *pInstance)
{
	uint8_t ret;

	ST25R3916_Write_NoResponseTimer(pInstance, 0);
	ret = ST25R3916_Transmit(pInstance, ST25R_14A3_HLTA_data, sizeof(ST25R_14A3_HLTA_data), 1);
	HAL_Delay(2); // TODO better

	return ret;
}
