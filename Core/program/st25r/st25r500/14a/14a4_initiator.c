/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a4_initiator.h"
#include <string.h>

uint8_t ST25R500_14A4_Rats(ST25R *pInstance, const uint8_t parameters, T4A_INFOS *pt4aInfos)
{
	uint8_t ret, buffer[2] = {K14A_RATS, parameters};

	ret = ST25R500_Transmit_then_Receive(pInstance, buffer, sizeof(buffer), 1);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		if (pInstance->cbData <= sizeof(pt4aInfos->ATS))
		{
			pt4aInfos->cbATS = (uint8_t) pInstance->cbData;
			memcpy(pt4aInfos->ATS, pInstance->pbData, pInstance->cbData);
			ST25R14A4_AdjustFromATS(pt4aInfos);
			ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_NRT_GPT_CONF, ST25R500_REG_NRT_GPT_CONF_nrt_step_4096fc);
			ST25R500_Write_NoResponseTimer(pInstance, 1 << pt4aInfos->FWI);

			HAL_Delay(1 + ((1 << pt4aInfos->SFGI) / 3)); // TODO better
		}
		else
		{
			ret = ST25T_STATUS_APPLICATION;
		}
	}

	return ret;
}

uint8_t ST25R500_14A4_Deselect(ST25R *pInstance)
{
	uint8_t ret, buffer = K14A_DESELECT;

	ret = ST25R500_Transmit_then_Receive(pInstance, &buffer, sizeof(buffer), 1);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		if(!((pInstance->cbData == sizeof(buffer)) && (pInstance->pbData[0] == K14A_DESELECT)))
		{
			ret = ST25T_STATUS_APPLICATION;
		}
		else
		{
			HAL_Delay(1); // TODO better
		}
	}

	return ret;
}

uint8_t ST25R500_14A4_AdjustBitRate(ST25R *pInstance, T4A_INFOS *pt4aInfos, ST25R_BITRATE bitrate)
{
	uint8_t ret, buffer[3] = {0xd0, 0x11, };

	if(bitrate > pt4aInfos->MaxBitRate)
	{
		bitrate = pt4aInfos->MaxBitRate;
	}
	buffer[2] = 0x0f & ((bitrate << 2) | bitrate);

	if(pt4aInfos->CurrentBitrate != bitrate)
	{
		ret = ST25R500_Transmit_then_Receive(pInstance, buffer, sizeof(buffer), 1);
		if(ret == ST25R_STATUS_NO_ERROR)
		{
			switch(bitrate)
			{
			case ST25R_BITRATE_848:
				ST25R500_14A4_TxRx848(pInstance);
				break;

			case ST25R_BITRATE_424:
				ST25R500_14A4_TxRx424(pInstance);
				break;

			case ST25R_BITRATE_212:
				ST25R500_14A4_TxRx212(pInstance);
				break;

			case ST25R_BITRATE_106:
			default:
				ST25R500_14A4_TxRx106(pInstance);
				break;
			}

			pt4aInfos->CurrentBitrate = bitrate;
		}
	}
	else
	{
		ret = ST25R_STATUS_NO_ERROR;
	}

	return ret;
}
