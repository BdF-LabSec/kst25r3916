/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a4_initiator.h"

uint8_t K14A4_Rats(ST25R *pInstance)
{
	uint8_t ret, buffer[2] = {K14A_RATS, 5 << 4};// 64 instead of 256 - 0x80};
    ret = ST25R3916_Transmit_then_Receive(pInstance, buffer, sizeof(buffer), 1);
    if(ret == ST25R_STATUS_NO_ERROR)
    {
    	ST25R3916_Write_NoResponseTimer(pInstance, 0x4000); // TODO: from RATS, here FWT 77.33 ms
    }

    return ret;
}

uint8_t K14A4_Deselect(ST25R *pInstance)
{
	uint8_t ret, buffer = K14A_DESELECT;

	ST25R3916_Write_NoResponseTimer(pInstance, 0x0170); // TODO
	ret = ST25R3916_Transmit_then_Receive(pInstance, &buffer, sizeof(buffer), 1);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		if(!((pInstance->cbData == sizeof(buffer)) && (pInstance->pbData[0] == K14A_DESELECT)))
		{
			ret = ST25T_STATUS_APPLICATION;
		}
		else
		{
			HAL_Delay(1);
		}
	}

	return ret;
}

void K14A_AdjustMaxBitRate(T4A_INFOS *pt4aInfos)
{
	pt4aInfos->MaxBitRate = ST25R_BITRATE_106;

	if(pt4aInfos->cbATS >= 4)
	{
		uint8_t tl = pt4aInfos->ATS[0];

		if(tl >= 3)
		{
			uint8_t t0 = pt4aInfos->ATS[1];
			if((t0 & 0b00110000) == 0b00110000)
			{
				uint8_t ta = pt4aInfos->ATS[2];//, tb = pt4aInfos->ATS[3];

				if((ta & 0x44) == 0x44)
				{
					pt4aInfos->MaxBitRate = ST25R_BITRATE_848;
				}
				else if((ta & 0x22) == 0x22)
				{
					pt4aInfos->MaxBitRate = ST25R_BITRATE_424;
				}
				else if((ta & 0x11) == 0x11)
				{
					pt4aInfos->MaxBitRate = ST25R_BITRATE_212;
				}
			}
		}
	}
}

uint8_t K14A4_AdjustBitRate(ST25R *pInstance, T4A_INFOS *pt4aInfos, ST25R_BITRATE bitrate)
{
	uint8_t ret, buffer[3] = {0xd0, 0x11, };

	if(bitrate > pt4aInfos->MaxBitRate)
	{
		bitrate = pt4aInfos->MaxBitRate;
	}
	buffer[2] = 0x0f & ((bitrate << 2) | bitrate);

	if(pt4aInfos->CurrentBitrate != bitrate)
	{
		ret = ST25R3916_Transmit_then_Receive(pInstance, buffer, sizeof(buffer), 1);
		if(ret == ST25R_STATUS_NO_ERROR)
		{
			switch(bitrate)
			{
			case ST25R_BITRATE_848:
				ST25R3916_14A4_TxRx848(pInstance);
				break;

			case ST25R_BITRATE_424:
				ST25R3916_14A4_TxRx424(pInstance);
				break;

			case ST25R_BITRATE_212:
				ST25R3916_14A4_TxRx212(pInstance);
				break;

			case ST25R_BITRATE_106:
			default:
				ST25R3916_14A4_TxRx106(pInstance);
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
