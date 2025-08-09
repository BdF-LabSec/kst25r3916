/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a.h"

const uint8_t ST25R_14A3_HLTA_data[2] = {K14A_HLTA, K14A_HLTA_2};

void ST25R_14A4_AdjustMaxBitRate(T4A_INFOS *pt4aInfos)
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
