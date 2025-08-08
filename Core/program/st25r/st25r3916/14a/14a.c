#include "14a.h"
#include <string.h>

uint8_t K14A_Anticoll(ST25R *pInstance, T4A_INFOS *infos)
{
	uint8_t ret;

	ret = K14A3_Anticoll(pInstance, &infos->t3a);
	if (ret == ST25R_STATUS_NO_ERROR)
	{
		if (infos->t3a.SAK & 0x20)
		{
			infos->CurrentBitrate = ST25R_BITRATE_106;
			ret = K14A4_Rats(pInstance, 0x50); // 64 b, CID 0
			if (ret == ST25R_STATUS_NO_ERROR)
			{
				if (pInstance->cbData <= sizeof(infos->ATS))
				{
					infos->cbATS = (uint8_t) pInstance->cbData;
					memcpy(infos->ATS, pInstance->pbData, pInstance->cbData);

					K14A_AdjustMaxBitRate(infos);
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
