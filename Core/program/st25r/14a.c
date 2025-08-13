/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a.h"

const uint8_t ST25R_14A3_HLTA_data[2] = {K14A_HLTA, K14A_HLTA_2};
const uint8_t ST25R_14A4_DESELECT_data[1] = {K14A_DESELECT};

const uint16_t ST25R_14A4_FSxI_to_FSx[] = {16, 24, 32, 40, 48, 64, 96, 128, 256};

void ST25R14A4_AdjustFromATS(T4A_INFOS *pt4aInfos)
{
	uint8_t curIdx, tl, t0, ta, tb;//, tc;

	pt4aInfos->FSCI = K14A_DEFAULT_FSCI;
	pt4aInfos->FWI = K14A_DEFAULT_FWI;
	pt4aInfos->SFGI = K14A_DEFAULT_SFGI;

	pt4aInfos->MaxBitRate = ST25R_BITRATE_106;

	if(pt4aInfos->cbATS)
	{
		tl = pt4aInfos->ATS[0];
		if(tl == pt4aInfos->cbATS)
		{
			if(tl > 1)
			{
				t0 = pt4aInfos->ATS[1];
				pt4aInfos->FSCI = t0 & 0b00001111;

				curIdx = 2;

				if(t0 & 0b00010000)
				{
					if(tl > curIdx)
					{
						ta = pt4aInfos->ATS[curIdx];
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
					curIdx++;
				}

				if(t0 & 0b00100000)
				{
					if(tl > curIdx)
					{
						tb = pt4aInfos->ATS[curIdx];
						pt4aInfos->FWI = tb >> 4;
						pt4aInfos->SFGI = tb & 0b00001111;
					}
					curIdx++;
				}

				if(t0 & 0b01000000)
				{
					if(tl > curIdx)
					{
						//tc = pt4aInfos->ATS[curIdx];
					}
					curIdx++;
				}

			}
		}
	}

	pt4aInfos->MaxFrameSize = (pt4aInfos->FSCI < sizeof(ST25R_14A4_FSxI_to_FSx)) ? ST25R_14A4_FSxI_to_FSx[pt4aInfos->FSCI] : ST25R_14A4_FSxI_to_FSx[K14A_DEFAULT_FSCI];
}
