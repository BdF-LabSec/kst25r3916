/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#pragma once
#include "14a.h"

uint8_t K14A4_Rats(ST25R *pInstance);
uint8_t K14A4_Deselect(ST25R *pInstance);
void K14A_AdjustMaxBitRate(T4A_INFOS *pt4aInfos);
uint8_t K14A4_AdjustBitRate(ST25R *pInstance, T4A_INFOS *pt4aInfos, ST25R_BITRATE bitrate);
