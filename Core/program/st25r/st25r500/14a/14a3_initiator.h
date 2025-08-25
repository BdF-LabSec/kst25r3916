/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#pragma once
#include "14a.h"

void ST25R500_14A3_WUPA(ST25R *pInstance);
uint8_t ST25R500_14A3_Anticoll(ST25R *pInstance, T3A_INFOS *pInfos);
uint8_t ST25R500_14A3_HLTA(ST25R *pInstance);
