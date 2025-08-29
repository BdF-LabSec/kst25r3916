/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#pragma once
#include "14a.h"

void ST25R500_14A_Target(ST25R *pInstance);

void ST25R500_14A3_Target_Prepare_AC_Buffer(ST25R *pInstance, const T3A_INFOS *pInfos);
void ST25R500_14A4_Target_Prepare_AC_Buffer(ST25R *pInstance, const T4A_INFOS *pInfos);
