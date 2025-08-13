/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#pragma once
#include "../st25r3916.h"
#include "../../14a.h"

#include "14a3_initiator.h"
#include "14a4_initiator.h"

uint8_t ST25R3916_14A_Anticoll(ST25R *pInstance, T4A_INFOS *infos);

void ST25R3916_14A_Initiator(ST25R *pInstance);

void ST25R3916_14A4_TxRx106(ST25R *pInstance);
void ST25R3916_14A4_TxRx212(ST25R *pInstance);
void ST25R3916_14A4_TxRx424(ST25R *pInstance);
void ST25R3916_14A4_TxRx848(ST25R *pInstance);
