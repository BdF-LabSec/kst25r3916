/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a3_target.h"
#include <string.h>

void ST25R500_14A_Target(ST25R *pInstance)
{
	ST25R500_Mask_IRQ(pInstance, /*ST25R500_IRQ_MASK_TXE | */ST25R500_IRQ_MASK_RXS, ST25R_IRQ_MASK_OP_ADD);

}

void ST25R500_14A3_Target_Prepare_AC_Buffer(ST25R *pInstance, const T3A_INFOS *pInfos)
{

}
