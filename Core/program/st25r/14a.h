/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#pragma once
#include "st25r.h"

#define K14A_ALL_REQ__WUPA  0x52
#define K14A_SENS_REQ__REQA 0x26
// SDD_REQ: SEL_CMD - SEL_PAR - data bit 1 ... data bit n
#define K14A_SDD_REQ_CL1    0x93
#define K14A_SDD_REQ_CL2    0x95
#define K14A_SDD_REQ_CL3    0x97

// SEL_REQ: SEL_CMD - 70h - NFCID1 CLn - BCC
#define K14A_HLTA    		0x50
#define K14A_HLTA_2  		0x00

#define K14A_RATS    		0xe0 // 0x..

#define K14A_DESELECT		0xc2

#define K14A_CASCADE_TAG	0x88

typedef struct _T3A_INFOS {
    uint16_t ATQA;
    uint8_t cbUID;
    uint8_t UID[10];
    uint8_t SAK;
} T3A_INFOS, *PT3A_INFOS;

typedef struct _T4A_INFOS {
	T3A_INFOS t3a;
	uint8_t cbATS;
	uint8_t ATS[20];
	ST25R_BITRATE MaxBitRate;
	ST25R_BITRATE CurrentBitrate;
} T4A_INFOS, *PT4A_INFOS;

extern const uint8_t ST25R_14A3_HLTA_data[2];

void ST25R_14A4_AdjustMaxBitRate(T4A_INFOS *pt4aInfos);
