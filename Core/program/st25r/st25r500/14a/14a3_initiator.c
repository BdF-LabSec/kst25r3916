/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "14a3_initiator.h"
#include <string.h>

uint8_t ST25R500_14A3_Anticoll_CL(ST25R *pInstance, uint8_t cl, uint8_t UID[10], uint8_t *pSAK, uint8_t *pUIDIdx)
{
	uint8_t ret, buffer[7], recIdx;

	buffer[0] = cl;
	buffer[1] = 0x02 << 4;

	/* We know here that we're in 106 kbps, so we can write ST25R500_REG_PROTOCOL_TX1 to disable CRC*/
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_PROTOCOL_TX1, ST25R500_REG_PROTOCOL_TX1_a_tx_par_on | ST25R500_REG_PROTOCOL_TX1_tx_crc_off | (0 << ST25R500_REG_PROTOCOL_TX1_p_len_shift)); // OOK
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_PROTOCOL_RX1, ST25R500_REG_PROTOCOL_RX1_b_rx_sof | ST25R500_REG_PROTOCOL_RX1_b_rx_eof | ST25R500_REG_PROTOCOL_RX1_a_rx_par_on | ST25R500_REG_PROTOCOL_RX1_rx_crc_off);
	ret = ST25R500_Transmit_then_Receive(pInstance, buffer, 2, 0);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		if(pInstance->cbData == 5)
		{
			if((pInstance->pbData[0] ^ pInstance->pbData[1] ^ pInstance->pbData[2] ^ pInstance->pbData[3]) == pInstance->pbData[4])
			{
				recIdx = (pInstance->pbData[0] == K14A_CASCADE_TAG);
				memcpy(UID + *pUIDIdx, pInstance->pbData + recIdx, 4 - recIdx);
				*pUIDIdx += 4 - recIdx;

				buffer[1] = 0x07 << 4;
				memcpy(buffer + 2, pInstance->pbData, 5);
				ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_PROTOCOL_TX1, ST25R500_REG_PROTOCOL_TX1_a_tx_par_on | ST25R500_REG_PROTOCOL_TX1_tx_crc_on | (0 << ST25R500_REG_PROTOCOL_TX1_p_len_shift)); // OOK
				ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_PROTOCOL_RX1, ST25R500_REG_PROTOCOL_RX1_b_rx_sof | ST25R500_REG_PROTOCOL_RX1_b_rx_eof | ST25R500_REG_PROTOCOL_RX1_a_rx_par_on | ST25R500_REG_PROTOCOL_RX1_rx_crc_on);
				ret = ST25R500_Transmit_then_Receive(pInstance, buffer, 7, 1);
				if(ret == ST25R_STATUS_NO_ERROR)
				{
					if(pInstance->cbData == 1)
					{
						*pSAK = pInstance->pbData[0];
						if(((*pSAK >> 2) & 1) ^ recIdx)
						{
							ret = ST25T_STATUS_APPLICATION;
						}
					}
					else
					{
						ret = ST25T_STATUS_APPLICATION;
					}
				}
			}
			else
			{
				ret = ST25T_STATUS_APPLICATION;
			}
		}
		else
		{
			ret = ST25T_STATUS_APPLICATION;
		}
	}

	return ret;
}

void ST25R500_14A3_WUPA(ST25R *pInstance)
{
	uint8_t buff[] = {ST25R500_MK_WRITE(ST25R500_REG_TX_FRAME1), 0, 0 | 7};

	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT(pInstance, buff, sizeof(buff));
	ST25R_SPI_COMM_RELEASE(pInstance);
	ST25R500_Fifo_Load(pInstance, ST25R_14A3_WUPA_data, sizeof(ST25R_14A3_WUPA_data));

	ST25R500_DirectCommand(pInstance, ST25R500_CMD_TRANSMIT);
}

uint8_t ST25R500_14A3_Anticoll(ST25R *pInstance, T3A_INFOS *pInfos)
{
	uint8_t ret;

	pInfos->cbUID = 0;

	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_PROTOCOL_RX1, ST25R500_REG_PROTOCOL_RX1_b_rx_sof | ST25R500_REG_PROTOCOL_RX1_b_rx_eof | ST25R500_REG_PROTOCOL_RX1_a_rx_par_on | ST25R500_REG_PROTOCOL_RX1_rx_crc_off);
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_NRT_GPT_CONF, ST25R500_REG_NRT_GPT_CONF_nrt_step_64fc);
	ST25R500_Write_NoResponseTimer(pInstance, 0x0080);

	ST25R500_14A3_WUPA(pInstance);
	ret = ST25R500_Transmit_then_Receive(pInstance, NULL, 0, 0);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		if(pInstance->cbData == sizeof(pInfos->ATQA))
		{
			pInfos->ATQA = *(uint16_t *) pInstance->pbData;
			if(pInfos->ATQA & 0x1f) // anticoll
			{
				ret = ST25R500_14A3_Anticoll_CL(pInstance, K14A_SDD_REQ_CL1, pInfos->UID, &pInfos->SAK, &pInfos->cbUID);
				if ((ret == ST25R_STATUS_NO_ERROR) && (pInfos->SAK & 0x04))
				{
					ret = ST25R500_14A3_Anticoll_CL(pInstance, K14A_SDD_REQ_CL2, pInfos->UID, &pInfos->SAK, &pInfos->cbUID);
					if ((ret == ST25R_STATUS_NO_ERROR) && (pInfos->SAK & 0x04))
					{
						ret = ST25R500_14A3_Anticoll_CL(pInstance, K14A_SDD_REQ_CL3, pInfos->UID, &pInfos->SAK, &pInfos->cbUID);
						if ((ret == ST25R_STATUS_NO_ERROR) && (pInfos->SAK & 0x04))
						{
							ret = ST25T_STATUS_APPLICATION; // not valid at this point
						}
					}
				}
			}
			else
			{
				ret = ST25T_STATUS_APPLICATION;
			}
		}
		else
		{
			ret = ST25T_STATUS_APPLICATION;
		}
	}

	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_PROTOCOL_RX1, ST25R500_REG_PROTOCOL_RX1_b_rx_sof | ST25R500_REG_PROTOCOL_RX1_b_rx_eof | ST25R500_REG_PROTOCOL_RX1_a_rx_par_on | ST25R500_REG_PROTOCOL_RX1_rx_crc_on);

	HAL_Delay(1 + ((1 << 2) / 3)); // TODO better

	return ret;
}

uint8_t ST25R500_14A3_HLTA(ST25R *pInstance)
{
	uint8_t ret;

	ST25R500_Write_NoResponseTimer(pInstance, 0);
	ret = ST25R500_Transmit(pInstance, ST25R_14A3_HLTA_data, sizeof(ST25R_14A3_HLTA_data), 1);
	HAL_Delay(2); // TODO better

	return ret;
}
