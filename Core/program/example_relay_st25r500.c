/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "st25r/st25r500/14a/14a4_initiator.h"
#include "st25r/st25r500/14a/14a3_target.h"

void Example_Relay_ST25R500(ST25R *pReader, ST25R *pEmulator)
{
	uint8_t ret, ce_status;
	T4A_INFOS tgInfos;
	TARGET_STATE tgState;
	uint8_t FSD_Max = 8, RATS_Param;

	ST25R500_Init(pReader);
	ST25R500_14A_Initiator(pReader);
	printf("NFC #0 IC identity: %hu.%hu (0x%02hx) - initiator\r\n", pReader->icIdentity >> ST25R_REG_IC_IDENTITY_shift_ic_type, pReader->icIdentity & ST25R_REG_IC_IDENTITY_mask_ic_rev, pReader->icIdentity);

	ST25R500_Init(pEmulator);
	ST25R500_14A_Target(pEmulator);
	printf("NFC #1 IC identity: %hu.%hu (0x%02hx) - target\r\n", pEmulator->icIdentity >> ST25R_REG_IC_IDENTITY_shift_ic_type, pEmulator->icIdentity & ST25R_REG_IC_IDENTITY_mask_ic_rev, pEmulator->icIdentity);

	ret = ST25R500_FieldOn_AC(pReader);
	if (ret == ST25R_STATUS_NO_ERROR)
	{
		do
		{
			LED_ON(LED_GREEN);
			ret = ST25R500_14A_Anticoll(pReader, &tgInfos);
			if (ret == ST25R_STATUS_NO_ERROR)
			{
				LED_OFF(LED_GREEN);

				printf("ATQA: 0x%04x / %02hx %02hx\r\nSAK : 0x%02hx\r\nUID : ", tgInfos.t3a.ATQA, ((uint8_t*) &tgInfos.t3a.ATQA)[0], ((uint8_t*) &tgInfos.t3a.ATQA)[1], tgInfos.t3a.SAK);
				kprinthex(tgInfos.t3a.UID, tgInfos.t3a.cbUID);
				if (tgInfos.t3a.SAK & 0x20)
				{
					printf("ATS : ");
					kprinthex(tgInfos.ATS, tgInfos.cbATS);

					printf("| MaxFrameSize         : %u - FSCI: %hu\r\n", tgInfos.MaxFrameSize, tgInfos.FSCI);
					printf("| FrameWaitingTime     : %u (4096/fc) - FWI: %hu\r\n", 1 << tgInfos.FWI, tgInfos.FWI);
					printf("| StartupFrameGuardTime: %u (4096/fc) - SFGI: %hu\r\n", 1 << tgInfos.SFGI, tgInfos.SFGI);
					printf("| Max bitrate          : %u kbps\r\n", ST25R_BITRATE_TO_KBPS(tgInfos.MaxBitRate));

					tgInfos.ATS[1] =  (tgInfos.ATS[1] & 0xf0) | MIN(tgInfos.FSCI, FSD_Max);
					tgInfos.ATS[2] = 0x80;
					tgInfos.ATS[3] += 0b00010001;

					printf("ATS*: ");
					kprinthex(tgInfos.ATS, tgInfos.cbATS);

					tgState = TARGET_STATE_T4;
					ST25R500_14A4_Target_Prepare_AC_Buffer(pEmulator, &tgInfos);
				}
				else
				{
					ret = ST25T_STATUS_APPLICATION;
				}

				if (ret == ST25R_STATUS_NO_ERROR)
				{
					ST25R500_Write_SingleRegister(pEmulator, ST25R500_REG_OPERATION, ST25R500_REG_OPERATION_en | ST25R500_REG_OPERATION_rx_en | ST25R500_REG_OPERATION_ce_en);
					ST25R500_Write_SingleRegister(pEmulator, ST25R500_REG_CE_CONFIG1, ST25R500_REG_CE_CONFIG1_en_ce4a | ST25R500_REG_CE_CONFIG1_en_106_ac_a);
					ST25R500_Mask_IRQ(pEmulator, ~(ST25R500_IRQ_MASK_COL | ST25R500_IRQ_MASK_EON | ST25R500_IRQ_MASK_EOF | ST25R500_IRQ_MASK_RX_ERR | ST25R500_IRQ_MASK_CE_SC), ST25R_IRQ_MASK_OP_ADD);

					do{
						ST25R500_Mask_IRQ(pEmulator, ST25R500_IRQ_MASK_TXE | ST25R500_IRQ_MASK_RXE, ST25R_IRQ_MASK_OP_ADD);
						if(tgState != TARGET_STATE_IDLE)
						{
							if(tgState == TARGET_STATE_T4)
							{
								ST25R500_14A4_Deselect(pReader);
							}
							else if(tgState == TARGET_STATE_T3)
							{
								ST25R500_14A3_HLTA(pReader);
							}

							if(tgInfos.CurrentBitrate != ST25R_BITRATE_106)
							{
								ST25R500_14A4_TxRx106(pReader);
								tgInfos.CurrentBitrate = ST25R_BITRATE_106;
							}
							ret = ST25R500_14A3_Anticoll(pReader, &tgInfos.t3a);
							if (ret == ST25R_STATUS_NO_ERROR)
							{
								tgState = TARGET_STATE_T3;
							}
						}

						do
						{
							ST25R500_WaitForIRQ(pEmulator);
							TRACE_RAM_Add(pEmulator, pEmulator->irqStatus, NULL, 0);

							if(pEmulator->irqStatus & ST25R500_IRQ_MASK_CE_SC)
							{
								ce_status = ST25R500_Read_SingleRegister(pEmulator, ST25R500_REG_CE_STATUS2);
								RATS_Param = (MIN(ce_status >> 4, FSD_Max) << 4) | (ce_status & 0b00001111);
								ret = ST25R500_14A4_Rats(pReader, RATS_Param, &tgInfos);
								if(ret == ST25R_STATUS_NO_ERROR)
								{
									ret = ST25R500_14A4_AdjustBitRate(pReader, &tgInfos, ST25R_BITRATE_848);
									if(ret == ST25R_STATUS_NO_ERROR)
									{
										ST25R500_Mask_IRQ(pEmulator, ST25R500_IRQ_MASK_TXE | ST25R500_IRQ_MASK_RXE, ST25R_IRQ_MASK_OP_DEL);
										tgState = TARGET_STATE_T4;
									}
								}

								if(ret != ST25R_STATUS_NO_ERROR)
								{
									ST25R500_Write_SingleRegister(pEmulator, ST25R500_REG_CE_STATUS1, ST25R500_REG_CE_STATUS1_ce_state_idle);
									break;
								}
							}
							else if(pEmulator->irqStatus & ST25R500_IRQ_MASK_RXE)
							{
								ret = ST25R500_Receive_NoIRQ(pEmulator, 1);
								if(ret == ST25R_STATUS_NO_ERROR)
								{
									TRACE_RAM_Add(pEmulator, pEmulator->irqStatus, pEmulator->pbData, pEmulator->cbData);
									if ((pEmulator->cbData == 1) && (pEmulator->pbData[0] == K14A_DESELECT))
									{
										ST25R500_Transmit(pEmulator, ST25R_14A4_DESELECT_data, sizeof(ST25R_14A4_DESELECT_data), 1);
										TRACE_RAM_Add(pEmulator, pEmulator->irqStatus, ST25R_14A4_DESELECT_data, sizeof(ST25R_14A4_DESELECT_data));
										ret = ST25T_STATUS_APPLICATION;
									}
									else if(pEmulator->cbData)
									{
										ret = ST25R500_Transmit_then_Receive(pReader, pEmulator->pbData, pEmulator->cbData, 1);
										if(ret == ST25R_STATUS_NO_ERROR)
										{
											ret = ST25R500_Transmit(pEmulator, pReader->pbData, pReader->cbData, 1);
											TRACE_RAM_Add(pEmulator, pEmulator->irqStatus, pReader->pbData, pReader->cbData);
										}
									}
									else
									{
										ret = ST25T_STATUS_BUFFER_ERR;
									}
								}

								if(ret != ST25R_STATUS_NO_ERROR)
								{
									ST25R500_Write_SingleRegister(pEmulator, ST25R500_REG_CE_STATUS1, ST25R500_REG_CE_STATUS1_ce_state_idle);
									break;
								}
							}
							else if(pEmulator->irqStatus == ST25R500_IRQ_MASK_EON) // TODO better
							{
								;
							}
							else
							{
								break;
							}

						} while(1);

					} while(1);
				}

			}
			else
			{
				printf("Error: 0x%02hx\r\n", ret);
				HAL_Delay(500);
				LED_OFF(LED_GREEN);
				HAL_Delay(500);
			}
		}
		while(1);

		ST25R500_FieldOff(pReader);
	}
}
