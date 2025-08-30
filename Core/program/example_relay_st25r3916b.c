/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "st25r/st25r3916b/14a/14a4_initiator.h"
#include "st25r/st25r3916b/14a/14a3_target.h"
#include <string.h>

const char EXAMPLE_RELAY_ST25R3916B_BANNER[] = "> Relay for 14A4 ( ST25R3916B / ST25R3917B / ST25R3919B / ST25R3920B )\r\n";

void Example_Relay_ST25R3916B(ST25R *pReader, ST25R *pEmulator)
{
	uint8_t ret;
	T4A_INFOS tgInfos;
	TARGET_STATE tgState;
	uint8_t ATS[20], cbATS, FSD_Max = 8, RATS_Param;

	puts(EXAMPLE_RELAY_ST25R3916B_BANNER);
	TRACE_FLASH_Describe(TRACE_FLASH_IRQ_Describe_ST25R3916B);

	ST25R3916B_Init(pReader);
	ST25R3916B_14A_Initiator(pReader);
	printf("NFC #0 IC identity: %hu.%hu (0x%02hx) - initiator\r\n", pReader->icIdentity >> ST25R_REG_IC_IDENTITY_shift_ic_type, pReader->icIdentity & ST25R_REG_IC_IDENTITY_mask_ic_rev, pReader->icIdentity);

	ST25R3916B_Init(pEmulator);
	ST25R3916B_14A_Target(pEmulator);
	printf("NFC #1 IC identity: %hu.%hu (0x%02hx) - target\r\n", pEmulator->icIdentity >> ST25R_REG_IC_IDENTITY_shift_ic_type, pEmulator->icIdentity & ST25R_REG_IC_IDENTITY_mask_ic_rev, pEmulator->icIdentity);

	ret = ST25R3916B_FieldOn_AC(pReader);
	if (ret == ST25R_STATUS_NO_ERROR)
	{
		do
		{
			LED_ON(LED_GREEN);
			ret = ST25R3916B_14A_Anticoll(pReader, &tgInfos);
			if (ret == ST25R_STATUS_NO_ERROR)
			{
				LED_OFF(LED_GREEN);

				ST25R3916B_14A3_Target_Prepare_AC_Buffer(pEmulator, &tgInfos.t3a);

				printf("ATQA: 0x%04x / %02hx %02hx\r\nSAK : 0x%02hx\r\nUID : ", tgInfos.t3a.ATQA, ((uint8_t*) &tgInfos.t3a.ATQA)[0], ((uint8_t*) &tgInfos.t3a.ATQA)[1], tgInfos.t3a.SAK);
				kprinthex(tgInfos.t3a.UID, tgInfos.t3a.cbUID);
				if (tgInfos.t3a.SAK & 0x20)
				{
					tgState = TARGET_STATE_T4;

					printf("ATS : ");
					kprinthex(tgInfos.ATS, tgInfos.cbATS);

					printf("| MaxFrameSize         : %u - FSCI: %hu\r\n", tgInfos.MaxFrameSize, tgInfos.FSCI);
					printf("| FrameWaitingTime     : %u (4096/fc) - FWI: %hu\r\n", 1 << tgInfos.FWI, tgInfos.FWI);
					printf("| StartupFrameGuardTime: %u (4096/fc) - SFGI: %hu\r\n", 1 << tgInfos.SFGI, tgInfos.SFGI);
					printf("| Max bitrate          : %u kbps\r\n", ST25R_BITRATE_TO_KBPS(tgInfos.MaxBitRate));

					memcpy(ATS, tgInfos.ATS, tgInfos.cbATS); // we use another variable as tgInfos will be reused and we need to send back modified ATS
					cbATS = tgInfos.cbATS;

					ATS[1] = (ATS[1] & 0xf0) | MIN(tgInfos.FSCI, FSD_Max);
					ATS[2] = 0x80;
					ATS[3] += 0b00010001;
					printf("ATS*: ");
					kprinthex(ATS, cbATS);
				}
				else
				{
					tgState = TARGET_STATE_T3;
					cbATS = 0;
				}

				if (ret == ST25R_STATUS_NO_ERROR)
				{
					ST25R3916B_Write_SingleRegister(pEmulator, ST25R3916B_REG_OP_CONTROL, ST25R3916B_REG_OP_CONTROL_en | ST25R3916B_REG_OP_CONTROL_en_fd_auto_efd | ST25R3916B_REG_OP_CONTROL_rx_en);
					ST25R3916B_Write_SingleRegister(pEmulator, ST25R3916B_REG_MODE, ST25R3916B_REG_MODE_targ_targ | ST25R3916B_REG_MODE_om_targ_nfca);
					ST25R3916B_Mask_IRQ(pEmulator, ~(ST25R3916B_IRQ_MASK_COL | ST25R3916B_IRQ_MASK_EON | ST25R3916B_IRQ_MASK_EOF | ST25R3916B_IRQ_MASK_CRC | ST25R3916B_IRQ_MASK_PAR | ST25R3916B_IRQ_MASK_ERR2 | ST25R3916B_IRQ_MASK_ERR1 | ST25R3916B_IRQ_MASK_WU_A | ST25R3916B_IRQ_MASK_WU_A_X), ST25R_IRQ_MASK_OP_ADD);

					do{
						ST25R3916B_Mask_IRQ(pEmulator, ST25R3916B_IRQ_MASK_TXE | ST25R3916B_IRQ_MASK_RXE, ST25R_IRQ_MASK_OP_ADD);
						ST25R3916B_DirectCommand(pEmulator, ST25R3916B_CMD_GOTO_SENSE);

						if(tgState != TARGET_STATE_IDLE)
						{
							if(tgState == TARGET_STATE_T4)
							{
								ST25R3916B_14A4_Deselect(pReader);
							}
							else if(tgState == TARGET_STATE_T3)
							{
								ST25R3916B_14A3_HLTA(pReader);
							}

							if(tgInfos.CurrentBitrate != ST25R_BITRATE_106)
							{
								ST25R3916B_14A4_TxRx106(pReader);
								tgInfos.CurrentBitrate = ST25R_BITRATE_106;
							}
							ret = ST25R3916B_14A3_Anticoll(pReader, &tgInfos.t3a);
							if (ret == ST25R_STATUS_NO_ERROR)
							{
								tgState = TARGET_STATE_T3;
							}
						}

						do
						{
							ST25R3916B_WaitForIRQ(pEmulator);
							TRACE_RAM_Add(pEmulator, pEmulator->irqStatus, NULL, 0);

							if(pEmulator->irqStatus & (ST25R3916B_IRQ_MASK_WU_A | ST25R3916B_IRQ_MASK_WU_A_X))
							{
								ST25R3916B_Mask_IRQ(pEmulator, ST25R3916B_IRQ_MASK_TXE | ST25R3916B_IRQ_MASK_RXE, ST25R_IRQ_MASK_OP_DEL);
							}
							else if(pEmulator->irqStatus & ST25R3916B_IRQ_MASK_RXE)
							{
								ret = ST25R3916B_Receive_NoIRQ(pEmulator, 1);
								if(ret == ST25R_STATUS_NO_ERROR)
								{
									TRACE_RAM_Add(pEmulator, pEmulator->irqStatus, pEmulator->pbData, pEmulator->cbData);
									if ((pEmulator->cbData == 2) && (pEmulator->pbData[0] == K14A_HLTA) && (pEmulator->pbData[1] == K14A_HLTA_2))
									{
										ret = ST25T_STATUS_APPLICATION;
									}
									else if ((pEmulator->cbData == 1) && (pEmulator->pbData[0] == K14A_DESELECT))
									{
										ST25R3916B_Transmit(pEmulator, ST25R_14A4_DESELECT_data, sizeof(ST25R_14A4_DESELECT_data), 1);
										TRACE_RAM_Add(pEmulator, pEmulator->irqStatus, ST25R_14A4_DESELECT_data, sizeof(ST25R_14A4_DESELECT_data));
										ret = ST25T_STATUS_APPLICATION;
									}
									else if ((pEmulator->cbData == 2) && (pEmulator->pbData[0] == K14A_RATS))
									{
										ret = ST25R3916B_Transmit(pEmulator, ATS, cbATS, 1);
										TRACE_RAM_Add(pEmulator, pEmulator->irqStatus, ATS, cbATS);
										if(ret == ST25R_STATUS_NO_ERROR)
										{
											RATS_Param = (MIN(pEmulator->pbData[1] >> 4, FSD_Max) << 4) | (pEmulator->pbData[1] & 0b00001111);
											ret = ST25R3916B_14A4_Rats(pReader, RATS_Param, &tgInfos);
											if(ret == ST25R_STATUS_NO_ERROR)
											{
												ret = ST25R3916B_14A4_AdjustBitRate(pReader, &tgInfos, ST25R_BITRATE_848);
												if(ret == ST25R_STATUS_NO_ERROR)
												{
													tgState = TARGET_STATE_T4;
												}
											}
										}
									}
									else if(pEmulator->cbData)
									{
										ret = ST25R3916B_Transmit_then_Receive(pReader, pEmulator->pbData, pEmulator->cbData, 1);
										if(ret == ST25R_STATUS_NO_ERROR)
										{
											ret = ST25R3916B_Transmit(pEmulator, pReader->pbData, pReader->cbData, 1);
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
									break;
								}
							}
							else if(pEmulator->irqStatus == ST25R3916B_IRQ_MASK_EON) // TODO better
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

		} while(1);

		ST25R3916B_FieldOff(pReader);
	}
}
