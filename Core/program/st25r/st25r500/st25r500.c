/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "st25r500.h"

void ST25R500_Init(ST25R *pInstance)
{
	ST25R500_DirectCommand(pInstance, ST25R500_CMD_SET_DEFAULT);

	pInstance->icIdentity = ST25R500_Read_SingleRegister(pInstance, ST25R500_REG_IC_ID);

	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_GENERAL, ST25R500_REG_GENERAL_miso_pd1 | ST25R500_REG_GENERAL_miso_pd2);
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_REGULATOR, 0x00);
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_DRIVER, ST25R500_REG_DRIVER_regd_350mV | 0);
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_GPIO, ST25R500_REG_GPIO_gpio_rw1 | ST25R500_REG_GPIO_gpio_rw0);

	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_OPERATION, ST25R500_REG_OPERATION_en);
	ST25R500_WaitForIRQ(pInstance);
	ST25R500_Mask_IRQ(pInstance, ST25R500_IRQ_MASK_OSC, ST25R_IRQ_MASK_OP_ADD);

	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_REGULATOR, 0x00);
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_OPERATION, ST25R500_REG_OPERATION_en | ST25R500_REG_OPERATION_vdddr_en);
	HAL_Delay(1); // TODO better !!! (10µS)
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_OPERATION, ST25R500_REG_OPERATION_en | ST25R500_REG_OPERATION_vdddr_en | ST25R500_REG_OPERATION_rx_en | ST25R500_REG_OPERATION_tx_en);
	ST25R500_DirectCommand(pInstance, ST25R500_CMD_ADJUST_REGULATORS);
	ST25R500_WaitForIRQ(pInstance);
	ST25R500_Mask_IRQ(pInstance, ST25R500_IRQ_MASK_DCT, ST25R_IRQ_MASK_OP_ADD);
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_OPERATION, ST25R500_REG_OPERATION_en);
}

void ST25R500_WaitForIRQ(ST25R *pInstance)
{
	while(!pInstance->irqFlag);
	pInstance->irqFlag = 0;
	ST25R500_Read_IRQ(pInstance);
}

uint8_t ST25R500_Generic_IRQ_toErr(uint32_t irq, ST25R *pInstance)
{
	uint8_t status = ST25R_STATUS_NO_ERROR, ss3;

	if(irq & ST25R500_IRQ_MASK_COL)
	{
		status |= ST25R_STATUS_COLLISION;
	}

	if(irq & ST25R500_IRQ_MASK_NRE)
	{
		status |= ST25R_STATUS_NO_RESPONSE;
	}

	if(irq & ST25R500_IRQ_MASK_RX_ERR) // check with ST25R500_REG_STATUS_STATIC3
	{
		if(pInstance)
		{
			ss3 = ST25R500_Read_SingleRegister(pInstance, ST25R500_REG_STATUS_STATIC3);
			if(ss3 & ST25R500_REG_STATUS_STATIC3_s_crc)
			{
				status |= ST25R_STATUS_CRC;
			}

			if(ss3 & (ST25R500_REG_STATUS_STATIC3_s_par | ST25R500_REG_STATUS_STATIC3_s_hfe | ST25R500_REG_STATUS_STATIC3_s_sfe))
			{
				status |= ST25R_STATUS_PARITY_FRAMING;
			}
		}
		else
		{
			status |= ST25R_STATUS_GENERIC_ERR;
		}
	}

	return status;
}

uint8_t ST25R500_WaitFor_SpecificIRQ(ST25R *pInstance, uint32_t SpecificIRQ)
{
	uint8_t ret;

	ST25R500_WaitForIRQ(pInstance);
	ret = ST25R500_Generic_IRQ_toErr(pInstance->irqStatus, pInstance);
	if((ret == ST25R_STATUS_NO_ERROR) && !(pInstance->irqStatus & SpecificIRQ))
	{
		ret = ST25T_STATUS_OTHER_IRQ;
	}

	return ret;
}

uint8_t ST25R500_FieldOn_AC(ST25R *pInstance)
{
	uint8_t ret;

	ST25R500_Mask_IRQ(pInstance, ST25R500_IRQ_MASK_DCT, ST25R_IRQ_MASK_OP_DEL);

	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_NRT_GPT_CONF, ST25R500_REG_NRT_GPT_CONF_gptc_fon);

	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_OPERATION, ST25R500_REG_OPERATION_en | ST25R500_REG_OPERATION_vdddr_en);

	ST25R500_DirectCommand(pInstance, ST25R500_CMD_NFC_FIELD_ON);
	ret = ST25R500_WaitFor_SpecificIRQ(pInstance, ST25R500_IRQ_MASK_DCT);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		ret = ST25R500_WaitFor_SpecificIRQ(pInstance, ST25R500_IRQ_MASK_GPE);
		if(ret == ST25R_STATUS_NO_ERROR)
		{
			ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_OPERATION, ST25R500_REG_OPERATION_en | ST25R500_REG_OPERATION_vdddr_en | ST25R500_REG_OPERATION_rx_en | ST25R500_REG_OPERATION_tx_en);
		}
	}

	ST25R500_Mask_IRQ(pInstance, ST25R500_IRQ_MASK_DCT, ST25R_IRQ_MASK_OP_ADD);

	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_NRT_GPT_CONF, 0);

	return ret;
}

void ST25R500_FieldOff(ST25R *pInstance)
{
	ST25R500_Write_SingleRegister(pInstance, ST25R500_REG_OPERATION, ST25R500_REG_OPERATION_en);
}

uint16_t ST25R500_Fifo_Status(ST25R * pInstance, uint8_t *pStatus2)
{
	uint8_t buff[3] = {ST25R500_MK_READ(ST25R500_REG_FIFO_STATUS1), };

	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT_RECEIVE(pInstance, buff, sizeof(buff));
	ST25R_SPI_COMM_RELEASE(pInstance);

	if(pStatus2)
	{
		*pStatus2 = buff[2] & ~ST25R500_REG_FIFO_STATUS2_fifo_b8;
	}

	return ((buff[2] & ST25R500_REG_FIFO_STATUS2_fifo_b8) << 2) | buff[1];;
}

uint8_t ST25R500_Transmit_NoIRQ(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC/* not used*/)
{
	uint8_t buff[] = {ST25R500_MK_WRITE(ST25R500_REG_TX_FRAME1), cbData >> (8 - ST25R500_REG_TX_FRAME2_ntx_shift), cbData << ST25R500_REG_TX_FRAME2_ntx_shift}, ret = ST25R_STATUS_NO_ERROR;

	(void) bWithCRC;

	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT(pInstance, buff, sizeof(buff));
	ST25R_SPI_COMM_RELEASE(pInstance);
	ST25R500_Fifo_Load(pInstance, pbData, cbData);

	ST25R500_DirectCommand(pInstance, ST25R500_CMD_TRANSMIT);

	return ret;
}

uint8_t ST25R500_Transmit(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC/* not used*/)
{
	uint8_t ret;

	ret = ST25R500_Transmit_NoIRQ(pInstance, pbData, cbData, bWithCRC);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		ret = ST25R500_WaitFor_SpecificIRQ(pInstance, ST25R500_IRQ_MASK_TXE);
	}

	return ret;
}

uint8_t ST25R500_Receive_NoIRQ(ST25R *pInstance, const uint8_t bWithCRC)
{
	uint8_t ret = ST25T_STATUS_BUFFER_ERR, status2;

	pInstance->cbData = ST25R500_Fifo_Status(pInstance, &status2);
	if(pInstance->cbData && !status2 && (pInstance->cbData <= sizeof(pInstance->pbData)))
	{
		ST25R500_Fifo_Read(pInstance, pInstance->pbData, pInstance->cbData);
		if(bWithCRC)
		{
			if(pInstance->cbData >= 2)
			{
				pInstance->cbData -= 2;
				ret = ST25R_STATUS_NO_ERROR;
			}
		}
		else
		{
			ret = ST25R_STATUS_NO_ERROR;
		}
	}

	return ret;
}

uint8_t ST25R500_Receive(ST25R *pInstance, const uint8_t bWithCRC)
{
	uint8_t ret;

	//pInstance->cbData = 0;
	ret = ST25R500_WaitFor_SpecificIRQ(pInstance, ST25R500_IRQ_MASK_RXE);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		ret = ST25R500_Receive_NoIRQ(pInstance, bWithCRC);
	}

	return ret;
}

uint8_t ST25R500_Transmit_then_Receive(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC)
{
	uint8_t ret;

	if(pbData && cbData)
	{
		ret = ST25R500_Transmit(pInstance, pbData, cbData, bWithCRC);
	}
	else // Specific for WUPA/REQA
	{
		ret = ST25R500_WaitFor_SpecificIRQ(pInstance, ST25R500_IRQ_MASK_TXE);
	}

	if(ret == ST25R_STATUS_NO_ERROR)
	{
		ret = ST25R500_Receive(pInstance, bWithCRC);
	}

	return ret;
}

typedef struct _ST25R_REG_DEFINITION {
	const uint8_t reg;
	const uint8_t defaultValue;
	const char *name;
} ST25R_REG_DEFINITION;

const ST25R_REG_DEFINITION ST25R500_REGS_DEFINITION[] = {
	{ST25R500_REG_OPERATION, 0x00, "OPERATION"},
	{ST25R500_REG_GENERAL, 0x00, "GENERAL"},
	{ST25R500_REG_REGULATOR, 0x7f, "REGULATOR"},
	{ST25R500_REG_DRIVER, 0x20, "DRIVER"},
	{ST25R500_REG_TX_MOD1, 0x70, "TX_MOD1"},
	{ST25R500_REG_TX_MOD2, 0x00, "TX_MOD2"},
	{ST25R500_REG_CE_TX_MOD1, 0x80, "CE_TX_MOD1"},
	{ST25R500_REG_CE_TX_MOD2, 0x01, "CE_TX_MOD2"},
	{ST25R500_REG_GPIO, 0x00, "GPIO"},
	{ST25R500_REG_RX_ANA1, 0x53, "RX_ANA1"},
	{ST25R500_REG_RX_ANA2, 0x08, "RX_ANA2"},
	{ST25R500_REG_RX_ANA3, 0x85, "RX_ANA3"},
	{ST25R500_REG_RX_ANA4, 0x00, "RX_ANA4"},
	{ST25R500_REG_RX_DIG, 0x00, "RX_DIG"},
	{ST25R500_REG_CORR1, 0x00, "CORR1"},
	{ST25R500_REG_CORR2, 0x00, "CORR2"},
	{ST25R500_REG_CORR3, 0x00, "CORR3"},
	{ST25R500_REG_CORR4, 0x00, "CORR4"},
	{ST25R500_REG_CORR5, 0x00, "CORR5"},
	{ST25R500_REG_CORR6, 0x30, "CORR6"},
	{ST25R500_REG_PROTOCOL, 0x01, "PROTOCOL"},
	{ST25R500_REG_PROTOCOL_TX1, 0x00, "PROTOCOL_TX1"},
	{ST25R500_REG_PROTOCOL_TX2, 0x00, "PROTOCOL_TX2"},
	{ST25R500_REG_PROTOCOL_RX1, 0x00, "PROTOCOL_RX1"},
	{ST25R500_REG_PROTOCOL_RX2, 0x00, "PROTOCOL_RX2"},
	{ST25R500_REG_PROTOCOL_RX3, 0x00, "PROTOCOL_RX3"},
	{ST25R500_REG_EMD1, 0x00, "EMD1"},
	{ST25R500_REG_EMD2, 0x00, "EMD2"},
	{ST25R500_REG_CE_CONFIG1, 0x1d, "CE_CONFIG1"},
	{ST25R500_REG_CE_CONFIG2, 0x00, "CE_CONFIG2"},
	{ST25R500_REG_CE_CONFIG3, 0x00, "CE_CONFIG3"},
	{ST25R500_REG_MRT1, 0x21, "MRT1"},
	{ST25R500_REG_MRT2, 0x0b, "MRT2"},
	{ST25R500_REG_SQT, 0x00, "SQT"},
	{ST25R500_REG_NRT_GPT_CONF, 0x00, "NRT_GPT_CONF"},
	{ST25R500_REG_NRT1, 0x00, "NRT1"},
	{ST25R500_REG_NRT2, 0x00, "NRT2"},
	{ST25R500_REG_GPT1, 0x00, "GPT1"},
	{ST25R500_REG_GPT2, 0x00, "GPT2"},
	{ST25R500_REG_WAKEUP_CONF1, 0x00, "WAKEUP_CONF1"},
	{ST25R500_REG_WAKEUP_CONF2, 0x00, "WAKEUP_CONF2"},
	{ST25R500_REG_WAKEUP_CONF3, 0x00, "WAKEUP_CONF3"},
	{ST25R500_REG_WU_I_CONF, 0x00, "WU_I_CONF"},
	{ST25R500_REG_WU_I_DELTA, 0x8a, "WU_I_DELTA"},
	{ST25R500_REG_WU_I_CAL, 0x00, "WU_I_CAL"},
	{ST25R500_REG_WU_I_ADC, 0x00, "WU_I_ADC"},
	{ST25R500_REG_WU_I_REF, 0x00, "WU_I_REF"},
	{ST25R500_REG_WU_Q_CONF, 0x00, "WU_Q_CONF"},
	{ST25R500_REG_WU_Q_DELTA, 0x8a, "WU_Q_DELTA"},
	{ST25R500_REG_WU_Q_CAL, 0x00, "WU_Q_CAL"},
	{ST25R500_REG_WU_Q_ADC, 0x00, "WU_Q_ADC"},
	{ST25R500_REG_WU_Q_REF, 0x00, "WU_Q_REF"},
	{ST25R500_REG_TX_FRAME1, 0x00, "TX_FRAME1"},
	{ST25R500_REG_TX_FRAME2, 0x00, "TX_FRAME2"},
	{ST25R500_REG_FIFO_STATUS1, 0x00, "FIFO_STATUS1"},
	{ST25R500_REG_FIFO_STATUS2, 0x00, "FIFO_STATUS2"},
	{ST25R500_REG_COLLISION, 0x00, "COLLISION"},
	{ST25R500_REG_IRQ_MASK1, 0x00, "IRQ_MASK1"},
	{ST25R500_REG_IRQ_MASK2, 0x00, "IRQ_MASK2"},
	{ST25R500_REG_IRQ_MASK3, 0x00, "IRQ_MASK3"},
	//{ST25R500_REG_IRQ1, 0x00, "IRQ1"},
	//{ST25R500_REG_IRQ2, 0x00, "IRQ2"},
	//{ST25R500_REG_IRQ3, 0x00, "IRQ3"},
	{ST25R500_REG_IC_ID, 0xb1, "IC_ID"},
	{ST25R500_REG_STATUS1, 0x00, "STATUS1"},
	{ST25R500_REG_STATUS2, 0x00, "STATUS2"},
	{ST25R500_REG_STATUS_STATIC1, 0x00, "STATUS_STATIC1"},
	{ST25R500_REG_STATUS_STATIC2, 0x00, "STATUS_STATIC2"},
	{ST25R500_REG_STATUS_STATIC3, 0x00, "STATUS_STATIC3"},
	{ST25R500_REG_CE_STATUS1, 0x00, "CE_STATUS1"},
	{ST25R500_REG_CE_STATUS2, 0x00, "CE_STATUS2"},
	{ST25R500_REG_WU_STATUS, 0x80, "WU_STATUS"},
	{ST25R500_REG_ANA_DISPLAY1, 0x7f, "ANA_DISPLAY1"},
	{ST25R500_REG_ANA_DISPLAY2, 0x00, "ANA_DISPLAY2"},
	{ST25R500_REG_RSSI_I, 0x00, "RSSI_I"},
	{ST25R500_REG_RSSI_Q, 0x00, "RSSI_Q"},
	{ST25R500_REG_SENSE_DISPLAY, 0x00, "SENSE_DISPLAY"},
	{ST25R500_REG_AWS_CONFIG1, 0x00, "AWS_CONFIG1"},
	{ST25R500_REG_AWS_CONFIG2, 0x00, "AWS_CONFIG2"},
	{ST25R500_REG_AWS_TIME1, 0x00, "AWS_TIME1"},
	{ST25R500_REG_AWS_TIME2, 0x00, "AWS_TIME2"},
	{ST25R500_REG_AWS_TIME3, 0x00, "AWS_TIME3"},
	{ST25R500_REG_AWS_TIME4, 0x00, "AWS_TIME4"},
	{ST25R500_REG_OVERSHOOT_CONF, 0x00, "OVERSHOOT_CONF"},
	{ST25R500_REG_UNDERSHOOT_CONF, 0x00, "UNDERSHOOT_CONF"},
	{ST25R500_REG_EFD_THRESHOLD, 0x23, "EFD_THRESHOLD"},
};
#include <stdio.h>
void displayRegisters(ST25R *pInstance, uint8_t bOnlyDiffFromDefault)
{
	uint8_t i, val;
	for(i = 0; i < (sizeof(ST25R500_REGS_DEFINITION) / sizeof(ST25R500_REGS_DEFINITION[0])); i++)
	{
		val = ST25R500_Read_SingleRegister(pInstance, ST25R500_REGS_DEFINITION[i].reg);
		if(!bOnlyDiffFromDefault || (val != ST25R500_REGS_DEFINITION[i].defaultValue))
		{
			printf("%hu - %s\t0x%02hx\r\n", ST25R500_REGS_DEFINITION[i].reg, ST25R500_REGS_DEFINITION[i].name, val);
		}
	}
}
