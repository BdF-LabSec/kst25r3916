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
	HAL_Delay(0); // TODO better !!! (10µS)
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
