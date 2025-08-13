/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "st25r3911b.h"

void ST25R3911B_Init(ST25R *pInstance)
{
	ST25R3911B_DirectCommand(pInstance, ST25R3911B_CMD_SET_DEFAULT);
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_OP_CONTROL, 0);

	pInstance->icIdentity = ST25R3911B_Read_SingleRegister(pInstance, ST25R3911B_REG_IC_IDENTITY);



	ST25R3911B_Write_Registers2_sep(pInstance, ST25R3911B_REG_IO_CONF1,
			ST25R3911B_REG_IO_CONF1_osc | ST25R3911B_REG_IO_CONF1_mask_out_cl | ST25R3911B_REG_IO_CONF1_lf_clk_off,
			ST25R3911B_REG_IO_CONF2_miso_pd2 | ST25R3911B_REG_IO_CONF2_miso_pd1
	);

	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_OP_CONTROL, ST25R3911B_REG_OP_CONTROL_en);
	ST25R3911B_WaitForIRQ(pInstance);
	ST25R3911B_Mask_IRQ(pInstance, ST25R3911B_IRQ_MASK_OSC, ST25R_IRQ_MASK_OP_ADD);


	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_REGULATOR_CONTROL, ST25R3911B_REG_REGULATOR_CONTROL_reg_s);
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_REGULATOR_CONTROL, 0x00);
	ST25R3911B_DirectCommand(pInstance, ST25R3911B_CMD_ADJUST_REGULATORS);
	ST25R3911B_WaitForIRQ(pInstance);
	ST25R3911B_Mask_IRQ(pInstance, ST25R3911B_IRQ_MASK_DCT, ST25R_IRQ_MASK_OP_ADD);


}

void ST25R3911B_WaitForIRQ(ST25R *pInstance)
{
	while(!pInstance->irqFlag);
	pInstance->irqFlag = 0;
	ST25R3911B_Read_IRQ(pInstance);
}

uint8_t ST25R3911B_Generic_IRQ_toErr(uint32_t irq)
{
	uint8_t status = ST25R_STATUS_NO_ERROR;

	if(irq & ST25R3911B_IRQ_MASK_CAC)
	{
		status |= ST25R_STATUS_COLLISION;
	}

	if(irq & ST25R3911B_IRQ_MASK_NRE)
	{
		status |= ST25R_STATUS_NO_RESPONSE;
	}

	if(irq & ST25R3911B_IRQ_MASK_CRC)
	{
		status |= ST25R_STATUS_CRC;
	}

	if(irq & (ST25R3911B_IRQ_MASK_PAR | ST25R3911B_IRQ_MASK_ERR2 | ST25R3911B_IRQ_MASK_ERR1))
	{
		status |= ST25R_STATUS_PARITY_FRAMING;
	}

	return status;
}

uint8_t ST25R3911B_WaitFor_SpecificIRQ(ST25R *pInstance, uint32_t SpecificIRQ)
{
	uint8_t ret;

	ST25R3911B_WaitForIRQ(pInstance);
	ret = ST25R3911B_Generic_IRQ_toErr(pInstance->irqStatus);
	if((ret == ST25R_STATUS_NO_ERROR) && !(pInstance->irqStatus & SpecificIRQ))
	{
		ret = ST25T_STATUS_OTHER_IRQ;
	}

	return ret;
}

uint8_t ST25R3911B_FieldOn_AC(ST25R *pInstance)
{
	uint8_t ret;

	ST25R3911B_DirectCommand(pInstance, ST25R3911B_CMD_INITIAL_RF_COLLISION);

	// no APON on 3911B ?

	ret = ST25R3911B_WaitFor_SpecificIRQ(pInstance, ST25R3911B_IRQ_MASK_CAT);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_OP_CONTROL, ST25R3911B_REG_OP_CONTROL_en | ST25R3911B_REG_OP_CONTROL_rx_man | ST25R3911B_REG_OP_CONTROL_rx_en | ST25R3911B_REG_OP_CONTROL_tx_en);
	}


	return ret;
}

void ST25R3911B_FieldOff(ST25R *pInstance)
{
	ST25R3911B_Write_SingleRegister(pInstance, ST25R3911B_REG_OP_CONTROL, ST25R3911B_REG_OP_CONTROL_en | ST25R3911B_REG_OP_CONTROL_rx_man);
}

uint8_t ST25R3911B_Fifo_Status(ST25R * pInstance, uint8_t *pStatus2)
{
	uint8_t buff[3] = {ST25R3911B_MK_READ(ST25R3911B_REG_FIFO_RX_STATUS1), };

	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT_RECEIVE(pInstance, buff, sizeof(buff));
	ST25R_SPI_COMM_RELEASE(pInstance);

	if(pStatus2)
	{
		*pStatus2 = buff[2] & 0x7f;
	}

	return buff[1] & 0x7f;
}

uint8_t ST25R3911B_Transmit_NoIRQ(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC)
{
	uint8_t buff[] = {ST25R3911B_MK_CMD(ST25R3911B_CMD_CLEAR_FIFO), ST25R3911B_MK_WRITE(ST25R3911B_REG_NUM_TX_BYTES1), cbData >> (8 - 3), cbData << 3}, ret = ST25R_STATUS_NO_ERROR;
	// CMD_CLEAR_FIFO needed ?
	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT(pInstance, buff, sizeof(buff));
	ST25R_SPI_COMM_RELEASE(pInstance);
	ST25R3911B_Fifo_Load(pInstance, pbData, cbData);

	ST25R_SPI_DirectCommand_internal(pInstance, bWithCRC ? ST25R3911B_MK_CMD(ST25R3911B_CMD_TRANSMIT_WITH_CRC) : ST25R3911B_MK_CMD(ST25R3911B_CMD_TRANSMIT_WITHOUT_CRC));

	return ret;
}

uint8_t ST25R3911B_Transmit(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC)
{
	uint8_t ret;

	ret = ST25R3911B_Transmit_NoIRQ(pInstance, pbData, cbData, bWithCRC);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		ret = ST25R3911B_WaitFor_SpecificIRQ(pInstance, ST25R3911B_IRQ_MASK_TXE);
	}

	return ret;
}

uint8_t ST25R3911B_Receive_NoIRQ(ST25R *pInstance, const uint8_t bWithCRC)
{
	uint8_t ret = ST25T_STATUS_BUFFER_ERR, status2;

	pInstance->cbData = ST25R3911B_Fifo_Status(pInstance, &status2);
	if(pInstance->cbData && !status2 && (pInstance->cbData <= sizeof(pInstance->pbData)))
	{
		ST25R3911B_Fifo_Read(pInstance, pInstance->pbData, pInstance->cbData);
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

uint8_t ST25R3911B_Receive(ST25R *pInstance, const uint8_t bWithCRC)
{
	uint8_t ret;

	//pInstance->cbData = 0;
	ret = ST25R3911B_WaitFor_SpecificIRQ(pInstance, ST25R3911B_IRQ_MASK_RXE);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		ret = ST25R3911B_Receive_NoIRQ(pInstance, bWithCRC);
	}

	return ret;
}

uint8_t ST25R3911B_Transmit_then_Receive(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC)
{
	uint8_t ret;

	if(pbData && cbData)
	{
		ret = ST25R3911B_Transmit(pInstance, pbData, cbData, bWithCRC);
	}
	else // Specific for WUPA/REQA
	{
		ret = ST25R3911B_WaitFor_SpecificIRQ(pInstance, ST25R3911B_IRQ_MASK_TXE);
	}

	if(ret == ST25R_STATUS_NO_ERROR)
	{
		ret = ST25R3911B_Receive(pInstance, bWithCRC);
	}

	return ret;
}
