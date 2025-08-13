/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#include "st25r3916b.h"

void ST25R3916B_Init(ST25R *pInstance) // 4.1 # Power-on sequence
{
	ST25R3916B_DirectCommand(pInstance, ST25R3916B_CMD_SET_DEFAULT);
	pInstance->icIdentity = ST25R3916B_Read_SingleRegister(pInstance, ST25R3916B_REG_IC_IDENTITY);




	// The IO configuration register 1 and IO configuration register 2 must be properly configured.
	ST25R3916B_Write_Registers2_sep(pInstance, ST25R3916B_REG_IO_CONF1,
			ST25R3916B_REG_IO_CONF1_out_cl_disabled | ST25R3916B_REG_IO_CONF1_lf_clk_off,
			ST25R3916B_REG_IO_CONF2_sup3V_5V | ST25R3916B_REG_IO_CONF2_aat_en | ST25R3916B_REG_IO_CONF2_miso_pd2 | ST25R3916B_REG_IO_CONF2_miso_pd1 | ST25R3916B_REG_IO_CONF2_io_drv_lvl
	);

	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_OP_CONTROL, ST25R3916B_REG_OP_CONTROL_en | ST25R3916B_REG_OP_CONTROL_en_fd_efd_off);
	ST25R3916B_WaitForIRQ(pInstance);
	ST25R3916B_Mask_IRQ(pInstance, ST25R3916B_IRQ_MASK_OSC, ST25R_IRQ_MASK_OP_ADD);

	// The internal voltage regulators have to be configured. It is recommended to use direct command Adjust regulators to improve the system PSRR.
	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_REGULATOR_CONTROL, ST25R3916B_REG_REGULATOR_CONTROL_reg_s);
	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_REGULATOR_CONTROL, 0x00);
	ST25R3916B_DirectCommand(pInstance, ST25R3916B_CMD_ADJUST_REGULATORS);
	ST25R3916B_WaitForIRQ(pInstance);
	ST25R3916B_Mask_IRQ(pInstance, ST25R3916B_IRQ_MASK_DCT, ST25R_IRQ_MASK_OP_ADD);

	// No AAT
}

void ST25R3916B_WaitForIRQ(ST25R *pInstance)
{
	while(!pInstance->irqFlag);
	pInstance->irqFlag = 0;
	ST25R3916B_Read_IRQ(pInstance);
}

uint8_t ST25R3916B_Generic_IRQ_toErr(uint32_t irq)
{
	uint8_t status = ST25R_STATUS_NO_ERROR;

	if(irq & ST25R3916B_IRQ_MASK_CAC)
	{
		status |= ST25R_STATUS_COLLISION;
	}

	if(irq & ST25R3916B_IRQ_MASK_NRE)
	{
		status |= ST25R_STATUS_NO_RESPONSE;
	}

	if(irq & ST25R3916B_IRQ_MASK_CRC)
	{
		status |= ST25R_STATUS_CRC;
	}

	if(irq & (ST25R3916B_IRQ_MASK_PAR | ST25R3916B_IRQ_MASK_ERR2 | ST25R3916B_IRQ_MASK_ERR1))
	{
		status |= ST25R_STATUS_PARITY_FRAMING;
	}

	return status;
}

uint8_t ST25R3916B_WaitFor_SpecificIRQ(ST25R *pInstance, uint32_t SpecificIRQ)
{
	uint8_t ret;

	ST25R3916B_WaitForIRQ(pInstance);
	ret = ST25R3916B_Generic_IRQ_toErr(pInstance->irqStatus);
	if((ret == ST25R_STATUS_NO_ERROR) && !(pInstance->irqStatus & SpecificIRQ))
	{
		ret = ST25T_STATUS_OTHER_IRQ;
	}

	return ret;
}

uint8_t ST25R3916B_FieldOn_AC(ST25R *pInstance)
{
	uint8_t ret;

	ST25R3916B_DirectCommand(pInstance, ST25R3916B_CMD_INITIAL_RF_COLLISION);
	ret = ST25R3916B_WaitFor_SpecificIRQ(pInstance, ST25R3916B_IRQ_MASK_APON);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		ret = ST25R3916B_WaitFor_SpecificIRQ(pInstance, ST25R3916B_IRQ_MASK_CAT);
		if(ret == ST25R_STATUS_NO_ERROR)
		{
			ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_OP_CONTROL, ST25R3916B_REG_OP_CONTROL_en | ST25R3916B_REG_OP_CONTROL_en_fd_efd_off | ST25R3916B_REG_OP_CONTROL_rx_en | ST25R3916B_REG_OP_CONTROL_tx_en);
		}
	}

	return ret;
}

void ST25R3916B_FieldOff(ST25R *pInstance)
{
	ST25R3916B_Write_SingleRegister(pInstance, ST25R3916B_REG_OP_CONTROL, ST25R3916B_REG_OP_CONTROL_en | ST25R3916B_REG_OP_CONTROL_en_fd_efd_off);
}

uint16_t ST25R3916B_Fifo_Status(ST25R * pInstance, uint8_t *pStatus2)
{
	uint8_t buff[3] = {ST25R3916B_MK_READ(ST25R3916B_REG_FIFO_STATUS1), };

	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT_RECEIVE(pInstance, buff, sizeof(buff));
	ST25R_SPI_COMM_RELEASE(pInstance);

	if(pStatus2)
	{
		*pStatus2 = buff[2] & ~ST25R3916B_REG_FIFO_STATUS2_fifo_b_mask;
	}

	return ((buff[2] & ST25R3916B_REG_FIFO_STATUS2_fifo_b_mask) << 2) | buff[1];;
}

uint8_t ST25R3916B_Transmit_NoIRQ(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC)
{
	uint8_t buff[] = {ST25R3916B_MK_WRITE(ST25R3916B_REG_NUM_TX_BYTES1), cbData >> (8 - 3), cbData << 3}, ret = ST25R_STATUS_NO_ERROR;

	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT(pInstance, buff, sizeof(buff));
	ST25R_SPI_COMM_RELEASE(pInstance);
	ST25R3916B_Fifo_Load(pInstance, pbData, cbData);

	ST25R_SPI_DirectCommand_internal(pInstance, bWithCRC ? ST25R3916B_MK_CMD(ST25R3916B_CMD_TRANSMIT_WITH_CRC) : ST25R3916B_MK_CMD(ST25R3916B_CMD_TRANSMIT_WITHOUT_CRC));

	return ret;
}

uint8_t ST25R3916B_Transmit(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC)
{
	uint8_t ret;

	ret = ST25R3916B_Transmit_NoIRQ(pInstance, pbData, cbData, bWithCRC);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		ret = ST25R3916B_WaitFor_SpecificIRQ(pInstance, ST25R3916B_IRQ_MASK_TXE);
	}

	return ret;
}

uint8_t ST25R3916B_Receive_NoIRQ(ST25R *pInstance, const uint8_t bWithCRC)
{
	uint8_t ret = ST25T_STATUS_BUFFER_ERR, status2;

	pInstance->cbData = ST25R3916B_Fifo_Status(pInstance, &status2);
	if(pInstance->cbData && !status2 && (pInstance->cbData <= sizeof(pInstance->pbData)))
	{
		ST25R3916B_Fifo_Read(pInstance, pInstance->pbData, pInstance->cbData);
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

uint8_t ST25R3916B_Receive(ST25R *pInstance, const uint8_t bWithCRC)
{
	uint8_t ret;

	//pInstance->cbData = 0;
	ret = ST25R3916B_WaitFor_SpecificIRQ(pInstance, ST25R3916B_IRQ_MASK_RXE);
	if(ret == ST25R_STATUS_NO_ERROR)
	{
		ret = ST25R3916B_Receive_NoIRQ(pInstance, bWithCRC);
	}

	return ret;
}

uint8_t ST25R3916B_Transmit_then_Receive(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC)
{
	uint8_t ret;

	if(pbData && cbData)
	{
		ret = ST25R3916B_Transmit(pInstance, pbData, cbData, bWithCRC);
	}
	else // Specific for WUPA/REQA
	{
		ret = ST25R3916B_WaitFor_SpecificIRQ(pInstance, ST25R3916B_IRQ_MASK_TXE);
	}

	if(ret == ST25R_STATUS_NO_ERROR)
	{
		ret = ST25R3916B_Receive(pInstance, bWithCRC);
	}

	return ret;
}
