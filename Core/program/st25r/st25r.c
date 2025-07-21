#include "st25r.h"

void ST25R_SPI_DirectCommand_internal(ST25R *pInstance, uint8_t CommandCode_Preparred)
{
	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT(pInstance, &CommandCode_Preparred, 1);
	ST25R_SPI_COMM_RELEASE(pInstance);
}

uint8_t ST25R_SPI_Read_SingleRegister_internal(ST25R *pInstance, const uint8_t PreCommandCode_Preparred, const uint8_t Register_Prepared)
{
	uint8_t buff[3] = {PreCommandCode_Preparred, Register_Prepared, }, offset = PreCommandCode_Preparred ? 0 : 1;

	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT_RECEIVE(pInstance, buff + offset, sizeof(buff) - offset);
	ST25R_SPI_COMM_RELEASE(pInstance);

	return buff[2];
}

void ST25R_SPI_Write_SingleRegister_internal(ST25R *pInstance, const uint8_t PreCommandCode_Preparred, const uint8_t Register_Prepared, const uint8_t Value)
{
	uint8_t buff[3] = {PreCommandCode_Preparred, Register_Prepared, Value}, offset = PreCommandCode_Preparred ? 0 : 1;
	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT(pInstance, buff + offset, sizeof(buff) - offset);
	ST25R_SPI_COMM_RELEASE(pInstance);
}

void ST25R_SPI_Write_Registers2_internal(ST25R *pInstance, const uint8_t PreCommandCode_Preparred, const uint8_t Register_Prepared, const uint16_t Value)
{
	uint8_t buff[4] = {PreCommandCode_Preparred, Register_Prepared, }, offset = PreCommandCode_Preparred ? 0 : 1;
	*(uint16_t *) (buff + 2) = Value;

	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT(pInstance, buff + offset, sizeof(buff) - offset);
	ST25R_SPI_COMM_RELEASE(pInstance);
}

void ST25R_SPI_Write_Registers4_internal(ST25R *pInstance, const uint8_t PreCommandCode_Preparred, const uint8_t Register_Prepared, const uint32_t Value)
{
	uint8_t buff[6] = {PreCommandCode_Preparred, Register_Prepared, }, offset = PreCommandCode_Preparred ? 0 : 1;
	*(uint32_t *) (buff + 2) = Value;

	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT(pInstance, buff + offset, sizeof(buff) - offset);
	ST25R_SPI_COMM_RELEASE(pInstance);
}


void ST25R_SPI_Read_Multiple_internal(ST25R *pInstance, const uint8_t PreCommandCode_Preparred, uint8_t *pbData, const uint16_t cbData)
{
	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT(pInstance, &PreCommandCode_Preparred, sizeof(PreCommandCode_Preparred));
	ST25R_SPI_COMM_RECEIVE(pInstance, pbData, cbData);
	ST25R_SPI_COMM_RELEASE(pInstance);
}

void ST25R_SPI_Write_Multiple_internal(ST25R * pInstance, const uint8_t PreCommandCode_Preparred, const uint8_t *pbData, const uint16_t cbData)
{
	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT(pInstance, &PreCommandCode_Preparred, sizeof(PreCommandCode_Preparred));
	ST25R_SPI_COMM_TRANSMIT(pInstance, pbData, cbData);
	ST25R_SPI_COMM_RELEASE(pInstance);
}

void ST25R_SPI_Read_IRQ_internal(ST25R *pInstance, const uint8_t Register_Prepared, const ST25R_IRQ_CB cb)
{
	uint8_t buff[5] = {Register_Prepared, };

	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT_RECEIVE(pInstance, buff, cb + 1);
	ST25R_SPI_COMM_RELEASE(pInstance);

	pInstance->irqStatus = (*(uint32_t *) (buff + 1)) & ~pInstance->irqMask;
	pInstance->irqFlag = 0;
}

void ST25R_SPI_Write_IRQ_Mask_internal(ST25R *pInstance, const uint8_t Register_Prepared, const ST25R_IRQ_CB cb)
{
	uint8_t buff[5] = {Register_Prepared, };

	*(uint32_t *) (buff + 1) = pInstance->irqMask;

	ST25R_SPI_COMM_ACQUIRE(pInstance);
	ST25R_SPI_COMM_TRANSMIT(pInstance, buff, cb + 1);
	ST25R_SPI_COMM_RELEASE(pInstance);
}

void ST25R_SPI_Write_IRQ_Mask_Operation_internal(ST25R *pInstance, const uint32_t mask, const ST25R_IRQ_MASK_OP op, const uint8_t Register_Prepared, const ST25R_IRQ_CB cb)
{
	uint8_t bFlag = 0x00;

	if(op == ST25R_IRQ_MASK_OP_SET)
	{
		if(pInstance->irqMask != mask)
		{
			pInstance->irqMask = mask;
			bFlag = 0x01;
		}
	}
	else if(op == ST25R_IRQ_MASK_OP_ADD)
	{
		if((mask & pInstance->irqMask) != mask)
		{
			pInstance->irqMask |= mask;
			bFlag = 0x01;
		}
	}
	else if(op == ST25R_IRQ_MASK_OP_DEL)
	{
		if(mask & pInstance->irqMask)
		{
			pInstance->irqMask &= ~mask;
			bFlag = 0x01;
		}
	}

	if(bFlag)
	{
		ST25R_SPI_Write_IRQ_Mask_internal(pInstance, Register_Prepared, cb);
	}
}
