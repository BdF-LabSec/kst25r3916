#include "board.h"
#include <stdio.h>
#include <string.h>

uint8_t RAM_TRACE[TRACE_SIZE] = {0x00, 0x00, 0x00, 0x00};
__attribute__((section(".trace"))) /*const */uint8_t FLASH_TRACE[TRACE_SIZE] = {0x00, 0x00, 0x00, 0x00};

typedef struct _TRACE_DATA {
	uint32_t timestamp;
	void *source;
	uint32_t irq;
	uint32_t optionalCbData;
} TRACE_DATA, *PTRACE_DATA;

HAL_StatusTypeDef TRACE_FLASH_Erase()
{
	HAL_StatusTypeDef status;
	FLASH_EraseInitTypeDef EraseInit = {
		.TypeErase = FLASH_TYPEERASE_SECTORS,
		.Banks = FLASH_BANK_2,
		.Sector = 240,
		.NbSectors = TRACE_NB_SECTORS,
		.VoltageRange = 0,
	};
	uint32_t SectorError;

	status = HAL_FLASHEx_Unlock_Bank2();
	if(status == HAL_OK)
	{
		status = HAL_FLASHEx_Erase(&EraseInit, &SectorError);
		if(status != HAL_OK)
		{
			LED_ON(LED_RED);
			printf("Error on sector %lu / 0x%08lx\r\n", SectorError, SectorError);
		}

		HAL_FLASHEx_Lock_Bank2();
	}

	return status;
}

void TRACE_RAM_Add(const void *source, const uint32_t irq, const uint8_t *pbData, const uint16_t cbData) // TODO: deal with size :')
{
	uint32_t *pRAM_TRACE_CB = (uint32_t *) RAM_TRACE;
	PTRACE_DATA pData = (PTRACE_DATA) (RAM_TRACE + sizeof(uint32_t) + *pRAM_TRACE_CB);

	pData->timestamp = HAL_GetTick();
	pData->source = (void *) source;
	pData->irq = irq;
	if(pbData && cbData)
	{
		pData->optionalCbData = cbData;
		memcpy(pData + 1, pbData, cbData);
	}
	else
	{
		pData->optionalCbData = 0;
	}

	*pRAM_TRACE_CB +=  __align_up(sizeof(TRACE_DATA) + pData->optionalCbData, __alignof__(TRACE_DATA));
}

const char * IRQ_DESC[] = {
	"RFU", "RX_REST", "COL", "TXE", "RXE", "RXS", "FWL", "OSC",
	"NFCT", "CAT", "CAC", "EOF", "EON", "GPE", "NRE", "DCT",
	"WCAP", "WPH", "WAM", "WT", "ERR1", "ERR2", "PAR", "CRC",
	"WU_A", "WU_A_X", "RFU2", "WU_F", "RXE_PTA", "APON", "SL_WL", "PPON2",
};
void TRACE_FLASH_IRQ_Describe(uint32_t irq)
{
	uint8_t i;
	for(i = 0; i < (sizeof(uint32_t) * 8); i++)
	{
		if(irq & ((uint32_t) 1 << i))
		{
			printf("%s ; ", IRQ_DESC[i]);
		}
	}
}

const char * IRQ_DESC_500[] = {
	"RX_ERR", "TXE", "RXS", "RXE", "RX_REST", "WL", "COL", "SUBC_START",
	"NFCT", "RXE_CE", "CE_SC", "RFU", "WPT_FOD", "WPT_STOP", "NRE", "GPE",
	"OSC", "WUT", "WUI", "WUQ", "DCT", "EON", "EOF", "WUTME",
};
void TRACE_FLASH_IRQ_Describe_500(uint32_t irq)
{
	uint8_t i;
	for(i = 0; i < ((sizeof(uint32_t) - 1) * 8); i++)
	{
		if(irq & ((uint32_t) 1 << i))
		{
			printf("%s ; ", IRQ_DESC_500[i]);
		}
	}
}

void TRACE_FLASH_Describe()
{
	uint32_t i, FLASH_TRACE_CB = *(uint32_t *) FLASH_TRACE;
	const TRACE_DATA *pData;

	printf("FLASH_TRACE @ %p (%lu)\r\n", FLASH_TRACE, FLASH_TRACE_CB);

	for(i = 0; i < FLASH_TRACE_CB; i += __align_up(sizeof(TRACE_DATA) + pData->optionalCbData, __alignof__(TRACE_DATA)))
	{
		pData = (PTRACE_DATA) (FLASH_TRACE + sizeof(uint32_t) + i);
		//printf("%p - %6lu 0x%08lx ", pData->source, pData->timestamp, pData->irq);
		printf("%6lu 0x%06lx ", pData->timestamp, pData->irq);
		TRACE_FLASH_IRQ_Describe_500(pData->irq);
		if(pData->optionalCbData)
		{
			kprinthex(pData + 1, pData->optionalCbData);
		}
		else
		{
			printf("\r\n");
		}
	}
}

HAL_StatusTypeDef TRACE_FLASH_Save()
{
	HAL_StatusTypeDef status;
	uint32_t i;
	uint32_t *pRAM_TRACE_CB = (uint32_t *) RAM_TRACE;

	status = TRACE_FLASH_Erase(0);
	if(status == HAL_OK)
	{
		status = HAL_FLASHEx_Unlock_Bank2();
		if(status == HAL_OK)
		{
			LED_ON(LED_YELLOW);
			if(*pRAM_TRACE_CB)
			{
				for(i = 0; (status == HAL_OK) && (i < TRACE_SIZE); i += 16)
				{
					status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t) (FLASH_TRACE + i), (uint32_t) (RAM_TRACE + i));
				}
				printf("RAM_TRACE to FLASH_TRACE: %u (%lu bytes)\r\n", status, *(uint32_t *) FLASH_TRACE);
			}
			else
			{
				status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t) FLASH_TRACE, (uint32_t) RAM_TRACE);
				printf("RAM_TRACE & FLASH_TRACE cleared: %u\r\n", status);
			}

			LED_OFF(LED_YELLOW);
			HAL_FLASHEx_Lock_Bank2();
		}
	}

	if(status != HAL_OK)
	{
		LED_ON(LED_RED);
	}

	return status;
}

void kprinthex(const void *lpData, const uint16_t cbData)
{
	uint16_t i;
	for(i = 0; i < cbData; i++)
	{
		printf("%02hx ", ((const uint8_t *)lpData)[i]);
	}
	printf("\r\n");
}
