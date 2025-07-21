#include "board.h"
#include <stdio.h>
#include <string.h>

uint8_t RAM_TRACE[TRACE_SIZE] = {0};
uint32_t RAM_TRACE_CB = 0;

__attribute__((section(".trace"))) const uint8_t FLASH_TRACE[TRACE_SIZE] = {TRACE_EVENT_INVALID};

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

void TRACE_Add_Event(TRACE_EVENT_TYPE type, const uint8_t *pbData, const uint16_t cbData)
{
	if((type == TRACE_EVENT_TYPE_DATA_IN) || (type == TRACE_EVENT_TYPE_DATA_OUT))
	{
		if((RAM_TRACE_CB + 1 + 1 + cbData + 1) <= sizeof(RAM_TRACE)) // + 1 field for Invalid when saving
		{
			RAM_TRACE[RAM_TRACE_CB++] = type;
			RAM_TRACE[RAM_TRACE_CB++] = (uint8_t) cbData;
			memcpy(RAM_TRACE + RAM_TRACE_CB, pbData, cbData);
			RAM_TRACE_CB += cbData;
		}
	}
	else if(type == TRACE_EVENT_TYPE_RAW_IRQ)
	{
		if((RAM_TRACE_CB + 1 + sizeof(uint32_t) + 1) <= sizeof(RAM_TRACE)) // + 1 field for Invalid when saving
		{
			RAM_TRACE[RAM_TRACE_CB++] = type;
			*(uint32_t *) (RAM_TRACE + RAM_TRACE_CB) = *(uint32_t *) pbData;
			RAM_TRACE_CB += sizeof(uint32_t);
		}
	}
	else
	{
		if((RAM_TRACE_CB + 1 + 1) <= sizeof(RAM_TRACE)) // + 1 field for Invalid when saving
		{
			RAM_TRACE[RAM_TRACE_CB++] = type;
		}
	}
}
extern void kprinthex(const void *lpData, const uint16_t cbData);

const char * IRQ_DESC[] = {
	"RFU", "RX_REST", "COL", "TXE", "RXE", "RXS", "FWL", "OSC",
	"NFCT", "CAT", "CAC", "EOF", "EON", "GPE", "NRE", "DCT",
	"WCAP", "WPH", "WAM", "WT", "ERR1", "ERR2", "PAR", "CRC",
	"WU_A", "WU_A_X", "RFU2", "WU_F", "RXE_PTA", "APON", "SL_WL", "PPON2",
};
void TRACE_Flash_IRQ_Desc(uint32_t irq)
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

void TRACE_Flash_Describe()
{
	uint32_t i;

	for(i = 0; (i < sizeof(FLASH_TRACE)) && (FLASH_TRACE[i] != TRACE_EVENT_INVALID); )
	{
		switch(FLASH_TRACE[i])
		{
		case TRACE_EVENT_TYPE_AC:
			printf("| AC / SELECT\r\n");
			i++;
			break;
		case TRACE_EVENT_TYPE_DATA_IN:
			printf("< ");
			kprinthex(FLASH_TRACE + i + 2, FLASH_TRACE[i + 1]);
			i += 1 + 1 + FLASH_TRACE[i + 1];
			break;
		case TRACE_EVENT_TYPE_DATA_OUT:
			printf("> ");
			kprinthex(FLASH_TRACE + i + 2, FLASH_TRACE[i + 1]);
			i += 1 + 1 + FLASH_TRACE[i + 1];
			break;
		case TRACE_EVENT_TYPE_FIELD_ON:
			printf("| FIELD ON\r\n");
			i++;
			break;
		case TRACE_EVENT_TYPE_FIELD_OFF:
			printf("| FIELD OFF\r\n");
			i++;
			break;
		case TRACE_EVENT_TYPE_RAW_IRQ:
			printf("| IRQ 0x%08lx -- ", *(uint32_t *)(FLASH_TRACE + i + 1));
			TRACE_Flash_IRQ_Desc(*(uint32_t *)(FLASH_TRACE + i + 1));
			printf("\r\n");
			i += 1 + sizeof(uint32_t);
			break;
		default:
			printf("????\r\n");
			LED_ON(LED_RED);
			while(1);
		}
	}
}

HAL_StatusTypeDef TRACE_FLASH_Save()
{
	HAL_StatusTypeDef status;
	uint32_t i;

	status = TRACE_FLASH_Erase(0);
	if(status == HAL_OK)
	{
		status = HAL_FLASHEx_Unlock_Bank2();
		if(status == HAL_OK)
		{
			LED_ON(LED_YELLOW);
			if(RAM_TRACE_CB && (RAM_TRACE_CB < sizeof(RAM_TRACE)))
			{
				RAM_TRACE[RAM_TRACE_CB++] = TRACE_EVENT_INVALID;

				for(i = 0; (status == HAL_OK) && (i < (TRACE_SIZE / 16)); i += 16)
				{
					status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t) (FLASH_TRACE + i), (uint32_t) (RAM_TRACE + i));
				}

				printf("RAM_TRACE to FLASH_TRACE (for %lu real bytes): %u\r\n", RAM_TRACE_CB, status);

				RAM_TRACE[0] = TRACE_EVENT_INVALID;
				RAM_TRACE_CB = 0;
			}
			else
			{
				RAM_TRACE[0] = TRACE_EVENT_INVALID;
				RAM_TRACE_CB = 0;
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
