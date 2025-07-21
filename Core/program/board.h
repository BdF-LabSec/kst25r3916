/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#pragma once
#include "main.h"
#include "st25r/st25r.h"

#define LED_ON(led)		HAL_GPIO_WritePin(led ## _GPIO_Port, led ## _Pin, GPIO_PIN_SET)
#define LED_OFF(led)	HAL_GPIO_WritePin(led ## _GPIO_Port, led ## _Pin, GPIO_PIN_RESET)
#define LED_TOGGLE(led)	HAL_GPIO_TogglePin(led ## _GPIO_Port, led ## _Pin)

#define TRACE_NB_SECTORS	(16)
#define TRACE_SIZE			(TRACE_NB_SECTORS * FLASH_SECTOR_SIZE)

typedef enum _TRACE_EVENT_TYPE {
	TRACE_EVENT_TYPE_AC,
	//TRACE_EVENT_TYPE_RATS,
	TRACE_EVENT_TYPE_DATA_IN,
	TRACE_EVENT_TYPE_DATA_OUT,
	TRACE_EVENT_TYPE_FIELD_ON,
	TRACE_EVENT_TYPE_FIELD_OFF,
	TRACE_EVENT_TYPE_RAW_IRQ,

	TRACE_EVENT_INVALID = 0xff,
} TRACE_EVENT_TYPE, *PTRACE_EVENT_TYPE;

__attribute__((section(".trace"))) extern /*const */uint8_t FLASH_TRACE[TRACE_SIZE];

void TRACE_Add_Event(TRACE_EVENT_TYPE type, const uint8_t *pbData, const uint16_t cbData);
void TRACE_Flash_Describe();
HAL_StatusTypeDef TRACE_FLASH_Erase();
HAL_StatusTypeDef TRACE_FLASH_Save();

