/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#pragma once
#include "main.h"
#include <stdio.h>
#include "st25r/st25r.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define LED_ON(led)		HAL_GPIO_WritePin(led ## _GPIO_Port, led ## _Pin, GPIO_PIN_SET)
#define LED_OFF(led)	HAL_GPIO_WritePin(led ## _GPIO_Port, led ## _Pin, GPIO_PIN_RESET)
#define LED_TOGGLE(led)	HAL_GPIO_TogglePin(led ## _GPIO_Port, led ## _Pin)

#define TRACE_NB_SECTORS	(16)
#define TRACE_SIZE			(TRACE_NB_SECTORS * FLASH_SECTOR_SIZE)

void TRACE_RAM_Add(const void *source, const uint32_t irq, const uint8_t *pbData, const uint16_t cbData);
void TRACE_FLASH_Describe();
HAL_StatusTypeDef TRACE_FLASH_Erase();
HAL_StatusTypeDef TRACE_FLASH_Save();

void kprinthex(const void *lpData, const uint16_t cbData);
