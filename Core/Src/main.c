/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "../program/st25r/st25r.h"
#include "../program/st25r/st25r3916/14a/14a.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t SPI_Mutex = 0;

ST25R nfc0 = {
	.pSPI = &hspi1,
	.GPIOx_CS = N0_CS_GPIO_Port,
	.GPIO_Pin_CS = N0_CS_Pin,
	.pSPI_Mutex = &SPI_Mutex,
	.irqStatus = 0,
	.irqMask = ST25R3916_IRQ_MASK_NONE,
	.irqFlag = 0,
};

ST25R nfc1 = {
	.pSPI = &hspi1,
	.GPIOx_CS = N1_CS_GPIO_Port,
	.GPIO_Pin_CS = N1_CS_Pin,
	.pSPI_Mutex = &SPI_Mutex,
	.irqStatus = 0,
	.irqMask = ST25R3916_IRQ_MASK_NONE,
	.irqFlag = 0,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void kprinthex(const void *lpData, const uint16_t cbData);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const char KIWI_BANNER[] = 	"\x1b[2J\x1b[3J\x1b[H\r\n"
	"  .#####.         kst25r3916 on NUCLEO-H7A3ZI-Q (STM32)\r\n"
	" .## ^ ##.__ _\r\n"
	" ## / \\ /   ('>-  /***\r\n"
	" ## \\ / | K  |     Benjamin DELPY `gentilkiwi` ( benjamin@gentilkiwi.com )\r\n"
	" '## v #\\____/\r\n"
	"  '#####' L\\_      ***/\r\n";

typedef enum _TARGET_STATE {
	TARGET_STATE_IDLE = 0,
	TARGET_STATE_T3	= 1,
	TARGET_STATE_T4 = 2,
} TARGET_STATE, *PTARGET_STATE;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t ret;
	T4A_INFOS tgInfos;
	TARGET_STATE tgState;
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	puts(KIWI_BANNER);
	//printf("Device ID   : 0x%04x rev: 0x%04x\r\n", (uint16_t )HAL_GetDEVID(), (uint16_t ) HAL_GetREVID());
	printf("Flash size  : %hu Kbytes\r\n", *(const uint16_t*)FLASHSIZE_BASE);
	printf("Device UID  : 0x%08lx%08lx%08lx\r\n", HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
	printf("SysClockFreq: %lu MHz\r\n\r\n", HAL_RCC_GetSysClockFreq() / 1000 / 1000);

	printf("Flash trace : %p\r\n\r\n", FLASH_TRACE);
	TRACE_Flash_Describe();

	ST25R3916_Init(&nfc0);
	ST25R3916_14A_Initiator(&nfc0);
	printf("NFC #0 IC identity: 0x%02hx (initiator)\r\n", nfc0.icIdentity);

	ST25R3916_Init(&nfc1);
	ST25R3916_14A_Target(&nfc1);
	printf("NFC #1 IC identity: 0x%02hx (target)\r\n", nfc1.icIdentity);

	ret = ST25R3916_FieldOn_AC(&nfc0);
	if (ret == ST25R_STATUS_NO_ERROR)
	{
		do
		{
			LED_ON(LED_GREEN);
			ret = K14A_Anticoll(&nfc0, &tgInfos);
			if (ret == ST25R_STATUS_NO_ERROR)
			{
				LED_OFF(LED_GREEN);

				K14A3_TG_Prepare_AC_Buffer(&nfc1, &tgInfos.t3a);

				printf("ATQA: 0x%04x / %02hx %02hx\r\nSAK : 0x%02hx\r\nUID : ", tgInfos.t3a.ATQA, ((uint8_t*) &tgInfos.t3a.ATQA)[0], ((uint8_t*) &tgInfos.t3a.ATQA)[1], tgInfos.t3a.SAK);
				kprinthex(tgInfos.t3a.UID, tgInfos.t3a.cbUID);
				if (tgInfos.t3a.SAK & 0x20)
				{
					printf("ATS : ");
					kprinthex(tgInfos.ATS, tgInfos.cbATS);

					tgInfos.ATS[1] = (tgInfos.ATS[1] & 0xf0) | 0b0101;
					tgInfos.ATS[2] = 0x80;
					printf("ATS*: ");
					kprinthex(tgInfos.ATS, tgInfos.cbATS);

					printf("| Max bitrate: %u kbps - will answer %u kbps\r\n", ST25R_BITRATE_TO_KBPS(tgInfos.MaxBitRate), ST25R_BITRATE_TO_KBPS(ST25R_BITRATE_106));
					tgState = TARGET_STATE_T4;
				}
				else
				{
					tgState = TARGET_STATE_T3;
				}

				if (ret == ST25R_STATUS_NO_ERROR)
				{
					ST25R3916_Write_SingleRegister(&nfc1, ST25R3916_REG_OP_CONTROL, ST25R3916_REG_OP_CONTROL_en | ST25R3916_REG_OP_CONTROL_en_fd_auto_efd | ST25R3916_REG_OP_CONTROL_rx_en);
					ST25R3916_Write_SingleRegister(&nfc1, ST25R3916_REG_MODE, ST25R3916_REG_MODE_targ_targ | ST25R3916_REG_MODE_om_targ_nfca);
					ST25R3916_Mask_IRQ(&nfc1, ~(ST25R3916_IRQ_MASK_COL | ST25R3916_IRQ_MASK_EON | ST25R3916_IRQ_MASK_EOF | ST25R3916_IRQ_MASK_CRC | ST25R3916_IRQ_MASK_PAR | ST25R3916_IRQ_MASK_ERR2 | ST25R3916_IRQ_MASK_ERR1 | ST25R3916_IRQ_MASK_WU_A | ST25R3916_IRQ_MASK_WU_A_X), ST25R_IRQ_MASK_OP_ADD);

					do{
						if(tgState != TARGET_STATE_IDLE)
						{
							if(tgState == TARGET_STATE_T4)
							{
								K14A4_Deselect(&nfc0);
							}
							else if(tgState == TARGET_STATE_T3)
							{
								K14A3_HLTA(&nfc0);
							}

							if(tgInfos.CurrentBitrate != ST25R_BITRATE_106)
							{
								ST25R3916_14A4_TxRx106(&nfc0);
								tgInfos.CurrentBitrate = ST25R_BITRATE_106;
							}
							K14A3_Anticoll(&nfc0, &tgInfos.t3a);
							tgState = TARGET_STATE_IDLE;
						}

						ST25R3916_Mask_IRQ(&nfc1, ST25R3916_IRQ_MASK_RXE, ST25R_IRQ_MASK_OP_ADD);
						ST25R3916_DirectCommand(&nfc1, ST25R3916_CMD_GOTO_SENSE);

						do
						{
							do
							{
								ST25R3916_WaitForIRQ(&nfc1);
							} while(!nfc1.irqStatus);

							if(nfc1.irqStatus & (ST25R3916_IRQ_MASK_WU_A | ST25R3916_IRQ_MASK_WU_A_X))
							{
								ST25R3916_Mask_IRQ(&nfc1, ST25R3916_IRQ_MASK_RXE, ST25R_IRQ_MASK_OP_DEL);
								ST25R3916_DirectCommand(&nfc1, ST25R3916_CMD_CLEAR_FIFO);
								tgState = TARGET_STATE_T3;
								TRACE_Add_Event(TRACE_EVENT_TYPE_AC, NULL, 0);
							}
							else if(nfc1.irqStatus & ST25R3916_IRQ_MASK_RXE)
							{
								ST25R3916_Receive(&nfc1, 1);
								if(nfc1.cbData)
								{
									if ((nfc1.cbData == 2) && (nfc1.pbData[0] == K14A_HLTA) && (nfc1.pbData[1] == K14A_HLTA_2))
									{
										TRACE_Add_Event(TRACE_EVENT_TYPE_DATA_IN, nfc1.pbData, nfc1.cbData);
										break;
									}
									else if ((nfc1.cbData == 1) && (nfc1.pbData[0] == K14A_DESELECT))
									{
										const uint8_t myDeselect = K14A_DESELECT;
										ST25R3916_Transmit(&nfc1, &myDeselect, sizeof(myDeselect), 1);
										TRACE_Add_Event(TRACE_EVENT_TYPE_DATA_IN, nfc1.pbData, nfc1.cbData);
										TRACE_Add_Event(TRACE_EVENT_TYPE_DATA_OUT, &myDeselect, sizeof(myDeselect));
										break;
									}
									else if ((nfc1.cbData == 2) && (nfc1.pbData[0] == K14A_RATS))
									{
										ST25R3916_Transmit(&nfc1, tgInfos.ATS, tgInfos.cbATS, 1);
										TRACE_Add_Event(TRACE_EVENT_TYPE_DATA_IN, nfc1.pbData, nfc1.cbData);
										TRACE_Add_Event(TRACE_EVENT_TYPE_DATA_OUT, tgInfos.ATS, tgInfos.cbATS);
										ret = K14A4_Rats(&nfc0);
										if(ret == ST25R_STATUS_NO_ERROR)
										{
											/*
											 * Because EV2 XL, don't ask PPS too soon...
											 */
											ST25R3916_Write_GeneralPurposeTimer(&nfc0, 0x0380);
											ST25R3916_DirectCommand(&nfc0, ST25R3916_CMD_START_GP_TIMER);
											ST25R3916_WaitForIRQ(&nfc0);

											K14A4_AdjustBitRate(&nfc0, &tgInfos, ST25R_BITRATE_848);
											tgState = TARGET_STATE_T4;
										}
									}
									else
									{
										ret = ST25R3916_Transmit_then_Receive(&nfc0, nfc1.pbData, nfc1.cbData, 1);
										TRACE_Add_Event(TRACE_EVENT_TYPE_DATA_IN, nfc1.pbData, nfc1.cbData);
										if(ret == ST25R_STATUS_NO_ERROR)
										{
											ST25R3916_Transmit(&nfc1, nfc0.pbData, nfc0.cbData, 1);
											TRACE_Add_Event(TRACE_EVENT_TYPE_DATA_OUT, nfc0.pbData, nfc0.cbData);
										}
										else
										{
											break;
										}
									}
								}
							}
							else if(nfc1.irqStatus & ST25R3916_IRQ_MASK_EOF)
							{
								TRACE_Add_Event(TRACE_EVENT_TYPE_FIELD_OFF, NULL, 0);
								break;
							}
							else if(nfc1.irqStatus & ST25R3916_IRQ_MASK_EON)
							{
								TRACE_Add_Event(TRACE_EVENT_TYPE_FIELD_ON, NULL, 0);
							}
							else
							{
//								LED_ON(LED_RED);
//								printf("Target AC error with IRQ: 0x%08lx -- DEBUG IT !\r\n", nfc1.irqStatus);
//								while(1);
								TRACE_Add_Event(TRACE_EVENT_TYPE_RAW_IRQ, (uint8_t *) &nfc1.irqStatus, sizeof(nfc1.irqStatus));
								break;
							}

						} while(1);

					} while(1);
				}
			}
			else
			{
				printf("Error: 0x%02hx\r\n", ret);
				HAL_Delay(100);
				LED_OFF(LED_GREEN);
				HAL_Delay(100);
			}
		}
		while(1);

		ST25R3916_FieldOff(&nfc0);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /*AXI clock gating */
  RCC->CKGAENR = 0xE003FFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 35;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, N0_LED1_Pin|N0_LED4_Pin|N0_LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|N0_LED3_Pin|LED_RED_Pin|N1_LED6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(N0_LED5_GPIO_Port, N0_LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, N1_LED2_Pin|N1_LED4_Pin|N1_LED3_Pin|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, N0_CS_Pin|N1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(N1_LED1_GPIO_Port, N1_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, N0_LED6_Pin|N1_LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : N0_LED1_Pin N0_LED4_Pin N0_LED2_Pin */
  GPIO_InitStruct.Pin = N0_LED1_Pin|N0_LED4_Pin|N0_LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : N0_IRQ_Pin */
  GPIO_InitStruct.Pin = N0_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(N0_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin N0_LED3_Pin LED_RED_Pin N1_LED6_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|N0_LED3_Pin|LED_RED_Pin|N1_LED6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : N0_LED5_Pin */
  GPIO_InitStruct.Pin = N0_LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(N0_LED5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : N1_LED2_Pin N1_LED4_Pin N1_LED3_Pin LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = N1_LED2_Pin|N1_LED4_Pin|N1_LED3_Pin|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : N0_CS_Pin N1_CS_Pin */
  GPIO_InitStruct.Pin = N0_CS_Pin|N1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : N1_LED1_Pin */
  GPIO_InitStruct.Pin = N1_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(N1_LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : N1_IRQ_Pin */
  GPIO_InitStruct.Pin = N1_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(N1_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : N0_LED6_Pin N1_LED5_Pin */
  GPIO_InitStruct.Pin = N0_LED6_Pin|N1_LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(N0_IRQ_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(N0_IRQ_EXTI_IRQn);

  HAL_NVIC_SetPriority(N1_IRQ_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(N1_IRQ_EXTI_IRQn);

  HAL_NVIC_SetPriority(BUTTON_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BUTTON_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart3, (uint8_t *) &ch, 1, HAL_MAX_DELAY);
	return ch;
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case N0_IRQ_Pin:
		nfc0.irqFlag = 1;
		break;
	case N1_IRQ_Pin:
		nfc1.irqFlag = 1;
		break;
	case BUTTON_Pin:
		TRACE_FLASH_Save();
		break;
	}
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
