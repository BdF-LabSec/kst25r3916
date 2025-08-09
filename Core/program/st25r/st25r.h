/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#pragma once
#include "../board.h"

#define ST25R_DEFAULT_FIFO_SIZE	(512)

typedef struct _ST25R {
	SPI_HandleTypeDef *pSPI;
	GPIO_TypeDef *GPIOx_CS;
	uint16_t GPIO_Pin_CS;
	uint8_t *pSPI_Mutex;
	uint32_t irqStatus;
	uint32_t irqMask;
	uint8_t pbData[ST25R_DEFAULT_FIFO_SIZE];
	uint16_t cbData;
	volatile uint8_t irqFlag;
	uint8_t icIdentity;
} ST25R, *PST25R;

#define ST25R_STATUS_NO_ERROR		0x00
#define ST25R_STATUS_NO_RESPONSE	0x01
#define ST25R_STATUS_COLLISION		0x02
#define ST25R_STATUS_CRC			0x04
#define ST25R_STATUS_PARITY_FRAMING	0x08

#define ST25T_STATUS_BUFFER_ERR		0x20
#define ST25T_STATUS_OTHER_IRQ		0x40
#define ST25T_STATUS_APPLICATION	0x80

typedef enum _ST25R_IRQ_CB {
	ST25R_IRQ_CB_3 = 3,
	ST25R_IRQ_CB_4 = 4,
} ST25R_IRQ_CB, *PST25R_IRQ_CB;

typedef enum _ST25R_IRQ_MASK_OP {
	ST25R_IRQ_MASK_OP_SET = 0,
	ST25R_IRQ_MASK_OP_ADD = 1,
	ST25R_IRQ_MASK_OP_DEL = 2,
} ST25R_IRQ_MASK_OP, *PST25R_IRQ_MASK_OP;

typedef enum _ST25R_BITRATE {
	ST25R_BITRATE_106 = 0,
	ST25R_BITRATE_212 = 1,
	ST25R_BITRATE_424 = 2,
	ST25R_BITRATE_848 = 3,
} ST25R_BITRATE, *PST25R_BITRATE;

#define ST25R_BITRATE_TO_KBPS(b)	((uint16_t) 106 * (1 << (b)))

#define ST25R_SPI_COMM_ACQUIRE(s)					do { \
		while(*((s)->pSPI_Mutex)); \
		(*((s)->pSPI_Mutex) = 1); \
		HAL_GPIO_WritePin((s)->GPIOx_CS, (s)->GPIO_Pin_CS, GPIO_PIN_RESET); \
	} while(0)

#define ST25R_SPI_COMM_RELEASE(s)					do { \
		HAL_GPIO_WritePin((s)->GPIOx_CS, (s)->GPIO_Pin_CS, GPIO_PIN_SET); \
		(*((s)->pSPI_Mutex) = 0); \
	} while(0)

#define ST25R_SPI_COMM_TRANSMIT(s, d, c)			HAL_SPI_Transmit((s)->pSPI, (d), (c), HAL_MAX_DELAY)
#define ST25R_SPI_COMM_RECEIVE(s, d, c)				HAL_SPI_Receive((s)->pSPI, (d), (c), HAL_MAX_DELAY)
#define ST25R_SPI_COMM_TRANSMIT_RECEIVE(s, d, c)	HAL_SPI_TransmitReceive((s)->pSPI, (d), (d), (c), HAL_MAX_DELAY)

void ST25R_SPI_DirectCommand_internal(ST25R *pInstance, const uint8_t CommandCode_Preparred);
uint8_t ST25R_SPI_Read_SingleRegister_internal(ST25R *pInstance, const uint8_t PreCommandCode_Preparred, const uint8_t Register_Prepared);
void ST25R_SPI_Write_SingleRegister_internal(ST25R *pInstance, const uint8_t PreCommandCode_Preparred, const uint8_t Register_Prepared, const uint8_t Value);
void ST25R_SPI_Write_Registers2_internal(ST25R *pInstance, const uint8_t PreCommandCode_Preparred, const uint8_t Register_Prepared, const uint16_t Value);
void ST25R_SPI_Write_Registers4_internal(ST25R *pInstance, const uint8_t PreCommandCode_Preparred, const uint8_t Register_Prepared, const uint32_t Value);

void ST25R_SPI_Read_Multiple_internal(ST25R *pInstance, const uint8_t PreCommandCode_Preparred, uint8_t *pbData, const uint16_t cbData);
void ST25R_SPI_Write_Multiple_internal(ST25R * pInstance, const uint8_t PreCommandCode_Preparred, const uint8_t *pbData, const uint16_t cbData);

void ST25R_SPI_Read_IRQ_internal(ST25R *pInstance, const uint8_t Register_Prepared, const ST25R_IRQ_CB cb);
void ST25R_SPI_Write_IRQ_Mask_internal(ST25R *pInstance, const uint8_t Register_Prepared, const ST25R_IRQ_CB cb);
void ST25R_SPI_Write_IRQ_Mask_Operation_internal(ST25R *pInstance, const uint32_t mask, const ST25R_IRQ_MASK_OP op, const uint8_t Register_Prepared, const ST25R_IRQ_CB cb);

#define ST25R_REG_IC_IDENTITY_shift_ic_type	(3U)
#define ST25R_REG_IC_IDENTITY_mask_ic_rev	(7U)
