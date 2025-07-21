#pragma once
#include "../st25r.h"
#include "operations.h"
#include "commands.h"
#include "registers.h"

#define ST25R3916_DirectCommand(s, cmd)                 		ST25R_SPI_DirectCommand_internal((s), MK_CMD(cmd))
#define ST25R3916_Read_SingleRegister(s, reg)					ST25R_SPI_Read_SingleRegister_internal((s), 0, MK_READ(reg))
#define ST25R3916_Write_SingleRegister(s, reg, value)			ST25R_SPI_Write_SingleRegister_internal((s), 0, MK_WRITE(reg), (value))

#define ST25R3916_Write_Registers2(s, reg, value)				ST25R_SPI_Write_Registers2_internal((s), 0, MK_WRITE(reg), (value))
#define ST25R3916_Write_Registers2_sep(s, reg, v1, v2)			ST25R3916_Write_Registers2(s, reg, \
			((v1) <<  0) | \
			((v2) <<  8) \
	)
#define ST25R3916_Write_Registers4(s, reg, value)				ST25R_SPI_Write_Registers4_internal((s), 0, MK_WRITE(reg), (value))
#define ST25R3916_Write_Registers4_sep(s, reg, v1, v2, v3, v4)	ST25R3916_Write_Registers4(s, reg, \
			((v1) <<  0) | \
			((v2) <<  8) | \
			((v3) << 16) | \
			((v4) << 24) \
	)

#define ST25R3916_Read_SingleRegisterB(s, reg)		            ST25R_SPI_Read_SingleRegister_internal((s), MK_CMD(ST25R3916_CMD_SPACE_B_ACCESS), MK_READ(reg))
#define ST25R3916_Write_SingleRegisterB(s, reg, value)     		ST25R_SPI_Write_SingleRegister_internal((s), MK_CMD(ST25R3916_CMD_SPACE_B_ACCESS), MK_WRITE(reg), (value))

#define ST25R3916_Write_RegistersB2(s, reg, value)				ST25R_SPI_Write_Registers2_internal((s), MK_CMD(ST25R3916_CMD_SPACE_B_ACCESS), MK_WRITE(reg), (value))
#define ST25R3916_Write_RegistersB2_sep(s, reg, v1, v2)			ST25R3916_Write_RegistersB2(s, reg, \
			((v1) <<  0) | \
			((v2) <<  8) \
	)
#define ST25R3916_Write_RegistersB4(s, reg, value)				ST25R_SPI_Write_Registers4_internal((s), MK_CMD(ST25R3916_CMD_SPACE_B_ACCESS), MK_WRITE(reg), (value))
#define ST25R3916_Write_RegistersB4_sep(s, reg, v1, v2, v3, v4)	ST25R3916_Write_RegistersB4(s, reg, \
			((v1) <<  0) | \
			((v2) <<  8) | \
			((v3) << 16) | \
			((v4) << 24) \
	)

#define ST25R3916_Read_SingleRegisterTEST(s, reg)          		ST25R_SPI_Read_SingleRegister_internal((s), MK_CMD(ST25R3916_CMD_TEST_ACCESS), MK_READ(reg))
#define ST25R3916_Write_SingleRegisterTEST(s, reg, value)  		ST25R_SPI_Write_SingleRegister_internal((s), MK_CMD(ST25R3916_CMD_TEST_ACCESS), MK_WRITE(reg), value)

#define ST25R3916_Fifo_Read(s, d, c)							ST25R_SPI_Read_Multiple_internal((s), ST25R3916_FIFO_READ, (d), (c))
#define ST25R3916_Fifo_Load(s, d, c)							ST25R_SPI_Write_Multiple_internal((s), ST25R3916_FIFO_LOAD, (d), (c))

#define ST25R3916_PT_A_Config_Load(s, d, c)						ST25R_SPI_Write_Multiple_internal((s), ST25R3916_PT_A_CONFIG_LOAD, (d), (c))

#define ST25R3916_Read_IRQ(s)									ST25R_SPI_Read_IRQ_internal((s), MK_READ(ST25R3916_REG_IRQ_MAIN), ST25R_IRQ_CB_4)
#define ST25R3916_Mask_IRQ(s, mask, op)							ST25R_SPI_Write_IRQ_Mask_Operation_internal((s), (mask), (op), MK_WRITE(ST25R3916_REG_IRQ_MASK_MAIN), ST25R_IRQ_CB_4)

#define ST25R3916_Write_NoResponseTimer(s, v)					ST25R3916_Write_Registers2((s), MK_WRITE(ST25R3916_REG_NO_RESPONSE_TIMER1), __builtin_bswap16(v))

void ST25R3916_Init(ST25R *pInstance);
void ST25R3916_WaitForIRQ(ST25R *pInstance);
uint8_t ST25R3916_Generic_IRQ_toErr(uint32_t irq);
uint8_t ST25R3916_FieldOn_AC(ST25R *pInstance);
void ST25R3916_FieldOff(ST25R *pInstance);
uint16_t ST25R3916_Fifo_Status(ST25R * pInstance, uint8_t *pStatus2);

void ST25R3916_Transmit(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC);
void ST25R3916_Receive(ST25R *pInstance, const uint8_t bWithCRC);
uint8_t ST25R3916_Transmit_then_Receive(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC);

void ST25R3916_14A_Initiator(ST25R *pInstance);
void ST25R3916_14A_Target(ST25R *pInstance);

void ST25R3916_14A4_TxRx106(ST25R *pInstance);
void ST25R3916_14A4_TxRx212(ST25R *pInstance);
void ST25R3916_14A4_TxRx424(ST25R *pInstance);
void ST25R3916_14A4_TxRx848(ST25R *pInstance);

void ST25R_DumpRegs(ST25R * pInstance, uint8_t onlyDefaultDiff);
