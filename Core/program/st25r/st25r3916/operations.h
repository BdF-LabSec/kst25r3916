#pragma once

#define ST25R3916_WRITE_MODE            (0U << 6)                      /*!< ST25R3916 Operation Mode: Write                                */
#define ST25R3916_READ_MODE             (1U << 6)                      /*!< ST25R3916 Operation Mode: Read                                 */
#define ST25R3916_CMD_MODE              (3U << 6)                      /*!< ST25R3916 Operation Mode: Direct Command                       */
#define ST25R3916_FIFO_LOAD             (0x80U)                        /*!< ST25R3916 Operation Mode: FIFO Load                            */
#define ST25R3916_FIFO_READ             (0x9fU)                        /*!< ST25R3916 Operation Mode: FIFO Read                            */
#define ST25R3916_PT_A_CONFIG_LOAD      (0xa0U)                        /*!< ST25R3916 Operation Mode: Passive Target Memory A-Config Load  */
#define ST25R3916_PT_F_CONFIG_LOAD      (0xa8U)                        /*!< ST25R3916 Operation Mode: Passive Target Memory F-Config Load  */
#define ST25R3916_PT_TSN_DATA_LOAD      (0xacU)                        /*!< ST25R3916 Operation Mode: Passive Target Memory TSN Load       */
#define ST25R3916_PT_MEM_READ           (0xbfU)                        /*!< ST25R3916 Operation Mode: Passive Target Memory Read           */

#define MK_CMD(cmd)												((cmd) | ST25R3916_CMD_MODE)
#define MK_READ(reg)											((reg) | ST25R3916_READ_MODE)
#define MK_WRITE(reg)											((reg) | ST25R3916_WRITE_MODE)
