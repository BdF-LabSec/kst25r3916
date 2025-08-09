/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#pragma once
#include "../st25r.h"

/* ST25R3911B, seems to support:
 * - ST25R3912
 * - ST25R3914
 * - ST25R3915
 * See: https://blog.gentilkiwi.com/Smartcards/NFC/st25r/Chips
 */

#define ST25R3911B_FIFO_DEPTH		96U

#define ST25R3911B_DirectCommand(s, cmd)						ST25R_SPI_DirectCommand_internal((s), ST25R3911B_MK_CMD(cmd))
#define ST25R3911B_Read_SingleRegister(s, reg)					ST25R_SPI_Read_SingleRegister_internal((s), 0, ST25R3911B_MK_READ(reg))
#define ST25R3911B_Write_SingleRegister(s, reg, value)			ST25R_SPI_Write_SingleRegister_internal((s), 0, ST25R3911B_MK_WRITE(reg), (value))
#define ST25R3911B_Write_Registers2(s, reg, value)				ST25R_SPI_Write_Registers2_internal((s), 0, ST25R3911B_MK_WRITE(reg), (value))
#define ST25R3911B_Write_Registers2_sep(s, reg, v1, v2)			ST25R3911B_Write_Registers2(s, reg, \
			((v1) <<  0) | \
			((v2) <<  8)   \
	)
#define ST25R3911B_Write_Registers4(s, reg, value)				ST25R_SPI_Write_Registers4_internal((s), 0, ST25R3911B_MK_WRITE(reg), (value))
#define ST25R3911B_Write_Registers4_sep(s, reg, v1, v2, v3, v4)	ST25R3911B_Write_Registers4(s, reg, \
			((v1) <<  0) | \
			((v2) <<  8) | \
			((v3) << 16) | \
			((v4) << 24)   \
	)

#define ST25R3911B_Read_SingleRegisterTEST(s, reg)				ST25R_SPI_Read_SingleRegister_internal((s), ST25R3911B_MK_CMD(ST25R3911B_CMD_TEST_ACCESS), ST25R3911B_MK_READ(reg))
#define ST25R3911B_Write_SingleRegisterTEST(s, reg, value)		ST25R_SPI_Write_SingleRegister_internal((s), ST25R3911B_MK_CMD(ST25R3911B_CMD_TEST_ACCESS), ST25R3911B_MK_WRITE(reg), value)

#define ST25R3911B_Fifo_Read(s, d, c)							ST25R_SPI_Read_Multiple_internal((s), ST25R3911B_FIFO_READ, (d), (c))
#define ST25R3911B_Fifo_Load(s, d, c)							ST25R_SPI_Write_Multiple_internal((s), ST25R3911B_FIFO_LOAD, (d), (c))

#define ST25R3911B_Read_IRQ(s)									ST25R_SPI_Read_IRQ_internal((s), ST25R3911B_MK_READ(ST25R3911B_REG_IRQ_MAIN), ST25R_IRQ_CB_3)
#define ST25R3911B_Mask_IRQ(s, mask, op)						ST25R_SPI_Write_IRQ_Mask_Operation_internal((s), (mask), (op), ST25R3911B_MK_WRITE(ST25R3911B_REG_IRQ_MASK_MAIN), ST25R_IRQ_CB_3)

#define ST25R3911B_Write_NoResponseTimer(s, v)					ST25R3911B_Write_Registers2((s), ST25R3911B_MK_WRITE(ST25R3911B_REG_NO_RESPONSE_TIMER1), __builtin_bswap16(v))
#define ST25R3911B_Write_GeneralPurposeTimer(s, v)				ST25R3911B_Write_Registers2((s), ST25R3911B_MK_WRITE(ST25R3911B_REG_GPT1), __builtin_bswap16(v))


void ST25R3911B_Init(ST25R *pInstance);
void ST25R3911B_WaitForIRQ(ST25R *pInstance);
uint8_t ST25R3911B_Generic_IRQ_toErr(uint32_t irq);
uint8_t ST25R3911B_WaitFor_SpecificIRQ(ST25R *pInstance, uint32_t SpecificIRQ);

uint8_t ST25R3911B_FieldOn_AC(ST25R *pInstance);
void ST25R3911B_FieldOff(ST25R *pInstance);

uint8_t ST25R3911B_Fifo_Status(ST25R * pInstance, uint8_t *pStatus2);

uint8_t ST25R3911B_Transmit_NoIRQ(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC);
uint8_t ST25R3911B_Transmit(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC);
uint8_t ST25R3911B_Receive_NoIRQ(ST25R *pInstance, const uint8_t bWithCRC);
uint8_t ST25R3911B_Receive(ST25R *pInstance, const uint8_t bWithCRC);
uint8_t ST25R3911B_Transmit_then_Receive(ST25R *pInstance, const uint8_t *pbData, const uint16_t cbData, const uint8_t bWithCRC);


/* DS11793 # 1.2.12 - Communication with an external microcontroller - Table 6 */

#define ST25R3911B_MASK_ADDRESS		0b00111111
#define ST25R3911B_MASK_COMMAND		0b00111111

#define ST25R3911B_WRITE_MODE		(0b00 << 6)
#define ST25R3911B_READ_MODE		(0b01 << 6)
#define ST25R3911B_CMD_MODE			(0b11 << 6)
#define ST25R3911B_FIFO_LOAD		0b10000000
#define ST25R3911B_FIFO_READ		0b10111111

#define ST25R3911B_MK_READ(reg)		(((reg) & ST25R3911B_MASK_ADDRESS) | ST25R3911B_READ_MODE)
#define ST25R3911B_MK_WRITE(reg)	(((reg) & ST25R3911B_MASK_ADDRESS) | ST25R3911B_WRITE_MODE)
#define ST25R3911B_MK_CMD(cmd)		(((cmd) & ST25R3911B_MASK_COMMAND) | ST25R3911B_CMD_MODE)


#define ST25R3911B_CMD_SET_DEFAULT              0x01U    /*!< Puts the chip in default state (same as after power-up)    */
#define ST25R3911B_CMD_CLEAR_FIFO               0x02U    /*!< Stops all activities and clears FIFO                       */
#define ST25R3911B_CMD_TRANSMIT_WITH_CRC        0x04U    /*!< Transmit with CRC                                          */
#define ST25R3911B_CMD_TRANSMIT_WITHOUT_CRC     0x05U    /*!< Transmit without CRC                                       */
#define ST25R3911B_CMD_TRANSMIT_REQA            0x06U    /*!< Transmit REQA                                              */
#define ST25R3911B_CMD_TRANSMIT_WUPA            0x07U    /*!< Transmit WUPA                                              */
#define ST25R3911B_CMD_INITIAL_RF_COLLISION     0x08U    /*!< NFC transmit with Initial RF Collision Avoidance           */
#define ST25R3911B_CMD_RESPONSE_RF_COLLISION_N  0x09U    /*!< NFC transmit with Response RF Collision Avoidance          */
#define ST25R3911B_CMD_RESPONSE_RF_COLLISION_0  0x0AU    /*!< NFC transmit with Response RF Collision Avoidance with n=0 */
#define ST25R3911B_CMD_NORMAL_NFC_MODE          0x0BU    /*!< NFC switch to normal NFC mode                              */
#define ST25R3911B_CMD_ANALOG_PRESET            0x0CU    /*!< Analog Preset                                              */
#define ST25R3911B_CMD_MASK_RECEIVE_DATA        0x10U    /*!< Mask receive data                                          */
#define ST25R3911B_CMD_UNMASK_RECEIVE_DATA      0x11U    /*!< Unmask receive data                                        */
#define ST25R3911B_CMD_MEASURE_AMPLITUDE        0x13U    /*!< Measure singal amplitude on RFI inputs                     */
#define ST25R3911B_CMD_SQUELCH                  0x14U    /*!< Squelch                                                    */
#define ST25R3911B_CMD_CLEAR_SQUELCH            0x15U    /*!< Clear Squelch                                              */
#define ST25R3911B_CMD_ADJUST_REGULATORS        0x16U    /*!< Adjust regulators                                          */
#define ST25R3911B_CMD_CALIBRATE_MODULATION     0x17U    /*!< Calibrate modulation depth                                 */
#define ST25R3911B_CMD_CALIBRATE_ANTENNA        0x18U    /*!< Calibrate antenna                                          */
#define ST25R3911B_CMD_MEASURE_PHASE            0x19U    /*!< Measure phase between RFO and RFI signal                   */
#define ST25R3911B_CMD_CLEAR_RSSI               0x1AU    /*!< clear RSSI bits and restart the measurement                */
#define ST25R3911B_CMD_TRANSPARENT_MODE         0x1CU    /*!< Transparent mode                                           */
#define ST25R3911B_CMD_CALIBRATE_C_SENSOR       0x1DU    /*!< Calibrate the capacitive sensor                            */
#define ST25R3911B_CMD_MEASURE_CAPACITANCE      0x1EU    /*!< Measure capacitance                                        */
#define ST25R3911B_CMD_MEASURE_VDD              0x1FU    /*!< Measure power supply voltage                               */
#define ST25R3911B_CMD_START_GP_TIMER           0x20U    /*!< Start the general purpose timer                            */
#define ST25R3911B_CMD_START_WUP_TIMER          0x21U    /*!< Start the wake-up timer                                    */
#define ST25R3911B_CMD_START_MASK_RECEIVE_TIMER 0x22U    /*!< Start the mask-receive timer                               */
#define ST25R3911B_CMD_START_NO_RESPONSE_TIMER  0x23U    /*!< Start the no-repsonse timer                                */
#define ST25R3911B_CMD_TEST_CLEARA              0x3AU    /*!< Clear Test register                                        */
#define ST25R3911B_CMD_TEST_CLEARB              0x3BU    /*!< Clear Test register                                        */
#define ST25R3911B_CMD_TEST_ACCESS              0x3CU    /*!< Enable R/W access to the test registers                    */
#define ST25R3911B_CMD_LOAD_PPROM               0x3DU    /*!< Load data from the poly fuses to RAM                       */
#define ST25R3911B_CMD_FUSE_PPROM               0x3EU    /*!< Fuse poly fuses with data from the RAM                     */


#define ST25R3911B_REG_IO_CONF1                     0x00U        /*!< RW IO Configuration Register 1 */
#define ST25R3911B_REG_IO_CONF2                     0x01U        /*!< RW IO Configuration Register 2 */
#define ST25R3911B_REG_OP_CONTROL                   0x02U        /*!< RW Operation Control Register */
#define ST25R3911B_REG_MODE                         0x03U        /*!< RW Mode Definition Register */
#define ST25R3911B_REG_BIT_RATE                     0x04U        /*!< RW Bit Rate Definition Register */
#define ST25R3911B_REG_ISO14443A_NFC                0x05U        /*!< RW ISO14443A and NFC 106 kBit/s Settings Register */
#define ST25R3911B_REG_ISO14443B_1                  0x06U        /*!< RW ISO14443B Settings Register 1 */
#define ST25R3911B_REG_ISO14443B_2                  0x07U        /*!< RW ISO14443B Settings Register 2 */
#define ST25R3911B_REG_STREAM_MODE                  0x08U        /*!< RW Stream Mode Definition Register */
#define ST25R3911B_REG_AUX                          0x09U        /*!< RW Auxiliary Definition Register */
#define ST25R3911B_REG_RX_CONF1                     0x0AU        /*!< RW Receiver Configuration Register 1 */
#define ST25R3911B_REG_RX_CONF2                     0x0BU        /*!< RW Receiver Configuration Register 2 */
#define ST25R3911B_REG_RX_CONF3                     0x0CU        /*!< RW Receiver Configuration Register 3 */
#define ST25R3911B_REG_RX_CONF4                     0x0DU        /*!< RW Receiver Configuration Register 4 */
#define ST25R3911B_REG_MASK_RX_TIMER                0x0EU        /*!< RW Mask Receive Timer Register */
#define ST25R3911B_REG_NO_RESPONSE_TIMER1           0x0FU        /*!< RW No-response Timer Register 1 */
#define ST25R3911B_REG_NO_RESPONSE_TIMER2           0x10U        /*!< RW No-response Timer Register 2 */
#define ST25R3911B_REG_GPT_CONTROL                  0x11U        /*!< RW General Purpose Timer Control Register */
#define ST25R3911B_REG_GPT1                         0x12U        /*!< RW General Purpose Timer Register 1 */
#define ST25R3911B_REG_GPT2                         0x13U        /*!< RW General Purpose Timer Register 2 */
#define ST25R3911B_REG_IRQ_MASK_MAIN                0x14U        /*!< RW Mask Main Interrupt Register */
#define ST25R3911B_REG_IRQ_MASK_TIMER_NFC           0x15U        /*!< RW Mask Timer and NFC Interrupt Register */
#define ST25R3911B_REG_IRQ_MASK_ERROR_WUP           0x16U        /*!< RW Mask Error and Wake-up Interrupt Register */
#define ST25R3911B_REG_IRQ_MAIN                     0x17U        /*!< R  Main Interrupt Register */
#define ST25R3911B_REG_IRQ_TIMER_NFC                0x18U        /*!< R  Timer and NFC Interrupt Register */
#define ST25R3911B_REG_IRQ_ERROR_WUP                0x19U        /*!< R  Error and Wake-up Interrupt Register */
#define ST25R3911B_REG_FIFO_RX_STATUS1              0x1AU        /*!< R  FIFO RX Status Register 1 */
#define ST25R3911B_REG_FIFO_RX_STATUS2              0x1BU        /*!< R  FIFO RX Status Register 2 */
#define ST25R3911B_REG_COLLISION_STATUS             0x1CU        /*!< R  Collision Display Register */
#define ST25R3911B_REG_NUM_TX_BYTES1                0x1DU        /*!< RW Number of Transmitted Bytes Register 1 */
#define ST25R3911B_REG_NUM_TX_BYTES2                0x1EU        /*!< RW Number of Transmitted Bytes Register 2 */
#define ST25R3911B_REG_NFCIP1_BIT_RATE              0x1FU        /*!< R  NFCIP Bit Rate Detection Display Register */
#define ST25R3911B_REG_AD_RESULT                    0x20U        /*!< R  A/D Converter Output Register */
#define ST25R3911B_REG_ANT_CAL_CONTROL              0x21U        /*!< RW Antenna Calibration Control Register */
#define ST25R3911B_REG_ANT_CAL_TARGET               0x22U        /*!< RW Antenna Calibration Target Register */
#define ST25R3911B_REG_ANT_CAL_RESULT               0x23U        /*!< R  Antenna Calibration Display Register */
#define ST25R3911B_REG_AM_MOD_DEPTH_CONTROL         0x24U        /*!< RW AM Modulation Depth Control Register */
#define ST25R3911B_REG_AM_MOD_DEPTH_RESULT          0x25U        /*!< R  AM Modulation Depth Display Register */
#define ST25R3911B_REG_RFO_AM_ON_LEVEL              0x26U        /*!< RW RFO AM Modulation (On) Level Definition Register */
#define ST25R3911B_REG_RFO_AM_OFF_LEVEL             0x27U        /*!< RW RFO Normal (AM Off) Level Definition Register */
#define ST25R3911B_REG_FIELD_THRESHOLD              0x29U        /*!< RW External Field Detector Threshold Register */
#define ST25R3911B_REG_REGULATOR_CONTROL            0x2AU        /*!< RW Regulated Voltage Control Register */
#define ST25R3911B_REG_REGULATOR_RESULT             0x2BU        /*!< R Regulator Display Register */
#define ST25R3911B_REG_RSSI_RESULT                  0x2CU        /*!< R RSSI Display Register*/
#define ST25R3911B_REG_GAIN_RED_STATE               0x2DU        /*!< R Gain Reduction State Register*/
#define ST25R3911B_REG_CAP_SENSOR_CONTROL           0x2EU        /*!< RW Capacitive Sensor Control Register */
#define ST25R3911B_REG_CAP_SENSOR_RESULT            0x2FU        /*!< R  Capacitive Sensor Display Register */
#define ST25R3911B_REG_AUX_DISPLAY                  0x30U        /*!< R Auxiliary Display Register */
#define ST25R3911B_REG_WUP_TIMER_CONTROL            0x31U        /*!< RW Wake-up Timer Control Register */
#define ST25R3911B_REG_AMPLITUDE_MEASURE_CONF       0x32U        /*!< RW Amplitude Measurement Configuration Register */
#define ST25R3911B_REG_AMPLITUDE_MEASURE_REF        0x33U        /*!< RW Amplitude Measurement Reference Register */
#define ST25R3911B_REG_AMPLITUDE_MEASURE_AA_RESULT  0x34U        /*!< R  Amplitude Measurement Auto Averaging Display Register */
#define ST25R3911B_REG_AMPLITUDE_MEASURE_RESULT     0x35U        /*!< R  Amplitude Measurement Display Register */
#define ST25R3911B_REG_PHASE_MEASURE_CONF           0x36U        /*!< RW Phase Measurement Configuration Register */
#define ST25R3911B_REG_PHASE_MEASURE_REF            0x37U        /*!< RW Phase Measurement Reference Register */
#define ST25R3911B_REG_PHASE_MEASURE_AA_RESULT      0x38U        /*!< R  Phase Measurement Auto Averaging Display Register */
#define ST25R3911B_REG_PHASE_MEASURE_RESULT         0x39U        /*!< R  Phase Measurement Display Register */
#define ST25R3911B_REG_CAPACITANCE_MEASURE_CONF     0x3AU        /*!< RW Capacitance Measurement Configuration Register */
#define ST25R3911B_REG_CAPACITANCE_MEASURE_REF      0x3BU        /*!< RW Capacitance Measurement Reference Register */
#define ST25R3911B_REG_CAPACITANCE_MEASURE_AA_RESULT 0x3CU       /*!< R  Capacitance Measurement Auto Averaging Display Register */
#define ST25R3911B_REG_CAPACITANCE_MEASURE_RESULT   0x3DU        /*!< R  Capacitance Measurement Display Register */
#define ST25R3911B_REG_IC_IDENTITY                  0x3FU        /*!< R  Chip Id: 0 for old silicon, v2 silicon: 0x09 */


#define ST25R3911B_REG_IO_CONF1_lf_clk_off                     (1U<<0)
#define ST25R3911B_REG_IO_CONF1_out_cl0                        (1U<<1)
#define ST25R3911B_REG_IO_CONF1_out_cl1                        (1U<<2)
#define ST25R3911B_REG_IO_CONF1_mask_out_cl                    (3U<<1)
#define ST25R3911B_REG_IO_CONF1_osc                            (1U<<3)
#define ST25R3911B_REG_IO_CONF1_fifo_lt                        (1U<<4)
#define ST25R3911B_REG_IO_CONF1_fifo_lt_32bytes                (0U<<4)
#define ST25R3911B_REG_IO_CONF1_fifo_lt_16bytes                (1U<<4)
#define ST25R3911B_REG_IO_CONF1_fifo_lr                        (1U<<5)
#define ST25R3911B_REG_IO_CONF1_fifo_lr_64bytes                (0U<<5)
#define ST25R3911B_REG_IO_CONF1_fifo_lr_80bytes                (1U<<5)
#define ST25R3911B_REG_IO_CONF1_rfo2                           (1U<<6)
#define ST25R3911B_REG_IO_CONF1_single                         (1U<<7)

#define ST25R3911B_REG_IO_CONF2_slow_up                        (1U<<0)
#define ST25R3911B_REG_IO_CONF2_io_18                          (1U<<2)
#define ST25R3911B_REG_IO_CONF2_miso_pd1                       (1U<<3)
#define ST25R3911B_REG_IO_CONF2_miso_pd2                       (1U<<4)
#define ST25R3911B_REG_IO_CONF2_vspd_off                       (1U<<6)
#define ST25R3911B_REG_IO_CONF2_sup3V                          (1U<<7)

#define ST25R3911B_REG_OP_CONTROL_wu                           (1U<<2)
#define ST25R3911B_REG_OP_CONTROL_tx_en                        (1U<<3)
#define ST25R3911B_REG_OP_CONTROL_rx_man                       (1U<<4)
#define ST25R3911B_REG_OP_CONTROL_rx_chn                       (1U<<5)
#define ST25R3911B_REG_OP_CONTROL_rx_en                        (1U<<6)
#define ST25R3911B_REG_OP_CONTROL_en                           (1U<<7)

#define ST25R3911B_REG_MODE_nfc_ar                             (1U<<0)
#define ST25R3911B_REG_MODE_nfc_ar_on                          (1U<<0)
#define ST25R3911B_REG_MODE_nfc_ar_off                         (0U<<0)
#define ST25R3911B_REG_MODE_mask_om                            (0xfU<<3)
#define ST25R3911B_REG_MODE_om_nfc                             (0x0U<<3)
#define ST25R3911B_REG_MODE_om_iso14443a                       (0x1U<<3)
#define ST25R3911B_REG_MODE_om_iso14443b                       (0x2U<<3)
#define ST25R3911B_REG_MODE_om_felica                          (0x3U<<3)
#define ST25R3911B_REG_MODE_om_topaz                           (0x4U<<3)
#define ST25R3911B_REG_MODE_om_subcarrier_stream               (0xeU<<3)
#define ST25R3911B_REG_MODE_om_bpsk_stream                     (0xfU<<3)
#define ST25R3911B_REG_MODE_om_bit_rate_detection              (0x0U<<3)
#define ST25R3911B_REG_MODE_om_nfcip1_normal_mode              (0x1U<<3)
#define ST25R3911B_REG_MODE_targ                               (1U<<7)
#define ST25R3911B_REG_MODE_targ_targ                          (1U<<7)
#define ST25R3911B_REG_MODE_targ_init                          (0U<<7)

#define ST25R3911B_REG_BIT_RATE_mask_txrate                    (0xfU<<4)
#define ST25R3911B_REG_BIT_RATE_shift_txrate                   (4U)
#define ST25R3911B_REG_BIT_RATE_txrate_106                     (0x0U<<4)
#define ST25R3911B_REG_BIT_RATE_txrate_212                     (0x1U<<4)
#define ST25R3911B_REG_BIT_RATE_txrate_424                     (0x2U<<4)
#define ST25R3911B_REG_BIT_RATE_txrate_848                     (0x3U<<4)
#define ST25R3911B_REG_BIT_RATE_txrate_1695                    (0x4U<<4)
#define ST25R3911B_REG_BIT_RATE_txrate_3390                    (0x5U<<4)
#define ST25R3911B_REG_BIT_RATE_txrate_6780                    (0x6U<<4)
#define ST25R3911B_REG_BIT_RATE_mask_rxrate                    (0xfU<<0)
#define ST25R3911B_REG_BIT_RATE_shift_rxrate                   (0U)
#define ST25R3911B_REG_BIT_RATE_rxrate_106                     (0x0U<<0)
#define ST25R3911B_REG_BIT_RATE_rxrate_212                     (0x1U<<0)
#define ST25R3911B_REG_BIT_RATE_rxrate_424                     (0x2U<<0)
#define ST25R3911B_REG_BIT_RATE_rxrate_848                     (0x3U<<0)
#define ST25R3911B_REG_BIT_RATE_rxrate_1695                    (0x4U<<0)
#define ST25R3911B_REG_BIT_RATE_rxrate_3390                    (0x5U<<0)
#define ST25R3911B_REG_BIT_RATE_rxrate_6780                    (0x6U<<0)

#define ST25R3911B_REG_ISO14443A_NFC_antcl                     (1U<<0)
#define ST25R3911B_REG_ISO14443A_NFC_mask_p_len                (0xfU<<1)
#define ST25R3911B_REG_ISO14443A_NFC_shift_p_len               (1U)
#define ST25R3911B_REG_ISO14443A_NFC_nfc_f0                    (1U<<5)
#define ST25R3911B_REG_ISO14443A_NFC_nfc_f0_off                (0U<<5)
#define ST25R3911B_REG_ISO14443A_NFC_no_rx_par                 (1U<<6)
#define ST25R3911B_REG_ISO14443A_NFC_no_rx_par_off             (0U<<6)
#define ST25R3911B_REG_ISO14443A_NFC_no_tx_par                 (1U<<7)
#define ST25R3911B_REG_ISO14443A_NFC_no_tx_par_off             (0U<<7)

#define ST25R3911B_REG_ISO14443B_1_mask_eof                    (1U<<2)
#define ST25R3911B_REG_ISO14443B_1_eof_10etu                   (0U<<2)
#define ST25R3911B_REG_ISO14443B_1_eof_11etu                   (1U<<2)
#define ST25R3911B_REG_ISO14443B_1_mask_sof                    (3U<<3)
#define ST25R3911B_REG_ISO14443B_1_mask_sof_0                  (1U<<4)
#define ST25R3911B_REG_ISO14443B_1_sof_0_10etu                 (0U<<4)
#define ST25R3911B_REG_ISO14443B_1_sof_0_11etu                 (1U<<4)
#define ST25R3911B_REG_ISO14443B_1_mask_sof_1                  (1U<<3)
#define ST25R3911B_REG_ISO14443B_1_sof_1_2etu                  (0U<<3)
#define ST25R3911B_REG_ISO14443B_1_sof_2_3etu                  (1U<<3)
#define ST25R3911B_REG_ISO14443B_1_mask_egt                    (7U<<5)
#define ST25R3911B_REG_ISO14443B_1_shift_egt                   (5U)

#define ST25R3911B_REG_ISO14443B_2_eof_12                      (1U<<3)
#define ST25R3911B_REG_ISO14443B_2_eof_12_10to11etu            (0U<<3)
#define ST25R3911B_REG_ISO14443B_2_eof_12_10to12etu            (1U<<3)
#define ST25R3911B_REG_ISO14443B_2_no_eof                      (1U<<4)
#define ST25R3911B_REG_ISO14443B_2_no_sof                      (1U<<5)
#define ST25R3911B_REG_ISO14443B_2_mask_tr1                    (3U<<6)
#define ST25R3911B_REG_ISO14443B_2_shift_tr1                   (6U)
#define ST25R3911B_REG_ISO14443B_2_tr1_0                       (1U<<6)
#define ST25R3911B_REG_ISO14443B_2_tr1_1                       (1U<<7)
#define ST25R3911B_REG_ISO14443B_2_tr1_80fs80fs                (0U<<6)
#define ST25R3911B_REG_ISO14443B_2_tr1_64fs32fs                (1U<<6)

#define ST25R3911B_REG_STREAM_MODE_mask_stx                    (7U<<0)
#define ST25R3911B_REG_STREAM_MODE_shift_stx                   (0U)
#define ST25R3911B_REG_STREAM_MODE_stx_106                     (0U<<0)
#define ST25R3911B_REG_STREAM_MODE_stx_212                     (1U<<0)
#define ST25R3911B_REG_STREAM_MODE_stx_424                     (2U<<0)
#define ST25R3911B_REG_STREAM_MODE_stx_848                     (3U<<0)
#define ST25R3911B_REG_STREAM_MODE_stx_1695                    (4U<<0)
#define ST25R3911B_REG_STREAM_MODE_stx_3390                    (5U<<0)
#define ST25R3911B_REG_STREAM_MODE_stx_6780                    (6U<<0)
#define ST25R3911B_REG_STREAM_MODE_mask_scp                    (3U<<3)
#define ST25R3911B_REG_STREAM_MODE_shift_scp                   (3U)
#define ST25R3911B_REG_STREAM_MODE_scp_1pulse                  (0U<<3)
#define ST25R3911B_REG_STREAM_MODE_scp_2pulses                 (1U<<3)
#define ST25R3911B_REG_STREAM_MODE_scp_4pulses                 (2U<<3)
#define ST25R3911B_REG_STREAM_MODE_scp_8pulses                 (3U<<3)
#define ST25R3911B_REG_STREAM_MODE_mask_scf                    (3U<<5)
#define ST25R3911B_REG_STREAM_MODE_shift_scf                   (5U)
#define ST25R3911B_REG_STREAM_MODE_scf_bpsk848                 (0U<<5)
#define ST25R3911B_REG_STREAM_MODE_scf_bpsk1695                (1U<<5)
#define ST25R3911B_REG_STREAM_MODE_scf_bpsk3390                (2U<<5)
#define ST25R3911B_REG_STREAM_MODE_scf_bpsk106                 (3U<<5)
#define ST25R3911B_REG_STREAM_MODE_scf_sc212                   (0U<<5)
#define ST25R3911B_REG_STREAM_MODE_scf_sc424                   (1U<<5)
#define ST25R3911B_REG_STREAM_MODE_scf_sc848                   (2U<<5)
#define ST25R3911B_REG_STREAM_MODE_scf_sc1695                  (3U<<5)

#define ST25R3911B_REG_AUX_mask_nfc_n                          (3U<<0)
#define ST25R3911B_REG_AUX_nfc_n0                              (1U<<0)
#define ST25R3911B_REG_AUX_nfc_n1                              (1U<<1)
#define ST25R3911B_REG_AUX_rx_tol                              (1U<<2)
#define ST25R3911B_REG_AUX_ook_hr                              (1U<<3)
#define ST25R3911B_REG_AUX_en_fd                               (1U<<4)
#define ST25R3911B_REG_AUX_tr_am                               (1U<<5)
#define ST25R3911B_REG_AUX_crc_2_fifo                          (1U<<6)
#define ST25R3911B_REG_AUX_no_crc_rx                           (1U<<7)

#define ST25R3911B_REG_RX_CONF1_z12k                           (1U<<0)
#define ST25R3911B_REG_RX_CONF1_h80                            (1U<<1)
#define ST25R3911B_REG_RX_CONF1_h200                           (1U<<2)
#define ST25R3911B_REG_RX_CONF1_mask_lp                        (7U<<3)
#define ST25R3911B_REG_RX_CONF1_lp_1200khz                     (0U<<3)
#define ST25R3911B_REG_RX_CONF1_lp_600khz                      (1U<<3)
#define ST25R3911B_REG_RX_CONF1_lp_300khz                      (2U<<3)
#define ST25R3911B_REG_RX_CONF1_lp_2000khz                     (4U<<3)
#define ST25R3911B_REG_RX_CONF1_lp_7000khz                     (5U<<3)
#define ST25R3911B_REG_RX_CONF1_amd_sel                        (1U<<6)
#define ST25R3911B_REG_RX_CONF1_ch_sel                         (1U<<7)

#define ST25R3911B_REG_RX_CONF2_sqm_dyn                        (1U<<1)
#define ST25R3911B_REG_RX_CONF2_agc_alg                        (1U<<2)
#define ST25R3911B_REG_RX_CONF2_agc_m                          (1U<<3)
#define ST25R3911B_REG_RX_CONF2_agc_en                         (1U<<4)
#define ST25R3911B_REG_RX_CONF2_lf_en                          (1U<<5)
#define ST25R3911B_REG_RX_CONF2_lf_op                          (1U<<6)
#define ST25R3911B_REG_RX_CONF2_rx_lp                          (1U<<7)
#define ST25R3911B_REG_RX_CONF3_rg_nfc                         (1U<<0)

#define ST25R3911B_REG_RX_CONF3_lim                            (1U<<1)
#define ST25R3911B_REG_RX_CONF3_shift_rg1_pm                   (2U)
#define ST25R3911B_REG_RX_CONF3_mask_rg1_pm                    (0x7U<<2)
#define ST25R3911B_REG_RX_CONF3_rg1_pm0                        (1U<<2)
#define ST25R3911B_REG_RX_CONF3_rg1_pm1                        (1U<<3)
#define ST25R3911B_REG_RX_CONF3_rg1_pm2                        (1U<<4)
#define ST25R3911B_REG_RX_CONF3_shift_rg1_am                   (5U)
#define ST25R3911B_REG_RX_CONF3_mask_rg1_am                    (0x7U<<5)
#define ST25R3911B_REG_RX_CONF3_rg1_am0                        (1U<<5)
#define ST25R3911B_REG_RX_CONF3_rg1_am1                        (1U<<6)
#define ST25R3911B_REG_RX_CONF3_rg1_am2                        (1U<<7)

#define ST25R3911B_REG_RX_CONF4_shift_rg2_pm                   (0U)
#define ST25R3911B_REG_RX_CONF4_mask_rg2_pm                    (0xfU<<0)
#define ST25R3911B_REG_RX_CONF4_rg2_pm0                        (1U<<0)
#define ST25R3911B_REG_RX_CONF4_rg2_pm1                        (1U<<1)
#define ST25R3911B_REG_RX_CONF4_rg2_pm2                        (1U<<2)
#define ST25R3911B_REG_RX_CONF4_rg2_pm3                        (1U<<3)
#define ST25R3911B_REG_RX_CONF4_shift_rg2_am                   (4U)
#define ST25R3911B_REG_RX_CONF4_mask_rg2_am                    (0xfU<<4)
#define ST25R3911B_REG_RX_CONF4_rg2_am0                        (1U<<4)
#define ST25R3911B_REG_RX_CONF4_rg2_am1                        (1U<<5)
#define ST25R3911B_REG_RX_CONF4_rg2_am2                        (1U<<6)
#define ST25R3911B_REG_RX_CONF4_rg2_am3                        (1U<<7)

#define ST25R3911B_REG_GPT_CONTROL_nrt_step                    (1U<<0)
#define ST25R3911B_REG_GPT_CONTROL_nrt_emv                     (1U<<1)
#define ST25R3911B_REG_GPT_CONTROL_gptc0                       (1U<<5)
#define ST25R3911B_REG_GPT_CONTROL_gptc1                       (1U<<6)
#define ST25R3911B_REG_GPT_CONTROL_gptc2                       (1U<<7)
#define ST25R3911B_REG_GPT_CONTROL_gptc_mask                   (0x7U<<5)
#define ST25R3911B_REG_GPT_CONTROL_gptc_no_trigger             (0x0U<<5)
#define ST25R3911B_REG_GPT_CONTROL_gptc_erx                    (0x1U<<5)
#define ST25R3911B_REG_GPT_CONTROL_gptc_srx                    (0x2U<<5)
#define ST25R3911B_REG_GPT_CONTROL_gptc_etx_nfc                (0x3U<<5)

#define ST25R3911B_REG_FIFO_RX_STATUS2_np_lb                   (1U<<0)
#define ST25R3911B_REG_FIFO_RX_STATUS2_mask_fifo_lb            (7U<<1)
#define ST25R3911B_REG_FIFO_RX_STATUS2_shift_fifo_lb           (1U)
#define ST25R3911B_REG_FIFO_RX_STATUS2_fifo_lb0                (1U<<1)
#define ST25R3911B_REG_FIFO_RX_STATUS2_fifo_lb1                (1U<<2)
#define ST25R3911B_REG_FIFO_RX_STATUS2_fifo_lb2                (1U<<3)
#define ST25R3911B_REG_FIFO_RX_STATUS2_fifo_ncp                (1U<<4)
#define ST25R3911B_REG_FIFO_RX_STATUS2_fifo_ovr                (1U<<5)
#define ST25R3911B_REG_FIFO_RX_STATUS2_fifo_unf                (1U<<6)

#define ST25R3911B_REG_COLLISION_STATUS_c_pb                   (1U<<0)
#define ST25R3911B_REG_COLLISION_STATUS_mask_c_bit             (3U<<1)
#define ST25R3911B_REG_COLLISION_STATUS_shift_c_bit            (1U)
#define ST25R3911B_REG_COLLISION_STATUS_mask_c_byte            (0xfU<<4)
#define ST25R3911B_REG_COLLISION_STATUS_shift_c_byte           (4U)

#define ST25R3911B_REG_NFCIP1_BIT_RATE_nfc_rate0               (1U<<4) /* TYPO FIXED ? */
#define ST25R3911B_REG_NFCIP1_BIT_RATE_nfc_rate1               (1U<<5)
#define ST25R3911B_REG_NFCIP1_BIT_RATE_nfc_rate2               (1U<<6)
#define ST25R3911B_REG_NFCIP1_BIT_RATE_nfc_rate3               (1U<<7)
#define ST25R3911B_REG_NFCIP1_BIT_RATE_nfc_rate_mask           (0xfU<<4)
#define ST25R3911B_REG_NFCIP1_BIT_RATE_nfc_rate_shift          (4U)

#define ST25R3911B_REG_ANT_CAL_CONTROL_mask_tre                (0xfU<<3)
#define ST25R3911B_REG_ANT_CAL_CONTROL_shift_tre               (3U)
#define ST25R3911B_REG_ANT_CAL_CONTROL_tre_0                   (1U<<3)
#define ST25R3911B_REG_ANT_CAL_CONTROL_tre_1                   (1U<<4)
#define ST25R3911B_REG_ANT_CAL_CONTROL_tre_2                   (1U<<5)
#define ST25R3911B_REG_ANT_CAL_CONTROL_tre_3                   (1U<<6)
#define ST25R3911B_REG_ANT_CAL_CONTROL_trim_s                  (1U<<7)

#define ST25R3911B_REG_ANT_CAL_RESULT_tri_err                  (1U<<3)
#define ST25R3911B_REG_ANT_CAL_RESULT_tri_0                    (1U<<4)
#define ST25R3911B_REG_ANT_CAL_RESULT_tri_1                    (1U<<5)
#define ST25R3911B_REG_ANT_CAL_RESULT_tri_2                    (1U<<6)
#define ST25R3911B_REG_ANT_CAL_RESULT_tri_3                    (1U<<7)

#define ST25R3911B_REG_AM_MOD_DEPTH_CONTROL_mask_mod           (0x3fU<<1)
#define ST25R3911B_REG_AM_MOD_DEPTH_CONTROL_shift_mod          (1U)
#define ST25R3911B_REG_AM_MOD_DEPTH_CONTROL_mod_8percent       (0xbU<<1)
#define ST25R3911B_REG_AM_MOD_DEPTH_CONTROL_mod_10percent      (0xeU<<1)
#define ST25R3911B_REG_AM_MOD_DEPTH_CONTROL_mod_14percent      (0x14U<<1)
#define ST25R3911B_REG_AM_MOD_DEPTH_CONTROL_mod_20percent      (0x20U<<1)
#define ST25R3911B_REG_AM_MOD_DEPTH_CONTROL_mod_25percent      (0x2aU<<1)
#define ST25R3911B_REG_AM_MOD_DEPTH_CONTROL_mod_30percent      (0x37U<<1)
#define ST25R3911B_REG_AM_MOD_DEPTH_CONTROL_mod_33percent      (0x3fU<<1)
#define ST25R3911B_REG_AM_MOD_DEPTH_CONTROL_am_s               (1U<<7)

#define ST25R3911B_REG_RFO_AM_MOD_LEVEL_dram0                  (1U<<0)
#define ST25R3911B_REG_RFO_AM_MOD_LEVEL_dram1                  (1U<<1)
#define ST25R3911B_REG_RFO_AM_MOD_LEVEL_dram2                  (1U<<2)
#define ST25R3911B_REG_RFO_AM_MOD_LEVEL_dram3                  (1U<<3)
#define ST25R3911B_REG_RFO_AM_MOD_LEVEL_dram4                  (1U<<4)
#define ST25R3911B_REG_RFO_AM_MOD_LEVEL_dram5                  (1U<<5)
#define ST25R3911B_REG_RFO_AM_MOD_LEVEL_dram6                  (1U<<6)
#define ST25R3911B_REG_RFO_AM_MOD_LEVEL_dram7                  (1U<<7)

#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_t0                  (1U<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_t1                  (1U<<1)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_t2                  (1U<<2)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_t3                  (1U<<3)
#define ST25R3911B_REG_FIELD_THRESHOLD_trg_l0                  (1U<<4)
#define ST25R3911B_REG_FIELD_THRESHOLD_trg_l1                  (1U<<5)
#define ST25R3911B_REG_FIELD_THRESHOLD_trg_l2                  (1U<<6)
#define ST25R3911B_REG_FIELD_THRESHOLD_mask_trg                (0x07U<<4)
#define ST25R3911B_REG_FIELD_THRESHOLD_trg_75mV                (0x00U<<4)
#define ST25R3911B_REG_FIELD_THRESHOLD_trg_105mV               (0x01U<<4)
#define ST25R3911B_REG_FIELD_THRESHOLD_trg_150mV               (0x02U<<4)
#define ST25R3911B_REG_FIELD_THRESHOLD_trg_205mV               (0x03U<<4)
#define ST25R3911B_REG_FIELD_THRESHOLD_trg_290mV               (0x04U<<4)
#define ST25R3911B_REG_FIELD_THRESHOLD_trg_400mV               (0x05U<<4)
#define ST25R3911B_REG_FIELD_THRESHOLD_trg_560mV               (0x06U<<4)
#define ST25R3911B_REG_FIELD_THRESHOLD_trg_800mV               (0x07U<<4)
#define ST25R3911B_REG_FIELD_THRESHOLD_mask_rfe                (0x0FU<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_75mV                (0x00U<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_105mV               (0x01U<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_150mV               (0x02U<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_205mV               (0x03U<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_290mV               (0x04U<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_400mV               (0x05U<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_560mV               (0x06U<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_800mV               (0x07U<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_25mV                (0x08U<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_33mV                (0x09U<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_47mV                (0x0AU<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_64mV                (0x0BU<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_90mV                (0x0CU<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_125mV               (0x0DU<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_175mV               (0x0EU<<0)
#define ST25R3911B_REG_FIELD_THRESHOLD_rfe_250mV               (0x0FU<<0)

#define ST25R3911B_REG_REGULATOR_CONTROL_shift_mpsv            (1U)
#define ST25R3911B_REG_REGULATOR_CONTROL_mask_mpsv             (3U<<1)
#define ST25R3911B_REG_REGULATOR_CONTROL_mpsv_vdd              (0U<<1)
#define ST25R3911B_REG_REGULATOR_CONTROL_mpsv_vsp_a            (1U<<1)
#define ST25R3911B_REG_REGULATOR_CONTROL_mpsv_vsp_d            (2U<<1)
#define ST25R3911B_REG_REGULATOR_CONTROL_mpsv_vsp_rf           (3U<<1)
#define ST25R3911B_REG_REGULATOR_CONTROL_mask_rege             (0xfU<<3)
#define ST25R3911B_REG_REGULATOR_CONTROL_shift_rege            (3U)
#define ST25R3911B_REG_REGULATOR_CONTROL_reg_s                 (1U<<7)
#define ST25R3911B_REG_REGULATOR_RESULT_mrt_on                 (1U<<0)
#define ST25R3911B_REG_REGULATOR_RESULT_nrt_on                 (1U<<1)
#define ST25R3911B_REG_REGULATOR_RESULT_gpt_on                 (1U<<2)
#define ST25R3911B_REG_REGULATOR_RESULT_mask_reg               (0xfU<<4)
#define ST25R3911B_REG_REGULATOR_RESULT_shift_reg              (4U)
#define ST25R3911B_REG_REGULATOR_RESULT_reg_0                  (1U<<4)
#define ST25R3911B_REG_REGULATOR_RESULT_reg_1                  (1U<<5)
#define ST25R3911B_REG_REGULATOR_RESULT_reg_2                  (1U<<6)
#define ST25R3911B_REG_REGULATOR_RESULT_reg_3                  (1U<<7)

#define ST25R3911B_REG_RSSI_RESULT_mask_rssi_pm                (0xfU)
#define ST25R3911B_REG_RSSI_RESULT_shift_rssi_pm               (0U)
#define ST25R3911B_REG_RSSI_RESULT_rssi_pm0                    (1U<<0)
#define ST25R3911B_REG_RSSI_RESULT_rssi_pm1                    (1U<<1)
#define ST25R3911B_REG_RSSI_RESULT_rssi_pm2                    (1U<<2)
#define ST25R3911B_REG_RSSI_RESULT_rssi_pm3                    (1U<<3)
#define ST25R3911B_REG_RSSI_RESULT_mask_rssi_am                (0xfU<<4)
#define ST25R3911B_REG_RSSI_RESULT_shift_rssi_am               (4U)
#define ST25R3911B_REG_RSSI_RESULT_rssi_am_0                   (1U<<4)
#define ST25R3911B_REG_RSSI_RESULT_rssi_am_1                   (1U<<5)
#define ST25R3911B_REG_RSSI_RESULT_rssi_am_2                   (1U<<6)
#define ST25R3911B_REG_RSSI_RESULT_rssi_am_3                   (1U<<7)

#define ST25R3911B_REG_GAIN_RED_STATE_mask_gs_pm               (0xfU)
#define ST25R3911B_REG_GAIN_RED_STATE_shift_gs_pm              (0U)
#define ST25R3911B_REG_GAIN_RED_STATE_gs_pm_0                  (1U<<0)
#define ST25R3911B_REG_GAIN_RED_STATE_gs_pm_1                  (1U<<1)
#define ST25R3911B_REG_GAIN_RED_STATE_gs_pm_2                  (1U<<2)
#define ST25R3911B_REG_GAIN_RED_STATE_gs_pm_3                  (1U<<3)
#define ST25R3911B_REG_GAIN_RED_STATE_mask_gs_am               (0xfU<<4)
#define ST25R3911B_REG_GAIN_RED_STATE_shift_gs_am              (4U)
#define ST25R3911B_REG_GAIN_RED_STATE_gs_am_0                  (1U<<4)
#define ST25R3911B_REG_GAIN_RED_STATE_gs_am_1                  (1U<<5)
#define ST25R3911B_REG_GAIN_RED_STATE_gs_am_2                  (1U<<6)
#define ST25R3911B_REG_GAIN_RED_STATE_gs_am_3                  (1U<<7)

#define ST25R3911B_REG_CAP_SENSOR_CONTROL_cs_g0                (1U<<0)
#define ST25R3911B_REG_CAP_SENSOR_CONTROL_cs_g1                (1U<<1)
#define ST25R3911B_REG_CAP_SENSOR_CONTROL_cs_g2                (1U<<2)
#define ST25R3911B_REG_CAP_SENSOR_CONTROL_mask_cs_g            (7U<<0)
#define ST25R3911B_REG_CAP_SENSOR_CONTROL_cs_mcal0             (1U<<3)
#define ST25R3911B_REG_CAP_SENSOR_CONTROL_cs_mcal1             (1U<<4)
#define ST25R3911B_REG_CAP_SENSOR_CONTROL_cs_mcal2             (1U<<5)
#define ST25R3911B_REG_CAP_SENSOR_CONTROL_cs_mcal3             (1U<<6)
#define ST25R3911B_REG_CAP_SENSOR_CONTROL_cs_mcal4             (1U<<7)
#define ST25R3911B_REG_CAP_SENSOR_CONTROL_mask_cs_mcal         (0x1fU<<3)
#define ST25R3911B_REG_CAP_SENSOR_CONTROL_shift_cs_mcal        (3U)
#define ST25R3911B_REG_CAP_SENSOR_RESULT_cs_cal_err            (1U<<1)
#define ST25R3911B_REG_CAP_SENSOR_RESULT_cs_cal_end            (1U<<2)
#define ST25R3911B_REG_CAP_SENSOR_RESULT_cs_cal0               (1U<<3)
#define ST25R3911B_REG_CAP_SENSOR_RESULT_cs_cal1               (1U<<4)
#define ST25R3911B_REG_CAP_SENSOR_RESULT_cs_cal2               (1U<<5)
#define ST25R3911B_REG_CAP_SENSOR_RESULT_cs_cal3               (1U<<6)
#define ST25R3911B_REG_CAP_SENSOR_RESULT_cs_cal4               (1U<<7)

#define ST25R3911B_REG_AUX_DISPLAY_en_ac                       (1U<<0)
#define ST25R3911B_REG_AUX_DISPLAY_nfc_t                       (1U<<1)
#define ST25R3911B_REG_AUX_DISPLAY_rx_act                      (1U<<2)
#define ST25R3911B_REG_AUX_DISPLAY_rx_on                       (1U<<3)
#define ST25R3911B_REG_AUX_DISPLAY_osc_ok                      (1U<<4)
#define ST25R3911B_REG_AUX_DISPLAY_tx_on                       (1U<<5)
#define ST25R3911B_REG_AUX_DISPLAY_efd_o                       (1U<<6)
#define ST25R3911B_REG_AUX_DISPLAY_a_cha                       (1U<<7)

#define ST25R3911B_REG_WUP_TIMER_CONTROL_wcap                  (1U<<0)
#define ST25R3911B_REG_WUP_TIMER_CONTROL_wph                   (1U<<1)
#define ST25R3911B_REG_WUP_TIMER_CONTROL_wam                   (1U<<2)
#define ST25R3911B_REG_WUP_TIMER_CONTROL_wto                   (1U<<3)
#define ST25R3911B_REG_WUP_TIMER_CONTROL_wut0                  (1U<<4)
#define ST25R3911B_REG_WUP_TIMER_CONTROL_wut1                  (1U<<5)
#define ST25R3911B_REG_WUP_TIMER_CONTROL_wut2                  (1U<<6)
#define ST25R3911B_REG_WUP_TIMER_CONTROL_shift_wut             (4U)
#define ST25R3911B_REG_WUP_TIMER_CONTROL_mask_wut              (7U<<4)
#define ST25R3911B_REG_WUP_TIMER_CONTROL_wur                   (1U<<7)

#define ST25R3911B_REG_AMPLITUDE_MEASURE_CONF_am_ae            (1U<<0)
#define ST25R3911B_REG_AMPLITUDE_MEASURE_CONF_am_aew0          (1U<<1)
#define ST25R3911B_REG_AMPLITUDE_MEASURE_CONF_am_aew1          (1U<<2)
#define ST25R3911B_REG_AMPLITUDE_MEASURE_CONF_shift_am_aew     (1U)
#define ST25R3911B_REG_AMPLITUDE_MEASURE_CONF_mask_am_aew      (3U<<1)
#define ST25R3911B_REG_AMPLITUDE_MEASURE_CONF_am_aam           (1U<<3)
#define ST25R3911B_REG_AMPLITUDE_MEASURE_CONF_am_d0            (1U<<4)
#define ST25R3911B_REG_AMPLITUDE_MEASURE_CONF_am_d1            (1U<<5)
#define ST25R3911B_REG_AMPLITUDE_MEASURE_CONF_am_d2            (1U<<6)
#define ST25R3911B_REG_AMPLITUDE_MEASURE_CONF_am_d3            (1U<<7)
#define ST25R3911B_REG_AMPLITUDE_MEASURE_CONF_shift_am_d       (4U)
#define ST25R3911B_REG_AMPLITUDE_MEASURE_CONF_mask_am_d        (0xfU<<4)

#define ST25R3911B_REG_PHASE_MEASURE_CONF_pm_ae                (1U<<0)
#define ST25R3911B_REG_PHASE_MEASURE_CONF_pm_aew0              (1U<<1)
#define ST25R3911B_REG_PHASE_MEASURE_CONF_pm_aew1              (1U<<2)
#define ST25R3911B_REG_PHASE_MEASURE_CONF_shift_pm_aew         (1U)
#define ST25R3911B_REG_PHASE_MEASURE_CONF_mask_pm_aew          (3U<<1)
#define ST25R3911B_REG_PHASE_MEASURE_CONF_pm_aam               (1U<<3)
#define ST25R3911B_REG_PHASE_MEASURE_CONF_pm_d0                (1U<<4)
#define ST25R3911B_REG_PHASE_MEASURE_CONF_pm_d1                (1U<<5)
#define ST25R3911B_REG_PHASE_MEASURE_CONF_pm_d2                (1U<<6)
#define ST25R3911B_REG_PHASE_MEASURE_CONF_pm_d3                (1U<<7)
#define ST25R3911B_REG_PHASE_MEASURE_CONF_shift_pm_d           (4U)
#define ST25R3911B_REG_PHASE_MEASURE_CONF_mask_pm_d            (0xfU<<4)

#define ST25R3911B_REG_CAPACITANCE_MEASURE_CONF_cm_ae          (1U<<0)
#define ST25R3911B_REG_CAPACITANCE_MEASURE_CONF_cm_aew0        (1U<<1)
#define ST25R3911B_REG_CAPACITANCE_MEASURE_CONF_cm_aew1        (1U<<2)
#define ST25R3911B_REG_CAPACITANCE_MEASURE_CONF_shift_cm_aew   (1U)
#define ST25R3911B_REG_CAPACITANCE_MEASURE_CONF_mask_cm_aew    (3U<<1)
#define ST25R3911B_REG_CAPACITANCE_MEASURE_CONF_cm_aam         (1U<<3)
#define ST25R3911B_REG_CAPACITANCE_MEASURE_CONF_cm_d0          (1U<<4)
#define ST25R3911B_REG_CAPACITANCE_MEASURE_CONF_cm_d1          (1U<<5)
#define ST25R3911B_REG_CAPACITANCE_MEASURE_CONF_cm_d2          (1U<<6)
#define ST25R3911B_REG_CAPACITANCE_MEASURE_CONF_cm_d3          (1U<<7)
#define ST25R3911B_REG_CAPACITANCE_MEASURE_CONF_shift_cm_d     (4U)

#define ST25R3911B_REG_IC_IDENTITY_v2                          (0x09U)
#define ST25R3911B_REG_IC_IDENTITY_ic_type                     (1U<<3)
#define ST25R3911B_REG_IC_IDENTITY_mask_ic_type                (0x1FU<<3)
#define ST25R3911B_REG_IC_IDENTITY_shift_ic_type               (3U)
#define ST25R3911B_REG_IC_IDENTITY_mask_ic_rev                 (7U)


#define ST25R3911B_IRQ_MASK_ALL             (uint32_t)(0xFFFFFFU) /*!< All ST25R3911 interrupt sources                              */
#define ST25R3911B_IRQ_MASK_NONE            (uint32_t)(0U)        /*!< No ST25R3911 interrupt source                                */

/* Main interrupt register. */
#define ST25R3911B_IRQ_MASK_OSC             (uint32_t)(0x80U)     /*!< ST25R3911 oscillator stable interrupt                        */
#define ST25R3911B_IRQ_MASK_FWL             (uint32_t)(0x40U)     /*!< ST25R3911 FIFO water level interrupt                         */
#define ST25R3911B_IRQ_MASK_RXS             (uint32_t)(0x20U)     /*!< ST25R3911 start of receive interrupt                         */
#define ST25R3911B_IRQ_MASK_RXE             (uint32_t)(0x10U)     /*!< ST25R3911 end of receive interrupt                           */
#define ST25R3911B_IRQ_MASK_TXE             (uint32_t)(0x08U)     /*!< ST25R3911 end of transmission interrupt                      */
#define ST25R3911B_IRQ_MASK_COL             (uint32_t)(0x04U)     /*!< ST25R3911 bit collision interrupt                            */
#define ST25R3911B_IRQ_MASK_TIM             (uint32_t)(0x02U)     /*!< additional interrupts in ST25R3911B_REG_IRQ_TIMER_NFC        */
#define ST25R3911B_IRQ_MASK_ERR             (uint32_t)(0x01U)     /*!< additional interrupts in ST25R3911B_REG_IRQ_ERROR_WUP        */

/* Timer and NFC interrupt register. */
#define ST25R3911B_IRQ_MASK_DCT             (uint32_t)(0x8000U)   /*!< ST25R3911 termination of direct command interrupt            */
#define ST25R3911B_IRQ_MASK_NRE             (uint32_t)(0x4000U)   /*!< ST25R3911 no-response timer expired interrupt                */
#define ST25R3911B_IRQ_MASK_GPE             (uint32_t)(0x2000U)   /*!< ST25R3911 general purpose timer expired interrupt            */
#define ST25R3911B_IRQ_MASK_EON             (uint32_t)(0x1000U)   /*!< ST25R3911 external field on interrupt                        */
#define ST25R3911B_IRQ_MASK_EOF             (uint32_t)(0x0800U)   /*!< ST25R3911 external field off interrupt                       */
#define ST25R3911B_IRQ_MASK_CAC             (uint32_t)(0x0400U)   /*!< ST25R3911 collision during RF collision avoidance interrupt  */
#define ST25R3911B_IRQ_MASK_CAT             (uint32_t)(0x0200U)   /*!< ST25R3911 minimum guard time expired interrupt               */
#define ST25R3911B_IRQ_MASK_NFCT            (uint32_t)(0x0100U)   /*!< ST25R3911 initiator bit rate recognized interrupt            */

/* Error and wake-up interrupt register. */
#define ST25R3911B_IRQ_MASK_CRC             (uint32_t)(0x800000U) /*!< ST25R3911 CRC error interrupt                                */
#define ST25R3911B_IRQ_MASK_PAR             (uint32_t)(0x400000U) /*!< ST25R3911 parity error interrupt                             */
#define ST25R3911B_IRQ_MASK_ERR2            (uint32_t)(0x200000U) /*!< ST25R3911 soft framing error interrupt                       */
#define ST25R3911B_IRQ_MASK_ERR1            (uint32_t)(0x100000U) /*!< ST25R3911 hard framing error interrupt                       */
#define ST25R3911B_IRQ_MASK_WT              (uint32_t)(0x080000U) /*!< ST25R3911 wake-up interrupt                                  */
#define ST25R3911B_IRQ_MASK_WAM             (uint32_t)(0x040000U) /*!< ST25R3911 wake-up due to amplitude interrupt                 */
#define ST25R3911B_IRQ_MASK_WPH             (uint32_t)(0x020000U) /*!< ST25R3911 wake-up due to phase interrupt                     */
#define ST25R3911B_IRQ_MASK_WCAP            (uint32_t)(0x010000U) /*!< ST25R3911 wake-up due to capacitance measurement             */
