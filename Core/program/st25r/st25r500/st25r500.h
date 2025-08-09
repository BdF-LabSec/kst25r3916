/*  Benjamin DELPY `gentilkiwi`
    https://blog.gentilkiwi.com
    benjamin@gentilkiwi.com
    Licence : https://creativecommons.org/licenses/by/4.0/
*/
#pragma once
#include "../st25r.h"

/* ST25R500, seems to support:
 * - ST25R300
 * - ST25R501
 * See: https://blog.gentilkiwi.com/Smartcards/NFC/st25r/Chips
 */


/* DS14655 # 5.15.2.1 - Serial peripheral interface (SPI) - Table 5 */

#define ST25R500_MASK_ADDRRESS		0b01111111
#define ST25R500_MASK_COMMAND		0b10011111


/* ST25R500 direct commands */
#define ST25R500_CMD_SET_DEFAULT              0x60U    /*!< Puts the chip in default state (same as after power-up) */
#define ST25R500_CMD_STOP                     0x62U    /*!< Stops all activities and clears FIFO                    */
#define ST25R500_CMD_CLEAR_FIFO               0x64U    /*!< Clears FIFO, Collision and IRQ status                   */
#define ST25R500_CMD_CLEAR_RXGAIN             0x66U    /*!< Clears FIFO, Collision and IRQ status                   */
#define ST25R500_CMD_ADJUST_REGULATORS        0x68U    /*!< Adjust regulators                                       */
#define ST25R500_CMD_TRANSMIT                 0x6AU    /*!< Transmit                                                */
#define ST25R500_CMD_TRANSMIT_EOF             0x6CU    /*!< Transmit ISO15693 EOF                                   */
#define ST25R500_CMD_NFC_FIELD_ON             0x6EU    /*!< Field On                                                */
#define ST25R500_CMD_MASK_RECEIVE_DATA        0x70U    /*!< Mask receive data                                       */
#define ST25R500_CMD_UNMASK_RECEIVE_DATA      0x72U    /*!< Unmask receive data                                     */
#define ST25R500_CMD_CALIBRATE_WU             0x74U    /*!< Calibrate Wake-up Measurement                           */
#define ST25R500_CMD_CLEAR_WU_CALIB           0x76U    /*!< Clear Wake-up Calibratation                             */
#define ST25R500_CMD_MEASURE_WU               0x78U    /*!< Measure Wake-up I and Q components                      */
#define ST25R500_CMD_MEASURE_IQ               0x7AU    /*!< Measure I and Q components                              */
#define ST25R500_CMD_SENSE_RF                 0x7CU    /*!< Sense RF on RFI pins                                    */
#define ST25R500_CMD_TRIGGER_WU_EV            0x7EU    /*!< Trigger Wake-up Event                             */
#define ST25R500_CMD_START_GP_TIMER           0xE2U    /*!< Start the general purpose timer                         */
#define ST25R500_CMD_START_WUT                0xE4U    /*!< Start the wake-up timer                                 */
#define ST25R500_CMD_START_MRT                0xE6U    /*!< Start the mask-receive timer                            */
#define ST25R500_CMD_START_NRT                0xE8U    /*!< Start the no-response timer                             */
#define ST25R500_CMD_STOP_NRT                 0xEAU    /*!< Stop No Response Timer                                  */
#define ST25R500_CMD_CALIBRATE_RC             0xEEU    /*!< Calibrate RC                                            */
#define ST25R500_CMD_TRIGGER_DIAG             0xF8U    /*!< Trigger Diagnostic Measurement                          */
#define ST25R500_CMD_TEST_ACCESS              0xFCU    /*!< Enable R/W access to the test registers                 */

#define ST25R500_BR_DO_NOT_SET                0xFFU    /*!< Indicates not to change this Bit Rate                   */
#define ST25R500_BR_106_26                    0x00U    /*!< ST25R500 Bit Rate  106 kbps (fc/128) / 26 kbps(fc/512)  */
#define ST25R500_BR_212_53                    0x01U    /*!< ST25R500 Bit Rate  212 kbps (fc/64)                     */
#define ST25R500_BR_424                       0x02U    /*!< ST25R500 Bit Rate  424 kbps (fc/32) / 53 kbps(fc/256)   */
#define ST25R500_BR_848                       0x03U    /*!< ST25R500 Bit Rate  848 kbps (fc/16)                     */

#define ST25R500_REG_DROP_200                 0U       /*!< ST25R500 target drop for regulator adjustment: 200mV    */
#define ST25R500_REG_DROP_250                 1U       /*!< ST25R500 target drop for regulator adjustment: 250mV    */
#define ST25R500_REG_DROP_300                 2U       /*!< ST25R500 target drop for regulator adjustment: 300mV    */
#define ST25R500_REG_DROP_350                 3U       /*!< ST25R500 target drop for regulator adjustment: 350mV    */
#define ST25R500_REG_DROP_400                 4U       /*!< ST25R500 target drop for regulator adjustment: 400mV    */
#define ST25R500_REG_DROP_450                 5U       /*!< ST25R500 target drop for regulator adjustment: 450mV    */
#define ST25R500_REG_DROP_500                 6U       /*!< ST25R500 target drop for regulator adjustment: 500mV    */
#define ST25R500_REG_DROP_550                 7U       /*!< ST25R500 target drop for regulator adjustment: 550mV    */
#define ST25R500_REG_DROP_DO_NOT_SET          0xFFU    /*!< Indicates not to change this setting (regd)             */

#define ST25R500_THRESHOLD_DO_NOT_SET         0xFFU    /*!< Indicates not to change this Threshold                  */

#define ST25R500_REG_LEN                      1U       /*!< Number of bytes in a ST25R500 register                  */
#define ST25R500_CMD_LEN                      1U       /*!< ST25R500 CMD length                                     */
#define ST25R500_FIFO_DEPTH                   256U     /*!< Depth of FIFO                                           */
#define ST25R500_TOUT_OSC_STABLE              5U       /*!< Timeout for Oscillator to get stable                    */

#define ST25R500_WRITE_MODE                   (0U << 7)           /*!< ST25R500 Operation Mode: Write               */
#define ST25R500_READ_MODE                    (1U << 7)           /*!< ST25R500 Operation Mode: Read                */
#define ST25R500_CMD_MODE                     ST25R500_WRITE_MODE /*!< ST25R500 Operation Mode: Direct Command      */
#define ST25R500_FIFO_ACCESS                  (0x5FU)             /*!< ST25R500 FIFO Access                         */


#define ST25R500_DIAG_MEAS_CMD                0x01U               /*!< ST25R500 Diagnostic Measurement cmd size     */
#define ST25R500_DIAG_MEAS_CMD_LEN            0x02U               /*!< ST25R500 Diagnostic Measurement cmd length   */
#define ST25R500_DIAG_MEAS_RES_LEN            0x04U               /*!< ST25R500 Diagnostic Measurement res length   */

#define ST25R500_DIAG_MEAS_I_VDD_DR           0x01U               /*!< ST25R500 Diagnostic Measurement: I_VDD_DR    */
#define ST25R500_DIAG_MEAS_VDD_TX             0x02U               /*!< ST25R500 Diagnostic Measurement: VDD_TX      */
#define ST25R500_DIAG_MEAS_VDD_DR             0x03U               /*!< ST25R500 Diagnostic Measurement: VDD_DR      */
#define ST25R500_DIAG_MEAS_VDD_IO             0x04U               /*!< ST25R500 Diagnostic Measurement: VDD_IO      */
#define ST25R500_DIAG_MEAS_VDD_D              0x0CU               /*!< ST25R500 Diagnostic Measurement: VDD_D       */
#define ST25R500_DIAG_MEAS_VDD_A              0x0DU               /*!< ST25R500 Diagnostic Measurement: VDD_A       */
#define ST25R500_DIAG_MEAS_VDD_VDD            0x11U               /*!< ST25R500 Diagnostic Measurement: VDD         */
#define ST25R500_DIAG_MEAS_VDD_AGD            0x12U               /*!< ST25R500 Diagnostic Measurement: AGD         */


#define ST25R500_FIFO_STATUS_LEN                               2        /*!< Number of FIFO Status Register                                    */
#define ST25R500_OBS_MODE_LEN                                  2        /*!< Number of Observation Mode Config Registers                       */

#define ST25R500_CEM_A_LEN                                     31U      /*!< Passive target memory A config length                             */
#define ST25R500_CEM_B_LEN                                     0U       /*!< Passive target memory B config length                             */
#define ST25R500_CEM_F_LEN                                     21U      /*!< Passive target memory F config length                             */


/*! Full CE memory length */
#define ST25R500_CEM_LEN                                       (ST25R500_CEM_A_LEN + ST25R500_CEM_B_LEN + ST25R500_CEM_F_LEN)

#define ST25R500_REG_OPERATION                                 0x00U    /*!< RW Operation Register                                             */
#define ST25R500_REG_GENERAL                                   0x01U    /*!< RW General Register                                               */
#define ST25R500_REG_REGULATOR                                 0x02U    /*!< RW Regulator Register                                             */
#define ST25R500_REG_DRIVER                                    0x03U    /*!< RW TX Driver Register                                             */
#define ST25R500_REG_TX_MOD1                                   0x04U    /*!< RW TX Modulation Register 1                                       */
#define ST25R500_REG_TX_MOD2                                   0x05U    /*!< RW TX Modulation Register 2                                       */
#define ST25R500_REG_CE_TX_MOD1                                0x06U    /*!< RW CE TX Modulation Register 1                                    */
#define ST25R500_REG_CE_TX_MOD2                                0x07U    /*!< RW CE TX Modulation Register 2                                    */
#define ST25R500_REG_GPIO                                      0x08U    /*!< RW GPIO control Register                                          */
#define ST25R500_REG_RX_ANA1                                   0x09U    /*!< RW RX Analog Register 1                                           */
#define ST25R500_REG_RX_ANA2                                   0x0AU    /*!< RW RX Analog Register 2                                           */
#define ST25R500_REG_RX_ANA3                                   0x0BU    /*!< RW RX Analog Register 3                                           */
#define ST25R500_REG_RX_ANA4                                   0x0CU    /*!< RW RX Analog Register 4                                           */
#define ST25R500_REG_RX_DIG                                    0x0DU    /*!< RW RX Digital Register                                            */
#define ST25R500_REG_CORR1                                     0x0EU    /*!< RW Correlator Register 1                                          */
#define ST25R500_REG_CORR2                                     0x0FU    /*!< RW Correlator Register 2                                          */
#define ST25R500_REG_CORR3                                     0x10U    /*!< RW Correlator Register 3                                          */
#define ST25R500_REG_CORR4                                     0x11U    /*!< RW Correlator Register 4                                          */
#define ST25R500_REG_CORR5                                     0x12U    /*!< RW Correlator Register 5                                          */
#define ST25R500_REG_CORR6                                     0x13U    /*!< RW Correlator Register 6                                          */
#define ST25R500_REG_PROTOCOL                                  0x14U    /*!< RW Protocol Register                                              */
#define ST25R500_REG_PROTOCOL_TX1                              0x15U    /*!< RW Protocol TX Register 1                                         */
#define ST25R500_REG_PROTOCOL_TX2                              0x16U    /*!< RW Protocol TX Register 2                                         */
#define ST25R500_REG_PROTOCOL_RX1                              0x17U    /*!< RW Protocol RX Register 1                                         */
#define ST25R500_REG_PROTOCOL_RX2                              0x18U    /*!< RW Protocol RX Register 2                                         */
#define ST25R500_REG_PROTOCOL_RX3                              0x19U    /*!< RW Protocol RX Register 3                                         */
#define ST25R500_REG_EMD1                                      0x1AU    /*!< RW Protocol EMD Register 1                                        */
#define ST25R500_REG_EMD2                                      0x1BU    /*!< RW Protocol EMD Register 2                                        */
#define ST25R500_REG_CE_CONFIG1                                0x1CU    /*!< RW CE Config Register 1                                           */
#define ST25R500_REG_CE_CONFIG2                                0x1DU    /*!< RW CE Config Register 2                                           */
#define ST25R500_REG_CE_CONFIG3                                0x1EU    /*!< RW CE Config Register 3                                           */
#define ST25R500_REG_MRT1                                      0x1FU    /*!< RW Mask Receive Timer Configuration Register 1                    */
#define ST25R500_REG_MRT2                                      0x20U    /*!< RW Mask Receive Timer Configuration Register 2                    */
#define ST25R500_REG_SQT                                       0x21U    /*!< RW Squelch Timer Register                                         */
#define ST25R500_REG_NRT_GPT_CONF                              0x22U    /*!< RW No Response and Genereal Purpose Timer Configuration Register  */
#define ST25R500_REG_NRT1                                      0x23U    /*!< RW No Response Timer (MSB) Register 1                             */
#define ST25R500_REG_NRT2                                      0x24U    /*!< RW No Response Timer (LSB) Register 2                             */
#define ST25R500_REG_GPT1                                      0x25U    /*!< RW General Purpose Timer (MSB) Register 1                         */
#define ST25R500_REG_GPT2                                      0x26U    /*!< RW General Purpose Timer (LSB) Register 2                         */
#define ST25R500_REG_WAKEUP_CONF1                              0x27U    /*!< RW Wake-up COnfiguration Register 1                               */
#define ST25R500_REG_WAKEUP_CONF2                              0x28U    /*!< RW Wake-up COnfiguration Register 2                               */
#define ST25R500_REG_WAKEUP_CONF3                              0x29U    /*!< RW Wake-up COnfiguration Register 3                               */
#define ST25R500_REG_WU_I_CONF                                 0x2AU    /*!< RW Wake-up I-Channel Configuration Register                       */
#define ST25R500_REG_WU_I_DELTA                                0x2BU    /*!< RW Wake-up I-Channel Delta Register                               */
#define ST25R500_REG_WU_I_CAL                                  0x2CU    /*!< RO Wake-up I-Channel Calibration Display Register                 */
#define ST25R500_REG_WU_I_ADC                                  0x2DU    /*!< RO Wake-up I-Channel ADC Display Register                         */
#define ST25R500_REG_WU_I_REF                                  0x2EU    /*!< RO Wake-up I-Channel Reference Display Register                   */
#define ST25R500_REG_WU_Q_CONF                                 0x2FU    /*!< RW Wake-up Q-Channel Configuration Register                       */
#define ST25R500_REG_WU_Q_DELTA                                0x30U    /*!< RW Wake-up Q-Channel Delta Register                               */
#define ST25R500_REG_WU_Q_CAL                                  0x31U    /*!< RO Wake-up Q-Channel Calibration Display Register                 */
#define ST25R500_REG_WU_Q_ADC                                  0x32U    /*!< RO Wake-up Q-Channel ADC Display Register                         */
#define ST25R500_REG_WU_Q_REF                                  0x33U    /*!< RO Wake-up Q-Channel Reference Display Register                   */
#define ST25R500_REG_TX_FRAME1                                 0x34U    /*!< RW TX Frame Register 1                                            */
#define ST25R500_REG_TX_FRAME2                                 0x35U    /*!< RW TX Frame Register 2                                            */
#define ST25R500_REG_FIFO_STATUS1                              0x36U    /*!< RO FIFO Status Register 1                                         */
#define ST25R500_REG_FIFO_STATUS2                              0x37U    /*!< RO FIFO Status Register 2                                         */
#define ST25R500_REG_COLLISION                                 0x38U    /*!< RO Collision Register 2                                           */
#define ST25R500_REG_IRQ_MASK1                                 0x39U    /*!< RO IRQ Mask Register 1                                            */
#define ST25R500_REG_IRQ_MASK2                                 0x3AU    /*!< RO IRQ Mask Register 2                                            */
#define ST25R500_REG_IRQ_MASK3                                 0x3BU    /*!< RO IRQ Mask Register 3                                            */
#define ST25R500_REG_IRQ1                                      0x3CU    /*!< RO IRQ Register 1                                                 */
#define ST25R500_REG_IRQ2                                      0x3DU    /*!< RO IRQ Register 2                                                 */
#define ST25R500_REG_IRQ3                                      0x3EU    /*!< RO IRQ Register 3                                                 */
#define ST25R500_REG_IC_ID                                     0x3FU    /*!< RO IC Identity Register                                           */

#define ST25R500_REG_STATUS1                                   0x40U    /*!< RO Status Register 1                                              */
#define ST25R500_REG_STATUS2                                   0x41U    /*!< RO Status Register 2                                              */
#define ST25R500_REG_STATUS_STATIC1                            0x42U    /*!< RO Status Static Register 1                                       */
#define ST25R500_REG_STATUS_STATIC2                            0x43U    /*!< RO Status Static Register 2                                       */
#define ST25R500_REG_STATUS_STATIC3                            0x44U    /*!< RO Status Static Register 3                                       */
#define ST25R500_REG_CE_STATUS1                                0x45U    /*!< RO CE Status Register 1                                           */
#define ST25R500_REG_CE_STATUS2                                0x46U    /*!< RO CE Status Register 2                                           */
#define ST25R500_REG_WU_STATUS                                 0x47U    /*!< RO WU Status Register                                             */
#define ST25R500_REG_ANA_DISPLAY1                              0x48U    /*!< RO Analog Display Register 1                                      */
#define ST25R500_REG_ANA_DISPLAY2                              0x49U    /*!< RO Analog Display Register 2                                      */
#define ST25R500_REG_RSSI_I                                    0x4AU    /*!< RO I RSSI Display Register                                        */
#define ST25R500_REG_RSSI_Q                                    0x4BU    /*!< RO Q RSSI Display Register                                        */
#define ST25R500_REG_SENSE_DISPLAY                             0x4CU    /*!< RO Display Register 3                                             */
#define ST25R500_REG_AWS_CONFIG1                               0x4DU    /*!< RW AWS Config Register 1                                          */
#define ST25R500_REG_AWS_CONFIG2                               0x4EU    /*!< RW AWS Config Register 2                                          */
#define ST25R500_REG_AWS_TIME1                                 0x4FU    /*!< RW AWS Time Register 1                                            */
#define ST25R500_REG_AWS_TIME2                                 0x50U    /*!< RW AWS Time Register 2                                            */
#define ST25R500_REG_AWS_TIME3                                 0x51U    /*!< RW AWS Time Register 3                                            */
#define ST25R500_REG_AWS_TIME4                                 0x52U    /*!< RW AWS Time Register 4                                            */
#define ST25R500_REG_OVERSHOOT_CONF                            0x53U    /*!< RW Overshoot Pattern Register                                     */
#define ST25R500_REG_UNDERSHOOT_CONF                           0x54U    /*!< RW Undershoot Pattern Register                                    */
#define ST25R500_REG_EFD_THRESHOLD                             0x57U    /*!< RW EFD Thresholds Register                                        */

#define ST25R500_REG_CEM_A                                     0x58U    /*!< RW CE Memory A                                                    */
#define ST25R500_REG_CEM_F                                     0x5AU    /*!< RW CE Memory F Register                                           */
#define ST25R500_REG_FIFO                                      0x5FU    /*!< RW FIFO address                                                   */

#define ST25R500_TEST_REG_ADC2CE                               0x08U    /*!< RW ADC to CE memory Register                                      */
#define ST25R500_TEST_REG_OSC_TIMING                           0x0EU    /*!< RW Oscillator timing control Register                             */
#define ST25R500_TEST_REG_MAN_TIMING                           0x13U    /*!< RW Manual timing Register                                         */
#define ST25R500_TEST_REG_DIAG_MEAS                            0x14U    /*!< RW Diagnostic Measurement Register                                */
#define ST25R500_TEST_REG_OVERLAP_CONTROL                      0x1EU    /*!< RW Overlap control Register                                       */
#define ST25R500_TEST_REG_RXS_DISP1                            0x30U    /*!< RO Receive start timer display Register 1                         */
#define ST25R500_TEST_REG_RXS_DISP2                            0x31U    /*!< RO Receive start timer display Register 2                         */
#define ST25R500_TEST_REG_RXE_DISP1                            0x32U    /*!< RO Receive end timer display Register 1                           */
#define ST25R500_TEST_REG_RXE_DISP2                            0x33U    /*!< RO Receive end timer display Register 2                           */



/*! Register bit definitions  \cond DOXYGEN_SUPPRESS */

#define ST25R500_REG_OPERATION_wpt_en                         (1U<<7)
#define ST25R500_REG_OPERATION_tx_en                          (1U<<6)
#define ST25R500_REG_OPERATION_rx_en                          (1U<<5)
#define ST25R500_REG_OPERATION_vdddr_en                       (1U<<4)
#define ST25R500_REG_OPERATION_en                             (1U<<3)
#define ST25R500_REG_OPERATION_ce_en                          (1U<<2)
#define ST25R500_REG_OPERATION_rfu                            (1U<<1)
#define ST25R500_REG_OPERATION_wu_en                          (1U<<0)

#define ST25R500_REG_GENERAL_rfu1                             (1U<<7)
#define ST25R500_REG_GENERAL_rfu0                             (1U<<6)
#define ST25R500_REG_GENERAL_single                           (1U<<5)
#define ST25R500_REG_GENERAL_rfo2                             (1U<<4)
#define ST25R500_REG_GENERAL_miso_pd2                         (1U<<3)
#define ST25R500_REG_GENERAL_miso_pd1                         (1U<<2)
#define ST25R500_REG_GENERAL_miso_pd_mask                     (0x3U<<2)
#define ST25R500_REG_GENERAL_miso_pd_shift                    (2U)
#define ST25R500_REG_GENERAL_nfc_n1                           (1U<<1)
#define ST25R500_REG_GENERAL_nfc_n0                           (1U<<0)
#define ST25R500_REG_GENERAL_nfc_n_mask                       (0x3U<<0)
#define ST25R500_REG_GENERAL_nfc_n_shift                      (0U)

#define ST25R500_REG_REGULATOR_reg_s                          (1U<<7)
#define ST25R500_REG_REGULATOR_rege6                          (1U<<6)
#define ST25R500_REG_REGULATOR_rege5                          (1U<<5)
#define ST25R500_REG_REGULATOR_rege4                          (1U<<4)
#define ST25R500_REG_REGULATOR_rege3                          (1U<<3)
#define ST25R500_REG_REGULATOR_rege2                          (1U<<2)
#define ST25R500_REG_REGULATOR_rege1                          (1U<<1)
#define ST25R500_REG_REGULATOR_rege0                          (1U<<0)
#define ST25R500_REG_REGULATOR_rege_mask                      (0x7FU<<0)
#define ST25R500_REG_REGULATOR_rege_shift                     (0U)

#define ST25R500_REG_DRIVER_rfu                               (1U<<7)
#define ST25R500_REG_DRIVER_regd2                             (1U<<6)
#define ST25R500_REG_DRIVER_regd1                             (1U<<5)
#define ST25R500_REG_DRIVER_regd0                             (1U<<4)
#define ST25R500_REG_DRIVER_regd_mask                         (0x7U<<4)
#define ST25R500_REG_DRIVER_regd_shift                        (4U)
#define ST25R500_REG_DRIVER_regd_200mV                        (0U<<4)
#define ST25R500_REG_DRIVER_regd_250mV                        (1U<<4)
#define ST25R500_REG_DRIVER_regd_300mV                        (2U<<4)
#define ST25R500_REG_DRIVER_regd_350mV                        (3U<<4)
#define ST25R500_REG_DRIVER_regd_400mV                        (4U<<4)
#define ST25R500_REG_DRIVER_regd_450mV                        (5U<<4)
#define ST25R500_REG_DRIVER_regd_500mV                        (6U<<4)
#define ST25R500_REG_DRIVER_regd_550mV                        (7U<<4)
#define ST25R500_REG_DRIVER_d_res3                            (1U<<3)
#define ST25R500_REG_DRIVER_d_res2                            (1U<<2)
#define ST25R500_REG_DRIVER_d_res1                            (1U<<1)
#define ST25R500_REG_DRIVER_d_res0                            (1U<<0)
#define ST25R500_REG_DRIVER_d_res_mask                        (0xFU<<0)
#define ST25R500_REG_DRIVER_d_res_shift                       (0U)

#define ST25R500_REG_TX_MOD1_am_mod3                          (1U<<7)
#define ST25R500_REG_TX_MOD1_am_mod2                          (1U<<6)
#define ST25R500_REG_TX_MOD1_am_mod1                          (1U<<5)
#define ST25R500_REG_TX_MOD1_am_mod0                          (1U<<4)
#define ST25R500_REG_TX_MOD1_am_mod_8percent                  (0x0U<<4)
#define ST25R500_REG_TX_MOD1_am_mod_10percent                 (0x1U<<4)
#define ST25R500_REG_TX_MOD1_am_mod_11percent                 (0x2U<<4)
#define ST25R500_REG_TX_MOD1_am_mod_12percent                 (0x3U<<4)
#define ST25R500_REG_TX_MOD1_am_mod_13percent                 (0x4U<<4)
#define ST25R500_REG_TX_MOD1_am_mod_14percent                 (0x5U<<4)
#define ST25R500_REG_TX_MOD1_am_mod_15percent                 (0x6U<<4)
#define ST25R500_REG_TX_MOD1_am_mod_20percent                 (0x7U<<4)
#define ST25R500_REG_TX_MOD1_am_mod_30percent                 (0x8U<<4)
#define ST25R500_REG_TX_MOD1_am_mod_40percent                 (0x9U<<4)
#define ST25R500_REG_TX_MOD1_am_mod_50percent                 (0xAU<<4)
#define ST25R500_REG_TX_MOD1_am_mod_60percent                 (0xBU<<4)
#define ST25R500_REG_TX_MOD1_am_mod_70percent                 (0xCU<<4)
#define ST25R500_REG_TX_MOD1_am_mod_80percent                 (0xDU<<4)
#define ST25R500_REG_TX_MOD1_am_mod_88percent                 (0xEU<<4)
#define ST25R500_REG_TX_MOD1_am_mod_97percent                 (0xFU<<4)
#define ST25R500_REG_TX_MOD1_am_mod_mask                      (0xFU<<4)
#define ST25R500_REG_TX_MOD1_am_mod_shift                     (4U)
#define ST25R500_REG_TX_MOD1_mod_state                        (1U<<3)
#define ST25R500_REG_TX_MOD1_rgs_am                           (1U<<2)
#define ST25R500_REG_TX_MOD1_res_am                           (1U<<1)
#define ST25R500_REG_TX_MOD1_rfu                              (1U<<0)

#define ST25R500_REG_TX_MOD2_rfu                              (1U<<7)
#define ST25R500_REG_TX_MOD2_md_res6                          (1U<<6)
#define ST25R500_REG_TX_MOD2_md_res5                          (1U<<5)
#define ST25R500_REG_TX_MOD2_md_res4                          (1U<<4)
#define ST25R500_REG_TX_MOD2_md_res3                          (1U<<3)
#define ST25R500_REG_TX_MOD2_md_res2                          (1U<<2)
#define ST25R500_REG_TX_MOD2_md_res1                          (1U<<1)
#define ST25R500_REG_TX_MOD2_md_res0                          (1U<<0)
#define ST25R500_REG_TX_MOD2_md_res_mask                      (0x7FU<<0)
#define ST25R500_REG_TX_MOD2_md_res_shift                     (0U)

#define ST25R500_REG_CE_TX_MOD1_cem_res3                      (1U<<7)
#define ST25R500_REG_CE_TX_MOD1_cem_res2                      (1U<<6)
#define ST25R500_REG_CE_TX_MOD1_cem_res1                      (1U<<5)
#define ST25R500_REG_CE_TX_MOD1_cem_res0                      (1U<<4)
#define ST25R500_REG_CE_TX_MOD1_cem_res_mask                  (0xFU<<4)
#define ST25R500_REG_CE_TX_MOD1_cem_res_shift                 (4U)
#define ST25R500_REG_CE_TX_MOD1_ce_res3                       (1U<<3)
#define ST25R500_REG_CE_TX_MOD1_ce_res2                       (1U<<2)
#define ST25R500_REG_CE_TX_MOD1_ce_res1                       (1U<<1)
#define ST25R500_REG_CE_TX_MOD1_ce_res0                       (1U<<0)
#define ST25R500_REG_CE_TX_MOD1_ce_res_mask                   (0xFU<<0)
#define ST25R500_REG_CE_TX_MOD1_ce_res_shift                  (0U)

#define ST25R500_REG_CE_TX_MOD2_gpio_en1                      (1U<<7)
#define ST25R500_REG_CE_TX_MOD2_gpio_en0                      (1U<<6)
#define ST25R500_REG_CE_TX_MOD2_gpio_en_mask                  (0x3U<<6)
#define ST25R500_REG_CE_TX_MOD2_gpio_en_shift                 (6U)
#define ST25R500_REG_CE_TX_MOD2_rfu                           (1U<<5)
#define ST25R500_REG_CE_TX_MOD2_tad_en                        (1U<<4)
#define ST25R500_REG_CE_TX_MOD2_lm_gpio1                      (1U<<3)
#define ST25R500_REG_CE_TX_MOD2_lm_gpio0                      (1U<<2)
#define ST25R500_REG_CE_TX_MOD2_lm_gpio_mask                  (0x3U<<2)
#define ST25R500_REG_CE_TX_MOD2_lm_gpio_shift                 (2U)
#define ST25R500_REG_CE_TX_MOD2_lm_trim                       (1U<<1)
#define ST25R500_REG_CE_TX_MOD2_lm_dri                        (1U<<0)

#define ST25R500_REG_GPIO_trim_ce1                            (1U<<7)
#define ST25R500_REG_GPIO_trim_ce0                            (1U<<6)
#define ST25R500_REG_GPIO_trim_ce_mask                        (0x3U<<6)
#define ST25R500_REG_GPIO_trim_ce_shift                       (6U)
#define ST25R500_REG_GPIO_gpio_ce1                            (1U<<5)
#define ST25R500_REG_GPIO_gpio_ce0                            (1U<<4)
#define ST25R500_REG_GPIO_gpio_ce_mask                        (0x3U<<4)
#define ST25R500_REG_GPIO_gpio_ce_shift                       (2U)
#define ST25R500_REG_GPIO_trim_rw1                            (1U<<3)
#define ST25R500_REG_GPIO_trim_rw0                            (1U<<2)
#define ST25R500_REG_GPIO_trim_rw_mask                        (0x3U<<2)
#define ST25R500_REG_GPIO_trim_rw_shift                       (2U)
#define ST25R500_REG_GPIO_gpio_rw1                            (1U<<1)
#define ST25R500_REG_GPIO_gpio_rw0                            (1U<<0)
#define ST25R500_REG_GPIO_gpio_rw_mask                        (0x3U<<0)
#define ST25R500_REG_GPIO_gpio_rw_shift                       (0U)

#define ST25R500_REG_RX_ANA1_dig_clk_dly3                     (1U<<7)
#define ST25R500_REG_RX_ANA1_dig_clk_dly2                     (1U<<6)
#define ST25R500_REG_RX_ANA1_dig_clk_dly1                     (1U<<5)
#define ST25R500_REG_RX_ANA1_dig_clk_dly0                     (1U<<4)
#define ST25R500_REG_RX_ANA1_dig_clk_dly_mask                 (0x0FU<<4)
#define ST25R500_REG_RX_ANA1_dig_clk_dly_shift                (4U)
#define ST25R500_REG_RX_ANA1_rfu                              (1U<<3)
#define ST25R500_REG_RX_ANA1_gain_boost                       (1U<<2)
#define ST25R500_REG_RX_ANA1_hpf_ctrl1                        (1U<<1)
#define ST25R500_REG_RX_ANA1_hpf_ctrl0                        (1U<<0)
#define ST25R500_REG_RX_ANA1_hpf_ctrl_mask                    (3U<<0)
#define ST25R500_REG_RX_ANA1_hpf_ctrl_shift                   (0U)

#define ST25R500_REG_RX_ANA2_afe_gain_rw3                     (1U<<7)
#define ST25R500_REG_RX_ANA2_afe_gain_rw2                     (1U<<6)
#define ST25R500_REG_RX_ANA2_afe_gain_rw1                     (1U<<5)
#define ST25R500_REG_RX_ANA2_afe_gain_rw0                     (1U<<4)
#define ST25R500_REG_RX_ANA2_afe_gain_rw_mask                 (0x0FU<<4)
#define ST25R500_REG_RX_ANA2_afe_gain_rw_shift                (4U)
#define ST25R500_REG_RX_ANA2_afe_gain_td3                     (1U<<3)
#define ST25R500_REG_RX_ANA2_afe_gain_td2                     (1U<<2)
#define ST25R500_REG_RX_ANA2_afe_gain_td1                     (1U<<1)
#define ST25R500_REG_RX_ANA2_afe_gain_td0                     (1U<<0)
#define ST25R500_REG_RX_ANA2_afe_gain_td_mask                 (0x0FU<<0)
#define ST25R500_REG_RX_ANA2_afe_gain_td_shift                (0U)

#define ST25R500_REG_RX_ANA3_afe_gain_ce3                     (1U<<7)
#define ST25R500_REG_RX_ANA3_afe_gain_ce2                     (1U<<6)
#define ST25R500_REG_RX_ANA3_afe_gain_ce1                     (1U<<5)
#define ST25R500_REG_RX_ANA3_afe_gain_ce0                     (1U<<4)
#define ST25R500_REG_RX_ANA3_afe_gain_ce_mask                 (0x0FU<<4)
#define ST25R500_REG_RX_ANA3_afe_gain_ce_shift                (4U)
#define ST25R500_REG_RX_ANA3_ook_thr_hi1                      (1U<<3)
#define ST25R500_REG_RX_ANA3_ook_thr_hi0                      (1U<<2)
#define ST25R500_REG_RX_ANA3_ook_thr_hi_mask                  (0x03U<<2)
#define ST25R500_REG_RX_ANA3_ook_thr_hi_shift                 (2U)
#define ST25R500_REG_RX_ANA3_ook_thr_lo1                      (1U<<1)
#define ST25R500_REG_RX_ANA3_ook_thr_lo0                      (1U<<0)
#define ST25R500_REG_RX_ANA3_ook_thr_lo_mask                  (0x03U<<0)
#define ST25R500_REG_RX_ANA3_ook_thr_lo_shift                 (0U)

#define ST25R500_REG_RX_ANA4_en_phase_deadzone                (1U<<7)
#define ST25R500_REG_RX_ANA4_en_rect_cor                      (1U<<6)
#define ST25R500_REG_RX_ANA4_rfu5                             (1U<<5)
#define ST25R500_REG_RX_ANA4_rfu4                             (1U<<4)
#define ST25R500_REG_RX_ANA4_rfu3                             (1U<<3)
#define ST25R500_REG_RX_ANA4_rfu2                             (1U<<2)
#define ST25R500_REG_RX_ANA4_rfu1                             (1U<<1)
#define ST25R500_REG_RX_ANA4_rfu0                             (1U<<0)

#define ST25R500_REG_RX_DIG_agc_en                            (1U<<7)
#define ST25R500_REG_RX_DIG_lpf_coef2                         (1U<<6)
#define ST25R500_REG_RX_DIG_lpf_coef1                         (1U<<5)
#define ST25R500_REG_RX_DIG_lpf_coef0                         (1U<<4)
#define ST25R500_REG_RX_DIG_lpf_coef_mask                     (7U<<4)
#define ST25R500_REG_RX_DIG_lpf_coef_shift                    (4U)
#define ST25R500_REG_RX_DIG_hpf_coef1                         (1U<<3)
#define ST25R500_REG_RX_DIG_hpf_coef0                         (1U<<2)
#define ST25R500_REG_RX_DIG_hpf_coef_mask                     (3U<<2)
#define ST25R500_REG_RX_DIG_hpf_coef_shift                    (2U)
#define ST25R500_REG_RX_DIG_hpf_ce_agc_freeze                 (1U<<1)
#define ST25R500_REG_RX_DIG_rfu                               (1U<<0)

#define ST25R500_REG_CORR1_iir_coef2_3                        (1U<<7)
#define ST25R500_REG_CORR1_iir_coef2_2                        (1U<<6)
#define ST25R500_REG_CORR1_iir_coef2_1                        (1U<<5)
#define ST25R500_REG_CORR1_iir_coef2_0                        (1U<<4)
#define ST25R500_REG_CORR1_iir_coef2_mask                     (0x0FU<<4)
#define ST25R500_REG_CORR1_iir_coef2_shift                    (4U)
#define ST25R500_REG_CORR1_iir_coef1_3                        (1U<<3)
#define ST25R500_REG_CORR1_iir_coef1_2                        (1U<<2)
#define ST25R500_REG_CORR1_iir_coef1_1                        (1U<<1)
#define ST25R500_REG_CORR1_iir_coef1_0                        (1U<<0)
#define ST25R500_REG_CORR1_iir_coef1_mask                     (0x0FU<<0)
#define ST25R500_REG_CORR1_iir_coef1_shift                    (0U)

#define ST25R500_REG_CORR2_agc_thr_squelch_3                  (1U<<7)
#define ST25R500_REG_CORR2_agc_thr_squelch_2                  (1U<<6)
#define ST25R500_REG_CORR2_agc_thr_squelch_1                  (1U<<5)
#define ST25R500_REG_CORR2_agc_thr_squelch_0                  (1U<<4)
#define ST25R500_REG_CORR2_agc_thr_squelch_mask               (0x0FU<<4)
#define ST25R500_REG_CORR2_agc_thr_squelch_shift              (4U)
#define ST25R500_REG_CORR2_agc_thr_3                          (1U<<3)
#define ST25R500_REG_CORR2_agc_thr_2                          (1U<<2)
#define ST25R500_REG_CORR2_agc_thr_1                          (1U<<1)
#define ST25R500_REG_CORR2_agc_thr_0                          (1U<<0)
#define ST25R500_REG_CORR2_agc_thr_mask                       (0x0FU<<0)
#define ST25R500_REG_CORR2_agc_thr_shift                      (0U)

#define ST25R500_REG_CORR3_rfu                                (1U<<7)
#define ST25R500_REG_CORR3_en_subc_end                        (1U<<6)
#define ST25R500_REG_CORR3_start_wait5                        (1U<<5)
#define ST25R500_REG_CORR3_start_wait4                        (1U<<4)
#define ST25R500_REG_CORR3_start_wait3                        (1U<<3)
#define ST25R500_REG_CORR3_start_wait2                        (1U<<2)
#define ST25R500_REG_CORR3_start_wait1                        (1U<<1)
#define ST25R500_REG_CORR3_start_wait0                        (1U<<0)
#define ST25R500_REG_CORR3_start_wait_mask                    (0x3FU<<0)
#define ST25R500_REG_CORR3_start_wait_shift                   (0U)

#define ST25R500_REG_CORR4_coll_lvl3                          (1U<<7)
#define ST25R500_REG_CORR4_coll_lvl2                          (1U<<6)
#define ST25R500_REG_CORR4_coll_lvl1                          (1U<<5)
#define ST25R500_REG_CORR4_coll_lvl0                          (1U<<4)
#define ST25R500_REG_CORR4_coll_lvl_mask                      (0x0FU<<4)
#define ST25R500_REG_CORR4_coll_lvl_shift                     (4U)
#define ST25R500_REG_CORR4_data_lvl3                          (1U<<3)
#define ST25R500_REG_CORR4_data_lvl2                          (1U<<2)
#define ST25R500_REG_CORR4_data_lvl1                          (1U<<1)
#define ST25R500_REG_CORR4_data_lvl0                          (1U<<0)
#define ST25R500_REG_CORR4_data_lvl_mask                      (0x0FU<<0)
#define ST25R500_REG_CORR4_data_lvl_shift                     (0U)

#define ST25R500_REG_CORR5_dis_noise_leak                     (1U<<7)
#define ST25R500_REG_CORR5_dis_noise_meas                     (1U<<6)
#define ST25R500_REG_CORR5_dis_soft_sq                        (1U<<5)
#define ST25R500_REG_CORR5_dis_agc_noise_meas                 (1U<<4)
#define ST25R500_REG_CORR5_no_phase                           (1U<<3)
#define ST25R500_REG_CORR5_dec_f2                             (1U<<2)
#define ST25R500_REG_CORR5_dec_f1                             (1U<<1)
#define ST25R500_REG_CORR5_dec_f0                             (1U<<0)
#define ST25R500_REG_CORR5_dec_f_mask                         (0x7U<<0)
#define ST25R500_REG_CORR5_dec_f_shift                        (0U)

#define ST25R500_REG_CORR6_init_noise_lvl3                    (1U<<7)
#define ST25R500_REG_CORR6_init_noise_lvl2                    (1U<<6)
#define ST25R500_REG_CORR6_init_noise_lvl1                    (1U<<5)
#define ST25R500_REG_CORR6_init_noise_lvl0                    (1U<<4)
#define ST25R500_REG_CORR6_init_noise_lvl_mask                (0x0FU<<4)
#define ST25R500_REG_CORR6_init_noise_lvl_shift               (4U)
#define ST25R500_REG_CORR6_agc_freeze_cnt3                    (1U<<3)
#define ST25R500_REG_CORR6_agc_freeze_cnt2                    (1U<<2)
#define ST25R500_REG_CORR6_agc_freeze_cnt1                    (1U<<1)
#define ST25R500_REG_CORR6_agc_freeze_cnt0                    (1U<<0)
#define ST25R500_REG_CORR6_agc_freeze_cnt_mask                (0x0FU<<0)
#define ST25R500_REG_CORR6_agc_freeze_cnt_shift               (0U)

#define ST25R500_REG_PROTOCOL_rx_rate1                        (1U<<7)
#define ST25R500_REG_PROTOCOL_rx_rate0                        (1U<<6)
#define ST25R500_REG_PROTOCOL_rx_rate_106_26                  (0x0U<<6)
#define ST25R500_REG_PROTOCOL_rx_rate_212_53                  (0x2U<<6)
#define ST25R500_REG_PROTOCOL_rx_rate_424                     (0x3U<<6)
#define ST25R500_REG_PROTOCOL_rx_rate_848                     (0x4U<<6)
#define ST25R500_REG_PROTOCOL_rx_rate_mask                    (0x03U<<6)
#define ST25R500_REG_PROTOCOL_rx_rate_shift                   (6U)
#define ST25R500_REG_PROTOCOL_tx_rate1                        (1U<<5)
#define ST25R500_REG_PROTOCOL_tx_rate0                        (1U<<4)
#define ST25R500_REG_PROTOCOL_tx_rate_106                     (0x0U<<4)
#define ST25R500_REG_PROTOCOL_tx_rate_212                     (0x1U<<4)
#define ST25R500_REG_PROTOCOL_tx_rate_424                     (0x2U<<4)
#define ST25R500_REG_PROTOCOL_tx_rate_848                     (0x3U<<4)
#define ST25R500_REG_PROTOCOL_tx_rate_mask                    (0x03U<<4)
#define ST25R500_REG_PROTOCOL_tx_rate_shift                   (4U)
#define ST25R500_REG_PROTOCOL_om3                             (1U<<3)
#define ST25R500_REG_PROTOCOL_om2                             (1U<<2)
#define ST25R500_REG_PROTOCOL_om1                             (1U<<1)
#define ST25R500_REG_PROTOCOL_om0                             (1U<<0)
#define ST25R500_REG_PROTOCOL_om_iso14443a                    (0x01U<<0)
#define ST25R500_REG_PROTOCOL_om_iso14443b                    (0x02U<<0)
#define ST25R500_REG_PROTOCOL_om_felica                       (0x03U<<0)
#define ST25R500_REG_PROTOCOL_om_topaz                        (0x04U<<0)
#define ST25R500_REG_PROTOCOL_om_iso15693                     (0x05U<<0)
#define ST25R500_REG_PROTOCOL_om_sbrdml                       (0x0dU<<0)
#define ST25R500_REG_PROTOCOL_om_sbrdm                        (0x0eU<<0)
#define ST25R500_REG_PROTOCOL_om_brdm                         (0x0fU<<0)
#define ST25R500_REG_PROTOCOL_om_mask                         (0x0FU<<0)
#define ST25R500_REG_PROTOCOL_om_shift                        (0U)

#define ST25R500_REG_PROTOCOL_TX1_a_nfc_f0                    (1U<<7)
#define ST25R500_REG_PROTOCOL_TX1_a_tx_par                    (1U<<6)
#define ST25R500_REG_PROTOCOL_TX1_a_tx_par_on                 (1U<<6)
#define ST25R500_REG_PROTOCOL_TX1_a_tx_par_off                (0U<<6)
#define ST25R500_REG_PROTOCOL_TX1_tx_crc                      (1U<<5)
#define ST25R500_REG_PROTOCOL_TX1_tx_crc_on                   (1U<<5)
#define ST25R500_REG_PROTOCOL_TX1_tx_crc_off                  (0U<<5)
#define ST25R500_REG_PROTOCOL_TX1_tr_am                       (1U<<4)
#define ST25R500_REG_PROTOCOL_TX1_p_len3                      (1U<<3)
#define ST25R500_REG_PROTOCOL_TX1_p_len2                      (1U<<2)
#define ST25R500_REG_PROTOCOL_TX1_p_len1                      (1U<<1)
#define ST25R500_REG_PROTOCOL_TX1_p_len0                      (1U<<0)
#define ST25R500_REG_PROTOCOL_TX1_p_len_mask                  (0x0FU<<0)
#define ST25R500_REG_PROTOCOL_TX1_p_len_shift                 (0U)

#define ST25R500_REG_PROTOCOL_TX2_rfu2                        (1U<<7)
#define ST25R500_REG_PROTOCOL_TX2_rfu1                        (1U<<6)
#define ST25R500_REG_PROTOCOL_TX2_rfu0                        (1U<<5)
#define ST25R500_REG_PROTOCOL_TX2_f_tx_len                    (1U<<4)
#define ST25R500_REG_PROTOCOL_TX2_b_tx_sof_0                  (1U<<3)
#define ST25R500_REG_PROTOCOL_TX2_b_tx_sof_1                  (1U<<2)
#define ST25R500_REG_PROTOCOL_TX2_b_tx_sof_0_10etu            (0U<<3)
#define ST25R500_REG_PROTOCOL_TX2_b_tx_sof_0_11etu            (1U<<3)
#define ST25R500_REG_PROTOCOL_TX2_b_tx_sof_1_2etu             (0U<<3)
#define ST25R500_REG_PROTOCOL_TX2_b_tx_sof_1_3etu             (1U<<3)
#define ST25R500_REG_PROTOCOL_TX2_b_tx_sof_mask               (3U<<2)
#define ST25R500_REG_PROTOCOL_TX2_b_tx_sof_shift              (2U)
#define ST25R500_REG_PROTOCOL_TX2_b_tx_eof                    (1U<<1)
#define ST25R500_REG_PROTOCOL_TX2_b_tx_eof_11etu              (1U<<1)
#define ST25R500_REG_PROTOCOL_TX2_b_tx_eof_10etu              (0U<<1)
#define ST25R500_REG_PROTOCOL_TX2_b_tx_half                   (1U<<0)

#define ST25R500_REG_PROTOCOL_RX1_rfu1                        (1U<<7)
#define ST25R500_REG_PROTOCOL_RX1_rfu0                        (1U<<5)
#define ST25R500_REG_PROTOCOL_RX1_b_rx_sof                    (1U<<5)
#define ST25R500_REG_PROTOCOL_RX1_b_rx_eof                    (1U<<4)
#define ST25R500_REG_PROTOCOL_RX1_a_rx_par                    (1U<<3)
#define ST25R500_REG_PROTOCOL_RX1_a_rx_par_on                 (1U<<3)
#define ST25R500_REG_PROTOCOL_RX1_a_rx_par_off                (0U<<3)
#define ST25R500_REG_PROTOCOL_RX1_rx_crc                      (1U<<2)
#define ST25R500_REG_PROTOCOL_RX1_rx_crc_on                   (1U<<2)
#define ST25R500_REG_PROTOCOL_RX1_rx_crc_off                  (0U<<2)
#define ST25R500_REG_PROTOCOL_RX1_rx_nbtx                     (1U<<1)
#define ST25R500_REG_PROTOCOL_RX1_antcl                       (1U<<0)
#define ST25R500_REG_PROTOCOL_RX1_antcl_on                    (1U<<0)
#define ST25R500_REG_PROTOCOL_RX1_antcl_off                   (0U<<0)

#define ST25R500_REG_PROTOCOL_RX2_rfu                         (1U<<7)
#define ST25R500_REG_PROTOCOL_RX2_tr1_min_len6                (1U<<6)
#define ST25R500_REG_PROTOCOL_RX2_tr1_min_len5                (1U<<5)
#define ST25R500_REG_PROTOCOL_RX2_tr1_min_len4                (1U<<4)
#define ST25R500_REG_PROTOCOL_RX2_tr1_min_len3                (1U<<3)
#define ST25R500_REG_PROTOCOL_RX2_tr1_min_len2                (1U<<2)
#define ST25R500_REG_PROTOCOL_RX2_tr1_min_len1                (1U<<1)
#define ST25R500_REG_PROTOCOL_RX2_tr1_min_len0                (1U<<0)
#define ST25R500_REG_PROTOCOL_RX2_tr1_min_len_mask            (0x7FU<<0)
#define ST25R500_REG_PROTOCOL_RX2_tr1_min_len_shift           (0U)

#define ST25R500_REG_PROTOCOL_RX3_tr1_max_len7                (1U<<7)
#define ST25R500_REG_PROTOCOL_RX3_tr1_max_len6                (1U<<6)
#define ST25R500_REG_PROTOCOL_RX3_tr1_max_len5                (1U<<5)
#define ST25R500_REG_PROTOCOL_RX3_tr1_max_len4                (1U<<4)
#define ST25R500_REG_PROTOCOL_RX3_tr1_max_len3                (1U<<3)
#define ST25R500_REG_PROTOCOL_RX3_tr1_max_len2                (1U<<2)
#define ST25R500_REG_PROTOCOL_RX3_tr1_max_len1                (1U<<1)
#define ST25R500_REG_PROTOCOL_RX3_tr1_max_len0                (1U<<0)
#define ST25R500_REG_PROTOCOL_RX3_tr1_max_len_mask            (0xFFU<<0)
#define ST25R500_REG_PROTOCOL_RX3_tr1_max_len_shift           (0U)

#define ST25R500_REG_EMD1_emd_thld3                           (1U<<7)
#define ST25R500_REG_EMD1_emd_thld2                           (1U<<6)
#define ST25R500_REG_EMD1_emd_thld1                           (1U<<5)
#define ST25R500_REG_EMD1_emd_thld0                           (1U<<4)
#define ST25R500_REG_EMD1_emd_thld_mask                       (0x0FU<<4)
#define ST25R500_REG_EMD1_emd_thld_shift                      (4U)
#define ST25R500_REG_EMD1_emd_thld_ff                         (1U<<3)
#define ST25R500_REG_EMD1_rfu1                                (1U<<2)
#define ST25R500_REG_EMD1_rfu0                                (1U<<1)
#define ST25R500_REG_EMD1_emd_en                              (1U<<0)
#define ST25R500_REG_EMD1_emd_en_on                           (1U<<0)
#define ST25R500_REG_EMD1_emd_en_off                          (0U<<0)

#define ST25R500_REG_EMD2_rfu7                                (1U<<7)
#define ST25R500_REG_EMD2_rfu6                                (1U<<6)
#define ST25R500_REG_EMD2_rfu5                                (1U<<5)
#define ST25R500_REG_EMD2_rfu4                                (1U<<4)
#define ST25R500_REG_EMD2_rfu3                                (1U<<3)
#define ST25R500_REG_EMD2_rfu2                                (1U<<2)
#define ST25R500_REG_EMD2_rfu1                                (1U<<1)
#define ST25R500_REG_EMD2_rfu0                                (1U<<0)

#define ST25R500_REG_CE_CONFIG1_rfu1                          (1U<<7)
#define ST25R500_REG_CE_CONFIG1_rfu0                          (1U<<6)
#define ST25R500_REG_CE_CONFIG1_ce_signal_all                 (1U<<5)
#define ST25R500_REG_CE_CONFIG1_en_other_idle                 (1U<<4)
#define ST25R500_REG_CE_CONFIG1_en_dsl_a                      (1U<<3)
#define ST25R500_REG_CE_CONFIG1_en_ce4a                       (1U<<2)
#define ST25R500_REG_CE_CONFIG1_en_212_424_1r                 (1U<<1)
#define ST25R500_REG_CE_CONFIG1_en_106_ac_a                   (1U<<0)

#define ST25R500_REG_CE_CONFIG2_fdel3                         (1U<<7)
#define ST25R500_REG_CE_CONFIG2_fdel2                         (1U<<6)
#define ST25R500_REG_CE_CONFIG2_fdel1                         (1U<<5)
#define ST25R500_REG_CE_CONFIG2_fdel0                         (1U<<4)
#define ST25R500_REG_CE_CONFIG2_fdel_mask                     (0xFU<<4)
#define ST25R500_REG_CE_CONFIG2_fdel_shift                    (4U)
#define ST25R500_REG_CE_CONFIG2_rfu2                          (1U<<3)
#define ST25R500_REG_CE_CONFIG2_rfu1                          (1U<<2)
#define ST25R500_REG_CE_CONFIG2_rfu0                          (1U<<1)
#define ST25R500_REG_CE_CONFIG2_nfc_id                        (1U<<0)
#define ST25R500_REG_CE_CONFIG2_nfc_id_4bytes                 (0U<<0)
#define ST25R500_REG_CE_CONFIG2_nfc_id_7bytes                 (1U<<0)

#define ST25R500_REG_CE_CONFIG3_rfu                           (1U<<7)
#define ST25R500_REG_CE_CONFIG3_tsn6                          (1U<<6)
#define ST25R500_REG_CE_CONFIG3_tsn5                          (1U<<5)
#define ST25R500_REG_CE_CONFIG3_tsn4                          (1U<<4)
#define ST25R500_REG_CE_CONFIG3_tsn3                          (1U<<3)
#define ST25R500_REG_CE_CONFIG3_tsn2                          (1U<<2)
#define ST25R500_REG_CE_CONFIG3_tsn1                          (1U<<1)
#define ST25R500_REG_CE_CONFIG3_tsn0                          (1U<<0)
#define ST25R500_REG_CE_CONFIG3_tsn_mask                      (0x7FU<<0)
#define ST25R500_REG_CE_CONFIG3_tsn_shift                     (0U)

#define ST25R500_REG_MRT1_sq_del1                             (1U<<7)
#define ST25R500_REG_MRT1_sq_del0                             (1U<<6)
#define ST25R500_REG_MRT1_sq_del_mask                         (3U<<6)
#define ST25R500_REG_MRT1_sq_del_shift                        (6U)
#define ST25R500_REG_MRT1_mrt_step1                           (1U<<5)
#define ST25R500_REG_MRT1_mrt_step0                           (1U<<4)
#define ST25R500_REG_MRT1_mrt_step_16fc                       (0U<<4)
#define ST25R500_REG_MRT1_mrt_step_32fc                       (1U<<4)
#define ST25R500_REG_MRT1_mrt_step_64fc                       (2U<<4)
#define ST25R500_REG_MRT1_mrt_step_512fc                      (3U<<4)
#define ST25R500_REG_MRT1_mrt_step_mask                       (3U<<4)
#define ST25R500_REG_MRT1_mrt_step_shift                      (4U)
#define ST25R500_REG_MRT1_rfu2                                (1U<<3)
#define ST25R500_REG_MRT1_rfu1                                (1U<<2)
#define ST25R500_REG_MRT1_rfu0                                (1U<<1)
#define ST25R500_REG_MRT1_sq_en                               (1U<<0)

#define ST25R500_REG_MRT2_mrt7                                (1U<<7)
#define ST25R500_REG_MRT2_mrt6                                (1U<<6)
#define ST25R500_REG_MRT2_mrt5                                (1U<<5)
#define ST25R500_REG_MRT2_mrt4                                (1U<<4)
#define ST25R500_REG_MRT2_mrt3                                (1U<<3)
#define ST25R500_REG_MRT2_mrt2                                (1U<<2)
#define ST25R500_REG_MRT2_mrt1                                (1U<<1)
#define ST25R500_REG_MRT2_mrt0                                (1U<<0)

#define ST25R500_REG_SQT_sqt7                                 (1U<<7)
#define ST25R500_REG_SQT_sqt6                                 (1U<<6)
#define ST25R500_REG_SQT_sqt5                                 (1U<<5)
#define ST25R500_REG_SQT_sqt4                                 (1U<<4)
#define ST25R500_REG_SQT_sqt3                                 (1U<<3)
#define ST25R500_REG_SQT_sqt2                                 (1U<<2)
#define ST25R500_REG_SQT_sqt1                                 (1U<<1)
#define ST25R500_REG_SQT_sqt0                                 (1U<<0)

#define ST25R500_REG_NRT_GPT_CONF_rfu2                        (1U<<7)
#define ST25R500_REG_NRT_GPT_CONF_gptc2                       (1U<<6)
#define ST25R500_REG_NRT_GPT_CONF_gptc1                       (1U<<5)
#define ST25R500_REG_NRT_GPT_CONF_gptc0                       (1U<<4)
#define ST25R500_REG_NRT_GPT_CONF_gptc_no_trigger             (0U<<4)
#define ST25R500_REG_NRT_GPT_CONF_gptc_erx                    (1U<<4)
#define ST25R500_REG_NRT_GPT_CONF_gptc_srx                    (2U<<4)
#define ST25R500_REG_NRT_GPT_CONF_gptc_etx                    (3U<<5)
#define ST25R500_REG_NRT_GPT_CONF_gptc_mask                   (7U<<4)
#define ST25R500_REG_NRT_GPT_CONF_gptc_shift                  (4U)
#define ST25R500_REG_NRT_GPT_CONF_rfu1                        (1U<<3)
#define ST25R500_REG_NRT_GPT_CONF_rfu0                        (1U<<2)
#define ST25R500_REG_NRT_GPT_CONF_nrt_emv                     (1U<<1)
#define ST25R500_REG_NRT_GPT_CONF_nrt_emv_on                  (1U<<1)
#define ST25R500_REG_NRT_GPT_CONF_nrt_emv_off                 (0U<<1)
#define ST25R500_REG_NRT_GPT_CONF_nrt_step                    (1U<<0)
#define ST25R500_REG_NRT_GPT_CONF_nrt_step_64fc               (0U<<0)
#define ST25R500_REG_NRT_GPT_CONF_nrt_step_4096fc             (1U<<0)

#define ST25R500_REG_NRT1_nrt15                               (1U<<7)
#define ST25R500_REG_NRT1_nrt14                               (1U<<6)
#define ST25R500_REG_NRT1_nrt13                               (1U<<5)
#define ST25R500_REG_NRT1_nrt12                               (1U<<4)
#define ST25R500_REG_NRT1_nrt11                               (1U<<3)
#define ST25R500_REG_NRT1_nrt10                               (1U<<2)
#define ST25R500_REG_NRT1_nrt9                                (1U<<1)
#define ST25R500_REG_NRT1_nrt8                                (1U<<0)

#define ST25R500_REG_NRT2_nrt7                                (1U<<7)
#define ST25R500_REG_NRT2_nrt6                                (1U<<6)
#define ST25R500_REG_NRT2_nrt5                                (1U<<5)
#define ST25R500_REG_NRT2_nrt4                                (1U<<4)
#define ST25R500_REG_NRT2_nrt3                                (1U<<3)
#define ST25R500_REG_NRT2_nrt2                                (1U<<2)
#define ST25R500_REG_NRT2_nrt1                                (1U<<1)
#define ST25R500_REG_NRT2_nrt0                                (1U<<0)

#define ST25R500_REG_GPT1_gpt15                               (1U<<7)
#define ST25R500_REG_GPT1_gpt14                               (1U<<6)
#define ST25R500_REG_GPT1_gpt13                               (1U<<5)
#define ST25R500_REG_GPT1_gpt12                               (1U<<4)
#define ST25R500_REG_GPT1_gpt11                               (1U<<3)
#define ST25R500_REG_GPT1_gpt10                               (1U<<2)
#define ST25R500_REG_GPT1_gpt9                                (1U<<1)
#define ST25R500_REG_GPT1_gpt8                                (1U<<0)

#define ST25R500_REG_GPT2_gpt7                                (1U<<7)
#define ST25R500_REG_GPT2_gpt6                                (1U<<6)
#define ST25R500_REG_GPT2_gpt5                                (1U<<5)
#define ST25R500_REG_GPT2_gpt4                                (1U<<4)
#define ST25R500_REG_GPT2_gpt3                                (1U<<3)
#define ST25R500_REG_GPT2_gpt2                                (1U<<2)
#define ST25R500_REG_GPT2_gpt1                                (1U<<1)
#define ST25R500_REG_GPT2_gpt0                                (1U<<0)

#define ST25R500_REG_WAKEUP_CONF1_wut3                        (1U<<7)
#define ST25R500_REG_WAKEUP_CONF1_wut2                        (1U<<6)
#define ST25R500_REG_WAKEUP_CONF1_wut1                        (1U<<5)
#define ST25R500_REG_WAKEUP_CONF1_wut0                        (1U<<4)
#define ST25R500_REG_WAKEUP_CONF1_wut_mask                    (0x0FU<<4)
#define ST25R500_REG_WAKEUP_CONF1_wut_shift                   (4U)
#define ST25R500_REG_WAKEUP_CONF1_rfu2                        (1U<<3)
#define ST25R500_REG_WAKEUP_CONF1_rfu1                        (1U<<2)
#define ST25R500_REG_WAKEUP_CONF1_rfu0                        (1U<<1)
#define ST25R500_REG_WAKEUP_CONF1_fast_efd_irq                (1U<<0)

#define ST25R500_REG_WAKEUP_CONF2_rfu1                        (1U<<7)
#define ST25R500_REG_WAKEUP_CONF2_wut_cal                     (1U<<6)
#define ST25R500_REG_WAKEUP_CONF2_wut_cal_len1                (1U<<5)
#define ST25R500_REG_WAKEUP_CONF2_wut_cal_len0                (1U<<4)
#define ST25R500_REG_WAKEUP_CONF2_wut_cal_len_mask            (3U<<4)
#define ST25R500_REG_WAKEUP_CONF2_wut_cal_len_shift           (4)
#define ST25R500_REG_WAKEUP_CONF2_weak_disch                  (1U<<3)
#define ST25R500_REG_WAKEUP_CONF2_rfu0                        (1U<<2)
#define ST25R500_REG_WAKEUP_CONF2_tagdet_len1                 (1U<<1)
#define ST25R500_REG_WAKEUP_CONF2_tagdet_len0                 (1U<<0)
#define ST25R500_REG_WAKEUP_CONF2_tagdet_len_mask             (3U<<0)
#define ST25R500_REG_WAKEUP_CONF2_tagdet_len_shift            (0U)

#define ST25R500_REG_WAKEUP_CONF3_skip_recal                  (1U<<7)
#define ST25R500_REG_WAKEUP_CONF3_skip_cal                    (1U<<6)
#define ST25R500_REG_WAKEUP_CONF3_skip_twcal                  (1U<<5)
#define ST25R500_REG_WAKEUP_CONF3_skip_twref                  (1U<<4)
#define ST25R500_REG_WAKEUP_CONF3_iq_aaref                    (1U<<3)
#define ST25R500_REG_WAKEUP_CONF3_td_mf                       (1U<<2)
#define ST25R500_REG_WAKEUP_CONF3_td_mt1                      (1U<<1)
#define ST25R500_REG_WAKEUP_CONF3_td_mt0                      (1U<<0)
#define ST25R500_REG_WAKEUP_CONF3_td_mt_mask                  (3U<<0)
#define ST25R500_REG_WAKEUP_CONF3_td_mt_shift                 (0U)

#define ST25R500_REG_WU_I_CONF_rfu                            (1U<<7)
#define ST25R500_REG_WU_I_CONF_i_iirqm                        (1U<<6)
#define ST25R500_REG_WU_I_CONF_i_aaw2                         (1U<<5)
#define ST25R500_REG_WU_I_CONF_i_aaw1                         (1U<<4)
#define ST25R500_REG_WU_I_CONF_i_aaw0                         (1U<<3)
#define ST25R500_REG_WU_I_CONF_i_aaw_mask                     (7U<<3)
#define ST25R500_REG_WU_I_CONF_i_aaw_shift                    (3U)
#define ST25R500_REG_WU_I_CONF_i_tdi_en2                      (1U<<2)
#define ST25R500_REG_WU_I_CONF_i_tdi_en1                      (1U<<1)
#define ST25R500_REG_WU_I_CONF_i_tdi_en0                      (1U<<0)
#define ST25R500_REG_WU_I_CONF_i_tdi_en_shift                 (0U)
#define ST25R500_REG_WU_I_CONF_i_tdi_en_mask                  (0x07U<<0)

#define ST25R500_REG_WU_I_DELTA_i_cal8                        (1U<<7)
#define ST25R500_REG_WU_I_DELTA_i_cal8_mask                   (0x01U<<7)
#define ST25R500_REG_WU_I_DELTA_i_cal8_shift                  (7U)
#define ST25R500_REG_WU_I_DELTA_i_rfu                         (1U<<6)
#define ST25R500_REG_WU_I_DELTA_i_diff5                       (1U<<5)
#define ST25R500_REG_WU_I_DELTA_i_diff4                       (1U<<4)
#define ST25R500_REG_WU_I_DELTA_i_diff3                       (1U<<3)
#define ST25R500_REG_WU_I_DELTA_i_diff2                       (1U<<2)
#define ST25R500_REG_WU_I_DELTA_i_diff1                       (1U<<1)
#define ST25R500_REG_WU_I_DELTA_i_diff0                       (1U<<0)
#define ST25R500_REG_WU_I_DELTA_i_diff_mask                   (0x3FU<<0)
#define ST25R500_REG_WU_I_DELTA_i_diff_shift                  (0U)

#define ST25R500_REG_WU_I_CAL_i_cal7                          (1U<<7)
#define ST25R500_REG_WU_I_CAL_i_cal6                          (1U<<6)
#define ST25R500_REG_WU_I_CAL_i_cal5                          (1U<<5)
#define ST25R500_REG_WU_I_CAL_i_cal4                          (1U<<4)
#define ST25R500_REG_WU_I_CAL_i_cal3                          (1U<<3)
#define ST25R500_REG_WU_I_CAL_i_cal2                          (1U<<2)
#define ST25R500_REG_WU_I_CAL_i_cal1                          (1U<<1)
#define ST25R500_REG_WU_I_CAL_i_cal0                          (1U<<0)

#define ST25R500_REG_WU_I_ADC_i_adc7                          (1U<<7)
#define ST25R500_REG_WU_I_ADC_i_adc6                          (1U<<6)
#define ST25R500_REG_WU_I_ADC_i_adc5                          (1U<<5)
#define ST25R500_REG_WU_I_ADC_i_adc4                          (1U<<4)
#define ST25R500_REG_WU_I_ADC_i_adc3                          (1U<<3)
#define ST25R500_REG_WU_I_ADC_i_adc2                          (1U<<2)
#define ST25R500_REG_WU_I_ADC_i_adc1                          (1U<<1)
#define ST25R500_REG_WU_I_ADC_i_adc0                          (1U<<0)

#define ST25R500_REG_WU_I_REF_i_ref7                          (1U<<7)
#define ST25R500_REG_WU_I_REF_i_ref6                          (1U<<6)
#define ST25R500_REG_WU_I_REF_i_ref5                          (1U<<5)
#define ST25R500_REG_WU_I_REF_i_ref4                          (1U<<4)
#define ST25R500_REG_WU_I_REF_i_ref3                          (1U<<3)
#define ST25R500_REG_WU_I_REF_i_ref2                          (1U<<2)
#define ST25R500_REG_WU_I_REF_i_ref1                          (1U<<1)
#define ST25R500_REG_WU_I_REF_i_ref0                          (1U<<0)

#define ST25R500_REG_WU_Q_CONF_rfu                            (1U<<7)
#define ST25R500_REG_WU_Q_CONF_q_iirqm                        (1U<<6)
#define ST25R500_REG_WU_Q_CONF_q_aaw2                         (1U<<5)
#define ST25R500_REG_WU_Q_CONF_q_aaw1                         (1U<<4)
#define ST25R500_REG_WU_Q_CONF_q_aaw0                         (1U<<3)
#define ST25R500_REG_WU_Q_CONF_q_aaw_mask                     (7U<<3)
#define ST25R500_REG_WU_Q_CONF_q_aaw_shift                    (3U)
#define ST25R500_REG_WU_Q_CONF_q_tdi_en2                      (1U<<2)
#define ST25R500_REG_WU_Q_CONF_q_tdi_en1                      (1U<<1)
#define ST25R500_REG_WU_Q_CONF_q_tdi_en0                      (1U<<0)
#define ST25R500_REG_WU_Q_CONF_q_tdi_en_shift                 (0U)
#define ST25R500_REG_WU_Q_CONF_q_tdi_en_mask                  (0x07U<<0)

#define ST25R500_REG_WU_Q_DELTA_q_cal8                        (1U<<7)
#define ST25R500_REG_WU_Q_DELTA_q_cal8_mask                   (0x01U<<7)
#define ST25R500_REG_WU_Q_DELTA_q_cal8_shift                  (7U)
#define ST25R500_REG_WU_Q_DELTA_q_rfu                         (1U<<6)
#define ST25R500_REG_WU_Q_DELTA_q_diff5                       (1U<<5)
#define ST25R500_REG_WU_Q_DELTA_q_diff4                       (1U<<4)
#define ST25R500_REG_WU_Q_DELTA_q_diff3                       (1U<<3)
#define ST25R500_REG_WU_Q_DELTA_q_diff2                       (1U<<2)
#define ST25R500_REG_WU_Q_DELTA_q_diff1                       (1U<<1)
#define ST25R500_REG_WU_Q_DELTA_q_diff0                       (1U<<0)
#define ST25R500_REG_WU_Q_DELTA_q_diff_mask                   (0x3FU<<0)
#define ST25R500_REG_WU_Q_DELTA_q_diff_shift                  (0U)

#define ST25R500_REG_WU_Q_CAL_q_cal7                          (1U<<7)
#define ST25R500_REG_WU_Q_CAL_q_cal6                          (1U<<6)
#define ST25R500_REG_WU_Q_CAL_q_cal5                          (1U<<5)
#define ST25R500_REG_WU_Q_CAL_q_cal4                          (1U<<4)
#define ST25R500_REG_WU_Q_CAL_q_cal3                          (1U<<3)
#define ST25R500_REG_WU_Q_CAL_q_cal2                          (1U<<2)
#define ST25R500_REG_WU_Q_CAL_q_cal1                          (1U<<1)
#define ST25R500_REG_WU_Q_CAL_q_cal0                          (1U<<0)

#define ST25R500_REG_WU_Q_ADC_q_adc7                          (1U<<7)
#define ST25R500_REG_WU_Q_ADC_q_adc6                          (1U<<6)
#define ST25R500_REG_WU_Q_ADC_q_adc5                          (1U<<5)
#define ST25R500_REG_WU_Q_ADC_q_adc4                          (1U<<4)
#define ST25R500_REG_WU_Q_ADC_q_adc3                          (1U<<3)
#define ST25R500_REG_WU_Q_ADC_q_adc2                          (1U<<2)
#define ST25R500_REG_WU_Q_ADC_q_adc1                          (1U<<1)
#define ST25R500_REG_WU_Q_ADC_q_adc0                          (1U<<0)

#define ST25R500_REG_WU_Q_REF_q_ref7                          (1U<<7)
#define ST25R500_REG_WU_Q_REF_q_ref6                          (1U<<6)
#define ST25R500_REG_WU_Q_REF_q_ref5                          (1U<<5)
#define ST25R500_REG_WU_Q_REF_q_ref4                          (1U<<4)
#define ST25R500_REG_WU_Q_REF_q_ref3                          (1U<<3)
#define ST25R500_REG_WU_Q_REF_q_ref2                          (1U<<2)
#define ST25R500_REG_WU_Q_REF_q_ref1                          (1U<<1)
#define ST25R500_REG_WU_Q_REF_q_ref0                          (1U<<0)

#define ST25R500_REG_TX_FRAME1_ntx12                          (1U<<7)
#define ST25R500_REG_TX_FRAME1_ntx11                          (1U<<6)
#define ST25R500_REG_TX_FRAME1_ntx10                          (1U<<5)
#define ST25R500_REG_TX_FRAME1_ntx9                           (1U<<4)
#define ST25R500_REG_TX_FRAME1_ntx8                           (1U<<3)
#define ST25R500_REG_TX_FRAME1_ntx7                           (1U<<2)
#define ST25R500_REG_TX_FRAME1_ntx6                           (1U<<1)
#define ST25R500_REG_TX_FRAME1_ntx5                           (1U<<0)

#define ST25R500_REG_TX_FRAME2_ntx4                           (1U<<7)
#define ST25R500_REG_TX_FRAME2_ntx3                           (1U<<6)
#define ST25R500_REG_TX_FRAME2_ntx2                           (1U<<5)
#define ST25R500_REG_TX_FRAME2_ntx1                           (1U<<4)
#define ST25R500_REG_TX_FRAME2_ntx0                           (1U<<3)
#define ST25R500_REG_TX_FRAME2_ntx_mask                       (0x1FU<<3)
#define ST25R500_REG_TX_FRAME2_ntx_shift                      (3U)
#define ST25R500_REG_TX_FRAME2_nbtx2                          (1U<<2)
#define ST25R500_REG_TX_FRAME2_nbtx1                          (1U<<1)
#define ST25R500_REG_TX_FRAME2_nbtx0                          (1U<<0)
#define ST25R500_REG_TX_FRAME2_nbtx_mask                      (7U<<0)
#define ST25R500_REG_TX_FRAME2_nbtx_shift                     (0U)

#define ST25R500_REG_FIFO_STATUS1_fifo_b7                     (1U<<7)
#define ST25R500_REG_FIFO_STATUS1_fifo_b6                     (1U<<6)
#define ST25R500_REG_FIFO_STATUS1_fifo_b5                     (1U<<5)
#define ST25R500_REG_FIFO_STATUS1_fifo_b4                     (1U<<4)
#define ST25R500_REG_FIFO_STATUS1_fifo_b3                     (1U<<3)
#define ST25R500_REG_FIFO_STATUS1_fifo_b2                     (1U<<2)
#define ST25R500_REG_FIFO_STATUS1_fifo_b1                     (1U<<1)
#define ST25R500_REG_FIFO_STATUS1_fifo_b0                     (1U<<0)

#define ST25R500_REG_FIFO_STATUS2_rfu                         (1U<<7)
#define ST25R500_REG_FIFO_STATUS2_fifo_b8                     (1U<<6)
#define ST25R500_REG_FIFO_STATUS2_fifo_b_shift                (6U)
#define ST25R500_REG_FIFO_STATUS2_fifo_unf                    (1U<<5)
#define ST25R500_REG_FIFO_STATUS2_fifo_ovr                    (1U<<4)
#define ST25R500_REG_FIFO_STATUS2_fifo_lb2                    (1U<<3)
#define ST25R500_REG_FIFO_STATUS2_fifo_lb1                    (1U<<2)
#define ST25R500_REG_FIFO_STATUS2_fifo_lb0                    (1U<<1)
#define ST25R500_REG_FIFO_STATUS2_fifo_lb_mask                (7U<<1)
#define ST25R500_REG_FIFO_STATUS2_fifo_lb_shift               (1U)
#define ST25R500_REG_FIFO_STATUS2_np_lb                       (1U<<0)

#define ST25R500_REG_COLLISION_c_byte3                        (1U<<7)
#define ST25R500_REG_COLLISION_c_byte2                        (1U<<6)
#define ST25R500_REG_COLLISION_c_byte1                        (1U<<5)
#define ST25R500_REG_COLLISION_c_byte0                        (1U<<4)
#define ST25R500_REG_COLLISION_c_byte_mask                    (0xFU<<4)
#define ST25R500_REG_COLLISION_c_byte_shift                   (4U)
#define ST25R500_REG_COLLISION_c_bit2                         (1U<<3)
#define ST25R500_REG_COLLISION_c_bit1                         (1U<<2)
#define ST25R500_REG_COLLISION_c_bit0                         (1U<<1)
#define ST25R500_REG_COLLISION_c_bit_mask                     (0x7U<<1)
#define ST25R500_REG_COLLISION_c_bit_shift                    (1U)
#define ST25R500_REG_COLLISION_c_pb                           (1U<<0)

#define ST25R500_REG_IRQ_MASK1_M_subc_start                   (1U<<7)
#define ST25R500_REG_IRQ_MASK1_M_col                          (1U<<6)
#define ST25R500_REG_IRQ_MASK1_M_wl                           (1U<<5)
#define ST25R500_REG_IRQ_MASK1_M_rx_rest                      (1U<<4)
#define ST25R500_REG_IRQ_MASK1_M_rxe                          (1U<<3)
#define ST25R500_REG_IRQ_MASK1_M_rxs                          (1U<<2)
#define ST25R500_REG_IRQ_MASK1_M_txe                          (1U<<1)
#define ST25R500_REG_IRQ_MASK1_M_rx_err                       (1U<<0)

#define ST25R500_REG_IRQ_MASK2_M_gpe                          (1U<<7)
#define ST25R500_REG_IRQ_MASK2_M_nre                          (1U<<6)
#define ST25R500_REG_IRQ_MASK2_M_wpt_stop                     (1U<<5)
#define ST25R500_REG_IRQ_MASK2_M_wpt_fod                      (1U<<4)
#define ST25R500_REG_IRQ_MASK2_rfu                            (1U<<3)
#define ST25R500_REG_IRQ_MASK2_M_ce_sc                        (1U<<2)
#define ST25R500_REG_IRQ_MASK2_M_rxe_cea                      (1U<<1)
#define ST25R500_REG_IRQ_MASK2_M_nfct                         (1U<<0)

#define ST25R500_REG_IRQ_MASK3_M_wutme                        (1U<<7)
#define ST25R500_REG_IRQ_MASK3_M_eof                          (1U<<6)
#define ST25R500_REG_IRQ_MASK3_M_eon                          (1U<<5)
#define ST25R500_REG_IRQ_MASK3_M_dct                          (1U<<4)
#define ST25R500_REG_IRQ_MASK3_M_wuq                          (1U<<3)
#define ST25R500_REG_IRQ_MASK3_M_wui                          (1U<<2)
#define ST25R500_REG_IRQ_MASK3_M_wut                          (1U<<1)
#define ST25R500_REG_IRQ_MASK3_M_osc                          (1U<<0)

#define ST25R500_REG_IRQ1_I_subc_start                        (1U<<7)
#define ST25R500_REG_IRQ1_I_col                               (1U<<6)
#define ST25R500_REG_IRQ1_I_wl                                (1U<<5)
#define ST25R500_REG_IRQ1_I_rx_rest                           (1U<<4)
#define ST25R500_REG_IRQ1_I_rxe                               (1U<<3)
#define ST25R500_REG_IRQ1_I_rxs                               (1U<<2)
#define ST25R500_REG_IRQ1_I_txe                               (1U<<1)
#define ST25R500_REG_IRQ1_I_rx_err                            (1U<<0)

#define ST25R500_REG_IRQ2_I_gpe                               (1U<<7)
#define ST25R500_REG_IRQ2_I_nre                               (1U<<6)
#define ST25R500_REG_IRQ2_I_wpt_stop                          (1U<<5)
#define ST25R500_REG_IRQ2_I_wpt_fod                           (1U<<4)
#define ST25R500_REG_IRQ2_rfu                                 (1U<<3)
#define ST25R500_REG_IRQ2_I_ce_sc                             (1U<<2)
#define ST25R500_REG_IRQ2_I_rxe_cea                           (1U<<1)
#define ST25R500_REG_IRQ2_I_nfct                              (1U<<0)

#define ST25R500_REG_IRQ3_I_wutme                             (1U<<7)
#define ST25R500_REG_IRQ3_I_eof                               (1U<<6)
#define ST25R500_REG_IRQ3_I_eon                               (1U<<5)
#define ST25R500_REG_IRQ3_I_dct                               (1U<<4)
#define ST25R500_REG_IRQ3_I_wuq                               (1U<<3)
#define ST25R500_REG_IRQ3_I_wui                               (1U<<2)
#define ST25R500_REG_IRQ3_I_wut                               (1U<<1)
#define ST25R500_REG_IRQ3_I_osc                               (1U<<0)

#define ST25R500_REG_IC_ID_ic_type4                           (1U<<7)
#define ST25R500_REG_IC_ID_ic_type3                           (1U<<6)
#define ST25R500_REG_IC_ID_ic_type2                           (1U<<5)
#define ST25R500_REG_IC_ID_ic_type1                           (1U<<4)
#define ST25R500_REG_IC_ID_ic_type0                           (1U<<3)
#define ST25R500_REG_IC_ID_ic_type_st25r500                   (22U<<3)
#define ST25R500_REG_IC_ID_ic_type_mask                       (0x1FU<<3)
#define ST25R500_REG_IC_ID_ic_type_shift                      (3U)
#define ST25R500_REG_IC_ID_ic_rev2                            (1U<<2)
#define ST25R500_REG_IC_ID_ic_rev1                            (1U<<1)
#define ST25R500_REG_IC_ID_ic_rev0                            (1U<<0)
#define ST25R500_REG_IC_ID_ic_rev_mask                        (7U<<0)
#define ST25R500_REG_IC_ID_ic_rev_shift                       (0U)

#define ST25R500_REG_STATUS1_rfu1                             (1U<<7)
#define ST25R500_REG_STATUS1_wut_on                           (1U<<6)
#define ST25R500_REG_STATUS1_agd_ok                           (1U<<5)
#define ST25R500_REG_STATUS1_osc_ok                           (1U<<4)
#define ST25R500_REG_STATUS1_rfu0                             (1U<<3)
#define ST25R500_REG_STATUS1_tmp_on                           (1U<<2)
#define ST25R500_REG_STATUS1_efd_out                          (1U<<1)
#define ST25R500_REG_STATUS1_efd_on                           (1U<<0)

#define ST25R500_REG_STATUS2_subc_on                          (1U<<7)
#define ST25R500_REG_STATUS2_gpt_on                           (1U<<6)
#define ST25R500_REG_STATUS2_nrt_on                           (1U<<5)
#define ST25R500_REG_STATUS2_mrt_on                           (1U<<4)
#define ST25R500_REG_STATUS2_rx_act                           (1U<<3)
#define ST25R500_REG_STATUS2_rx_on                            (1U<<2)
#define ST25R500_REG_STATUS2_tx_on                            (1U<<1)
#define ST25R500_REG_STATUS2_rfu                              (1U<<0)

#define ST25R500_REG_STATUS_STATIC1_rfu4                      (1U<<7)
#define ST25R500_REG_STATUS_STATIC1_s_eof                     (1U<<6)
#define ST25R500_REG_STATUS_STATIC1_s_eon                     (1U<<5)
#define ST25R500_REG_STATUS_STATIC1_s_dct                     (1U<<4)
#define ST25R500_REG_STATUS_STATIC1_rfu3                      (1U<<3)
#define ST25R500_REG_STATUS_STATIC1_rfu2                      (1U<<2)
#define ST25R500_REG_STATUS_STATIC1_rfu1                      (1U<<1)
#define ST25R500_REG_STATUS_STATIC1_rfu0                      (1U<<0)

#define ST25R500_REG_STATUS_STATIC2_rfu2                      (1U<<7)
#define ST25R500_REG_STATUS_STATIC2_rfu1                      (1U<<6)
#define ST25R500_REG_STATUS_STATIC2_s_rx_err                  (1U<<5)
#define ST25R500_REG_STATUS_STATIC2_s_rxe                     (1U<<4)
#define ST25R500_REG_STATUS_STATIC2_s_rx_rest                 (1U<<3)
#define ST25R500_REG_STATUS_STATIC2_s_col                     (1U<<2)
#define ST25R500_REG_STATUS_STATIC2_s_rxs                     (1U<<1)
#define ST25R500_REG_STATUS_STATIC2_rfu0                      (1U<<0)

#define ST25R500_REG_STATUS_STATIC3_s_crc                     (1U<<7)
#define ST25R500_REG_STATUS_STATIC3_s_par                     (1U<<6)
#define ST25R500_REG_STATUS_STATIC3_s_hfe                     (1U<<5)
#define ST25R500_REG_STATUS_STATIC3_s_sfe                     (1U<<4)
#define ST25R500_REG_STATUS_STATIC3_s_mask                    (0xFU<<4)
#define ST25R500_REG_STATUS_STATIC3_rfu3                      (1U<<3)
#define ST25R500_REG_STATUS_STATIC3_rfu2                      (1U<<2)
#define ST25R500_REG_STATUS_STATIC3_rfu1                      (1U<<1)
#define ST25R500_REG_STATUS_STATIC3_rfu0                      (1U<<0)

#define ST25R500_REG_CE_STATUS1_nfc_rate_lock1                 (1U<<7)
#define ST25R500_REG_CE_STATUS1_nfc_rate_lock0                 (1U<<6)
#define ST25R500_REG_CE_STATUS1_nfc_rate_lock_mask             (3U<<6)
#define ST25R500_REG_CE_STATUS1_nfc_rate_lock_shift            (6U)
#define ST25R500_REG_CE_STATUS1_nfc_rate1                      (1U<<5)
#define ST25R500_REG_CE_STATUS1_nfc_rate0                      (1U<<4)
#define ST25R500_REG_CE_STATUS1_nfc_rate_mask                  (3U<<4)
#define ST25R500_REG_CE_STATUS1_nfc_rate_shift                 (4U)
#define ST25R500_REG_CE_STATUS1_ce_state3                      (1U<<3)
#define ST25R500_REG_CE_STATUS1_ce_state2                      (1U<<2)
#define ST25R500_REG_CE_STATUS1_ce_state1                      (1U<<1)
#define ST25R500_REG_CE_STATUS1_ce_state0                      (1U<<0)
#define ST25R500_REG_CE_STATUS1_ce_state_mask                  (0xFU<<0)
#define ST25R500_REG_CE_STATUS1_ce_state_shift                 (0U)
#define ST25R500_REG_CE_STATUS1_ce_state_power_off             (0x0U<<0)
#define ST25R500_REG_CE_STATUS1_ce_state_idle                  (0x1U<<0)
#define ST25R500_REG_CE_STATUS1_ce_state_ready_a               (0x2U<<0)
#define ST25R500_REG_CE_STATUS1_ce_state_ready_a_              (0x3U<<0)
#define ST25R500_REG_CE_STATUS1_ce_state_active_a              (0x5U<<0)
#define ST25R500_REG_CE_STATUS1_ce_state_ce_a                  (0x6U<<0)
#define ST25R500_REG_CE_STATUS1_ce_state_sleep_a               (0x9U<<0)
#define ST25R500_REG_CE_STATUS1_ce_state_ready_ax              (0xAU<<0)
#define ST25R500_REG_CE_STATUS1_ce_state_ready_a_x             (0xBU<<0)
#define ST25R500_REG_CE_STATUS1_ce_state_active_ax             (0xDU<<0)
#define ST25R500_REG_CE_STATUS1_ce_state_ce_f                  (0xFU<<0)

#define ST25R500_REG_WU_STATUS_rfu1                           (1U<<7)
#define ST25R500_REG_WU_STATUS_q_tdi2                         (1U<<6)
#define ST25R500_REG_WU_STATUS_q_tdi1                         (1U<<5)
#define ST25R500_REG_WU_STATUS_q_tdi0                         (1U<<4)
#define ST25R500_REG_WU_STATUS_q_tdi_mask                     (7U<<4)
#define ST25R500_REG_WU_STATUS_q_tdi_shift                    (4U)
#define ST25R500_REG_WU_STATUS_rfu0                           (1U<<3)
#define ST25R500_REG_WU_STATUS_i_tdi2                         (1U<<2)
#define ST25R500_REG_WU_STATUS_i_tdi1                         (1U<<1)
#define ST25R500_REG_WU_STATUS_i_tdi0                         (1U<<0)
#define ST25R500_REG_WU_STATUS_i_tdi_mask                     (7U<<0)
#define ST25R500_REG_WU_STATUS_i_tdi_shift                    (0U)

#define ST25R500_REG_ANA_DISPLAY1_i_lim                       (1U<<7)
#define ST25R500_REG_ANA_DISPLAY1_regc6                       (1U<<6)
#define ST25R500_REG_ANA_DISPLAY1_regc5                       (1U<<5)
#define ST25R500_REG_ANA_DISPLAY1_regc4                       (1U<<4)
#define ST25R500_REG_ANA_DISPLAY1_regc3                       (1U<<3)
#define ST25R500_REG_ANA_DISPLAY1_regc2                       (1U<<2)
#define ST25R500_REG_ANA_DISPLAY1_regc1                       (1U<<1)
#define ST25R500_REG_ANA_DISPLAY1_regc0                       (1U<<0)
#define ST25R500_REG_ANA_DISPLAY1_regc_mask                   (0x7FU<<0)
#define ST25R500_REG_ANA_DISPLAY1_regc_shift                  (0U)

#define ST25R500_REG_ANA_DISPLAY2_rfu3                        (1U<<7)
#define ST25R500_REG_ANA_DISPLAY2_rfu2                        (1U<<6)
#define ST25R500_REG_ANA_DISPLAY2_rfu1                        (1U<<5)
#define ST25R500_REG_ANA_DISPLAY2_rfu0                        (1U<<4)
#define ST25R500_REG_ANA_DISPLAY2_afe_gain3                   (1U<<3)
#define ST25R500_REG_ANA_DISPLAY2_afe_gain2                   (1U<<2)
#define ST25R500_REG_ANA_DISPLAY2_afe_gain1                   (1U<<1)
#define ST25R500_REG_ANA_DISPLAY2_afe_gain0                   (1U<<0)
#define ST25R500_REG_ANA_DISPLAY2_afe_gain_mask               (0x0FU<<0)
#define ST25R500_REG_ANA_DISPLAY2_afe_gain_shift              (0U)

#define ST25R500_REG_RSSI_I_rssi_i7                           (1U<<7)
#define ST25R500_REG_RSSI_I_rssi_i6                           (1U<<6)
#define ST25R500_REG_RSSI_I_rssi_i5                           (1U<<5)
#define ST25R500_REG_RSSI_I_rssi_i4                           (1U<<4)
#define ST25R500_REG_RSSI_I_rssi_i3                           (1U<<3)
#define ST25R500_REG_RSSI_I_rssi_i2                           (1U<<2)
#define ST25R500_REG_RSSI_I_rssi_i1                           (1U<<1)
#define ST25R500_REG_RSSI_I_rssi_i0                           (1U<<0)
#define ST25R500_REG_RSSI_I_rssi_i_mask                       (0xFFU<<0)
#define ST25R500_REG_RSSI_I_rssi_i_shift                      (0U)

#define ST25R500_REG_RSSI_Q_rssi_q7                           (1U<<7)
#define ST25R500_REG_RSSI_Q_rssi_q6                           (1U<<6)
#define ST25R500_REG_RSSI_Q_rssi_q5                           (1U<<5)
#define ST25R500_REG_RSSI_Q_rssi_q4                           (1U<<4)
#define ST25R500_REG_RSSI_Q_rssi_q3                           (1U<<3)
#define ST25R500_REG_RSSI_Q_rssi_q2                           (1U<<2)
#define ST25R500_REG_RSSI_Q_rssi_q1                           (1U<<1)
#define ST25R500_REG_RSSI_Q_rssi_q0                           (1U<<0)
#define ST25R500_REG_RSSI_Q_rssi_q_mask                       (0xFFU<<0)
#define ST25R500_REG_RSSI_Q_rssi_q_shift                      (0U)

#define ST25R500_REG_SENSE_DISPLAY_sense_adc7                 (1U<<7)
#define ST25R500_REG_SENSE_DISPLAY_sense_adc6                 (1U<<6)
#define ST25R500_REG_SENSE_DISPLAY_sense_adc5                 (1U<<5)
#define ST25R500_REG_SENSE_DISPLAY_sense_adc4                 (1U<<4)
#define ST25R500_REG_SENSE_DISPLAY_sense_adc3                 (1U<<3)
#define ST25R500_REG_SENSE_DISPLAY_sense_adc2                 (1U<<2)
#define ST25R500_REG_SENSE_DISPLAY_sense_adc1                 (1U<<1)
#define ST25R500_REG_SENSE_DISPLAY_sense_adc0                 (1U<<0)

#define ST25R500_REG_AWS_CONFIG1_dig_aws_en                   (1U<<7)
#define ST25R500_REG_AWS_CONFIG1_dyn_ilim_aws                 (1U<<6)
#define ST25R500_REG_AWS_CONFIG1_dyn_sink_offset              (1U<<5)
#define ST25R500_REG_AWS_CONFIG1_dyn_pass_sink                (1U<<4)
#define ST25R500_REG_AWS_CONFIG1_sc_prot_en                   (1U<<3)
#define ST25R500_REG_AWS_CONFIG1_sink_offset_en               (1U<<2)
#define ST25R500_REG_AWS_CONFIG1_act_sink_en                  (1U<<1)
#define ST25R500_REG_AWS_CONFIG1_rfu                          (1U<<0)

#define ST25R500_REG_AWS_CONFIG2_am_fall3                     (1U<<7)
#define ST25R500_REG_AWS_CONFIG2_am_fall2                     (1U<<6)
#define ST25R500_REG_AWS_CONFIG2_am_fall1                     (1U<<5)
#define ST25R500_REG_AWS_CONFIG2_am_fall0                     (1U<<4)
#define ST25R500_REG_AWS_CONFIG2_am_fall_mask                 (0xFU<<4)
#define ST25R500_REG_AWS_CONFIG2_am_fall_shift                (4U)
#define ST25R500_REG_AWS_CONFIG2_am_rise3                     (1U<<3)
#define ST25R500_REG_AWS_CONFIG2_am_rise2                     (1U<<2)
#define ST25R500_REG_AWS_CONFIG2_am_rise1                     (1U<<1)
#define ST25R500_REG_AWS_CONFIG2_am_rise0                     (1U<<0)
#define ST25R500_REG_AWS_CONFIG2_am_rise_mask                 (0xFU<<0)
#define ST25R500_REG_AWS_CONFIG2_am_rise_shift                (0U)

#define ST25R500_REG_AWS_TIME1_tentx1_3                       (1U<<7)
#define ST25R500_REG_AWS_TIME1_tentx1_2                       (1U<<6)
#define ST25R500_REG_AWS_TIME1_tentx1_1                       (1U<<5)
#define ST25R500_REG_AWS_TIME1_tentx1_0                       (1U<<4)
#define ST25R500_REG_AWS_TIME1_tentx1_mask                    (0xFU<<4)
#define ST25R500_REG_AWS_TIME1_tentx1_shift                   (4U)
#define ST25R500_REG_AWS_TIME1_tdres1_3                       (1U<<3)
#define ST25R500_REG_AWS_TIME1_tdres1_2                       (1U<<2)
#define ST25R500_REG_AWS_TIME1_tdres1_1                       (1U<<1)
#define ST25R500_REG_AWS_TIME1_tdres1_0                       (1U<<0)
#define ST25R500_REG_AWS_TIME1_tdres1_mask                    (0xFU<<0)
#define ST25R500_REG_AWS_TIME1_tdres1_shift                   (0U)

#define ST25R500_REG_AWS_TIME2_tpasssinkx1_3                  (1U<<7)
#define ST25R500_REG_AWS_TIME2_tpasssinkx1_2                  (1U<<6)
#define ST25R500_REG_AWS_TIME2_tpasssinkx1_1                  (1U<<5)
#define ST25R500_REG_AWS_TIME2_tpasssinkx1_0                  (1U<<4)
#define ST25R500_REG_AWS_TIME2_tpasssinkx1_mask               (0xFU<<4)
#define ST25R500_REG_AWS_TIME2_tpasssinkx1_shift              (4U)
#define ST25R500_REG_AWS_TIME2_tsinkoff1_3                    (1U<<3)
#define ST25R500_REG_AWS_TIME2_tsinkoff1_2                    (1U<<2)
#define ST25R500_REG_AWS_TIME2_tsinkoff1_1                    (1U<<1)
#define ST25R500_REG_AWS_TIME2_tsinkoff1_0                    (1U<<0)
#define ST25R500_REG_AWS_TIME2_tsinkoff1_mask                 (0xFU<<0)
#define ST25R500_REG_AWS_TIME2_tsinkoff1_shift                (0U)

#define ST25R500_REG_AWS_TIME3_tilim2_3                       (1U<<7)
#define ST25R500_REG_AWS_TIME3_tilim2_2                       (1U<<6)
#define ST25R500_REG_AWS_TIME3_tilim2_1                       (1U<<5)
#define ST25R500_REG_AWS_TIME3_tilim2_0                       (1U<<4)
#define ST25R500_REG_AWS_TIME3_tilim2_mask                    (0xFU<<4)
#define ST25R500_REG_AWS_TIME3_tilim2_shift                   (4U)
#define ST25R500_REG_AWS_TIME3_tdres2_3                       (1U<<3)
#define ST25R500_REG_AWS_TIME3_tdres2_2                       (1U<<2)
#define ST25R500_REG_AWS_TIME3_tdres2_1                       (1U<<1)
#define ST25R500_REG_AWS_TIME3_tdres2_0                       (1U<<0)
#define ST25R500_REG_AWS_TIME3_tdres2_mask                    (0xFU<<0)
#define ST25R500_REG_AWS_TIME3_tdres2_shift                   (0U)

#define ST25R500_REG_AWS_TIME4_tpasssinkx2_3                  (1U<<7)
#define ST25R500_REG_AWS_TIME4_tpasssinkx2_2                  (1U<<6)
#define ST25R500_REG_AWS_TIME4_tpasssinkx2_1                  (1U<<5)
#define ST25R500_REG_AWS_TIME4_tpasssinkx2_0                  (1U<<4)
#define ST25R500_REG_AWS_TIME4_tpasssinkx2_mask               (0xFU<<4)
#define ST25R500_REG_AWS_TIME4_tpasssinkx2_shift              (4U)
#define ST25R500_REG_AWS_TIME4_tsinkoff2_3                    (1U<<3)
#define ST25R500_REG_AWS_TIME4_tsinkoff2_2                    (1U<<2)
#define ST25R500_REG_AWS_TIME4_tsinkoff2_1                    (1U<<1)
#define ST25R500_REG_AWS_TIME4_tsinkoff2_0                    (1U<<0)
#define ST25R500_REG_AWS_TIME4_tsinkoff2_mask                 (0xFU<<0)
#define ST25R500_REG_AWS_TIME4_tsinkoff2_shift                (0U)

#define ST25R500_REG_OVERSHOOT_CONF_ov_pattern7               (1U<<7)
#define ST25R500_REG_OVERSHOOT_CONF_ov_pattern6               (1U<<6)
#define ST25R500_REG_OVERSHOOT_CONF_ov_pattern5               (1U<<5)
#define ST25R500_REG_OVERSHOOT_CONF_ov_pattern4               (1U<<4)
#define ST25R500_REG_OVERSHOOT_CONF_ov_pattern3               (1U<<3)
#define ST25R500_REG_OVERSHOOT_CONF_ov_pattern2               (1U<<2)
#define ST25R500_REG_OVERSHOOT_CONF_ov_pattern1               (1U<<1)
#define ST25R500_REG_OVERSHOOT_CONF_ov_pattern0               (1U<<0)

#define ST25R500_REG_UNDERSHOOT_CONF_uv_pattern7              (1U<<7)
#define ST25R500_REG_UNDERSHOOT_CONF_uv_pattern6              (1U<<6)
#define ST25R500_REG_UNDERSHOOT_CONF_uv_pattern5              (1U<<5)
#define ST25R500_REG_UNDERSHOOT_CONF_uv_pattern4              (1U<<4)
#define ST25R500_REG_UNDERSHOOT_CONF_uv_pattern3              (1U<<3)
#define ST25R500_REG_UNDERSHOOT_CONF_uv_pattern2              (1U<<2)
#define ST25R500_REG_UNDERSHOOT_CONF_uv_pattern1              (1U<<1)
#define ST25R500_REG_UNDERSHOOT_CONF_uv_pattern0              (1U<<0)

#define ST25R500_REG_EFD_THRESHOLD_efd_dt3                    (1U<<7)
#define ST25R500_REG_EFD_THRESHOLD_efd_dt2                    (1U<<6)
#define ST25R500_REG_EFD_THRESHOLD_efd_dt1                    (1U<<5)
#define ST25R500_REG_EFD_THRESHOLD_efd_dt0                    (1U<<4)
#define ST25R500_REG_EFD_THRESHOLD_efd_dt_mask                (0xFU<<4)
#define ST25R500_REG_EFD_THRESHOLD_efd_dt_shift               (4U)
#define ST25R500_REG_EFD_THRESHOLD_efd_at3                    (1U<<3)
#define ST25R500_REG_EFD_THRESHOLD_efd_at2                    (1U<<2)
#define ST25R500_REG_EFD_THRESHOLD_efd_at1                    (1U<<1)
#define ST25R500_REG_EFD_THRESHOLD_efd_at0                    (1U<<0)
#define ST25R500_REG_EFD_THRESHOLD_efd_at_mask                (0xFU<<0)
#define ST25R500_REG_EFD_THRESHOLD_efd_at_shift               (0U)

#define ST25R500_TEST_REG_ADC2CE_adc_cemem_src                (1U<<3)
#define ST25R500_TEST_REG_ADC2CE_adc_cemem_rate               (1U<<2)
#define ST25R500_TEST_REG_ADC2CE_adc_cemem_mask               (0x3U<<0)
#define ST25R500_TEST_REG_ADC2CE_adc_cemem_shift              (0U)

#define ST25R500_TEST_REG_OSC_TIMING_wait_ok_count_set_mask   (0x3FU<<0)
#define ST25R500_TEST_REG_OSC_TIMING_wait_ok_count_set_shift  (0U)

#define ST25R500_TEST_REG_MAN_TIMING_main_wait_ok             (1U<<6)

#define ST25R500_TEST_REG_OVERLAP_CONTROL_rfo_non_mode_mask   (0x3U<<1)
#define ST25R500_TEST_REG_OVERLAP_CONTROL_rfo_non_mode_shift  (1U)
#define ST25R500_TEST_REG_OVERLAP_CONTROL_rfo_non_mode_low    (1U<<1)
#define ST25R500_TEST_REG_OVERLAP_CONTROL_rfo_non_mode_high   (2U<<1)
#define ST25R500_TEST_REG_OVERLAP_CONTROL_rfo_non_mode_en     (1U<<0)

#define ST25R500_TEST_REG_DIAG_MEAS_discon_tad_out            (1U<<2)


#define ST25R500_IRQ_MASK_ALL             (uint32_t)(0xFFFFFFFFUL)  /*!< All ST25R500 interrupt sources                             */
#define ST25R500_IRQ_MASK_NONE            (uint32_t)(0x00000000UL)  /*!< No ST25R500 interrupt source                               */

/* Main interrupt register */
#define ST25R500_IRQ_MASK_SUBC_START      (uint32_t)(0x00000080U)   /*!< ST25R500 subcarrier start interrupt                        */
#define ST25R500_IRQ_MASK_COL             (uint32_t)(0x00000040U)   /*!< ST25R500 bit collision interrupt                           */
#define ST25R500_IRQ_MASK_WL              (uint32_t)(0x00000020U)   /*!< ST25R500 FIFO water level interrupt                        */
#define ST25R500_IRQ_MASK_RX_REST         (uint32_t)(0x00000010U)   /*!< ST25R500 automatic reception restart interrupt             */
#define ST25R500_IRQ_MASK_RXE             (uint32_t)(0x00000008U)   /*!< ST25R500 end of receive interrupt                          */
#define ST25R500_IRQ_MASK_RXS             (uint32_t)(0x00000004U)   /*!< ST25R500 start of receive interrupt                        */
#define ST25R500_IRQ_MASK_TXE             (uint32_t)(0x00000002U)   /*!< ST25R500 end of transmission interrupt                     */
#define ST25R500_IRQ_MASK_RX_ERR          (uint32_t)(0x00000001U)   /*!< ST25R500 Reception error interrupt                         */

/* Timer and Error interrupt register */
#define ST25R500_IRQ_MASK_GPE             (uint32_t)(0x00008000U)   /*!< ST25R500 general purpose timer expired interrupt           */
#define ST25R500_IRQ_MASK_NRE             (uint32_t)(0x00004000U)   /*!< ST25R500 no-response timer expired interrupt               */
#define ST25R500_IRQ_MASK_WPT_STOP        (uint32_t)(0x00002000U)   /*!< ST25R500 WPT stop received interrupt                       */
#define ST25R500_IRQ_MASK_WPT_FOD         (uint32_t)(0x00001000U)   /*!< ST25R500 WPT FOD received interrupt                        */
#define ST25R500_IRQ_MASK_RFU             (uint32_t)(0x00000800U)   /*!< ST25R500 RFU                                               */
#define ST25R500_IRQ_MASK_CE_SC           (uint32_t)(0x00000400U)   /*!< ST25R500 CE state change interrupt                         */
#define ST25R500_IRQ_MASK_RXE_CE          (uint32_t)(0x00000200U)   /*!< ST25R500 end of reception in CE interrupt                  */
#define ST25R500_IRQ_MASK_NFCT            (uint32_t)(0x00000100U)   /*!< ST25R500 bit rate was recognized interrupt                 */

/* Wake-up interrupt register */
#define ST25R500_IRQ_MASK_WUTME           (uint32_t)(0x00800000U)   /*!< ST25R500 WU Measure event interrupt                        */
#define ST25R500_IRQ_MASK_EOF             (uint32_t)(0x00400000U)   /*!< ST25R500 External Field Off interrupt                      */
#define ST25R500_IRQ_MASK_EON             (uint32_t)(0x00200000U)   /*!< ST25R500 External Field On interrupt                       */
#define ST25R500_IRQ_MASK_DCT             (uint32_t)(0x00100000U)   /*!< ST25R500 termination of direct command interrupt           */
#define ST25R500_IRQ_MASK_WUQ             (uint32_t)(0x00080000U)   /*!< ST25R500 wake-up Q-Channel interrupt                       */
#define ST25R500_IRQ_MASK_WUI             (uint32_t)(0x00040000U)   /*!< ST25R500 wake-up I-Channel interrupt                       */
#define ST25R500_IRQ_MASK_WUT             (uint32_t)(0x00020000U)   /*!< ST25R500 wake-up timer interrupt                           */
#define ST25R500_IRQ_MASK_OSC             (uint32_t)(0x00010000U)   /*!< ST25R500 oscillator stable interrupt                       */
