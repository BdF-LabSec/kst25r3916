#pragma once

/* ST25R3916 direct commands */
#define ST25R3916_CMD_SET_DEFAULT              0x01U    /*!< Puts the chip in default state (same as after power-up) */
#define ST25R3916_CMD_STOP                     0x02U    /*!< Stops all activities and clears FIFO                    */
#define ST25R3916_CMD_TRANSMIT_WITH_CRC        0x04U    /*!< Transmit with CRC                                       */
#define ST25R3916_CMD_TRANSMIT_WITHOUT_CRC     0x05U    /*!< Transmit without CRC                                    */
#define ST25R3916_CMD_TRANSMIT_REQA            0x06U    /*!< Transmit REQA                                           */
#define ST25R3916_CMD_TRANSMIT_WUPA            0x07U    /*!< Transmit WUPA                                           */
#define ST25R3916_CMD_INITIAL_RF_COLLISION     0x08U    /*!< NFC transmit with Initial RF Collision Avoidance        */
#define ST25R3916_CMD_RESPONSE_RF_COLLISION_N  0x09U    /*!< NFC transmit with Response RF Collision Avoidance       */
#define ST25R3916_CMD_GOTO_SENSE               0x0DU    /*!< Passive target logic to Sense/Idle state                */
#define ST25R3916_CMD_GOTO_SLEEP               0x0EU    /*!< Passive target logic to Sleep/Halt state                */
#define ST25R3916_CMD_MASK_RECEIVE_DATA        0x10U    /*!< Mask receive data                                       */
#define ST25R3916_CMD_UNMASK_RECEIVE_DATA      0x11U    /*!< Unmask receive data                                     */
#define ST25R3916_CMD_AM_MOD_STATE_CHANGE      0x12U    /*!< AM Modulation state change                              */
#define ST25R3916_CMD_MEASURE_AMPLITUDE        0x13U    /*!< Measure singal amplitude on RFI inputs                  */
#define ST25R3916_CMD_RESET_RXGAIN             0x15U    /*!< Reset RX Gain                                           */
#define ST25R3916_CMD_ADJUST_REGULATORS        0x16U    /*!< Adjust regulators                                       */
#define ST25R3916_CMD_CALIBRATE_DRIVER_TIMING  0x18U    /*!< Starts the sequence to adjust the driver timing         */
#define ST25R3916_CMD_MEASURE_PHASE            0x19U    /*!< Measure phase between RFO and RFI signal                */
#define ST25R3916_CMD_CLEAR_RSSI               0x1AU    /*!< Clear RSSI bits and restart the measurement             */
#define ST25R3916_CMD_CLEAR_FIFO               0x1BU    /*!< Clears FIFO, Collision and IRQ status                   */
#define ST25R3916_CMD_TRANSPARENT_MODE         0x1CU    /*!< Transparent mode                                        */
#define ST25R3916_CMD_CALIBRATE_C_SENSOR       0x1DU    /*!< Calibrate the capacitive sensor                         */
#define ST25R3916_CMD_MEASURE_CAPACITANCE      0x1EU    /*!< Measure capacitance                                     */
#define ST25R3916_CMD_MEASURE_VDD              0x1FU    /*!< Measure power supply voltage                            */
#define ST25R3916_CMD_START_GP_TIMER           0x20U    /*!< Start the general purpose timer                         */
#define ST25R3916_CMD_START_WUP_TIMER          0x21U    /*!< Start the wake-up timer                                 */
#define ST25R3916_CMD_START_MASK_RECEIVE_TIMER 0x22U    /*!< Start the mask-receive timer                            */
#define ST25R3916_CMD_START_NO_RESPONSE_TIMER  0x23U    /*!< Start the no-response timer                             */
#define ST25R3916_CMD_START_PPON2_TIMER        0x24U    /*!< Start PPon2 timer                                       */
#define ST25R3916_CMD_STOP_NRT                 0x28U    /*!< Stop No Response Timer                                  */
#define ST25R3916_CMD_SPACE_B_ACCESS           0x3BU    /*!< Enable R/W access to the space B registers              */
#define ST25R3916_CMD_TEST_ACCESS              0x3CU    /*!< Enable R/W access to the test registers                 */
