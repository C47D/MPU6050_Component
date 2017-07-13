#ifndef `$INSTANCE_NAME`_REGS_H
#define `$INSTANCE_NAME`_REGS_H

#include "cytypes.h"
#include "cyfitter.h"

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define MPU6050_ADDRESS_COMPASS 0x0C

/* Register Map for MPU-60X0 */

#define MPU6050_RA_SELF_TEST_X     0x0D
#define MPU6050_RA_SELF_TEST_Y     0x0E
#define MPU6050_RA_SELF_TEST_Z     0x0F
#define MPU6050_RA_SELF_TEST_A     0x10
#define MPU6050_RA_SMPLRT_DIV      0x19
#define MPU6050_RA_CONFIG          0x1A
#define MPU6050_RA_GYRO_CONFIG     0x1B
#define MPU6050_RA_ACCEL_CONFIG    0x1C
#define MPU6050_RA_MOT_THR         0x1F
#define MPU6050_RA_FIFO_EN         0x23
#define MPU6050_RA_I2C_MST_CTRL    0x24
#define MPU6050_RA_I2C_SLV0_ADDR   0x25
#define MPU6050_RA_I2C_SLV0_REG    0x26
#define MPU6050_RA_I2C_SLV0_CTRL   0x27
#define MPU6050_RA_I2C_SLV1_ADDR   0x28
#define MPU6050_RA_I2C_SLV1_REG    0x29
#define MPU6050_RA_I2C_SLV1_CTRL   0x2A
#define MPU6050_RA_I2C_SLV2_ADDR   0x2B
#define MPU6050_RA_I2C_SLV2_REG    0x2C
#define MPU6050_RA_I2C_SLV2_CTRL   0x2D
#define MPU6050_RA_I2C_SLV3_ADDR   0x2E
#define MPU6050_RA_I2C_SLV3_REG    0x2F
#define MPU6050_RA_I2C_SLV3_CTRL   0x30
#define MPU6050_RA_I2C_SLV4_ADDR   0x31
#define MPU6050_RA_I2C_SLV4_REG    0x32
#define MPU6050_RA_I2C_SLV4_DO     0x33
#define MPU6050_RA_I2C_SLV4_CTRL   0x34
#define MPU6050_RA_I2C_SLV4_DI     0x35
#define MPU6050_RA_I2C_MST_STATUS  0x36
#define MPU6050_RA_INT_PIN_CFG     0x37
#define MPU6050_RA_INT_ENABLE      0x38
#define MPU6050_RA_INT_STATUS      0x3A
#define MPU6050_RA_ACCEL_XOUT_H    0x3B
#define MPU6050_RA_ACCEL_XOUT_L    0x3C
#define MPU6050_RA_ACCEL_YOUT_H    0x3D
#define MPU6050_RA_ACCEL_YOUT_L    0x3E
#define MPU6050_RA_ACCEL_ZOUT_H    0x3F
#define MPU6050_RA_ACCEL_ZOUT_L    0x40
#define MPU6050_RA_TEMP_OUT_H      0x41
#define MPU6050_RA_TEMP_OUT_L      0x42
#define MPU6050_RA_GYRO_XOUT_H     0x43
#define MPU6050_RA_GYRO_XOUT_L     0x44
#define MPU6050_RA_GYRO_YOUT_H     0x45
#define MPU6050_RA_GYRO_YOUT_L     0x46
#define MPU6050_RA_GYRO_ZOUT_H     0x47
#define MPU6050_RA_GYRO_ZOUT_L     0x48
#define MPU6050_RA_EXT_SENS_DATA_00    0x49
#define MPU6050_RA_EXT_SENS_DATA_01    0x4A
#define MPU6050_RA_EXT_SENS_DATA_02    0x4B
#define MPU6050_RA_EXT_SENS_DATA_03    0x4C
#define MPU6050_RA_EXT_SENS_DATA_04    0x4D
#define MPU6050_RA_EXT_SENS_DATA_05    0x4E
#define MPU6050_RA_EXT_SENS_DATA_06    0x4F
#define MPU6050_RA_EXT_SENS_DATA_07    0x50
#define MPU6050_RA_EXT_SENS_DATA_08    0x51
#define MPU6050_RA_EXT_SENS_DATA_09    0x52
#define MPU6050_RA_EXT_SENS_DATA_10    0x53
#define MPU6050_RA_EXT_SENS_DATA_11    0x54
#define MPU6050_RA_EXT_SENS_DATA_12    0x55
#define MPU6050_RA_EXT_SENS_DATA_13    0x56
#define MPU6050_RA_EXT_SENS_DATA_14    0x57
#define MPU6050_RA_EXT_SENS_DATA_15    0x58
#define MPU6050_RA_EXT_SENS_DATA_16    0x59
#define MPU6050_RA_EXT_SENS_DATA_17    0x5A
#define MPU6050_RA_EXT_SENS_DATA_18    0x5B
#define MPU6050_RA_EXT_SENS_DATA_19    0x5C
#define MPU6050_RA_EXT_SENS_DATA_20    0x5D
#define MPU6050_RA_EXT_SENS_DATA_21    0x5E
#define MPU6050_RA_EXT_SENS_DATA_22    0x5F
#define MPU6050_RA_EXT_SENS_DATA_23    0x60
#define MPU6050_RA_I2C_SLV0_DO     0x63
#define MPU6050_RA_I2C_SLV1_DO     0x64
#define MPU6050_RA_I2C_SLV2_DO     0x65
#define MPU6050_RA_I2C_SLV3_DO     0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL  0x67
#define MPU6050_RA_SIGNAL_PATH_RESET   0x68
#define MPU6050_RA_MOT_DETECT_CTRL     0x69
#define MPU6050_RA_USER_CTRL       0x6A
#define MPU6050_RA_PWR_MGMT_1      0x6B
#define MPU6050_RA_PWR_MGMT_2      0x6C
#define MPU6050_RA_FIFO_COUNTH     0x72
#define MPU6050_RA_FIFO_COUNTL     0x73
#define MPU6050_RA_FIFO_R_W        0x74
#define MPU6050_RA_WHO_AM_I        0x75

/* Undocumented Registers */
#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD

#define MPU6050_TC_PWR_MODE_Pos 7
#define MPU6050_TC_OFFSET_Pos 6
#define MPU6050_TC_OFFSET_Len 6
#define MPU6050_TC_OTP_BNK_VLD_Pos 0

#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18

#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E

#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22

#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_DMP_INT_STATUS_5_Pos 5
#define MPU6050_DMP_INT_STATUS_4_Pos 4
#define MPU6050_DMP_INT_STATUS_3_Pos 3
#define MPU6050_DMP_INT_STATUS_2_Pos 2
#define MPU6050_DMP_INT_STATUS_1_Pos 1
#define MPU6050_DMP_INT_STATUS_0_Pos 0

#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_MOT_DETECT_STATUS_XNEG_Pos 7
#define MPU6050_MOT_DETECT_STATUS_XPOS_Pos 6
#define MPU6050_MOT_DETECT_STATUS_YNEG_Pos 5
#define MPU6050_MOT_DETECT_STATUS_YPOS_Pos 4
#define MPU6050_MOT_DETECT_STATUS_ZNEG_Pos 3
#define MPU6050_MOT_DETECT_STATUS_ZPOS_Pos 2
#define MPU6050_MOT_DETECT_STATUS_ZRMOT_Pos 0

#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71

/**
*   SELF_TEST_X Register
*   R/W
*   Address = 0x0D
*/
#define MPU6050_SELF_TEST_X_XA_TEST_Pos 7
#define MPU6050_SELF_TEST_X_XA_TEST_Len 3
#define MPU6050_SELF_TEST_X_XG_TEST_Pos 4
#define MPU6050_SELF_TEST_X_XG_TEST_Len 5

/**
*   SELF_TEST_Y Register
*   R/W
*   Address = 0x0E
*/
#define MPU6050_SELF_TEST_Y_YA_TEST_Pos 7
#define MPU6050_SELF_TEST_Y_YA_TEST_Msk 3
#define MPU6050_SELF_TEST_Y_YG_TEST_Pos 4
#define MPU6050_SELF_TEST_Y_YG_TEST_Msk 5

/**
*   SELF_TEST_Z Register
*   R/W
*   Address = 0x0F
*/
#define MPU6050_SELF_TEST_Z_ZA_TEST_Pos 7
#define MPU6050_SELF_TEST_Z_ZA_TEST_Len 3
#define MPU6050_SELF_TEST_Z_ZG_TEST_Pos 4
#define MPU6050_SELF_TEST_Z_ZG_TEST_Len 5

/**
*   SELF_TEST_A Register
*   R/W
*   Address = 0x10
*/
#define MPU6050_SELF_TEST_A_XA_TEST_Pos 5
#define MPU6050_SELF_TEST_A_XA_TEST_Len 2
#define MPU6050_SELF_TEST_A_YA_TEST_Pos 3
#define MPU6050_SELF_TEST_A_YA_TEST_Len 2
#define MPU6050_SELF_TEST_A_ZA_TEST_Pos 1
#define MPU6050_SELF_TEST_A_ZA_TEST_Len 2

/**
*   SMPLRT_DIV Register
*   R/W
*   Address = 0x19
*/
#define MPU6050_SMPLRT_DIV_SMPLRT_DIV_Pos 7
#define MPU6050_SMPLRT_DIV_SMPLRT_DIV_Len 8

/**
*   CONFIG Register
*   R/W
*   Address = 0x1A
*/
#define MPU6050_CONFIG_EXT_SYNC_SET_Pos 5
#define MPU6050_CONFIG_EXT_SYNC_SET_Len 3
#define MPU6050_CONFIG_DLPF_CFG_Pos 2
#define MPU6050_CONFIG_DLPF_CFG_Len 3
#define MPU6050_CONFIG_EXT_SYNC_SET_INPUT_DISABLED 0
#define MPU6050_CONFIG_EXT_SYNC_SET_TEMP_OUT 1
#define MPU6050_CONFIG_EXT_SYNC_SET_GYRO_XOUT_L 2
#define MPU6050_CONFIG_EXT_SYNC_SET_GYRO_YOUT_L 3
#define MPU6050_CONFIG_EXT_SYNC_SET_GYRO_ZOUT_L 4
#define MPU6050_CONFIG_EXT_SYNC_SET_ACCEL_XOUT_L 5
#define MPU6050_CONFIG_EXT_SYNC_SET_ACCEL_YOUT_L 6
#define MPU6050_CONFIG_EXT_SYNC_SET_ACCEL_ZOUT_L 7
/**
*   GYRO_CONFIG Register
*   R/W
*   Address = 0x1B
*/
#define MPU6050_GYRO_CONFIG_FS_SEL_Pos 4
#define MPU6050_GYRO_CONFIG_FS_SEL_Len 2
#define MPU6050_GYRO_CONFIG_FS_SEL_250 0
#define MPU6050_GYRO_CONFIG_FS_SEL_500 1
#define MPU6050_GYRO_CONFIG_FS_SEL_1000 2
#define MPU6050_GYRO_CONFIG_FS_SEL_2000 3

/**
*   ACCEL_CONFIG Register
*   R/W
*   Address = 0x1C
*/
#define MPU6050_ACCEL_CONFIG_XA_ST_Pos 7
#define MPU6050_ACCEL_CONFIG_XA_ST_Len 1
#define MPU6050_ACCEL_CONFIG_YA_ST_Pos 6
#define MPU6050_ACCEL_CONFIG_YA_ST_Len 1
#define MPU6050_ACCEL_CONFIG_ZA_ST_Pos 5
#define MPU6050_ACCEL_CONFIG_ZA_ST_Len 1
#define MPU6050_ACCEL_CONFIG_AFS_SEL_Pos 4
#define MPU6050_ACCEL_CONFIG_AFS_SEL_Len 2
#define MPU6050_ACCEL_CONFIG_AFS_SEL_2G 0
#define MPU6050_ACCEL_CONFIG_AFS_SEL_4G 1
#define MPU6050_ACCEL_CONFIG_AFS_SEL_8G 2
#define MPU6050_ACCEL_CONFIG_AFS_SEL_16G 3
#define MPU6050_ACCEL_CONFIG_HPF_Pos 2
#define MPU6050_ACCEL_CONFIG_HPF_Len 3
#define MPU6050_ACCEL_CONFIG_HPF_CUT_NONE 0
#define MPU6050_ACCEL_CONFIG_HPF_CUT_5HZ 1
#define MPU6050_ACCEL_CONFIG_HPF_CUT_2_5HZ 2
#define MPU6050_ACCEL_CONFIG_HPF_CUT_1_25HZ 3
#define MPU6050_ACCEL_CONFIG_HPF_CUT_0_63HZ 4
#define MPU6050_ACCEL_CONFIG_HPF_CUT_HOLD 7

/**
*   MOT_THR Register
*   R/W
*   Address = 0x1F
*/
#define MPU6050_MOT_THR_MOT_THR_Pos 7
#define MPU6050_MOT_THR_MOT_THR_Len 8

/**
*   FIFO_EN Register
*   R/W
*   Address = 0x23
*/
#define MPU6050_FIFO_EN_TEMP_FIFO_EN_Pos 7
#define MPU6050_FIFO_EN_TEMP_FIFO_EN_Len 1
#define MPU6050_FIFO_EN_XG_FIFO_EN_Pos 6
#define MPU6050_FIFO_EN_XG_FIFO_EN_Len 1
#define MPU6050_FIFO_EN_YG_FIFO_EN_Pos 5
#define MPU6050_FIFO_EN_YG_FIFO_EN_Len 1
#define MPU6050_FIFO_EN_ZG_FIFO_EN_Pos 4
#define MPU6050_FIFO_EN_ZG_FIFO_EN_Len 1
#define MPU6050_FIFO_EN_ACCEL_FIFO_EN_Pos 3
#define MPU6050_FIFO_EN_ACCEL_FIFO_EN_Len 1
#define MPU6050_FIFO_EN_SLV2_FIFO_EN_Pos 2
#define MPU6050_FIFO_EN_SLV2_FIFO_EN_Len 1
#define MPU6050_FIFO_EN_SLV1_FIFO_EN_Pos 1
#define MPU6050_FIFO_EN_SLV1_FIFO_EN_Len 1
#define MPU6050_FIFO_EN_SLV0_FIFO_EN_Pos 0
#define MPU6050_FIFO_EN_SLV0_FIFO_EN_Len 1

/**
*   I2C_MST_CTRL Register
*   R/W
*   Address = 0x24
*/
#define MPU6050_I2C_MST_CTRL_MULT_MST_EN_Pos 7
#define MPU6050_I2C_MST_CTRL_MULT_MST_EN_Len 1
#define MPU6050_I2C_MST_CTRL_WAIT_FOR_ES_Pos 6
#define MPU6050_I2C_MST_CTRL_WAIT_FOR_ES_Len 1
#define MPU6050_I2C_MST_CTRL_SLV3_FIFO_EN_Pos 5
#define MPU6050_I2C_MST_CTRL_SLV3_FIFO_EN_Len 1
#define MPU6050_I2C_MST_CTRL_I2C_MST_P_NSR_Pos 4
#define MPU6050_I2C_MST_CTRL_I2C_MST_P_NSR_Len 1
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_Pos 3
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_Len 4
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_348KHZ 0
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_333KHZ 1
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_320KHZ 2
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_308KHZ 3
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_296KHZ 4
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_286KHZ 5
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_276KHZ 6
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_267KHZ 7
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_258KHZ 8
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_500KHZ 9
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_471KHZ 10
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_444KHZ 11
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_421KHZ 12
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_400KHZ 13
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_381KHZ 14
#define MPU6050_I2C_MST_CTRL_I2C_MST_CLK_364KHZ 15

/**
* Added for mpu6050 lib compability
*/

#define MPU6050_I2C_SLV_RW_Pos 7
#define MPU6050_I2C_SLV_ADDR_Pos 6
#define MPU6050_I2C_SLV0_ADDR_Len 7
#define MPU6050_I2C_SLV_EN_Pos 7
#define MPU6050_I2C_SLV_BYTE_SW_Pos 6
#define MPU6050_I2C_SLV_REG_DIS_Pos 5
#define MPU6050_I2C_SLV_GRP_Pos 4
#define MPU6050_I2C_SLV_LEN_Pos 3
#define MPU6050_I2C_SLV_LEN_Len 4

/**
*   I2C_SLV0_ADDR Register
*   R/W
*   Address = 0x25
*/
#define MPU6050_I2C_SLV0_ADDR_I2C_SLV0_RW_Pos 7
#define MPU6050_I2C_SLV0_ADDR_I2C_SLV0_RW_Len 1
#define MPU6050_I2C_SLV0_ADDR_I2C_SLV0_ADDR_Pos 6
#define MPU6050_I2C_SLV0_ADDR_I2C_SLV0_ADDR_Len 7

/**
*   I2C_SLV0_REG Register
*   R/W
*   Address = 0x26
*/
#define MPU6050_I2C_SLV0_REG_I2C_SLV0_REG_Pos 7
#define MPU6050_I2C_SLV0_REG_I2C_SLV0_REG_Len 8

/**
*   I2C_SLV0_CTRL Register
*   R/W
*   Address = 0x27
*/
#define MPU6050_I2C_SLV0_CTRL_I2C_SLV0_EN_Pos 7
#define MPU6050_I2C_SLV0_CTRL_I2C_SLV0_EN_Len 1
#define MPU6050_I2C_SLV0_CTRL_I2C_SLV0_BYTE_SW_Pos 6
#define MPU6050_I2C_SLV0_CTRL_I2C_SLV0_BYTE_SW_Len 1
#define MPU6050_I2C_SLV0_CTRL_I2C_SLV0_REG_DIS_Pos 5
#define MPU6050_I2C_SLV0_CTRL_I2C_SLV0_REG_DIS_Len 1
#define MPU6050_I2C_SLV0_CTRL_I2C_SLV0_GRP_Pos 4
#define MPU6050_I2C_SLV0_CTRL_I2C_SLV0_GRP_Len 1
#define MPU6050_I2C_SLV0_CTRL_I2C_SLV0_LEN_Pos 3
#define MPU6050_I2C_SLV0_CTRL_I2C_SLV0_LEN_Len 4

/**
*   I2C_SLV1_ADDR Register
*   R/W
*   Address = 0x28
*/
#define MPU6050_I2C_SLV1_ADDR_I2C_SLV1_RW_Pos 7
#define MPU6050_I2C_SLV1_ADDR_I2C_SLV1_RW_Len 1
#define MPU6050_I2C_SLV1_ADDR_I2C_SLV1_ADDR_Pos 6
#define MPU6050_I2C_SLV1_ADDR_I2C_SLV1_ADDR_Len 7

/**
*   I2C_SLV1_REG Register
*   R/W
*   Address = 0x29
*/
#define MPU6050_I2C_SLV1_REG_I2C_SLV1_REG_Pos 7
#define MPU6050_I2C_SLV1_REG_I2C_SLV1_REG_Len 8

/**
*   I2C_SLV1_CTRL Register
*   R/W
*   Address = 0x2A
*/
#define MPU6050_I2C_SLV1_CTRL_I2C_SLV1_EN_Pos 7
#define MPU6050_I2C_SLV1_CTRL_I2C_SLV1_EN_Len 1
#define MPU6050_I2C_SLV1_CTRL_I2C_SLV1_BYTE_SW_Pos 6
#define MPU6050_I2C_SLV1_CTRL_I2C_SLV1_BYTE_SW_Len 1
#define MPU6050_I2C_SLV1_CTRL_I2C_SLV1_REG_DIS_Pos 5
#define MPU6050_I2C_SLV1_CTRL_I2C_SLV1_REG_DIS_Len 1
#define MPU6050_I2C_SLV1_CTRL_I2C_SLV1_GRP_Pos 4
#define MPU6050_I2C_SLV1_CTRL_I2C_SLV1_GRP_Len 1
#define MPU6050_I2C_SLV1_CTRL_I2C_SLV1_LEN_Pos 3
#define MPU6050_I2C_SLV1_CTRL_I2C_SLV1_LEN_Len 4

/**
*   I2C_SLV2_ADDR Register
*   R/W
*   Address = 0x2B
*/
#define MPU6050_I2C_SLV2_ADDR_I2C_SLV2_RW_Pos 7
#define MPU6050_I2C_SLV2_ADDR_I2C_SLV2_RW_Len 1
#define MPU6050_I2C_SLV2_ADDR_I2C_SLV2_ADDR_Pos 6
#define MPU6050_I2C_SLV2_ADDR_I2C_SLV2_ADDR_Len 7

/**
*   I2C_SLV2_REG Register
*   R/W
*   Address = 0x2C
*/
#define MPU6050_I2C_SLV2_REG_I2C_SLV2_REG_Pos 7
#define MPU6050_I2C_SLV2_REG_I2C_SLV2_REG_Len 8

/**
*   I2C_SLV2_CTRL Register
*   R/W
*   Address = 0x2D
*/
#define MPU6050_I2C_SLV2_CTRL_I2C_SLV2_EN_Pos 7
#define MPU6050_I2C_SLV2_CTRL_I2C_SLV2_EN_Len 1
#define MPU6050_I2C_SLV2_CTRL_I2C_SLV2_BYTE_SW_Pos 6
#define MPU6050_I2C_SLV2_CTRL_I2C_SLV2_BYTE_SW_Len 1
#define MPU6050_I2C_SLV2_CTRL_I2C_SLV2_REG_DIS_Pos 5
#define MPU6050_I2C_SLV2_CTRL_I2C_SLV2_REG_DIS_Len 1
#define MPU6050_I2C_SLV2_CTRL_I2C_SLV2_GRP_Pos 4
#define MPU6050_I2C_SLV2_CTRL_I2C_SLV2_GRP_Len 1
#define MPU6050_I2C_SLV2_CTRL_I2C_SLV2_LEN_Pos 3
#define MPU6050_I2C_SLV2_CTRL_I2C_SLV2_LEN_Len 4

/**
*   I2C_SLV3_ADDR Register
*   R/W
*   Address = 0x2E
*/
#define MPU6050_I2C_SLV3_ADDR_I2C_SLV3_RW_Pos 7
#define MPU6050_I2C_SLV3_ADDR_I2C_SLV3_RW_Len 1
#define MPU6050_I2C_SLV3_ADDR_I2C_SLV3_ADDR_Pos 6
#define MPU6050_I2C_SLV3_ADDR_I2C_SLV3_ADDR_Len 7

/**
*   I2C_SLV3_REG Register
*   R/W
*   Address = 0x2F
*/
#define MPU6050_I2C_SLV3_REG_I2C_SLV3_REG_Pos 7
#define MPU6050_I2C_SLV3_REG_I2C_SLV3_REG_Len 8

/**
*   I2C_SLV3_CTRL Register
*   R/W
*   Address = 0x30
*/
#define MPU6050_I2C_SLV3_CTRL_I2C_SLV3_EN_Pos 7
#define MPU6050_I2C_SLV3_CTRL_I2C_SLV3_EN_Len 1
#define MPU6050_I2C_SLV3_CTRL_I2C_SLV3_BYTE_SW_Pos 6
#define MPU6050_I2C_SLV3_CTRL_I2C_SLV3_BYTE_SW_Len 1
#define MPU6050_I2C_SLV3_CTRL_I2C_SLV3_REG_DIS_Pos 5
#define MPU6050_I2C_SLV3_CTRL_I2C_SLV3_REG_DIS_Len 1
#define MPU6050_I2C_SLV3_CTRL_I2C_SLV3_GRP_Pos 4
#define MPU6050_I2C_SLV3_CTRL_I2C_SLV3_GRP_Len 1
#define MPU6050_I2C_SLV3_CTRL_I2C_SLV3_LEN_Pos 3
#define MPU6050_I2C_SLV3_CTRL_I2C_SLV3_LEN_Len 4

/**
*   I2C_SLV4_ADDR Register
*   R/W
*   Address = 0x31
*/
#define MPU6050_I2C_SLV4_ADDR_I2C_SLV4_RW_Pos 7
#define MPU6050_I2C_SLV4_ADDR_I2C_SLV4_RW_Len 1
#define MPU6050_I2C_SLV4_ADDR_I2C_SLV4_ADDR_Pos 6
#define MPU6050_I2C_SLV4_ADDR_I2C_SLV4_ADDR_Len 7

/**
*   I2C_SLV4_REG Register
*   R/W
*   Address = 0x32
*/
#define MPU6050_I2C_SLV4_REG_I2C_SLV4_REG_Pos 7
#define MPU6050_I2C_SLV4_REG_I2C_SLV4_REG_Len 8

/**
*   I2C_SLV4_DO Register
*   R/W
*   Address = 0x33
*/
#define MPU6050_I2C_SLV4_DO_I2C_SLV4_DO_Pos 7
#define MPU6050_I2C_SLV4_DO_I2C_SLV4_DO_Len 8

/**
*   I2C_SLV4_CTRL Register
*   R/W
*   Address = 0x34
*/
#define MPU6050_I2C_SLV4_CTRL_I2C_SLV4_EN_Pos 7
#define MPU6050_I2C_SLV4_CTRL_I2C_SLV4_EN_Len 1
#define MPU6050_I2C_SLV4_CTRL_I2C_SLV4_INT_EN_Pos 6
#define MPU6050_I2C_SLV4_CTRL_I2C_SLV4_INT_EN_Len 1
#define MPU6050_I2C_SLV4_CTRL_I2C_SLV4_REG_DIS_Pos 5
#define MPU6050_I2C_SLV4_CTRL_I2C_SLV4_REG_DIS_Len 1
#define MPU6050_I2C_SLV4_CTRL_I2C_MST_DLY_Pos 4
#define MPU6050_I2C_SLV4_CTRL_I2C_MST_DLY_Len 5

/**
*   I2C_SLV4_DI Register
*   R/W
*   Address = 0x35
*/
#define MPU6050_I2C_SLV4_DI_I2C_SLV0_DI_Pos 7
#define MPU6050_I2C_SLV4_DI_I2C_SLV0_DI_Len 8

/**
*   I2C_MST_STATUS Register
*   R/W
*   Address = 0x36
*/
#define MPU6050_I2C_MST_STATUS_PASS_THROUGH_Pos 7
#define MPU6050_I2C_MST_STATUS_PASS_THROUGH_Len 1
#define MPU6050_I2C_MST_STATUS_I2C_SLV4_DONE_Pos 6
#define MPU6050_I2C_MST_STATUS_I2C_SLV4_DONE_Len 1
#define MPU6050_I2C_MST_STATUS_I2C_LOST_ARB_Pos 5
#define MPU6050_I2C_MST_STATUS_I2C_LOST_ARB_Len 1
#define MPU6050_I2C_MST_STATUS_I2C_SLV4_NACK_Pos 4
#define MPU6050_I2C_MST_STATUS_I2C_SLV4_NACK_Len 1
#define MPU6050_I2C_MST_STATUS_I2C_SLV3_NACK_Pos 3
#define MPU6050_I2C_MST_STATUS_I2C_SLV3_NACK_Len 1
#define MPU6050_I2C_MST_STATUS_I2C_SLV2_NACK_Pos 2
#define MPU6050_I2C_MST_STATUS_I2C_SLV2_NACK_Len 1
#define MPU6050_I2C_MST_STATUS_I2C_SLV1_NACK_Pos 1
#define MPU6050_I2C_MST_STATUS_I2C_SLV1_NACK_Len 1
#define MPU6050_I2C_MST_STATUS_I2C_SLV0_NACK_Pos 0
#define MPU6050_I2C_MST_STATUS_I2C_SLV0_NACK_Len 1

/**
*   INT_PIN_CFG Register
*   R/W
*   Address = 0x37
*/
#define MPU6050_INT_PIN_CFG_INT_LEVEL_Pos 7
#define MPU6050_INT_PIN_CFG_INT_LEVEL_Len 1
#define MPU6050_INT_PIN_CFG_INT_OPEN_Pos 6
#define MPU6050_INT_PIN_CFG_INT_OPEN_Len 1
#define MPU6050_INT_PIN_CFG_LATCH_INT_EN_Pos 5
#define MPU6050_INT_PIN_CFG_LATCH_INT_EN_Len 1
#define MPU6050_INT_PIN_CFG_INT_RD_CLEAR_Pos 4
#define MPU6050_INT_PIN_CFG_INT_RD_CLEAR_Len 1
#define MPU6050_INT_PIN_CFG_FSYNC_INT_LEVEL_Pos 3
#define MPU6050_INT_PIN_CFG_FSYNC_INT_LEVEL_Len 1
#define MPU6050_INT_PIN_CFG_FSYNC_INT_EN_Pos 2
#define MPU6050_INT_PIN_CFG_FSYNC_INT_EN_Len 1
#define MPU6050_INT_PIN_CFG_I2C_BYPASS_EN_Pos 1
#define MPU6050_INT_PIN_CFG_I2C_BYPASS_EN_Len 1
#define MPU6050_INT_PIN_CFG_I2C_CLKOUT_EN_Pos 0 /* Undocumented */
#define MPU6050_INT_PIN_CFG_I2C_CLKOUT_EN_Len 1

/**
*   INT_ENABLE Register
*   R/W
*   Address = 0x38
*/
#define MPU6050_INT_ENABLE_FF_EN_Pos 7 /* Undocumented */
#define MPU6050_INT_ENABLE_FF_EN_Len 1
#define MPU6050_INT_ENABLE_MOT_EN_Pos 6
#define MPU6050_INT_ENABLE_MOT_EN_Len 1
#define MPU6050_INT_ENABLE_ZMOT_EN_Pos 5 /* Undocumented */
#define MPU6050_INT_ENABLE_ZMOT_EN_Len 1
#define MPU6050_INT_ENABLE_FIFO_OFLOW_EN_Pos 4
#define MPU6050_INT_ENABLE_FIFO_OFLOW_EN_Len 1
#define MPU6050_INT_ENABLE_I2C_MST_INT_EN_Pos 3
#define MPU6050_INT_ENABLE_I2C_MST_INT_EN_Len 1
#define MPU6050_INT_ENABLE_PLL_RDY_INT_Pos 2 /* Undocumented */
#define MPU6050_INT_ENABLE_PLL_RDY_INT_Len 1
#define MPU6050_INT_ENABLE_DMP_INT_Pos 1 /* Undocumented */
#define MPU6050_INT_ENABLE_DMP_INT_Len 1
#define MPU6050_INT_ENABLE_DATA_RDY_EN_Pos 0
#define MPU6050_INT_ENABLE_DATA_RDY_EN_Len 1

/**
*   INT_STATUS Register
*   R/W
*   Address = 0x3A
*/
#define MPU6050_INT_STATUS_FF_EN_Pos 7 /* Undocumented */
#define MPU6050_INT_STATUS_FF_EN_Len 1
#define MPU6050_INT_STATUS_MOT_INT_Pos 6
#define MPU6050_INT_STATUS_MOT_INT_Len 1
#define MPU6050_INT_STATUS_ZMOT_EN_Pos 5 /* Undocumented */
#define MPU6050_INT_STATUS_ZMOT_EN_Len 1
#define MPU6050_INT_STATUS_FIFO_OFLOW_INT_Pos 4
#define MPU6050_INT_STATUS_FIFO_OFLOW_INT_Len 1
#define MPU6050_INT_STATUS_I2C_MST_INT_INT_Pos 3
#define MPU6050_INT_STATUS_I2C_MST_INT_Len 1
#define MPU6050_INT_STATUS_DATA_RDY_INT_Pos 0
#define MPU6050_INT_STATUS_DATA_RDY_INT_Len 1

/**
*   ACCEL_XOUT_H Register
*   R/W
*   Address = 0x3B
*/
#define MPU6050_ACCEL_XOUT_H_ACCEL_XOUT_Pos 7
#define MPU6050_ACCEL_XOUT_H_ACCEL_XOUT_Len 8

/**
*   ACCEL_XOUT_L Register
*   R/W
*   Address = 0x3C
*/
#define MPU6050_ACCEL_XOUT_L_ACCEL_XOUT_Pos 7
#define MPU6050_ACCEL_XOUT_L_ACCEL_XOUT_Len 8

/**
*   ACCEL_YOUT_H Register
*   R/W
*   Address = 0x3D
*/
#define MPU6050_ACCEL_YOUT_H_ACCEL_YOUT_Pos 7
#define MPU6050_ACCEL_YOUT_H_ACCEL_YOUT_Len 8

/**
*   ACCEL_YOUT_L Register
*   R/W
*   Address = 0x3E
*/
#define MPU6050_ACCEL_YOUT_L_ACCEL_YOUT_Pos 7
#define MPU6050_ACCEL_YOUT_L_ACCEL_YOUT_Len 8

/**
*   ACCEL_ZOUT_H Register
*   R/W
*   Address = 0x3F
*/
#define MPU6050_ACCEL_ZOUT_H_ACCEL_ZOUT_Pos 7
#define MPU6050_ACCEL_ZOUT_H_ACCEL_ZOUT_Len 8

/**
*   ACCEL_ZOUT_L Register
*   R/W
*   Address = 0x40
*/
#define MPU6050_ACCEL_ZOUT_L_ACCEL_ZOUT_Pos 7
#define MPU6050_ACCEL_ZOUT_L_ACCEL_ZOUT_Len 8

/**
*   TEMP_OUT_H Register
*   R/W
*   Address = 0x41
*/
#define MPU6050_TEMP_OUT_H_TEMP_OUT_Pos 7
#define MPU6050_TEMP_OUT_H_TEMP_OUT_Len 8

/**
*   TEMP_OUT_L Register
*   R/W
*   Address = 0x42
*/
#define MPU6050_TEMP_OUT_L_TEMP_OUT_Pos 7
#define MPU6050_TEMP_OUT_L_TEMP_OUT_Len 8

/**
*   GYRO_XOUT_H Register
*   R/W
*   Address = 0x43
*/
#define MPU6050_GYRO_XOUT_H_GYRO_XOUT_Pos 7
#define MPU6050_GYRO_XOUT_H_GYRO_XOUT_Len 8

/**
*   GYRO_XOUT_L Register
*   R/W
*   Address = 0x44
*/
#define MPU6050_GYRO_XOUT_L_GYRO_XOUT_Pos 7
#define MPU6050_GYRO_XOUT_L_GYRO_XOUT_Len 8

/**
*   GYRO_YOUT_H Register
*   R/W
*   Address = 0x45
*/
#define MPU6050_GYRO_YOUT_H_GYRO_YOUT_Pos 7
#define MPU6050_GYRO_YOUT_H_GYRO_YOUT_Len 8

/**
*   GYRO_YOUT_L Register
*   R/W
*   Address = 0x46
*/
#define MPU6050_GYRO_YOUT_L_GYRO_YOUT_Pos 7
#define MPU6050_GYRO_YOUT_L_GYRO_YOUT_Len 8

/**
*   GYRO_ZOUT_H Register
*   R/W
*   Address = 0x47
*/
#define MPU6050_GYRO_ZOUT_H_GYRO_ZOUT_Pos 7
#define MPU6050_GYRO_ZOUT_H_GYRO_ZOUT_Len 8

/**
*   GYRO_ZOUT_L Register
*   R/W
*   Address = 0x48
*/
#define MPU6050_GYRO_ZOUT_L_GYRO_ZOUT_Pos 7
#define MPU6050_GYRO_ZOUT_L_GYRO_ZOUT_Len 8

/**
*   EXT_SENS_DATA_00 Register
*   R/W
*   Address = 0x49
*/
#define MPU6050_EXT_SENS_DATA_00_EXT_SENS_DATA_00_Pos 7
#define MPU6050_EXT_SENS_DATA_00_EXT_SENS_DATA_00_Len 8

/**
*   EXT_SENS_DATA_02 Register
*   R/W
*   Address = 0x4A
*/
#define MPU6050_EXT_SENS_DATA_01_EXT_SENS_DATA_01_Pos 7
#define MPU6050_EXT_SENS_DATA_01_EXT_SENS_DATA_01_Len 8

/**
*   EXT_SENS_DATA_02 Register
*   R/W
*   Address = 0x4B
*/
#define MPU6050_EXT_SENS_DATA_02_EXT_SENS_DATA_02_Pos 7
#define MPU6050_EXT_SENS_DATA_02_EXT_SENS_DATA_02_Len 8

/**
*   EXT_SENS_DATA_03 Register
*   R/W
*   Address = 0x4C
*/
#define MPU6050_EXT_SENS_DATA_03_EXT_SENS_DATA_03_Pos 7
#define MPU6050_EXT_SENS_DATA_03_EXT_SENS_DATA_03_Len 8

/**
*   EXT_SENS_DATA_04 Register
*   R/W
*   Address = 0x4D
*/
#define MPU6050_EXT_SENS_DATA_04_EXT_SENS_DATA_04_Pos 7
#define MPU6050_EXT_SENS_DATA_04_EXT_SENS_DATA_04_Len 8

/**
*   EXT_SENS_DATA_05 Register
*   R/W
*   Address = 0x4E
*/
#define MPU6050_EXT_SENS_DATA_05_EXT_SENS_DATA_05_Pos 7
#define MPU6050_EXT_SENS_DATA_05_EXT_SENS_DATA_05_Len 8

/**
*   EXT_SENS_DATA_06 Register
*   R/W
*   Address = 0x4F
*/
#define MPU6050_EXT_SENS_DATA_06_EXT_SENS_DATA_06_Pos 7
#define MPU6050_EXT_SENS_DATA_06_EXT_SENS_DATA_06_Len 8

/**
*   EXT_SENS_DATA_07 Register
*   R/W
*   Address = 0x50
*/
#define MPU6050_EXT_SENS_DATA_07_EXT_SENS_DATA_07_Pos 7
#define MPU6050_EXT_SENS_DATA_07_EXT_SENS_DATA_07_Len 8

/**
*   EXT_SENS_DATA_08 Register
*   R/W
*   Address = 0x51
*/
#define MPU6050_EXT_SENS_DATA_08_EXT_SENS_DATA_08_Pos 7
#define MPU6050_EXT_SENS_DATA_08_EXT_SENS_DATA_08_Len 8

/**
*   EXT_SENS_DATA_09 Register
*   R/W
*   Address = 0x52
*/
#define MPU6050_EXT_SENS_DATA_09_EXT_SENS_DATA_09_Pos 7
#define MPU6050_EXT_SENS_DATA_09_EXT_SENS_DATA_09_Len 8

/**
*   EXT_SENS_DATA_10 Register
*   R/W
*   Address = 0x53
*/
#define MPU6050_EXT_SENS_DATA_10_EXT_SENS_DATA_10_Pos 7
#define MPU6050_EXT_SENS_DATA_10_EXT_SENS_DATA_10_Len 8

/**
*   EXT_SENS_DATA_11 Register
*   R/W
*   Address = 0x54
*/
#define MPU6050_EXT_SENS_DATA_11_EXT_SENS_DATA_11_Pos 7
#define MPU6050_EXT_SENS_DATA_11_EXT_SENS_DATA_11_Len 8

/**
*   EXT_SENS_DATA_12 Register
*   R/W
*   Address = 0x55
*/
#define MPU6050_EXT_SENS_DATA_12_EXT_SENS_DATA_12_Pos 7
#define MPU6050_EXT_SENS_DATA_12_EXT_SENS_DATA_12_Len 8

/**
*   EXT_SENS_DATA_13 Register
*   R/W
*   Address = 0x56
*/
#define MPU6050_EXT_SENS_DATA_13_EXT_SENS_DATA_13_Pos 7
#define MPU6050_EXT_SENS_DATA_13_EXT_SENS_DATA_13_Len 8

/**
*   EXT_SENS_DATA_14 Register
*   R/W
*   Address = 0x57
*/
#define MPU6050_EXT_SENS_DATA_14_EXT_SENS_DATA_14_Pos 7
#define MPU6050_EXT_SENS_DATA_14_EXT_SENS_DATA_14_Len 8

/**
*   EXT_SENS_DATA_15 Register
*   R/W
*   Address = 0x58
*/
#define MPU6050_EXT_SENS_DATA_15_EXT_SENS_DATA_15_Pos 7
#define MPU6050_EXT_SENS_DATA_15_EXT_SENS_DATA_15_Len 8

/**
*   EXT_SENS_DATA_16 Register
*   R/W
*   Address = 0x59
*/
#define MPU6050_EXT_SENS_DATA_16_EXT_SENS_DATA_16_Pos 7
#define MPU6050_EXT_SENS_DATA_16_EXT_SENS_DATA_16_Len 8

/**
*   EXT_SENS_DATA_17 Register
*   R/W
*   Address = 0x5A
*/
#define MPU6050_EXT_SENS_DATA_17_EXT_SENS_DATA_17_Pos 7
#define MPU6050_EXT_SENS_DATA_17_EXT_SENS_DATA_17_Len 8

/**
*   EXT_SENS_DATA_18 Register
*   R/W
*   Address = 0x5B
*/
#define MPU6050_EXT_SENS_DATA_18_EXT_SENS_DATA_18_Pos 7
#define MPU6050_EXT_SENS_DATA_18_EXT_SENS_DATA_18_Len 8

/**
*   EXT_SENS_DATA_19 Register
*   R/W
*   Address = 0x5C
*/
#define MPU6050_EXT_SENS_DATA_19_EXT_SENS_DATA_19_Pos 7
#define MPU6050_EXT_SENS_DATA_19_EXT_SENS_DATA_19_Len 8

/**
*   EXT_SENS_DATA_20 Register
*   R/W
*   Address = 0x5D
*/
#define MPU6050_EXT_SENS_DATA_20_EXT_SENS_DATA_20_Pos 7
#define MPU6050_EXT_SENS_DATA_20_EXT_SENS_DATA_20_Len 8

/**
*   EXT_SENS_DATA_21 Register
*   R/W
*   Address = 0x5E
*/
#define MPU6050_EXT_SENS_DATA_21_EXT_SENS_DATA_21_Pos 7
#define MPU6050_EXT_SENS_DATA_21_EXT_SENS_DATA_21_Len 8

/**
*   EXT_SENS_DATA_22 Register
*   R/W
*   Address = 0x5F
*/
#define MPU6050_EXT_SENS_DATA_22_EXT_SENS_DATA_22_Pos 7
#define MPU6050_EXT_SENS_DATA_22_EXT_SENS_DATA_22_Len 8

/**
*   EXT_SENS_DATA_23 Register
*   R/W
*   Address = 0x60
*/
#define MPU6050_EXT_SENS_DATA_23_EXT_SENS_DATA_23_Pos 7
#define MPU6050_EXT_SENS_DATA_23_EXT_SENS_DATA_23_Len 8

/**
*   I2C_SLV0_DO Register
*   R/W
*   Address = 0x63
*/
#define MPU6050_I2C_SLV0_DO_I2C_SLV0_DO_Pos 7
#define MPU6050_I2C_SLV0_DO_I2C_SLV0_DO_Len 8

/**
*   I2C_SLV1_DO Register
*   R/W
*   Address = 0x64
*/
#define MPU6050_I2C_SLV1_DO_I2C_SLV1_DO_Pos 7
#define MPU6050_I2C_SLV1_DO_I2C_SLV1_DO_Len 8

/**
*   I2C_SLV2_DO Register
*   R/W
*   Address = 0x65
*/
#define MPU6050_I2C_SLV2_DO_I2C_SLV2_DO_Pos 7
#define MPU6050_I2C_SLV2_DO_I2C_SLV2_DO_Len 8

/**
*   I2C_SLV3_DO Register
*   R/W
*   Address = 0x66
*/
#define MPU6050_I2C_SLV3_DO_I2C_SLV3_DO_Pos 7
#define MPU6050_I2C_SLV3_DO_I2C_SLV3_DO_Len 8

/**
*   I2C_MST_DELAY_CT_RL Register
*   R/W
*   Address = 0x67
*/
#define MPU6050_I2C_MST_DELAY_CT_RL_DELAY_ES_SHADOW_Pos 7
#define MPU6050_I2C_MST_DELAY_CT_RL_DELAY_ES_SHADOW_Len 1
#define MPU6050_I2C_MST_DELAY_CT_RL_I2C_SLV4_DLY_EN_Pos 4
#define MPU6050_I2C_MST_DELAY_CT_RL_I2C_SLV4_DLY_EN_Len 1
#define MPU6050_I2C_MST_DELAY_CT_RL_I2C_SLV3_DLY_EN_Pos 3
#define MPU6050_I2C_MST_DELAY_CT_RL_I2C_SLV3_DLY_EN_Len 1
#define MPU6050_I2C_MST_DELAY_CT_RL_I2C_SLV2_DLY_EN_Pos 2
#define MPU6050_I2C_MST_DELAY_CT_RL_I2C_SLV2_DLY_EN_Len 1
#define MPU6050_I2C_MST_DELAY_CT_RL_I2C_SLV1_DLY_EN_Pos 1
#define MPU6050_I2C_MST_DELAY_CT_RL_I2C_SLV1_DLY_EN_Len 1
#define MPU6050_I2C_MST_DELAY_CT_RL_I2C_SLV0_DLY_EN_Pos 0
#define MPU6050_I2C_MST_DELAY_CT_RL_I2C_SLV0_DLY_EN_Len 1

/**
*   SIGNAL_PATH_RESET Register
*   R/W
*   Address = 0x68
*/
#define MPU6050_SIGNAL_PATH_RESET_GYRO_RESET_Pos 2
#define MPU6050_SIGNAL_PATH_RESET_GYRO_RESET_Len 1
#define MPU6050_SIGNAL_PATH_RESET_ACCEL_RESET_Pos 1
#define MPU6050_SIGNAL_PATH_RESET_ACCEL_RESET_Len 1
#define MPU6050_SIGNAL_PATH_RESET_TEMP_RESET_Pos 0
#define MPU6050_SIGNAL_PATH_RESET_TEMP_RESET_Len 1

/**
*   MOT_DETECT_CTRL Register
*   R/W
*   Address = 0x69
*/
#define MPU6050_MOT_DETECT_CTRL_ACCEL_ON_DELAY_Pos 5
#define MPU6050_MOT_DETECT_CTRL_ACCEL_ON_DELAY_Len 2
#define MPU6050_MOT_DETECT_CTRL_FF_COUNT_Pos 3 /* Undocumented */
#define MPU6050_MOT_DETECT_CTRL_FF_COUNT_Len 2
#define MPU6050_MOT_DETECT_CTRL_MOT_COUNT_Pos 1 /* Undocumented */
#define MPU6050_MOT_DETECT_CTRL_MOT_COUNT_Len 2

/**
*   USER_CTRL Register
*   R/W
*   Address = 0x6A
*/
#define MPU6050_USER_CTRL_DMP_EN_Pos 7 /* Undocumented */
#define MPU6050_USER_CTRL_DMP_EN_Len 1
#define MPU6050_USER_CTRL_FIFO_EN_Pos 6
#define MPU6050_USER_CTRL_FIFO_EN_Len 1
#define MPU6050_USER_CTRL_I2C_MST_EN_Pos 5
#define MPU6050_USER_CTRL_I2C_MST_EN_Len 1
#define MPU6050_USER_CTRL_I2C_IF_DIS_Pos 4
#define MPU6050_USER_CTRL_I2C_IF_DIS_Len 1
#define MPU6050_USER_CTRL_DMP_RESET_Pos 3 /* Undocumented */
#define MPU6050_USER_CTRL_DMP_RESET_Len 1
#define MPU6050_USER_CTRL_FIFO_RESET_Pos 2
#define MPU6050_USER_CTRL_FIFO_RESET_Len 1
#define MPU6050_USER_CTRL_I2C_MST_RESET_Pos 1
#define MPU6050_USER_CTRL_I2C_MST_RESET_Len 1
#define MPU6050_USER_CTRL_SIG_COND_RESET_Pos 0
#define MPU6050_USER_CTRL_SIG_COND_RESET_Len 1

/**
*   PWR_MGMT_1 Register
*   R/W
*   Address = 0x6B
*   Default value = 0x40
*/
#define MPU6050_PWR_MGMT_1_DEVICE_RESET_Pos 7
#define MPU6050_PWR_MGMT_1_DEVICE_RESET_Len 1
#define MPU6050_PWR_MGMT_1_SLEEP_Pos 6
#define MPU6050_PWR_MGMT_1_SLEEP_Len 1
#define MPU6050_PWR_MGMT_1_CYCLE_Pos 5
#define MPU6050_PWR_MGMT_1_CYCLE_Len 1
#define MPU6050_PWR_MGMT_1_TEMP_DIS_Pos 3
#define MPU6050_PWR_MGMT_1_TEMP_DIS_Len 1
#define MPU6050_PWR_MGMT_1_CLK_SEL_Pos 2
#define MPU6050_PWR_MGMT_1_CLK_SEL_Len 3
#define MPU6050_PWR_MGMT_1_CLK_SEL_INTERNAL 0
#define MPU6050_PWR_MGMT_1_CLK_SEL_XAXIS 1
#define MPU6050_PWR_MGMT_1_CLK_SEL_YAXIS 2
#define MPU6050_PWR_MGMT_1_CLK_SEL_ZAXIS 3
#define MPU6050_PWR_MGMT_1_CLK_SEL_EXTERNAL_32KHZ 4
#define MPU6050_PWR_MGMT_1_CLK_SEL_EXTERNAL_192MHZ 5
#define MPU6050_PWR_MGMT_1_CLK_SEL_RESERVED 6
#define MPU6050_PWR_MGMT_1_CLK_SEL_STOP 7

/**
*   PWR_MGMT_2 Register
*   R/W
*   Address = 0x6C
*/
#define MPU6050_PWR_MGMT_2_LP_WAKE_CTRL_Pos 7
#define MPU6050_PWR_MGMT_2_LP_WAKE_CTRL_Len 2
#define MPU6050_PWR_MGMT_2_STBY_XA_Pos 5
#define MPU6050_PWR_MGMT_2_STBY_XA_Len 1
#define MPU6050_PWR_MGMT_2_STBY_YA_Pos 4
#define MPU6050_PWR_MGMT_2_STBY_YA_Len 1
#define MPU6050_PWR_MGMT_2_STBY_ZA_Pos 3
#define MPU6050_PWR_MGMT_2_STBY_ZA_Len 1
#define MPU6050_PWR_MGMT_2_STBY_XG_Pos 2
#define MPU6050_PWR_MGMT_2_STBY_XG_Len 1
#define MPU6050_PWR_MGMT_2_STBY_YG_Pos 1
#define MPU6050_PWR_MGMT_2_STBY_YG_Len 1
#define MPU6050_PWR_MGMT_2_STBY_ZG_Pos 0
#define MPU6050_PWR_MGMT_2_STBY_ZG_Len 1
#define MPU6050_PWR_MGMT_2_LP_WAKE_CTRL_12_5HZ 0
#define MPU6050_PWR_MGMT_2_LP_WAKE_CTRL_5HZ 1
#define MPU6050_PWR_MGMT_2_LP_WAKE_CTRL_20HZ 2
#define MPU6050_PWR_MGMT_2_LP_WAKE_CTRL_40HZ 3

/**
*   FIFO_COUNTH Register
*   R/W
*   Address = 0x72
*/
#define MPU6050_FIFO_COUNTH_FIFO_COUNT_Pos 7
#define MPU6050_FIFO_COUNTH_FIFO_COUNT_Len 8

/**
*   FIFO_COUNTL Register
*   R/W
*   Address = 0x73
*/
#define MPU6050_FIFO_COUNTL_FIFO_COUNT_Pos 7
#define MPU6050_FIFO_COUNTL_FIFO_COUNT_Len 8


/**
*   FIFO_R_W Register
*   R/W
*   Address = 0x74
*/
#define MPU6050_FIFO_R_W_FIFO_DATA_Pos 7
#define MPU6050_FIFO_R_W_FIFO_DATA_Len 8

/**
*   WHO_AM_I Register
*   R Only
*   Address = 0x75
*   Default value = 0x68
*/
#define MPU6050_WHO_AM_I_WHO_AM_I_Pos 6
#define MPU6050_WHO_AM_I_WHO_AM_I_Len 6

/* Undocumented registers */
#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16
    
#endif

/* [] END OF FILE */
