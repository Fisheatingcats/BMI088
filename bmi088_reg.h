/*
 * @Desc:
 * @Author: LIUBIN
 * @version: 0.1
 * @Date: 2024-12-28 22:38:00
 */
#ifndef BMI088_REG_H
#define BMI088_REG_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#ifndef DRV_BYTE_ORDER
#ifndef __BYTE_ORDER__

#define DRV_LITTLE_ENDIAN 1234 /* little endian:小端序列*/
#define DRV_BIG_ENDIAN 4321    /* big endian */

/** if _BYTE_ORDER is not defined, choose the endianness of your architecture
 * by uncommenting the define which fits your platform endianness
 */
// #define DRV_BYTE_ORDER    DRV_BIG_ENDIAN
#define DRV_BYTE_ORDER DRV_LITTLE_ENDIAN

#else /* defined __BYTE_ORDER__ */

#define DRV_LITTLE_ENDIAN __ORDER_LITTLE_ENDIAN__
#define DRV_BIG_ENDIAN __ORDER_BIG_ENDIAN__
#define DRV_BYTE_ORDER __BYTE_ORDER__

#endif /* __BYTE_ORDER__*/
#endif /* DRV_BYTE_ORDER */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t bit0 : 1;
    uint8_t bit1 : 1;
    uint8_t bit2 : 1;
    uint8_t bit3 : 1;
    uint8_t bit4 : 1;
    uint8_t bit5 : 1;
    uint8_t bit6 : 1;
    uint8_t bit7 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
    uint8_t bit7 : 1;
    uint8_t bit6 : 1;
    uint8_t bit5 : 1;
    uint8_t bit4 : 1;
    uint8_t bit3 : 1;
    uint8_t bit2 : 1;
    uint8_t bit1 : 1;
    uint8_t bit0 : 1;
#endif /* DRV_BYTE_ORDER */
} bitwise_t;

/** @addtogroup  Interfaces_Functions
 * @brief       This section provide a set of functions used to read and
 *              write a generic register of the device.
 *              MANDATORY: return 0 -> no Error.
 * @{
 *
 */

typedef int32_t (*dev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*dev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef void (*dev_mdelay_ptr)(uint32_t millisec);

typedef struct
{
    /** Component mandatory fields **/
    dev_write_ptr write_reg;
    dev_read_ptr read_reg;
    /** Component optional fields **/
    dev_mdelay_ptr mdelay;
    /** Customizable optional pointer **/
    void *handle;
} dev_ctx_t;
#endif /* MEMS_SHARED_TYPES */
#define ACC_CHIP_ID 0x1E
#define GYRO_CHIP_ID 0x0F

typedef enum
{
    ACC_CHIP_ID_REG = 0x00,
    ACC_ERR_REG = 0x02,
    ACC_STATUS_REG = 0x03,
    ACC_X_LSB_REG = 0x12,
    ACC_X_MSB_REG = 0x13,
    ACC_Y_LSB_REG = 0x14,
    ACC_Y_MSB_REG = 0x15,
    ACC_Z_LSB_REG = 0x16,
    ACC_Z_MSB_REG = 0x17,
    ACC_INIT_STAT_1_REG = 0x1D,
    TEMP_MSB_REG = 0x22,
    TEMP_LSB_REG = 0x23,
    ACC_CONF_REG = 0x40,
    ACC_RANGE_REG = 0x41,
    INT1_IO_CTRL_REG = 0x53,
    INT2_IO_CTRL_REG = 0x54,
    ACC_SELF_TEST_REG = 0x6D,
    ACC_PWR_CONF_REG = 0x7C,
    ACC_PWR_CTRL_REG = 0x7D,
    ACC_SOFTRESET_REG = 0x7E
} bmi088a_reg_list_t;

int32_t bmi088_acc_softreset(const dev_ctx_t *ctx);
int32_t bmi088_acc_id_get(const dev_ctx_t *ctx, uint8_t *buff);
int32_t bmi088_acc_err_rge_get(const dev_ctx_t *ctx, uint8_t *buff);
int32_t bmi088_acc_status_get(const dev_ctx_t *ctx, uint8_t *buff);
int32_t bmi088_acc_raw_get(const dev_ctx_t *ctx, int *val);
typedef enum
{
    ACC_DISABLE = 0,
    ACC_ENABLE = 1

} bmi088_acc_power_mode_t;
int32_t bmi088_acc_power_mode_set(const dev_ctx_t *ctx, uint8_t mode);

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t acc_odr : 4;
    uint8_t acc_bwp : 3;
    uint8_t reserved : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
    uint8_t reserved : 1;
    uint8_t acc_bwp : 3;
    uint8_t acc_odr : 4;
#endif /* DRV_BYTE_ORDER */
} bmi088_acc_conf_reg_t;

typedef enum
{
    ACC_DATA_RATE_12_5HZ = 0x05,
    ACC_DATA_RATE_25HZ,
    ACC_DATA_RATE_50HZ,
    ACC_DATA_RATE_100HZ,
    ACC_DATA_RATE_200HZ,
    ACC_DATA_RATE_400HZ,
    ACC_DATA_RATE_800HZ,
    ACC_DATA_RATE_1600HZ
} bmi088_acc_out_rate_t;

typedef enum
{
    OSR4 = 0x00,
    OSR2,
    NORMAL
} bmi088_acc_bandwith_t;

int32_t bmi088_acc_conf_get(const dev_ctx_t *ctx, uint8_t *Rate, uint8_t *Bandwidth);
int32_t bmi088_acc_conf_set(const dev_ctx_t *ctx, uint8_t Rate, uint8_t Bandwidth);

typedef enum
{
    ACC_RANGE_3G = 0x00,
    ACC_RANGE_6G,
    ACC_RANGE_12G,
    ACC_RANGE_24G
} bmi088_acc_range_t;

int32_t bmi088_acc_range_set(const dev_ctx_t *ctx, uint8_t range);
int32_t bmi088_acc_sensitivity_get(const dev_ctx_t *ctx, float *const acce_sensitivity);
int32_t bmi088_acc_get(const dev_ctx_t *ctx, float *val);
int32_t bmi088_temperature_get(const dev_ctx_t *ctx, float *val);

typedef enum
{
    GYRO_CHIP_ID_REG = 0x00,
    GYRO_X_LSB_REG = 0x02,
    GYRO_X_MSB_REG = 0x03,
    GYRO_Y_LSB_REG = 0x04,
    GYRO_Y_MSB_REG = 0x05,
    GYRO_Z_LSB_REG = 0x06,
    GYRO_Z_MSB_REG = 0x07,
    GYRO_INT_STAT_1_REG = 0x0A,
    GYRO_RANGE_REG = 0x0F,
    GYRO_BANDWIDTH_REG = 0x10,
    GYRO_LPM1_REG = 0x11,
    GYRO_SOFTRESET_REG = 0x14,
    GYRO_INT_CTRL_REG = 0x15
} bmi088g_reg_list_t;

int32_t bmi088_gyro_softreset(const dev_ctx_t *ctx);
int32_t bmi088_gyro_id_get(const dev_ctx_t *ctx, uint8_t *buff);
int32_t bmi088_gyro_status_get(const dev_ctx_t *ctx, uint8_t *buff);

typedef enum
{
    GYRO_ODR_2000HZ_BW_532HZ = 0x00,
    GYRO_ODR_2000HZ_BW_230HZ,
    GYRO_ODR_1000HZ_BW_116HZ,
    GYRO_ODR_400HZ_BW_47HZ,
    GYRO_ODR_200HZ_BW_23HZ,
    GYRO_ODR_100HZ_BW_12HZ,
    GYRO_ODR_200HZ_BW_64HZ,
    GYRO_ODR_100HZ_BW_32HZ
} bmi088_gyro_outrate_bandwidth_t;

int32_t bmi088_gyro_conf_get(const dev_ctx_t *ctx, uint8_t *val);
int32_t bmi088_gyro_conf_set(const dev_ctx_t *ctx, uint8_t val);

typedef enum
{
    GYRO_RANGE_2000_DPS = 0x00,
    GYRO_RANGE_1000_DPS,
    GYRO_RANGE_500_DPS,
    GYRO_RANGE_250_DPS,
    GYRO_RANGE_125_DPS
} bmi088_gyro_range_t;

int32_t bmi088_gyro_range_set(const dev_ctx_t *ctx, uint8_t range);
int32_t bmi088_gyro_sensitivity_get(const dev_ctx_t *ctx, float *const gyro_sensitivity);
int32_t bmi088_gyro_raw_get(const dev_ctx_t *ctx, int *val);
int32_t bmi088_gyro_get(const dev_ctx_t *ctx, float *val);


#endif
