/*
 * @Desc:
 * @Author: LIUBIN
 * @version: 0.1
 * @Date: 2024-12-28 22:38:11
 */
#include "bmi088_reg.h"

#define ACC_SOFT_REST_CMD 0xB6
#define GYRO_SOFT_REST_CMD 0xB6

#define TEMP_LSB_REG 0x23 // temperature lsb register : temperature[2:0]
#define TEMP_MSB_REG 0x22 // temperature msb register : temperature[10:3]

#define SENSORTIME_2_REG 0x1A // sensortime 2 register : sensortime[23:16]
#define SENSORTIME_1_REG 0x19 // sensortime 1 register : sensortime[15:8]
#define SENSORTIME_0_REG 0x18 // sensortime 1 register : sensortime[15:8]

/* CMD: accel power save */
#define ACC_PWR_ACTIVE_CMD 0x00
#define ACC_PWR_SUSPEND_CMD 0x03

/* CMD: accel power control */
#define ACC_POWER_DISABLE_CMD 0x00
#define ACC_POWER_ENABLE_CMD 0x04

/**
 * @brief  Read generic device register
 *
 * @param  ctx   read / write interface definitions(ptr)
 * @param  reg   register to read
 * @param  data  pointer to buffer that store the data read(ptr)
 * @param  len   number of consecutive register to read
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */

int32_t __weak bmi088_read_reg(const dev_ctx_t *ctx, uint8_t reg,
                               uint8_t *data,
                               uint16_t len)
{
    int32_t ret;

    if (ctx == NULL)
    {
        return -1;
    }

    ret = ctx->read_reg(ctx->handle, reg, data, len);

    return ret;
}

/**
 * @brief  Write generic device register
 *
 * @param  ctx   read / write interface definitions(ptr)
 * @param  reg   register to write
 * @param  data  pointer to data to write in register reg(ptr)
 * @param  len   number of consecutive register to write
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t __weak bmi088_write_reg(const dev_ctx_t *ctx, uint8_t reg,
                                uint8_t *data,
                                uint16_t len)
{
    int32_t ret;

    if (ctx == NULL)
    {
        return -1;
    }

    ret = ctx->write_reg(ctx->handle, reg, data, len);

    return ret;
}

int32_t bmi088_acc_softreset(const dev_ctx_t *ctx)
{
    int32_t ret;
    uint8_t cmd;
    cmd = ACC_SOFT_REST_CMD;
    ret = bmi088_write_reg(ctx, ACC_SOFTRESET_REG, &cmd, 1);
    return ret;
}

int32_t bmi088_acc_id_get(const dev_ctx_t *ctx, uint8_t *buff)
{
    int32_t ret;
    uint8_t temp[2];
    ret = bmi088_read_reg(ctx, ACC_CHIP_ID_REG, temp, 2);
    *buff = temp[1];
    return ret;
}

int32_t bmi088_acc_err_rge_get(const dev_ctx_t *ctx, uint8_t *buff)
{
    int32_t ret;
    uint8_t temp[2];
    ret = bmi088_read_reg(ctx, ACC_ERR_REG, temp, 2);
    return ret;
}
int32_t bmi088_acc_status_get(const dev_ctx_t *ctx, uint8_t *buff)
{
    int32_t ret;
    uint8_t temp[2];
    ret = bmi088_read_reg(ctx, ACC_STATUS_REG, temp, 2);
    return ret;
}

int32_t bmi088_acc_power_mode_set(const dev_ctx_t *ctx, uint8_t mode)
{
    int32_t ret;
    uint8_t cmd[2];

    if (mode)
    {
        cmd[0] = ACC_PWR_ACTIVE_CMD;
        cmd[1] = ACC_POWER_ENABLE_CMD;
    }
    else
    {
        cmd[0] = ACC_PWR_SUSPEND_CMD;
        cmd[1] = ACC_POWER_DISABLE_CMD;
    }
    ret = bmi088_write_reg(ctx, ACC_PWR_CONF_REG, &cmd[0], 1);
    ctx->mdelay(1);
    ret = bmi088_write_reg(ctx, ACC_PWR_CTRL_REG, &cmd[1], 1);

    return ret;
}

int32_t bmi088_acc_conf_get(const dev_ctx_t *ctx, uint8_t *Rate, uint8_t *Bandwidth)
{
    int32_t ret;
    uint8_t data[2];
    ret = bmi088_read_reg(ctx, ACC_CONF_REG, data, 2);
    bmi088_acc_conf_reg_t *acc_conf = (bmi088_acc_conf_reg_t *)&data[1];

    *Rate = acc_conf->acc_odr;
    *Bandwidth = acc_conf->acc_bwp;

    return ret;
}

int32_t bmi088_acc_conf_set(const dev_ctx_t *ctx, uint8_t Rate, uint8_t Bandwidth)
{
    int32_t ret;

    bmi088_acc_conf_reg_t acc_conf;
    acc_conf.reserved = 1;
    acc_conf.acc_odr = Rate;
    acc_conf.acc_bwp = Bandwidth;

    ret = bmi088_write_reg(ctx, ACC_CONF_REG, (uint8_t *)&acc_conf, 1);

    return ret;
}

int32_t bmi088_acc_raw_get(const dev_ctx_t *ctx, int *val)
{
    int8_t buff[7];
    int32_t ret;

    ret = bmi088_read_reg(ctx, ACC_X_LSB_REG, (uint8_t *)buff, 7);

    val[0] = (int)((buff[2] * 256) + (int)buff[1]);
    val[1] = (int)((buff[4] * 256) + (int)buff[3]);
    val[2] = (int)((buff[6] * 256) + (int)buff[5]);

    return ret;
}

int32_t bmi088_acc_range_set(const dev_ctx_t *ctx, uint8_t range)
{
    int32_t ret;
    ret = bmi088_write_reg(ctx, ACC_RANGE_REG, &range, 1);
    return ret;
}

int32_t bmi088_acc_sensitivity_get(const dev_ctx_t *ctx, float *const acce_sensitivity)
{
    int32_t ret;
    uint8_t temp[2];
    ret = bmi088_read_reg(ctx, ACC_RANGE_REG, temp, 2);

    switch (temp[1])
    {
    case ACC_RANGE_3G:
        *acce_sensitivity = 10920;
        break;

    case ACC_RANGE_6G:
        *acce_sensitivity = 5460;
        break;

    case ACC_RANGE_12G:
        *acce_sensitivity = 2730;
        break;

    case ACC_RANGE_24G:
        *acce_sensitivity = 1365;
        break;

    default:
        break;
    }
    return ret;
}

int32_t bmi088_acc_get(const dev_ctx_t *ctx, float *val)
{
    int32_t ret;
    float acc_sensitivity;
    int acc_raw_data[3];
    ret = bmi088_acc_sensitivity_get(ctx, &acc_sensitivity);

    ret = bmi088_acc_raw_get(ctx, acc_raw_data);

    val[0] = (float)(acc_raw_data[0] + 6) / acc_sensitivity;
    val[1] = (float)(acc_raw_data[1] - 75) / acc_sensitivity;
    val[2] = (float)(acc_raw_data[2] + 41) / acc_sensitivity;

    return ret;
}

/**
 * @brief: Read temperature data from the BMI088 sensor and convert it to Celsius.
 * @description: This function reads raw temperature data from the BMI088 sensor via I2C or SPI interface
 *               and converts it to a floating-point value in degrees Celsius. The converted temperature is
 *               returned through the pointer parameter.
 *               Temperature calculation formula: temp = (raw_data * 0.125) + 23.
 * @param {const dev_ctx_t *} ctx: Pointer to the device context structure containing configuration information
 *               for I2C/SPI communication.
 * @param {float *} val: Pointer to the float where the converted temperature value will be stored.
 * @return {int32_t}: Operation result, 0 indicates success, non-zero indicates failure.
 */
int32_t bmi088_temperature_get(const dev_ctx_t *ctx, float *val)
{
    int32_t ret;
    uint8_t data[3];

    // Read 3 bytes of temperature register data
    ret = bmi088_read_reg(ctx, TEMP_MSB_REG, data, 3);

    // Combine the higher 8 bits and lower 3 bits to form an 11-bit raw temperature data
    int16_t temp = (data[1] << 3) | (data[2] >> 5);

    // Handle the sign bit of the temperature data
    if (temp > 1023)
        temp = temp - 2048; // Process negative temperature values
    else
        temp = temp + 1024; // Positive temperature offset

    // Convert the raw temperature data to Celsius
    *val = temp * 0.125 + 23;

    return ret;
}

int32_t bmi088_gyro_softreset(const dev_ctx_t *ctx)
{
    int32_t ret;
    uint8_t cmd;
    cmd = GYRO_SOFT_REST_CMD;
    ret = bmi088_write_reg(ctx, GYRO_SOFTRESET_REG, &cmd, 1);
    return ret;
}

int32_t bmi088_gyro_id_get(const dev_ctx_t *ctx, uint8_t *buff)
{
    int32_t ret;
    uint8_t temp[1];
    ret = bmi088_read_reg(ctx, GYRO_CHIP_ID_REG, temp, 1);
    *buff = temp[0];
    return ret;
}

int32_t bmi088_gyro_status_get(const dev_ctx_t *ctx, uint8_t *buff)
{
    int32_t ret;
    ret = bmi088_read_reg(ctx, GYRO_INT_STAT_1_REG, buff, 1);
    return ret;
}
int32_t bmi088_gyro_conf_get(const dev_ctx_t *ctx, uint8_t *val)
{
    int32_t ret;
    uint8_t temp[1];
    ret = bmi088_read_reg(ctx, GYRO_BANDWIDTH_REG, temp, 1);
    *val = temp[0];
    return ret;
}

int32_t bmi088_gyro_conf_set(const dev_ctx_t *ctx, uint8_t val)
{
    int32_t ret;
    ret = bmi088_write_reg(ctx, GYRO_BANDWIDTH_REG, (uint8_t *)&val, 1);
    return ret;
}

int32_t bmi088_gyro_range_set(const dev_ctx_t *ctx, uint8_t range)
{
    int32_t ret;
    ret = bmi088_write_reg(ctx, GYRO_RANGE_REG, &range, 1);
    return ret;
}

int32_t bmi088_gyro_sensitivity_get(const dev_ctx_t *ctx, float *const gyro_sensitivity)
{
    int32_t ret;
    uint8_t temp[1];
    ret = bmi088_read_reg(ctx, GYRO_RANGE_REG, temp, 1);

    switch (temp[0])
    {
    case GYRO_RANGE_2000_DPS:
        *gyro_sensitivity = 16.384;
        break;

    case GYRO_RANGE_1000_DPS:
        *gyro_sensitivity = 32.768;
        break;

    case GYRO_RANGE_500_DPS:
        *gyro_sensitivity = 65.536;
        break;

    case GYRO_RANGE_250_DPS:
        *gyro_sensitivity = 131.072;
        break;
    case GYRO_RANGE_125_DPS:
        *gyro_sensitivity = 262.144;
        break;

    default:
        break;
    }
    return ret;
}

int32_t bmi088_gyro_raw_get(const dev_ctx_t *ctx, int *val)
{
    int8_t buff[6];
    int32_t ret;

    ret = bmi088_read_reg(ctx, GYRO_X_LSB_REG, (uint8_t *)buff, 6);

    val[0] = (int)((buff[1] * 256) + (int)buff[0]);
    val[1] = (int)((buff[3] * 256) + (int)buff[1]);
    val[2] = (int)((buff[5] * 256) + (int)buff[2]);

    return ret;
}

int32_t bmi088_gyro_get(const dev_ctx_t *ctx, float *val)
{
    int32_t ret;
    float gyro_sensitivity;
    ret = bmi088_gyro_sensitivity_get(ctx, &gyro_sensitivity);

    int gyro_raw_data[3];

    ret = bmi088_gyro_raw_get(ctx, gyro_raw_data);

    val[0] = (float)(gyro_raw_data[0] ) / gyro_sensitivity;
    val[1] = (float)(gyro_raw_data[1] ) / gyro_sensitivity;
    val[2] = (float)(gyro_raw_data[2] ) / gyro_sensitivity;

    return ret;
}