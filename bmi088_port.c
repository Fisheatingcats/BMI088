/*
 * @Desc:
 * @Author: LIUBIN
 * @version: 0.1
 * @Date: 2024-12-28 22:41:11
 */

#include "bmi088_port.h"

#include <string.h>
#include <stdio.h>
#include "bmi088_reg.h"

/* cubemx init */
#include "spi.h"
#include "usart.h"
#include "main.h"
#include "rtthread.h"


#define BUS_HANDLE hspi2
#define TIME_OUT 100

#define BOOT_TIME 15 // ms
#define TX_BUF_DIM 1000

static int32_t acc_write(void *handle, uint8_t reg, const uint8_t *bufp,
						 uint16_t len);
static int32_t acc_read(void *handle, uint8_t reg, uint8_t *bufp,
						uint16_t len);

static int32_t gyro_write(void *handle, uint8_t reg, const uint8_t *bufp,
						  uint16_t len);
static int32_t gyro_read(void *handle, uint8_t reg, uint8_t *bufp,
						 uint16_t len);

static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

/*
 * @brief  Write acc device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t acc_write(void *handle, uint8_t reg, const uint8_t *bufp,
						 uint16_t len)
{
	// HAL_I2C_Mem_Write(handle, LSM6DS3TR_C_I2C_ADD_H, reg,
	//                   I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000);
	// HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

	reg &= 0x7F;
	HAL_GPIO_WritePin(BMI088_CS1_GPIO_Port, BMI088_CS1_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(handle, &reg, 1, TIME_OUT);
	HAL_SPI_Transmit(handle, (uint8_t *)bufp, len, TIME_OUT);
	HAL_GPIO_WritePin(BMI088_CS1_GPIO_Port, BMI088_CS1_Pin, GPIO_PIN_SET);
	return 0;
}

/*
 * @brief  Read acc device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t acc_read(void *handle, uint8_t reg, uint8_t *bufp,
						uint16_t len)
{
	reg |= 0x80;

	HAL_GPIO_WritePin(BMI088_CS1_GPIO_Port, BMI088_CS1_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(handle, &reg, 1, TIME_OUT);
	HAL_SPI_Receive(handle, bufp, len, TIME_OUT);
	HAL_GPIO_WritePin(BMI088_CS1_GPIO_Port, BMI088_CS1_Pin, GPIO_PIN_SET);

	// HAL_I2C_Mem_Read(handle, LSM6DS3TR_C_I2C_ADD_H, reg,
	//                  I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

static int32_t gyro_write(void *handle, uint8_t reg, const uint8_t *bufp,
						  uint16_t len)
{
	// HAL_I2C_Mem_Write(handle, LSM6DS3TR_C_I2C_ADD_H, reg,
	//                   I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000);
	// HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

	reg &= 0x7F;
	HAL_GPIO_WritePin(BMI088_CS2_GPIO_Port, BMI088_CS2_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(handle, &reg, 1, TIME_OUT);
	HAL_SPI_Transmit(handle, (uint8_t *)bufp, len, TIME_OUT);
	HAL_GPIO_WritePin(BMI088_CS2_GPIO_Port, BMI088_CS2_Pin, GPIO_PIN_SET);
	return 0;
}

/*
 * @brief  Read acc device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t gyro_read(void *handle, uint8_t reg, uint8_t *bufp,
						 uint16_t len)
{
	reg |= 0x80;

	HAL_GPIO_WritePin(BMI088_CS2_GPIO_Port, BMI088_CS2_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(handle, &reg, 1, TIME_OUT);
	HAL_SPI_Receive(handle, bufp, len, TIME_OUT);
	HAL_GPIO_WritePin(BMI088_CS2_GPIO_Port, BMI088_CS2_Pin, GPIO_PIN_SET);

	// HAL_I2C_Mem_Read(handle, LSM6DS3TR_C_I2C_ADD_H, reg,
	//                  I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
	HAL_UART_Transmit(&huart1, tx_buffer, len, 1000);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
	HAL_Delay(ms);
	// rt_thread_delay(ms);
}

static void platform_init(void)
{
}

/* bmi088 demo */

dev_ctx_t acc;
void acc_init(void)
{
	acc.handle = &BUS_HANDLE;
	acc.write_reg = acc_write;
	acc.read_reg = acc_read;
	acc.mdelay = platform_delay;

	uint8_t acc_id;
	bmi088_acc_softreset(&acc);
	acc.mdelay(BOOT_TIME);

	bmi088_acc_id_get(&acc, &acc_id);
	bmi088_acc_id_get(&acc, &acc_id);
	printf("bmi088 acc id: 0x%x\r\n", acc_id);

	if (acc_id != ACC_CHIP_ID)
	{
		printf("acc_id error\r\n");
	}

	bmi088_acc_power_mode_set(&acc, ACC_ENABLE); // enable ACC
	bmi088_acc_out_rate_t acc_out_rate;
	bmi088_acc_bandwith_t acc_bandwith;

	bmi088_acc_conf_set(&acc, ACC_DATA_RATE_100HZ, NORMAL);
	bmi088_acc_conf_get(&acc, &acc_out_rate, &acc_bandwith);// set data rate and bandwidth
	printf("Rate: 0x%x, Bandwidth: 0x%x", acc_out_rate, acc_bandwith);

	float acc_sensitivity;
	bmi088_acc_sensitivity_get(&acc, &acc_sensitivity); // get acc sensitivity
	printf("acc sensitivity: %f", acc_sensitivity);

	float temperature;
	bmi088_temperature_get(&acc,&temperature); // get temperature
	printf("temperature: %.1f", temperature);
}
dev_ctx_t gyro;
void gyro_init(void)
{
	gyro.handle = &BUS_HANDLE;
	gyro.write_reg = gyro_write;
	gyro.read_reg = gyro_read;
	gyro.mdelay = platform_delay;

	uint8_t gyro_id;

	bmi088_gyro_softreset(&gyro);
	gyro.mdelay(BOOT_TIME);

	bmi088_gyro_id_get(&gyro,&gyro_id);
	printf("bmi088 gyro id: 0x%2x\r\n", gyro_id);
	if(gyro_id == GYRO_CHIP_ID)
	{
		printf("gyro init ok\r\n");
	}
	uint8_t gyro_out_rate;
	bmi088_gyro_conf_set(&gyro,GYRO_ODR_100HZ_BW_32HZ);
	bmi088_gyro_conf_get(&gyro,&gyro_out_rate);
	printf("gyro_out_rate: 0x%2x\r\n", gyro_out_rate);
}

float acc_data[3];
float gyro_data[3];
void bmi088_task(void)
{
	acc_init();
	gyro_init();
	while (1)
	{
		bmi088_acc_get(&acc, acc_data);
		printf("%.2f, %.2f, %.2f,", acc_data[0]*9.81, acc_data[1]*9.81, acc_data[2]*9.81);

		bmi088_gyro_get(&gyro, gyro_data);
		printf("%.2f, %.2f, %.2f\n", gyro_data[0], gyro_data[1], gyro_data[2]);
		HAL_Delay(10);
	}
}