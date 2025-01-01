# BMI088

BMI088 driver

SPI接口使用流程:（已测试，均正常）

1.在bmi088_port.c中添加头文件

```
/* cubemx init */
#include "spi.h"
#include "usart.h"
#include "main.h"
#include "rtthread.h"
```

2.修改BMI088 CS引脚相关定义：

BMI088_CS1_GPIO_Port BMI088_CS1_Pin

BMI088_CS2_GPIO_Port BMI088_CS2_Pin

3.修改到对应的spi handle

#defineBUS_HANDLE hspi2

4.找到/* bmi088 demo */这行注释，将对应的函数或句柄初始化。

```
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
```

注意:

1.bmi088  的加速度计和陀螺仪是作为两个spi设备拥有独立的chip_id。

2.关于数据单位

int32_tbmi088_acc_get(constdev_ctx_t*ctx,float*val);//获取到的数据单位为g

int32_tbmi088_gyro_get(constdev_ctx_t*ctx,float*val);//获取到的数据单位为 dps

3.如果使用I2C接口，只需要将函数 acc_read、acc_write、gyro_read、acc_write中的SPI接口修改成I2C接口
