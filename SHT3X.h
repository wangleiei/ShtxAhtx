#ifndef __SHT3x_H__
#define __SHT3x_H__

// #include "sys.h"
// #include "stm32l0xx_hal.h"
// #include "stm8l15x.h"
#include <stdint.h>
typedef void (*sht_read)(uint8_t i2caddr,uint16_t regaddr,uint8_t*databuf,uint8_t datalen);
typedef void (*sht_write)(uint8_t i2caddr,uint16_t regaddr,uint8_t*databuf,uint8_t datalen);
// typedef enum {DEV0X44 = 0X44,DEV0X45 = 0X45}SHT3X_DEVADDR;
typedef enum {SHTX = 0,AHTX = 1,HTU21D = 2}SH_DEV_TYPE;
typedef enum {IDLE = 0,TEMPCMD_SENDED = 1,TEMP_GET = 2,RHCMD_SENDED = 3,RH_GET = 4}HTU21D_STA_TYPE;
typedef struct SHT3X_DEV
{
	sht_read read;
	sht_write write;
	/*建议用联合体*/
	void(*AhtIicRead)(uint8_t *,uint16_t);//AHT

	void(*Htu21dIic_write_devaddr)(uint8_t);//HTU21D
	void(*Htu21dIic_read_devaddr)(uint8_t *,uint16_t);//HTU21D
	HTU21D_STA_TYPE HTU21D_read_sta;//读取温度的状态
	uint8_t HTU21D_read_sta_count;//读取温度的状态
	/*建议用联合体*/

	void (*delayms)(uint8_t);
	uint8_t delayms_sta_count;
	uint8_t i2cdev_addr;
	float temperature;
	float humidity;
	uint16_t badrh_crc_count;
	uint16_t goodrh_crc_count;
	uint16_t badtp_crc_count;
	uint16_t goodtp_crc_count;
	SH_DEV_TYPE sh_dev_type
}SHT3X_DEV;
//------------------------------------------------------------------------
uint8_t Sht3xInit(SHT3X_DEV* base,
	void (*read)(uint8_t,uint16_t,uint8_t*,uint8_t),//标准IIC读写函数 设备地址 寄存器地址 数据buf，数据buf长度
	void (*write)(uint8_t,uint16_t,uint8_t*,uint8_t),//标准IIC读写函数 设备地址 寄存器地址 数据buf，数据buf长度
	uint8_t i2cdev_addr,//shtx设备地址有0x45 0x44 aht的设备地址有0x38(0111 000rw)
	// HTU21D地址0x40
	SH_DEV_TYPE sh_dev_type,
	void (*delayms)(uint8_t)
	);
// 使用AHT时候会用到非标准IIC接口，为什么这些烂东西会自己想一个IIC接口
void Sht3xInitAddInterface(SHT3X_DEV* base,void(*AhtIicRead)(uint8_t *,uint16_t));
// 使用HTU21D时候会用到非标准IIC接口，为什么这些烂东西会自己想一个IIC接口，还申明是标准IIC !!!
void Sht3xInitAddInterface_HTU21D(SHT3X_DEV* base,
	void(*Htu21dIic_write_devaddr)(uint8_t),//参考手册读取温度一节。在模拟iic程序中有做函数接口。
	void(*Htu21dIic_read_devaddr)(uint8_t *,uint16_t));

uint8_t CheckSht3xInLine(SHT3X_DEV* base);
void Sht3xSoftReset(SHT3X_DEV* base);
int Sht3xTemperatureHumidity(SHT3X_DEV* base,float *temp_adcval,float *rh_adcval);
void SetSht3xPeriodicMode(SHT3X_DEV* base,uint8_t mps,uint8_t refresh);			//设置周期性输出方式;


//------------------------------------------------------------------------

#endif

