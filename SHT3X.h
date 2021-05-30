#ifndef __SHT3x_H__
#define __SHT3x_H__
//#include "stm32l0xx_hal.h"
#include "stm8l15x.h"
typedef void (*sht_read)(uint8_t i2caddr,uint16_t regaddr,uint8_t*databuf,uint8_t datalen);
typedef void (*sht_write)(uint8_t i2caddr,uint16_t regaddr,uint8_t*databuf,uint8_t datalen);
// typedef enum {DEV0X44 = 0X44,DEV0X45 = 0X45}SHT3X_DEVADDR;
typedef enum {SHTX ,AHTX}SH_DEV_TYPE;
typedef struct SHT3X_DEV
{
	sht_read read;
	sht_write write;
	uint8_t i2cdev_addr;
	float temperature;
	float humidity;
	uint16_t badrh_crc_count;
	uint16_t goodrh_crc_count;
	uint16_t badtp_crc_count;
	uint16_t goodtp_crc_count;
}SHT3X_DEV;
//------------------------------------------------------------------------
uint8_t Sht3xInit(SHT3X_DEV* base,
	void (*read)(uint8_t,uint16_t,uint8_t*,uint8_t),
	void (*write)(uint8_t,uint16_t,uint8_t*,uint8_t),
	uint8_t i2cdev_addr,//shtx设备地址有0x45 0x44 aht的设备地址有0x70
	SH_DEV_TYPE sh_dev_type
	);
uint8_t CheckSht3xInLine(SHT3X_DEV* base);
void Sht3xSoftReset(SHT3X_DEV* base);
int Sht3xTemperatureHumidity(SHT3X_DEV* base,float *temp_adcval,float *rh_adcval);
void SetSht3xPeriodicMode(SHT3X_DEV* base,uint8_t mps,uint8_t refresh);			//设置周期性输出方式;


//------------------------------------------------------------------------

#endif

