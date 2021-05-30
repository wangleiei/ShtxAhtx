/*************************************************************************************
Srh读值，RH转换后的湿度值
Tsr读值, T(C)摄氏度,T(F)华氏度

RH= 100*Srh / 65535    

T(C) = -45 + 175 * Tsr / 65535
T(F) = -49 + 315 * Tsr / 65535

温湿度传感器都使用I2C1进行通迅 通信scl速率设置成50hz比较合适，很低,即使如此也有10%的错误率

***********************************************************************************/
#include "SHT3X.h"

#define MPS_05HZ  	1			//2S1次
#define MPS_1HZ  	2			//1S1次
#define MPS_2HZ 	3			//1S2次
#define MPS_4HZ  	4			//1S4次
#define MPS_10HZ  	5			//1S10次

#define REFRESH_HIGH	1		//刷新率高(重复率高)
#define REFRESH_MID 	2		//刷新率中(重复率中)
#define REFRESH_LOW 	3		//刷新率低(重复率低)

#define SHTX_TC(St)	(175 * (float)St / 65535 -45)
#define SHTX_TF(St)  (315 * (float)St / 65535 -49)
#define SHTX_RH(St)  (100 * (float)St / 65535)

const uint8_t Measurementcommands[] = {0x2032,0x2024,0x202F,0x2130,0x2126,0x212D,0x2236,0x2220,0x222B,0x2334,0x2322,0x2329,0x2737,0x2721,0x272A};
static uint16_t SHT3X_crc8(unsigned char *addr,uint8_t num);

/***********************************************************************
* 描    述: SHT3X温湿度传感初始化
* 入口参数: read 函数指针（该函数第一个参数是传入i2c器件地址，第二个参数是传入寄存器地址，8位数据指针，数据长度）使用i2c读取一段数据的函数
		   write 函数指针使用i2c写入一段数据，
* 出口参数: 
* 附加信息: 
* 说    明: 由于SHTX是使用双字节地址，所以需要实现双地址的i2c接口，
  			不采用crc校验。降低i2c通信速率可以得到很好的数据
************************************************************************/
uint8_t Sht3xInit(SHT3X_DEV* base,
	void (*read)(uint8_t,uint16_t,uint8_t*,uint8_t),
	void (*write)(uint8_t,uint16_t,uint8_t*,uint8_t),
	uint8_t i2cdev_addr,
	SH_DEV_TYPE sh_dev_type,
	void (*delayms)(uint8_t)){
	base->read = read;
	base->AhtIicRead = 0;
	base->write = write;

	base->delayms = delayms;
	base->delayms_sta_count = 0;

	base->badrh_crc_count = base->goodrh_crc_count = base->badtp_crc_count = base->goodtp_crc_count = 0;

	base->i2cdev_addr = i2cdev_addr;
	
	base->sh_dev_type = sh_dev_type;

	if(base->sh_dev_type == SHTX){
		Sht3xSoftReset(base);
		SetSht3xPeriodicMode(base, MPS_1HZ,REFRESH_HIGH);			//设置温湿度传感器输出方式		
	}else if(base->sh_dev_type == AHTX){
		uint8_t temp[2] = {0x08,0x00};//数据手册上没有这个 但是很多地方的例子都有
		base->i2cdev_addr = 0x38;
		Sht3xSoftReset(base);
		base->delayms(30);
		// 5.4 软复位这个命令（见表9）用于在无需关闭和再次打开电源的情况下，重新启动传感器系统。在接收到这个命令之后，
		// 传感器系统开始重新初始化，并恢复默认设置状态，软复位所需时间不超过20毫秒。
		base->write(base->i2cdev_addr,0xe1,temp,2);
	}
	return 0;
}

void Sht3xInitAddInterface(SHT3X_DEV* base,void(*AhtIicRead)(uint8_t *,uint16_t)){
	base->AhtIicRead = AhtIicRead;
}
//***********************************************************************************************
// 函 数 名 : Check_SHT3X
// 输入参数 : NONE
// 返回参数 : stauts 在线TRUE,不在线FALSH
// 说    明 : 查找温湿度传感器是否在线
//            随便读取一个SHT中的寄存器，若有返回 0：在线
//***********************************************************************************************
uint8_t CheckSht3xInLine(SHT3X_DEV* base)
{
	uint8_t ReadBuf[2];
	return 0;
	base->read(base->i2cdev_addr,0xE000,ReadBuf,2);	
	if((ReadBuf[0] != 0) && (ReadBuf[0] != 0xff))
	{
		return 0;
	}
	return 1;
}

//***********************************************************************************************
// 函 数 名 : SHT3X_soft_reset
// 输入参数 : InExt(内外部湿度)
// 返回参数 : NONE
// 说    明 : 软件复位
//***********************************************************************************************
void Sht3xSoftReset(SHT3X_DEV* base)
{
	uint8_t buf;
	uint16_t temp = 0;
	if(base->sh_dev_type == SHTX){
		temp = 0x30A2;
	}else if(base->sh_dev_type == AHTX){
		temp = 0xba;		
	}
	base->write(base->i2cdev_addr,temp,&buf,0);
}
/**********************************************************************************************************
*	函 数 名: void SHT3X_temperature_humidity(SHT3X_DEV* base,float *TEMP_ADCVal,float *RH_ADCVal)
*	功能说明: 读取温度函数
*	传    参: float *TEMP_ADCVal 温度比如23.9摄氏度
			  float *RH_ADCVal 89 百分之89湿度
*	返 回 值: 0:成功采集数据1：失败,2：采集中
*   说    明: 
*********************************************************************************************************/
int Sht3xTemperatureHumidity(SHT3X_DEV* base,float *temp_adcval,float *rh_adcval)
{
	uint8_t ReadBuf[8] = {0x00};	
	uint16_t temp = 0;
	uint16_t remp = 0;
	if(base->sh_dev_type == SHTX){
		base->read(base->i2cdev_addr,0xE000,ReadBuf,6);	

		temp = (ReadBuf[0]<<8) + ReadBuf[1];
		remp = (ReadBuf[3]<<8) + ReadBuf[4];

		if((SHT3X_crc8(&ReadBuf[0],2) == ReadBuf[2])){
			*temp_adcval = base->temperature = SHTX_TC(temp);
			base->goodtp_crc_count++;
		}else{
			base->badtp_crc_count++;

			*temp_adcval = 0;
			temp = 1;
		}
		if(SHT3X_crc8(&ReadBuf[3],2) == ReadBuf[5]){		
			*rh_adcval = base->humidity = SHTX_RH(remp);
			base->goodrh_crc_count++;
		}
		else{
			base->badrh_crc_count++;
			*rh_adcval = 0;
			temp = 1;
		}
		return temp;
	}else if(base->sh_dev_type == AHTX){
		uint8_t merse[2] = {0x33,0x00};
		uint32_t temp = 0;
		uint32_t hump = 0;
		// 发出测量命令
		if(base->delayms_sta_count == 0){
			base->write(base->i2cdev_addr,0xAC,merse,2);
		}
		base->delayms(1);
		// 注：传感器在采集时需要时间,主机发出测量指令（0xAC）后,延时75毫秒以上再读取转换后的数据并判断返回的状
		// 态位是否正常。
		base->delayms_sta_count ++;
		if(base->delayms_sta_count >= 100){
			base->delayms_sta_count = 0;
		}else{
			return 2;
		}
		if(base->AhtIicRead == 0){
			return 1;//错误 没有调用AHT额外的接口函数
		}
		base->AhtIicRead(ReadBuf,6);
		if(0x80 & ReadBuf[0] ){
			return 1;
		}
		
		hump |= (0XFF&(ReadBuf[1]));hump <<= 8;
		hump |= (0XFF&(ReadBuf[2]));hump <<= 4;
		hump |= ((ReadBuf[3]&0xf0) >> 4);
		
		temp |= (ReadBuf[3]&0x0f); temp <<= 8;
		temp |= (0XFF&(ReadBuf[4])); temp <<= 8;
		temp |= (0XFF&(ReadBuf[5]));

		base->temperature = *temp_adcval = temp *200.0/1024.0/1024.0-50;//计算得到温度值
		base->humidity = *rh_adcval = hump*100.0/1024.0/1024.0;  //计算得到湿度值
		
		return 0;
	}		
}

//***********************************************************************************************
// 函 数 名 : set_SHT3x_Periodic_mode
// 输入参数 : mps(采样率),refresh(刷新率),InExt(内外部湿度)
// 返回参数 : NONE
// 说    明 : 设置周期性输出方式
//            刷新率越低,数值跳变越明显
//            mps,采样率有:0.5Hz,1Hz,2Hz,4Hz,10Hz(MPS_05HZ,MPS_1HZ,MPS_2HZ...)
//            refresh刷新率,高中低(REFRESH_HIGH,REFRESH_MID,REFRESH_LOW)
//***********************************************************************************************
void SetSht3xPeriodicMode(SHT3X_DEV* base,uint8_t mps,uint8_t refresh)			//设置周期性输出方式
{
	uint8_t buf[1];	
	uint8_t index = 0;
	if(base->sh_dev_type == AHTX) return;

	index = (mps-MPS_05HZ)*3;
	index = index + refresh;
	if(index >= sizeof(Measurementcommands))
	{
		index = 0;
	}

	base->write(base->i2cdev_addr,Measurementcommands[index],buf,0);
}


//*************************************************************************************************
// 函 数 名 : SHT3X_crc8
// 输入参数 : 参与运算的值addr,长度
// 返回参数 : CRC
// 说    明 : SHT3X CRC校验
// 						P(x) = x^8 + x^5 + x^4 + 1
//此CRC计算方式仅适用小数据量运算,多数据运算时,请使用查表方式
//***********************************************************************************************
static uint16_t SHT3X_crc8(unsigned char *addr,uint8_t num) 
{  
	uint8_t  i;  
	uint8_t crc =  0xFF;
	
	for (; num > 0; num--)              // Step through bytes in memory   
	{  
		crc ^= (*addr++);
		for (i = 0; i < 8; i++)           // Prepare to rotate 8 bits   
		{  
			if (crc & 0x80)            			// b7 is set...   
				crc = (crc << 1) ^  0x31; // P(x) = x^8 + x^5 + x^4 + 1 = 100110001
			else                          	// b7 is clear...   
				crc <<= 1;                  // just rotate   
		}                             		// Loop for 8 bits   
		crc &= 0xFF;                  		// Ensure CRC remains 16-bit value   
	}                               		// Loop until num=0   
	return(crc);                    		// Return updated CRC   
}

