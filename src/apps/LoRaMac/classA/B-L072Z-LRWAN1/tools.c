#include "tools.h"

void printDouble(double v, int decimalDigits)
{
  int i = 1;
  int intPart, fractPart;
  for (;decimalDigits!=0; i*=10, decimalDigits--);
  intPart = (int)v;
  fractPart = (int)((v-(double)(int)v)*i);
  if(fractPart < 0) fractPart *= -1;
  printf("%i.%i", intPart, fractPart);
}

uint8_t WriteI2cSW( uint8_t addr ){
	uint8_t rasc=0,t,addrBit=addr;
	GpioWrite(&SDA,0);
	GpioWrite(&SCL,0);	
	for (t=0;t<8;t++){		
		GpioWrite(&SDA,addrBit>>7);
		GpioWrite(&SCL,1);
		GpioWrite(&SCL,0);	
		addrBit=addrBit<<1;	
	}
	GpioWrite(&SDA,1);
	GpioWrite(&SCL,1);
	if (GpioRead(&SDA)==1) rasc=1;
	GpioWrite(&SCL,0);	
	return rasc;	
}

uint8_t ReadI2cSW( uint8_t addr ){
	uint8_t t;
	uint8_t addrBit=0;
	GpioWrite(&SDA,1);
	GpioWrite(&SCL,1);
	WriteI2cSW((addr<<1)+1);	
	for (t=0;t<8;t++){		
		addrBit=addrBit<<1;			
		GpioWrite(&SCL,1);
		addrBit=addrBit+GpioRead(&SDA);
		GpioWrite(&SCL,0);			
	}				
	//MAK
	GpioWrite(&SDA,1);
	GpioWrite(&SCL,1);
	GpioWrite(&SCL,0);
	//Stop
	GpioWrite(&SDA,0);
	GpioWrite(&SCL,1);
	GpioWrite(&SDA,1);	
	return (addrBit);		
}

uint8_t  HTS221_Init(){	
	HTS221.ADDR 			= 	0x5F;
	HTS221.DEV_ID			=	0xBC;
	HTS221.WHO_I_AM 		= 	0x0F;
	HTS221.AV_CONF 			= 	0x10;
	HTS221.CTRL_REG1 		=	0x20;
	HTS221.CTRL_REG2 		=	0x21;
	HTS221.CTRL_REG3 		=	0x22;
	HTS221.STATUS_REG		=	0x27;
	HTS221.HUMIDITY_OUT_L	=	0x28;
	HTS221.HUMIDITY_OUT_H	=	0x29;
	HTS221.TEMP_OUT_L		=	0x2A;
	HTS221.TEMP_OUT_H		=	0x2B;	
	HTS221.H0_rH_x2_REG     =   0x30;
	HTS221.H1_rH_x2_REG     =   0x31;
	HTS221.T0_degC_x8_REG   =   0x32;
	HTS221.T1_degC_x8_REG   =   0x33;
	HTS221.T1_T0_MSB_REG    =   0x35;
	HTS221.H0_T0_OUT_REG    =   0x36;
	HTS221.H1_T0_OUT_REG    =   0x3a;
	HTS221.T0_OUT_REG       =   0x3c;
	HTS221.T1_OUT_REG       =   0x3e;
	uint8_t WIA=0;
	GpioInit( &SDA, PB_9, PIN_OUTPUT,  PIN_OPEN_DRAIN, PIN_NO_PULL, 1);
	GpioInit( &SCL, PB_8, PIN_OUTPUT,  PIN_OPEN_DRAIN, PIN_NO_PULL, 1);	
	if (WriteI2cSW( HTS221.ADDR<<1)==0)	WriteI2cSW(HTS221.WHO_I_AM);	
	WIA=ReadI2cSW(HTS221.ADDR);
	if (WIA==HTS221.DEV_ID) {
		HTS221_Calibration_Read();
		return 1;}
	else return 0;	
}

uint16_t HTS221_Temp(){
	uint8_t HI,LOW;
	if (WriteI2cSW( HTS221.ADDR<<1)==0)	WriteI2cSW(HTS221.TEMP_OUT_H);		
	HI=ReadI2cSW(HTS221.ADDR);
	if (WriteI2cSW( HTS221.ADDR<<1)==0)	WriteI2cSW(HTS221.TEMP_OUT_L);		
	LOW=ReadI2cSW(HTS221.ADDR);
	return ((HI<<8)+LOW);
}

uint16_t HTS221_Humid(){
	uint8_t HI,LOW;
	if (WriteI2cSW( HTS221.ADDR<<1)==0)	WriteI2cSW(HTS221.HUMIDITY_OUT_H);		
	HI=ReadI2cSW(HTS221.ADDR);
	if (WriteI2cSW( HTS221.ADDR<<1)==0)	WriteI2cSW(HTS221.HUMIDITY_OUT_L);		
	LOW=ReadI2cSW(HTS221.ADDR);
	return ((HI<<8)+LOW);
}

void HTS221_Calibration_Read(){
	
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.H0_rH_x2_REG);			
	uint8_t h0rH = ReadI2cSW(HTS221.ADDR);
	printf("\nh0rH = %X",h0rH);
	
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.H1_rH_x2_REG);	
	uint8_t h1rH = ReadI2cSW(HTS221.ADDR);
	printf("\nh1rH = %X",h1rH);
	
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.T0_degC_x8_REG);	
	uint16_t t0degC = ReadI2cSW(HTS221.ADDR);
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.T1_T0_MSB_REG);		
	t0degC=t0degC | ((ReadI2cSW(HTS221.ADDR) & 0x03) << 8);	
	printf("\nt0degC = %lX",t0degC);
	
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.T1_degC_x8_REG);	
	uint16_t t1degC = ReadI2cSW(HTS221.ADDR);
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.T1_T0_MSB_REG);		
	t1degC=t1degC | ((ReadI2cSW(HTS221.ADDR) & 0x0c) << 6);	
	printf("\nt1degC = %lX",t1degC);
	
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.H0_T0_OUT_REG);	
	uint16_t h0t0Out = ReadI2cSW(HTS221.ADDR);
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.H0_T0_OUT_REG+1);	
	h0t0Out=(ReadI2cSW(HTS221.ADDR)<<8)|h0t0Out;	
	printf("\nh0t0Out = %lX",h0t0Out); 
	
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.H1_T0_OUT_REG);	
	uint16_t h1t0Out = ReadI2cSW(HTS221.ADDR);
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.H1_T0_OUT_REG+1);	
	h1t0Out=(ReadI2cSW(HTS221.ADDR)<<8)|h1t0Out;
	printf("\nh1t0Out = %lX",h1t0Out); 
	
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.T0_OUT_REG);	
	uint16_t t0Out = ReadI2cSW(HTS221.ADDR);
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.T0_OUT_REG+1);	
	t0Out=(ReadI2cSW(HTS221.ADDR)<<8)|t0Out;
	printf("\nt0Out = %lX",t0Out); 
	
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.T1_OUT_REG);	
	uint16_t t1Out = ReadI2cSW(HTS221.ADDR);
	WriteI2cSW( HTS221.ADDR<<1);
	WriteI2cSW(HTS221.T1_OUT_REG+1);	
	t1Out=(ReadI2cSW(HTS221.ADDR)<<8)|t1Out;
	printf("\nt1Out = %lX",t1Out); 

	// calculate slopes and 0 offset from calibration values,
	// for future calculations: value = a * X + b

	_hts221HumiditySlope = (h1rH - h0rH) / (2.0 * (h1t0Out - h0t0Out));
	_hts221HumidityZero = (h0rH / 2.0) - _hts221HumiditySlope * h0t0Out;

	_hts221TemperatureSlope = (t1degC - t0degC) / (8.0 * (t1Out - t0Out));
	_hts221TemperatureZero = (t0degC / 8.0) - _hts221TemperatureSlope * t0Out;
}



//////******************************LPS22HB************************////////

uint8_t  LPS22HB_Init(){	
	LPS22HB.ADDR 			= 	0x5D;
	LPS22HB.DEV_ID			=	0xB1;
	LPS22HB.WHO_I_AM 		= 	0x0F;	
	LPS22HB.PRESS_OUT_XL	=	0x28;
	LPS22HB.PRESS_OUT_L		=	0x29;
	LPS22HB.PRESS_OUT_H		=	0x2A;
	LPS22HB.TEMP_OUT_L		=	0x2B;
	LPS22HB.TEMP_OUT_H		=	0x2C;
 	
	uint8_t WIA=0;
	GpioInit( &SDA, PB_9, PIN_OUTPUT,  PIN_OPEN_DRAIN, PIN_NO_PULL, 1);
	GpioInit( &SCL, PB_8, PIN_OUTPUT,  PIN_OPEN_DRAIN, PIN_NO_PULL, 1);	
	if (WriteI2cSW( LPS22HB.ADDR<<1)==0)	WriteI2cSW(LPS22HB.WHO_I_AM);	
	WIA=ReadI2cSW(LPS22HB.ADDR);	
	if (WIA==LPS22HB.DEV_ID) return 1; else return 0;	
}

uint16_t LPS22HB_Temp(){
	uint8_t HI,LOW;
	if (WriteI2cSW( LPS22HB.ADDR<<1)==0)	WriteI2cSW(LPS22HB.TEMP_OUT_H);		
	HI=ReadI2cSW(LPS22HB.ADDR);
	if (WriteI2cSW( LPS22HB.ADDR<<1)==0)	WriteI2cSW(LPS22HB.TEMP_OUT_L);		
	LOW=ReadI2cSW(LPS22HB.ADDR);
	return ((HI<<8)+LOW);
}


uint32_t LPS22HB_Press(){
	uint8_t HI,MID,LOW; 
	float rasc;
	if (WriteI2cSW( LPS22HB.ADDR<<1)==0)	WriteI2cSW(LPS22HB.PRESS_OUT_XL);		
	LOW=ReadI2cSW(LPS22HB.ADDR);
	if (WriteI2cSW( LPS22HB.ADDR<<1)==0)	WriteI2cSW(LPS22HB.PRESS_OUT_L);		
	MID=ReadI2cSW(LPS22HB.ADDR);
	if (WriteI2cSW( LPS22HB.ADDR<<1)==0)	WriteI2cSW(LPS22HB.PRESS_OUT_H);		
	HI=ReadI2cSW(LPS22HB.ADDR);	
	rasc=((HI<<16)+(MID<<8)+LOW)*1.0;
	printf("Pressao:%4.2f", (float) rasc/4096.0);	
	return ((HI<<16)+(MID<<8)+LOW);
	
	//Para fornecer em hPa basta dividor por 4096
}

