/*!
 * \file      tools.c
 *
 * \brief     sensor functions implementation
 *	
 * \author    Alexandro Vanderley dos Santos
 */

#include "tools.h"
#include <stdio.h>
#include <stdlib.h>

void printDouble(double v, int decimalDigits){
	  int i = 1;
	  int intPart, fractPart;
	  for (;decimalDigits!=0; i*=10, decimalDigits--);
	  intPart = (int)v;
	  fractPart = (int)((v-(double)(int)v)*i);
	  if(fractPart < 0) fractPart *= -1;
	  printf("%i.%i", intPart, fractPart);
}

void WriteI2cSW_8bits(uint8_t addr, uint8_t reg, uint8_t data){	
	uint8_t x;
	uint8_t addr_Write=addr<<1;
	uint8_t internal_Reg=reg;
	uint8_t Reg_data=data;	
	//i2c_start();              	// send start sequence
	GpioWrite(&SDA,1);
	GpioWrite(&SCL,1);
	GpioWrite(&SDA,0);
	GpioWrite(&SCL,0);
	//i2c_tx(addr W);             	// SRF08 I2C address with R/W bit clear	
	for(x=8; x; x--) {
		if(addr_Write&0x80) GpioWrite(&SDA,1);
		else GpioWrite(&SDA,0);
		GpioWrite(&SCL,1);
		addr_Write <<= 1;
		GpioWrite(&SCL,0);
	}
	GpioWrite(&SDA,1);
	GpioWrite(&SCL,1);	
	GpioRead(&SDA);          		// possible ACK bit	
	GpioWrite(&SCL,0);		
	//i2c_tx(reg);             		// SRF08 light sensor register address	
	for(x=8; x; x--) {
		if(internal_Reg&0x80) GpioWrite(&SDA,1);
		else GpioWrite(&SDA,0);
		GpioWrite(&SCL,1);
		internal_Reg <<= 1;
		GpioWrite(&SCL,0);
	}
	GpioWrite(&SDA,1);
	GpioWrite(&SCL,1);	
	GpioRead(&SDA);          		// possible ACK bit
	GpioWrite(&SCL,0);	
	//i2c_tx(data);             	// SRF08 light sensor register address	
	for(x=8; x; x--) {
		if(Reg_data&0x80) GpioWrite(&SDA,1);
		else GpioWrite(&SDA,0);
		GpioWrite(&SCL,1);
		Reg_data <<= 1;
		GpioWrite(&SCL,0);
	}
	GpioWrite(&SDA,1);
	GpioWrite(&SCL,1);	
	GpioRead(&SDA);          		// possible ACK bit
	GpioWrite(&SCL,0);	
	//i2c_stop(); 	
	GpioWrite(&SDA,0);
	GpioWrite(&SCL,1);
	GpioWrite(&SDA,1);	 
}

uint8_t ReadI2cSW_8bits(uint8_t addr, uint8_t reg){
	uint8_t data=0;
	uint8_t addr_Write=addr<<1;	
	uint8_t addr_Read=addr_Write+0x01;	
	uint8_t	internal_Reg=reg;
	uint8_t x;
	
	//start()
	GpioWrite(&SDA,1);             // i2c start bit sequence 
	GpioWrite(&SCL,1); 
	GpioWrite(&SDA,0); 
	GpioWrite(&SCL,0);        
	//i2c_tx(addr_Write);  	
	for(x=8; x; x--) {
		if(addr_Write&0x80) GpioWrite(&SDA,1);
		else GpioWrite(&SDA,0);
		GpioWrite(&SCL,1);
		addr_Write <<= 1;
		GpioWrite(&SCL,0);
	}
	GpioWrite(&SDA,1);
	GpioWrite(&SCL,1);
	GpioRead(&SDA);          		// possible ACK bit	
	GpioWrite(&SCL,0);	
	//i2c_tx(internal_Reg);
	for(x=8; x; x--) {
		if(internal_Reg&0x80) GpioWrite(&SDA,1);
		else GpioWrite(&SDA,0);
		GpioWrite(&SCL,1);
		internal_Reg <<= 1;
		GpioWrite(&SCL,0);
	}
	GpioWrite(&SDA,1);
	GpioWrite(&SCL,1);
	GpioRead(&SDA);          		// possible ACK bit		
	GpioWrite(&SCL,0);
	//GpioWrite(&SCL,0);			//delay	
	//start()	
	GpioWrite(&SCL,1); 
	//GpioWrite(&SCL,1);  			//delay
	GpioWrite(&SDA,0); 
	GpioWrite(&SCL,0);               
	//i2c_tx(addr_Read);
	for(x=8; x; x--) {
		if(addr_Read&0x80) GpioWrite(&SDA,1);
		else GpioWrite(&SDA,0);
		GpioWrite(&SCL,1);
		addr_Read <<= 1;
		GpioWrite(&SCL,0);
	}
	GpioWrite(&SDA,1);
	GpioWrite(&SCL,1);
	GpioRead(&SDA);          		// possible ACK bit		
	GpioWrite(&SCL,0);         
	//d = i2c_rx(0);  		
	for(x=0; x<8; x++) {
		data <<= 1;
		GpioWrite(&SCL,1);
		while(GpioRead(&SCL)==0);    // wait for any SCL clock stretching		
		if(GpioRead(&SDA)) data |= 1;
		GpioWrite(&SCL,0);
	}	
	GpioWrite(&SCL,1);	
	GpioWrite(&SCL,0);		
	//stop()
	GpioWrite(&SDA,0);             	// i2c stop bit sequence
	GpioWrite(&SCL,1);
	GpioWrite(&SDA,1);	
	return data; 
}

uint16_t ReadI2cSW_16bits(uint8_t addr, uint8_t reg){
	uint8_t data_L=0;
	uint8_t data_H=0;
	uint8_t addr_Write=addr<<1;	
	uint8_t addr_Read=addr_Write+0x01;	
	uint8_t	internal_Reg=reg|0x80;
	uint8_t x;
	uint16_t value=0;	
	//start()
	GpioWrite(&SDA,1);             // i2c start bit sequence 
	GpioWrite(&SCL,1); 
	GpioWrite(&SDA,0); 
	GpioWrite(&SCL,0);        
	//i2c_tx(addr_Write);  	
	for(x=8; x; x--) {
		if(addr_Write&0x80) GpioWrite(&SDA,1);
		else GpioWrite(&SDA,0);
		GpioWrite(&SCL,1);
		addr_Write <<= 1;
		GpioWrite(&SCL,0);
	}
	GpioWrite(&SDA,1);
	GpioWrite(&SCL,1);
	GpioRead(&SDA);          		// possible ACK bit	
	GpioWrite(&SCL,0);	
	//i2c_tx(internal_Reg);
	for(x=8; x; x--) {
		if(internal_Reg&0x80) GpioWrite(&SDA,1);
		else GpioWrite(&SDA,0);
		GpioWrite(&SCL,1);
		internal_Reg <<= 1;
		GpioWrite(&SCL,0);
	}
	GpioWrite(&SDA,1);
	GpioWrite(&SCL,1);
	GpioRead(&SDA);          		// possible ACK bit		
	GpioWrite(&SCL,0);
	//GpioWrite(&SCL,0);			//delay	
	//start()	
	GpioWrite(&SCL,1); 
	//GpioWrite(&SCL,1);  			//delay
	GpioWrite(&SDA,0); 
	GpioWrite(&SCL,0);               
	//i2c_tx(addr_Read);
	for(x=8; x; x--) {
		if(addr_Read&0x80) GpioWrite(&SDA,1);
		else GpioWrite(&SDA,0);
		GpioWrite(&SCL,1);
		addr_Read <<= 1;
		GpioWrite(&SCL,0);
	}
	GpioWrite(&SDA,1);
	GpioWrite(&SCL,1);
	GpioRead(&SDA);          		// possible ACK bit		
	GpioWrite(&SCL,0);         
	//d = i2c_rx(0);  		
	for(x=0; x<8; x++) {
		data_L <<= 1;
		GpioWrite(&SCL,1);
		while(GpioRead(&SCL)==0);  	// wait for any SCL clock stretching		
		if(GpioRead(&SDA)) data_L |= 1;
		GpioWrite(&SCL,0);
	}	
	GpioWrite(&SDA,0); 
	GpioWrite(&SCL,1);	
	GpioWrite(&SCL,0);
	GpioWrite(&SDA,1); 	
	//dada_L = i2c_rx(0);  		
	for(x=0; x<8; x++) {
		data_H <<= 1;
		GpioWrite(&SCL,1);
		while(GpioRead(&SCL)==0);    // wait for any SCL clock stretching		
		if(GpioRead(&SDA)) data_H |= 1;
		GpioWrite(&SCL,0);
	}
	GpioWrite(&SCL,1);	
	GpioWrite(&SCL,0);		
	//stop()
	GpioWrite(&SDA,0);            	 // i2c stop bit sequence
	GpioWrite(&SCL,1);
	GpioWrite(&SDA,1);		
	value=(data_H<<8)+data_L;
	return (value); 
}


//*************************HTS221 function*******************************
uint8_t HTS221_begin(){	
	GpioInit( &SDA, PB_9, PIN_OUTPUT,  PIN_OPEN_DRAIN, PIN_NO_PULL, 1);
	GpioInit( &SCL, PB_8, PIN_OUTPUT,  PIN_OPEN_DRAIN, PIN_NO_PULL, 1);	
	if (ReadI2cSW_8bits(HTS221_ADDRESS, HTS221_WHO_AM_I) != HTS221_DEVICE_ID) {		 
		printf("\nDispositivo HTS221 desconectado!!!\n");
		return 0;
	}
	printf("\nDispositivo HTS221 conectado!!!\n");
	HTS221_Read_Calibration();		
	return 1;
}

void HTS221_Read_Calibration(){
	  uint8_t h0rH = ReadI2cSW_8bits(HTS221_ADDRESS, HTS221_H0_rH_x2 );
	  uint8_t h1rH = ReadI2cSW_8bits(HTS221_ADDRESS, HTS221_H1_rH_x2 );
	  uint16_t t0degC = ReadI2cSW_8bits(HTS221_ADDRESS, HTS221_T0_degC_x8 ) | ((ReadI2cSW_8bits(HTS221_ADDRESS, HTS221_T1_T0_MSB ) & 0x03) << 8);
	  uint16_t t1degC = ReadI2cSW_8bits(HTS221_ADDRESS, HTS221_T1_degC_x8 ) | ((ReadI2cSW_8bits(HTS221_ADDRESS, HTS221_T1_T0_MSB ) & 0x0c) << 6);
	  int16_t h0t0Out = ReadI2cSW_16bits(HTS221_ADDRESS, HTS221_H0_T0_OUT );
	  int16_t h1t0Out = ReadI2cSW_16bits(HTS221_ADDRESS, HTS221_H1_T0_OUT );
	  int16_t t0Out = ReadI2cSW_16bits(HTS221_ADDRESS, HTS221_T0_OUT );
	  int16_t t1Out = ReadI2cSW_16bits(HTS221_ADDRESS, HTS221_T1_OUT );
	  _hts221HumiditySlope = (h1rH - h0rH) / (2.0 * (h1t0Out - h0t0Out));
	  _hts221HumidityZero = (h0rH / 2.0) - _hts221HumiditySlope * h0t0Out;
	  _hts221TemperatureSlope = (t1degC - t0degC) / (8.0 * (t1Out - t0Out));
	  _hts221TemperatureZero = (t0degC / 8.0) - _hts221TemperatureSlope * t0Out;
	}

float HTS221_Temperature(int units){  
	  // read value and convert in Fahrenheit or Celsius
	  int16_t tout = ReadI2cSW_16bits(HTS221_ADDRESS, HTS221_TEMP_OUT_L);
	  float reading = (tout * _hts221TemperatureSlope + _hts221TemperatureZero);
	  if (units == FAHRENHEIT) { // Fahrenheit = (Celsius * 9 / 5) + 32
			return (reading * 9.0 / 5.0) + 32.0;
	  } else {
			return reading;
	  }
}

float HTS221_Humidity(){
	  // read value
	  int16_t hout = ReadI2cSW_16bits(HTS221_ADDRESS, HTS221_HUMIDITY_OUT_L);
	  return (hout * _hts221HumiditySlope + _hts221HumidityZero);
}

uint16_t HTS221_Temperature_Hex(){
	int16_t tout = ReadI2cSW_16bits(HTS221_ADDRESS, HTS221_TEMP_OUT_L);
	return tout;
}

uint16_t HTS221_Humidity_Hex(){
	  // read value
	  int16_t hout = ReadI2cSW_16bits(HTS221_ADDRESS, HTS221_HUMIDITY_OUT_L);
	  return (hout);
}


//*************************LPS22HB function*******************************
uint8_t LPS22HB_begin(){	
	GpioInit( &SDA, PB_9, PIN_OUTPUT,  PIN_OPEN_DRAIN, PIN_NO_PULL, 1);
	GpioInit( &SCL, PB_8, PIN_OUTPUT,  PIN_OPEN_DRAIN, PIN_NO_PULL, 1);	
	if (ReadI2cSW_8bits(LPS22HB_ADDRESS, LPS22HB_WHO_AM_I) != LPS22HB_DEVICE_ID) {		 
		printf("\nDispositivo LPS22HB desconectado!!!\n");
		return 0;
	}
	WriteI2cSW_8bits(LPS22HB_ADDRESS,LPS22HB_RES_CONF,0x01);
	WriteI2cSW_8bits(LPS22HB_ADDRESS,LPS22HB_CTRL_REG1,0x32);
	printf("\nDispositivo LPS22HB conectado!!!\n");	
	return 1;
}

float LPS22HB_Pressure(){
	uint32_t value=0;
	float pressure=0.0;
	// read value and convert in hPa
	int8_t HI = ReadI2cSW_8bits(LPS22HB_ADDRESS, LPS22HB_PRESS_OUT_H);
	int8_t MID = ReadI2cSW_8bits(LPS22HB_ADDRESS, LPS22HB_PRESS_OUT_L);
	int8_t LOW = ReadI2cSW_8bits(LPS22HB_ADDRESS, LPS22HB_PRESS_OUT_XL);
	value=(HI<<16)+(MID<<8)+LOW;
	pressure=value/4096.0;		
	return (pressure);
}

uint32_t LPS22HB_Pressure_Hex(){
	uint32_t value=0;	
	// read value
	int8_t HI = ReadI2cSW_8bits(LPS22HB_ADDRESS, LPS22HB_PRESS_OUT_H);
	int8_t MID = ReadI2cSW_8bits(LPS22HB_ADDRESS, LPS22HB_PRESS_OUT_L);
	int8_t LOW = ReadI2cSW_8bits(LPS22HB_ADDRESS, LPS22HB_PRESS_OUT_XL);
	value=(HI<<16)+(MID<<8)+LOW;
	return (value);
}

uint16_t LPS22HB_Temperature_Hex(){
	uint16_t value=0;  
	// read value
	int8_t HI = ReadI2cSW_8bits(LPS22HB_ADDRESS, LPS22HB_TEMP_OUT_H);
	int8_t LOW = ReadI2cSW_8bits(LPS22HB_ADDRESS, LPS22HB_TEMP_OUT_L);
	value=(HI<<8)+LOW;
	return (value);
}


