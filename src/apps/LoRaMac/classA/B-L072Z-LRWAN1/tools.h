/*!
 * \file      tools.h
 *
 * \brief     parameters and sensor functions
 *
 * \author    Alexandro Vanderley dos Santos
 */

#ifndef __TOOLS_H__
#define __TOOLS_H__

#include <stdio.h>
#include "gpio.h"

//Registradores do Sesor HTS221
#define HTS221_ADDRESS   			0x5F
#define HTS221_WHO_AM_I          	0x0F
#define HTS221_DEVICE_ID			0xBC
#define HTS221_AV_CONF				0x10
#define HTS221_CTRL1             	0x20
#define HTS221_CTRL2             	0x21
#define HTS221_STATUS            	0x27
#define HTS221_HUMIDITY_OUT_L    	0x28
#define HTS221_TEMP_OUT_L        	0x2A
#define HTS221_TEMP_OUT_H        	0x2B
#define HTS221_H0_rH_x2          	0x30
#define HTS221_H1_rH_x2          	0x31
#define HTS221_T0_degC_x8        	0x32
#define HTS221_T1_degC_x8        	0x33
#define HTS221_T1_T0_MSB         	0x35
#define HTS221_H0_T0_OUT         	0x36
#define HTS221_H1_T0_OUT         	0x3A
#define HTS221_T0_OUT            	0x3C
#define HTS221_T1_OUT            	0x3E

//Registradores do Sesor LPS22HB
#define	LPS22HB_ADDRESS				0x5D
#define LPS22HB_DEVICE_ID			0xB1
#define	LPS22HB_INTERRUPT_CFG 	 	0x0B
#define	LPS22HB_THS_P_L 	 		0x0C
#define	LPS22HB_THS_P_H  			0x0D
#define	LPS22HB_WHO_AM_I 			0x0F
#define	LPS22HB_CTRL_REG1  			0x10
#define	LPS22HB_CTRL_REG2  			0x11
#define	LPS22HB_CTRL_REG3  			0x12
#define	LPS22HB_FIFO_CTRL  			0x14
#define	LPS22HB_REF_P_XL  			0x15
#define	LPS22HB_REF_P_L  			0x16
#define	LPS22HB_REF_P_H  			0x17
#define	LPS22HB_RPDS_L  			0x18
#define	LPS22HB_RPDS_H  			0x19
#define	LPS22HB_RES_CONF  			0x1A
#define	LPS22HB_INT_SOURCE  		0x25
#define	LPS22HB_FIFO_STATUS  		0x26
#define	LPS22HB_STATUS  			0x27 
#define	LPS22HB_PRESS_OUT_XL  		0x28
#define	LPS22HB_PRESS_OUT_L  		0x29
#define	LPS22HB_PRESS_OUT_H  		0x2A
#define	LPS22HB_TEMP_OUT_L  		0x2B
#define	LPS22HB_TEMP_OUT_H  		0x2C
#define	LPS22HB_LPFP_RES			0x33

//Cria os objetos referentes aos pinos de comunicacao i2c
Gpio_t SCL;
Gpio_t SDA;

//Variaveis de calibracao do sensor HTS221
float _hts221HumiditySlope;
float _hts221HumidityZero;
float _hts221TemperatureSlope;
float _hts221TemperatureZero;

enum {
  FAHRENHEIT,
  CELSIUS
};


//Funcao para imprimir double e float
void printDouble(double v, int decimalDigits);

//Protocolo i2c via software
void WriteI2cSW_8bits(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t ReadI2cSW_8bits(uint8_t addr, uint8_t reg);
uint16_t ReadI2cSW_16bits(uint8_t addr, uint8_t reg);

// Funcoes para o sensor HTS221
uint8_t HTS221_begin();
void HTS221_Read_Calibration();
float HTS221_Temperature(int units);
float HTS221_Humidity();
uint16_t HTS221_Temperature_Hex();
uint16_t HTS221_Humidity_Hex();

// Funcoes para o sensor LPS22HB
uint8_t LPS22HB_begin();
float LPS22HB_Pressure();
uint32_t LPS22HB_Pressure_Hex();
uint16_t LPS22HB_Temperature_Hex();

#endif // __TOOLS_H__
