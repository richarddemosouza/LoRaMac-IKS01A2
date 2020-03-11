#ifndef __TOOLS_H__
#define __TOOLS_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gpio.h"
typedef struct 
        {
        uint8_t ADDR;
        uint8_t DEV_ID;
        uint8_t WHO_I_AM;
		uint8_t AV_CONF;
		uint8_t CTRL_REG1;
		uint8_t CTRL_REG2;
		uint8_t CTRL_REG3;
		uint8_t STATUS_REG;
		uint8_t HUMIDITY_OUT_L;
		uint8_t HUMIDITY_OUT_H;
		uint8_t TEMP_OUT_L;
		uint8_t TEMP_OUT_H;	
		uint8_t H0_rH_x2_REG;
		uint8_t H1_rH_x2_REG;
        uint8_t T0_degC_x8_REG;
		uint8_t T1_degC_x8_REG;       
		uint8_t T1_T0_MSB_REG;         
		uint8_t H0_T0_OUT_REG;        
		uint8_t H1_T0_OUT_REG;       
		uint8_t T0_OUT_REG;           
		uint8_t T1_OUT_REG;            
        } Hts221_t;  
        
typedef struct 
        {
        uint8_t ADDR;
        uint8_t DEV_ID;
        uint8_t WHO_I_AM;
		uint8_t INTERRUPT_CFG;
		uint8_t THS_P_L;
		uint8_t THS_P_H;
		uint8_t CTRL_REG1;
		uint8_t CTRL_REG2;
		uint8_t CTRL_REG3;
		uint8_t FIFO_CTRL;
		uint8_t REF_P_XL;
		uint8_t REF_P_L;
		uint8_t REF_P_H;
		uint8_t RPDS_L;
		uint8_t RPDS_H;
		uint8_t RES_CONF;
		uint8_t INT_SOURCE;
		uint8_t FIFO_STATUS;
		uint8_t STATUS;
		uint8_t PRESS_OUT_XL;
		uint8_t PRESS_OUT_L;
		uint8_t PRESS_OUT_H;
		uint8_t TEMP_OUT_L;
		uint8_t TEMP_OUT_H;
		uint8_t LPFP_RES;
        } Lps22hb_t;     

Gpio_t SDA;
Gpio_t SCL;
Hts221_t HTS221; 
Lps22hb_t LPS22HB;

float _hts221HumiditySlope;
float _hts221HumidityZero;
float _hts221TemperatureSlope;
float _hts221TemperatureZero;

void printDouble(double v, int decimalDigits);

uint8_t WriteI2cSW( uint8_t data );

uint8_t ReadI2cSW( uint8_t addr );

uint8_t  HTS221_Init();

uint16_t HTS221_Temp();

uint16_t HTS221_Humid();

void HTS221_Calibration_Read();

uint8_t  LPS22HB_Init();

uint16_t LPS22HB_Temp();

uint32_t LPS22HB_Press();

#endif // __TOOLS_H__
