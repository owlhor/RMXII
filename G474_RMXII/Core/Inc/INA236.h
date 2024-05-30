/*
 * INA236.h
 *
 *  Created on: May 30, 2024
 *      Author: owl_hor
 */

#ifndef INC_INA236_H_
#define INC_INA236_H_

#include "stm32g4xx_hal.h"
#include <math.h>

//// ******** This library is referenced from my INA219 library, some part may me incomplete

#define INA236_R_SHUNT_Val  0.01 // ohm
#define INA236_MAX_Expect_Current 10 // Ampere

/* Define calibrate_EQ6 to enable Equation 6 Calibrate & Scaling
 * Do the experiment with some test circuit using
 * the external Ampmeter to get these parameter
 *
 * MeaShuntCurrent_ExtMeter <- Parameter measures from the meter
 * INA219_Current_Raw 		<- Raw current read from INA219_RG_Current
 * 								when there're no any calibrate applied
*/
#define calibrate_EQ6
#define MeaShuntCurrent_ExtMeter 	0.8  // Ampere
#define INA236_Current_Raw 			2.7  // Ampere

#define INA236_Config_Reset 0x4127

//////// Device Address ---------------------------------------------
#define INA236_A_AD_1 0x40 << 1  // 1000000
#define INA236_A_AD_2 0x41 << 1  // 1000001
#define INA236_A_AD_3 0x42 << 1  // 1000010
#define INA236_A_AD_4 0x43 << 1  // 1000011

#define INA236_B_AD_1 0x48 << 1  // 1001000
#define INA236_B_AD_2 0x49 << 1  // 1001001
#define INA236_B_AD_3 0x4A << 1  // 1001010
#define INA236_B_AD_4 0x4B << 1  // 1001011
//// Register Address ---------------------------------------------
#define INA236_RG_Config	0x00
#define INA236_RG_ShuntV    0x01
#define INA236_RG_BusV 		0x02
#define INA236_RG_PoWer		0x03
#define INA236_RG_Current   0x04
#define INA236_RG_Calibra 	0x05
#define INA236_RG_Mask_EN 	0x06
#define INA236_RG_Alr_Lim 	0x07

#define INA236_RG_Manu_ID 	0x3E
#define INA236_RG_Devi_ID 	0x3F

//// --------------------------Mode Setting ------------------------

//typedef enum {
//	BRNG_FSR_16V,
//	BRNG_FSR_32V}
//INA219_BRNG_FSR_Set;
//
//typedef enum {
//	PGA_GainD1_40mv,
//	PGA_GainD2_80mv,
//	PGA_GainD4_160mv,
//	PGA_GainD8_320mv}
//INA219_PG_Set;
//
//typedef enum{
//	ADCI_9bit_84uS ,
//	ADCI_10bit_148uS,
//	ADCI_11bit_276uS,
//	ADCI_12bit_532uS, // default
//	ADCI_12bit_532uSs = 0b1000,
//	ADCI_2sx_1ms,
//	ADCI_4sx_2ms,
//	ADCI_8sx_4ms,
//	ADCI_16sx_8ms,
//	ADCI_32sx_17ms,
//	ADCI_64sx_34ms,
//	ADCI_128sx_68ms}
//INA219_ADC_Set;
//
//typedef enum{
//	INAM_PWR_Down,
//	INAM_ShuntV_Triggr,
//	INAM_BusV_Triggr,
//	INAM_ShuntBusV_Triggr,
//	INAM_ADCOFF,
//	INAM_ShuntV_Continuous,
//	INAM_BusV_Continuous,
//	INAM_ShuntBusV_Continuous}
//INA219_Mode_Set;

typedef struct {

	uint16_t Config ;

	float SHUNT_V;
	uint16_t Bus_V;
	float POWER;
	int16_t CURRENT;

	uint16_t Calibra; // For read back
	uint16_t mask_en;
	uint16_t alrt_lim;

	uint16_t Manu_ID;
	uint16_t Dev_ID;

}INA236_Read_Set;

//typedef union _INA219_Conf_Strc{
//	struct INA219CF{
//
//		uint16_t Mode : 3;
//		uint16_t SADC : 4;
//		uint16_t BADC : 4;
//		uint16_t PGA  : 2;
//		uint16_t BRNG : 1;
//		uint16_t rsrv : 1;
//		uint16_t reset: 1;
//
//		}INA219CF;
//	uint8_t D8[2];
//}INA219_Conf_Strc;

uint16_t INA236Read_cx(I2C_HandleTypeDef *hi2c,uint8_t dv_addr, uint8_t ina_rg);
uint16_t INA236Read_BusV(I2C_HandleTypeDef *hi2c,uint8_t dv_addr);
uint16_t INA236Read_Current(I2C_HandleTypeDef *hi2c,uint8_t dv_addr);
float INA236Read_Power(I2C_HandleTypeDef *hi2c,uint8_t dv_addr);
float INA236Read_ShuntV(I2C_HandleTypeDef *hi2c,uint8_t dv_addr);

void INA236_Calibrate(I2C_HandleTypeDef *hi2c,uint8_t dv_addr);
//void INA219_INIT_Calibrate(I2C_HandleTypeDef *hi2c,uint8_t dv_addr);
//void INA219_INIT(I2C_HandleTypeDef *hi2c,uint8_t dv_addr, INA219_Conf_Strc cfgra);
//void INA219_BitReset(I2C_HandleTypeDef *hi2c,uint8_t dv_addr);


#endif /* INC_INA236_H_ */
