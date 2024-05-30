/*
 * INA236.c
 *
 *  Created on: May 30, 2024
 *      Author: owl_hor
 */
#include "INA236.h"

union{
	uint32_t D32;
	uint16_t D16[2];
	uint8_t D8[4];
}INACBffr;

/*  how data is stored & merge using INACBffr
 *  DataIN :  ABCD => D8.1, D8.2
 *  D16[0] :  AB 00  // D8[ 1 0 ]
 *  D16[1] :  00 CD  // D8[ 3 2 ]
 *  D16[1] | D16[0] = ABCD
 * */


uint16_t INA236Read_cx(I2C_HandleTypeDef *hi2c, uint8_t dv_addr, uint8_t ina_rg){
	/* @brief : General register read
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA219 device in the bus line
	 * @param : ina_rg - register address need to access
	 * @Retval: raw bit value
	 * @ex. answer = INA236Read_cx(&hi2c1, INA236_ADDR_1, INA236_RG_Current);
	 * */

	INACBffr.D32 = 0; //// buffer clear
	HAL_I2C_Mem_Read(hi2c, dv_addr, ina_rg, I2C_MEMADD_SIZE_8BIT, &INACBffr.D8[1], 2, 10);
	return INACBffr.D16[1] | INACBffr.D16[0];
}

void INA236_BitReset(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
	/* @brief : generates a system reset that is the same as power-on reset.
	 * 			Don't forget to  Re-calibrate or zero Current & power will be returned
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA219 device in the bus line
	 * */
	uint8_t resetx[2] = {0x80, 0x00};
	HAL_I2C_Mem_Write(hi2c, dv_addr, INA236_RG_Config, I2C_MEMADD_SIZE_8BIT, &resetx[0], 2, 10);
}

void INA236_Calibrate(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
	/* @brief : Insert calibration parameter
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA236 device in the busline
	 * */

	////// -------------------- Calibration -------------------------------------
	union{
		uint8_t  U8[2];
		uint16_t U16;
	}calibrator;

	float current_LSB = INA236_MAX_Expect_Current / 32768.0; // 2^15

	calibrator.U16 = 0.00512 / (current_LSB * INA236_R_SHUNT_Val);

	uint8_t calibrator_si2c[2] = {calibrator.U8[1], calibrator.U8[0]}; //// switch byte to send high order first in I2C
	HAL_I2C_Mem_Write(hi2c, dv_addr, INA236_RG_Calibra, I2C_MEMADD_SIZE_8BIT, &calibrator_si2c[0], 2, 10);

}

//// ******************** Reference only, incomplete *****************************88
//INA236_Conf_Strc configura;
//void INA219_INIT_Calibrate(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
//	/* @brief : Set initial config and insert calibration parameter
//	 * @param : hi2c - HAL_I2C used to read
//	 * @param : dv_addr - address of INA219 device in the busline
//	 * */
//
//	////// -------------------- Configuration -------------------------------------
//	configura.INA219CF.reset = 0;
//	configura.INA219CF.BRNG = BRNG_FSR_32V;
//	configura.INA219CF.PGA = PGA_GainD4_160mv;
//	configura.INA219CF.BADC = ADCI_12bit_532uS;
//	configura.INA219CF.SADC = ADCI_12bit_532uS;
//	configura.INA219CF.Mode = INAM_ShuntBusV_Continuous;
//
//	uint8_t confictor_si2c[2] = {configura.D8[1], configura.D8[0]};
//	HAL_I2C_Mem_Write(hi2c, dv_addr, INA219_RG_Config, I2C_MEMADD_SIZE_8BIT, &confictor_si2c[0], 2, 10);
//
//	////// -------------------- Calibration -------------------------------------
//	union{
//		uint8_t  U8[2];
//		uint16_t U16;
//	}calibrator;
//
//	// float current_LSB = INA219_MAX_Expect_Current / 32768.0; // 2^15
//	//calibrator.U16 = (int16_t)(trunc( 0.04096 / (current_LSB * INA219_R_SHUNT_Val))) << 1;
//	calibrator.U16 = trunc( 0.04096 / (current_LSB * INA219_R_SHUNT_Val));
//#ifdef calibrate_EQ6
//	calibrator.U16 = trunc((calibrator.U16 * MeaShuntCurrent_ExtMeter) / INA219_Current_Raw);
//#endif
//	uint8_t calibrator_si2c[2] = {calibrator.U8[1], calibrator.U8[0]}; //// switch byte to send high order first in I2C
//	////  ex calibrator(I = 3A, 0.1Rshunt) = 4473 = 0x1179
//
//	HAL_I2C_Mem_Write(hi2c, dv_addr, INA219_RG_Calibra, I2C_MEMADD_SIZE_8BIT, &calibrator_si2c[0], 2, 10);
//
//}

uint16_t INA236Read_BusV(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
	/* @brief : Bus Voltage read & calculate
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA236 device in the busline
	 * @Retval: bus voltage in mV,
	 *
	 * RESOLUTION: 1.6 mV / LSB
	 * @ex.
	 * */

	INACBffr.D32 = 0; //// buffer clear
	HAL_I2C_Mem_Read(hi2c, dv_addr, INA236_RG_BusV, I2C_MEMADD_SIZE_8BIT, &INACBffr.D8[1], 2, 10);

	return (INACBffr.D16[1] | INACBffr.D16[0]) * 1.6;
}

uint16_t INA236Read_Current(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
	/* @brief : Current read
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA236 device in the busline
	 * @Retval: current in mA, two's compliment
	 * @ex.
	 *
	 * RESOLUTION: 500uA/LSB = 0.5mA/LSB = 1/2mA
	 *
	 * The value of the Current Register is calculated by multiplying the decimal value
	 * in the Shunt Voltage Register with the decimal value of the Calibration Register.
	 * */
	INACBffr.D32 = 0; //// buffer clear
	HAL_I2C_Mem_Read(hi2c, dv_addr, INA236_RG_Current, I2C_MEMADD_SIZE_8BIT, &INACBffr.D8[1], 2, 10);

	return (INACBffr.D16[1] | INACBffr.D16[0]) / 2;
}

//float INA236Read_ShuntV(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
//	/* @brief : Shunt Voltage read & calculate
//	 * @param : hi2c - HAL_I2C used to read
//	 * @param : dv_addr - address of INA236 device in the busline
//	 * @Retval: shunt voltage in mV (.2f)
//	 * @ex.
//	 * */
//	INACBffr.D32 = 0; //// buffer clear
//	HAL_I2C_Mem_Read(hi2c, dv_addr, INA236_RG_ShuntV, I2C_MEMADD_SIZE_8BIT, &INACBffr.D8[1], 2, 10);
//
//	//// Convert rawdata To V shunt from Table 7.Shunt Voltage Register Format
//
//    int16_t rawshunt = INACBffr.D16[1] | INACBffr.D16[0];
//    return rawshunt;
//}

float INA236Read_Power(I2C_HandleTypeDef *hi2c,uint8_t dv_addr){
	/* @brief : Power read
	 * @param : hi2c - HAL_I2C used to read
	 * @param : dv_addr - address of INA236 device in the busline
	 * @Retval: Power in W,
	 * @ex.
	 *
	 *  The Power Register records power in Watts by multiplying the decimal values
	 *  of the Current Register with the decimal value of the Bus Voltage Register.
	 *  This is an unsigned result.
	 *
	 * */
	INACBffr.D32 = 0; //// buffer clear
	HAL_I2C_Mem_Read(hi2c, dv_addr, INA236_RG_PoWer, I2C_MEMADD_SIZE_8BIT, &INACBffr.D8[1], 2, 10);
	////  * 20, power_LSB = 20 x current_LSB & x 1000 make unit in mW
	return INACBffr.D16[1] | INACBffr.D16[0];
}
