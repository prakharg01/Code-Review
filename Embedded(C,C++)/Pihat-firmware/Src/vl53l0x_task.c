/*
 * vl53l0x_task.c
 *
 *  Created on: 23-Apr-2021
 *      Author: Prakhar Goel
 */

#include "vl53l0x_task.h"
#include "register_bank.h"
#include "timer.h"

#define SENSOR_TIMEOUT 65535

typedef enum{
	eState_wait,
	eState_read1,
	eState_read2,
	eState_read3,
	eState_read4,
	eState_read5,
	eState_read6,
	eState_storeData
} eStates;

eStates fsm_state;
uint32_t last_read_tick=0;
uint16_t rawValue[6]= {0};

struct VL53L0X myTOFsensor1 = {.io_2v8 = true, .address = 0x29, .io_timeout = 100, .did_timeout = false};
struct VL53L0X myTOFsensor2 = {.io_2v8 = true, .address = 0x29, .io_timeout = 100, .did_timeout = false};
struct VL53L0X myTOFsensor3 = {.io_2v8 = true, .address = 0x29, .io_timeout = 100, .did_timeout = false};
struct VL53L0X myTOFsensor4 = {.io_2v8 = true, .address = 0x29, .io_timeout = 100, .did_timeout = false};
struct VL53L0X myTOFsensor5 = {.io_2v8 = true, .address = 0x29, .io_timeout = 100, .did_timeout = false};
struct VL53L0X myTOFsensor6 = {.io_2v8 = true, .address = 0x29, .io_timeout = 100, .did_timeout = false};


/**
  * @brief This function provides method for initializing the sensors
  * @note Sets the address and starts continuous mode
  * @param  None
  * @retval None
  */
void init_sensors(void)
{

//	   uint8_t addressSensor[6] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
	   int errorFlag = 0;

	   //shutdown sensors
	   VL53L0X_Xshut_Deactivate(sensor_1);
	   VL53L0X_Xshut_Deactivate(sensor_2);
       VL53L0X_Xshut_Deactivate(sensor_3);
       VL53L0X_Xshut_Deactivate(sensor_4);
       VL53L0X_Xshut_Deactivate(sensor_5);
       VL53L0X_Xshut_Deactivate(sensor_6);

       delay(100);
       //address change routine

       VL53L0X_Xshut_Activate(sensor_1);
       delay(10);
	   VL53L0X_setAddress(&myTOFsensor1, ToF_ADDR1);
	   delay(10);

	   VL53L0X_Xshut_Activate(sensor_2);
	   delay(10);
	   VL53L0X_setAddress(&myTOFsensor2, ToF_ADDR2);
	   delay(10);

	   VL53L0X_Xshut_Activate(sensor_3);
	   delay(10);
	   VL53L0X_setAddress(&myTOFsensor3, ToF_ADDR3);
	   delay(10);

	   VL53L0X_Xshut_Activate(sensor_4);
	   delay(10);
	   VL53L0X_setAddress(&myTOFsensor4, ToF_ADDR4);
	   delay(10);

	   VL53L0X_Xshut_Activate(sensor_5);
	   delay(10);
	   VL53L0X_setAddress(&myTOFsensor5, ToF_ADDR5);
	   delay(10);

	   VL53L0X_Xshut_Activate(sensor_6);
	   delay(10);
	   VL53L0X_setAddress(&myTOFsensor6, ToF_ADDR6);
	   delay(50);


	   if ( VL53L0X_init(&myTOFsensor1) )
	     {
	   	  print_msg("success\r\n");
	     }
	     else
	     {
	   	 print_msg("failed\r\n");
	   	 errorFlag = 1;
	     }

	   if ( VL53L0X_init(&myTOFsensor2) )
	  	     {
	  	   	  print_msg("success\r\n");
	  	     }
	  	     else
	  	     {
	  	   	 print_msg("failed\r\n");
	  	   	 errorFlag = 1;
	  	     }

	   if ( VL53L0X_init(&myTOFsensor3) )
	  	     {
	  	   	  print_msg("success\r\n");
	  	     }
	  	     else
	  	     {
	  	   	 print_msg("failed\r\n");
	  	   	 errorFlag = 1;
	  	     }

	   if ( VL53L0X_init(&myTOFsensor4) )
	  	     {
	  	   	  print_msg("success\r\n");
	  	     }
	  	     else
	  	     {
	  	   	 print_msg("failed\r\n");
	  	   	 errorFlag = 1;
	  	     }
	   if ( VL53L0X_init(&myTOFsensor5) )
	  	     {
	  	   	  print_msg("success\r\n");
	  	     }
	  	     else
	  	     {
	  	   	 print_msg("failed\r\n");
	  	   	 errorFlag = 1;
	  	     }
	   if ( VL53L0X_init(&myTOFsensor6) )
	  	     {
	  	   	  print_msg("success\r\n");
	  	     }
	  	     else
	  	     {
	  	   	 print_msg("failed\r\n");
	  	   	 errorFlag = 1;
	  	     }


	   if(errorFlag == 1)
	   {
	     print_msg("error\r\n");
	     errorFlag = 0;
	   }

	   VL53L0X_setMeasurementTimingBudget(&myTOFsensor1, 70000);//20000
	   VL53L0X_setMeasurementTimingBudget(&myTOFsensor2, 70000);//20000
	   VL53L0X_setMeasurementTimingBudget(&myTOFsensor3, 70000);//20000
	   VL53L0X_setMeasurementTimingBudget(&myTOFsensor4, 70000);//20000
	   VL53L0X_setMeasurementTimingBudget(&myTOFsensor5, 70000);//20000
	   VL53L0X_setMeasurementTimingBudget(&myTOFsensor6, 70000);//20000

	   VL53L0X_startContinuous(&myTOFsensor1, 0); //0 or 100
	   VL53L0X_startContinuous(&myTOFsensor2, 0);
	   VL53L0X_startContinuous(&myTOFsensor3, 0);
	   VL53L0X_startContinuous(&myTOFsensor4, 0);
	   VL53L0X_startContinuous(&myTOFsensor5, 0);
	   VL53L0X_startContinuous(&myTOFsensor6, 0);

	   fsm_state = eState_wait;
	   last_read_tick = get_tick_micro();
}
/**
  * @brief This function provides method for returning the sensor values after correction for 6 sensors
  * @note Reads in continuous mode
  * @param  None
  * @retval None // can be float*
  */
void read_sensors(void)
{

	uint32_t current_tick;

	switch(fsm_state) {
	case eState_wait:

		current_tick = get_tick_micro();
		if ((current_tick - last_read_tick) > 70000)
		{
			last_read_tick = current_tick;
			fsm_state = eState_read1;
		}
		break;

	case eState_read1:
		rawValue[0] = VL53L0X_readRangeContinuousMillimeters(&myTOFsensor1);
		rawValue[0] = (uint16_t)((float)rawValue[0] * CORRECTION_FACTOR);

		if ( VL53L0X_timeoutOccurred(&myTOFsensor1) )
			rawValue[0]=SENSOR_TIMEOUT;

		fsm_state = eState_read2;
		break;

	case eState_read2:
		rawValue[1] = VL53L0X_readRangeContinuousMillimeters(&myTOFsensor2);
		rawValue[1] = (uint16_t)((float)rawValue[1] * CORRECTION_FACTOR);

		if ( VL53L0X_timeoutOccurred(&myTOFsensor2) )
			rawValue[1]=SENSOR_TIMEOUT;

		fsm_state = eState_read3;
		break;

	case eState_read3:
		rawValue[2] = VL53L0X_readRangeContinuousMillimeters(&myTOFsensor3);
		rawValue[2] = (uint16_t)((float)rawValue[2] * CORRECTION_FACTOR);

		if ( VL53L0X_timeoutOccurred(&myTOFsensor3) )
			rawValue[2]=SENSOR_TIMEOUT;

		fsm_state = eState_read4;
		break;

	case eState_read4:
		rawValue[3] = VL53L0X_readRangeContinuousMillimeters(&myTOFsensor4);
		rawValue[3] = (uint16_t)((float)rawValue[3] * CORRECTION_FACTOR);

		if ( VL53L0X_timeoutOccurred(&myTOFsensor4) )
			rawValue[3]=SENSOR_TIMEOUT;

		fsm_state = eState_read5;
		break;

	case eState_read5:
		rawValue[4] = VL53L0X_readRangeContinuousMillimeters(&myTOFsensor5);
		rawValue[4] = (uint16_t)((float)rawValue[4] * CORRECTION_FACTOR);

		if ( VL53L0X_timeoutOccurred(&myTOFsensor5) )
			rawValue[4]=SENSOR_TIMEOUT;

		fsm_state = eState_read6;
		break;

	case eState_read6:
		rawValue[5] = VL53L0X_readRangeContinuousMillimeters(&myTOFsensor6);
		rawValue[5] = (uint16_t)((float)rawValue[5] * CORRECTION_FACTOR);

		if ( VL53L0X_timeoutOccurred(&myTOFsensor6) )
			rawValue[5]=SENSOR_TIMEOUT;

		fsm_state = eState_storeData;
		break;

	case eState_storeData:

		write_sensor_data(TOF_ADDR, (uint8_t*) rawValue, 6*2);

		fsm_state = eState_wait;
		break;
	}

}
