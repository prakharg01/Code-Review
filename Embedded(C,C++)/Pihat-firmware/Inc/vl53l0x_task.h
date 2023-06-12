/*
 * vl53l0x_task.h
 *
 *  Created on: 23-Apr-2021
 *      Author: Prakhar Goel
 */

#ifndef INC_VL53L0X_TASK_H_
#define INC_VL53L0X_TASK_H_
#include <vl53l0x.h>


extern struct VL53L0X myTOFsensor1;
extern struct VL53L0X myTOFsensor2;
extern struct VL53L0X myTOFsensor3;
extern struct VL53L0X myTOFsensor4;
extern struct VL53L0X myTOFsensor5;
extern struct VL53L0X myTOFsensor6;

enum sensor_id {sensor_1=1,sensor_2=2,sensor_3=3,sensor_4=4,sensor_5=5,sensor_6=6};

void init_sensors(void);
void read_sensors(void);


#endif /* INC_VL53L0X_TASK_H_ */

