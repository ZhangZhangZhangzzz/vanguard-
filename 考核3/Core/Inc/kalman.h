/*
 * kalman.h
 *
 *  Created on: Mar 21, 2022
 *      Author: LX
 */





#ifndef __KALMAN_H
#define __KALMAN_H

//卡尔曼解算法库

#include "main.h"
#include "mpu6050.h"


extern float Angle_X_Final;			//解算后俯仰角
extern float Angle_Y_Final;			//解算后横滚角
extern IMU_Parameter IMU_Data;
extern uint8_t Exam_i;


void Angle_Calcu(void);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);
void Yaw();

#endif

