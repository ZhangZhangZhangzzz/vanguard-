/*
 * pid.h
 *
 *  Created on: Feb 24, 2024
 *      Author: Admin
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef struct _pid_struct_t
{
  float kp;//比例
  float ki;//积分
  float kd;//微分
  float i_max;//积分限幅
  float out_max;//输出限幅

  float ref;      // target value目标角度
  float fdb;      // feedback value设定角度
  float err[2];   // error and last error差值

  float p_out;//比例输出
  float i_out;//积分输出
  float d_out;//微分输出
  float output;//pid总输出
}pid_struct_t;

extern pid_struct_t gimbal_yaw_speed_pid;
extern pid_struct_t gimbal_yaw_angle_pid;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);

void gimbal_PID_init(void);
float pid_calc(pid_struct_t *pid, float ref, float fdb);
double msp(double x, double in_min, double in_max, double out_min, double out_max);//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
#endif /* INC_PID_H_ */
