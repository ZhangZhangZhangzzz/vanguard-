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
  float kp;//����
  float ki;//����
  float kd;//΢��
  float i_max;//�����޷�
  float out_max;//����޷�

  float ref;      // target valueĿ��Ƕ�
  float fdb;      // feedback value�趨�Ƕ�
  float err[2];   // error and last error��ֵ

  float p_out;//�������
  float i_out;//�������
  float d_out;//΢�����
  float output;//pid�����
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
double msp(double x, double in_min, double in_max, double out_min, double out_max);//ӳ�亯��������������ֵ��0~8191��ת��Ϊ�����ƵĽǶ�ֵ��-pi~pi��
#endif /* INC_PID_H_ */
