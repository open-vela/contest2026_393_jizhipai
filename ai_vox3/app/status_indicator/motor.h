/****************************************************************************
 * motor.h — 马达震动模式封装
 *
 * 驱动接口：/dev/pwm0 (GPTIM1_CH3)
 * 非阻塞：lv_timer 驱动状态机
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#ifndef __MOTOR_H
#define __MOTOR_H

void motor_init(void);
void motor_on(void);
void motor_off(void);
void motor_notify(void);   /* 单次 200ms */
void motor_alert(void);    /* 三短震 200ms×3 间隔150ms */
void motor_wakeup(void);   /* 单次 100ms */
void motor_deinit(void);

#endif
