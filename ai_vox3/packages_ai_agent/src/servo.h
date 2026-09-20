/*
 * servo.h — AI-VOX3 舵机控制模块 (v102)
 *
 * 板卡: emakefun AI-VOX3 / ESP32-S3-R8
 * 舵机: PAN = GPIO3, TILT = GPIO4  (NuttX 板级 esp32s3_bringup.c 映射:
 *        /dev/pwm0 -> GPIO3 = PAN, /dev/pwm1 -> GPIO4 = TILT;
 *        Arduino 参考固件用 42/43，但本 NuttX 固件已映射到 3/4，以本文件为准)
 * 驱动: NuttX ESP32-S3 LEDC PWM -> /dev/pwm0 (PAN), /dev/pwm1 (TILT)
 *       defconfig 已含:
 *         CONFIG_ESP32S3_LEDC=y
 *         CONFIG_ESP32S3_LEDC_CHANNEL0_PIN=3    (pwm0 = PAN)
 *         CONFIG_ESP32S3_LEDC_CHANNEL1_PIN=4    (pwm1 = TILT)
 *         CONFIG_PWM=y
 *
 * 标准模拟舵机: 50Hz, 脉宽 1000us(0°) .. 2000us(180°), 中位 1500us(90°)
 */
#ifndef __PACKAGES_AI_AGENT_SRC_SERVO_H
#define __PACKAGES_AI_AGENT_SRC_SERVO_H

#ifdef __cplusplus
extern "C"
{
#endif

/* 设备节点: pwm0=PAN(左右), pwm1=TILT(上下) */
#define SERVO_PAN_DEV   "/dev/pwm0"
#define SERVO_TILT_DEV  "/dev/pwm1"

#define SERVO_FREQ_HZ     50        /* 标准模拟舵机频率 */
#define SERVO_PERIOD_US   20000     /* 50Hz 周期 = 20ms */

/* 角度范围 0..180，中位 90。
 * 若实际安装方向相反，把对应 INVERT 改为 1 即可（无需改硬件）。*/
#define SERVO_PAN_INVERT  0
#define SERVO_TILT_INVERT 0

/* ---- 对外 API ---- */
int  servo_init(void);            /* 打开设备、START、回中位。失败仅记日志不致命 */
void servo_deinit(void);
int  servo_pan(int angle);        /* 0=右 180=左 (INVERT 时相反) */
int  servo_tilt(int angle);       /* 0=下 180=上 (INVERT 时相反) */
int  servo_center(void);          /* pan=90 tilt=90 */
int  servo_nod(void);             /* 点一下头 (TILT 动画) */
int  servo_shake(void);           /* 摇一下头 (PAN 动画) */
int  servo_self_test(void);       /* 上电自检: 两轴 0->180->90 扫一遍，便于肉眼验证 */

#ifdef __cplusplus
}
#endif
#endif /* __PACKAGES_AI_AGENT_SRC_SERVO_H */
