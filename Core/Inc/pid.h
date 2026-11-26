/**
  ******************************************************************************
  * @file    pid.h
  * @brief   位置式 / 增量式通用 PID（裸机版，去掉队内 BSP 的 FreeRTOS 依赖）
  ******************************************************************************
  * 用法：
  *     pid_type spd;
  *     const float p[3] = {15.0f, 0.3f, 0.0f};      // Kp, Ki, Kd
  *     PID_Parameter_Init(&spd, PID_POSITION, p, 6000.0f, 4000.0f, 0.0f);
  *     out = Pid_cal(&spd, target, feedback);       // 固定周期调用，如 1ms
  *
  * 注意：函数内部没有 dt，采样周期隐含在增益里。把调用周期从 1ms 改成 2ms，
  *       Ki 和 Kd 要相应减半 / 加倍，否则表现会变。
  ******************************************************************************
  */
#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

typedef enum
{
    PID_POSITION = 0,   /* 位置式：输出为绝对量，掉电重启时输出归零 */
    PID_DELTA           /* 增量式：输出在上一拍基础上累加，适合速度环 */
} PID_MODE;

typedef struct
{
    PID_MODE pidmode;

    float kp;
    float ki;
    float kd;

    float error[3];     /* [0] 本次误差  [1] 上次  [2] 上上次 */
    float sum_error;    /* 积分累加 */

    float Pout;
    float Iout;
    float Dout;
    float Out;

    float MaxOut;       /* 总输出限幅（必须给） */
    float MaxIout;      /* 积分项限幅（防积分饱和） */
    float DeadZone;     /* 误差死区，0 表示关闭 */
} pid_type;

void  PID_Parameter_Init(pid_type *pid, PID_MODE mode, const float para[3],
                         float max_out, float max_iout, float dead_zone);
void  PID_Parameter_Update(pid_type *pid, const float para[3]);
void  PID_clear(pid_type *pid);
float Pid_cal(pid_type *pid, float dst, float real);

#endif /* __PID_H__ */
