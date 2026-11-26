/**
  ******************************************************************************
  * @file    pid.c
  * @brief   位置式 / 增量式通用 PID
  ******************************************************************************
  */
#include "pid.h"

/**
  * @brief  限幅
  */
static float pid_limit(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/**
  * @brief  PID 初始化
  * @param  para[0]=Kp  para[1]=Ki  para[2]=Kd
  * @param  max_out  总输出限幅
  * @param  max_iout 积分项限幅
  * @param  dead_zone 误差死区
  */
void PID_Parameter_Init(pid_type *pid, PID_MODE mode, const float para[3],
                        float max_out, float max_iout, float dead_zone)
{
    if (pid == 0) return;

    pid->pidmode = mode;

    pid->kp = para[0];
    pid->ki = para[1];
    pid->kd = para[2];

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->sum_error = 0.0f;

    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->Out  = 0.0f;

    pid->MaxOut   = (max_out  > 0.0f) ? max_out  : 0.0f;
    pid->MaxIout  = (max_iout > 0.0f) ? max_iout : 0.0f;
    pid->DeadZone = (dead_zone > 0.0f) ? dead_zone : 0.0f;
}

/**
  * @brief  运行中改参数
  */
void PID_Parameter_Update(pid_type *pid, const float para[3])
{
    if (pid == 0) return;
    pid->kp = para[0];
    pid->ki = para[1];
    pid->kd = para[2];
}

/**
  * @brief  清零历史量
  */
void PID_clear(pid_type *pid)
{
    if (pid == 0) return;

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->sum_error = 0.0f;
    pid->Pout = pid->Iout = pid->Dout = pid->Out = 0.0f;
}

/**
  * @brief  PID 计算，返回限幅后的输出
  */
float Pid_cal(pid_type *pid, float dst, float real)
{
    if (pid == 0) return 0.0f;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = dst - real;

    /* 死区：抑制抖动 */
    if (pid->DeadZone > 0.0f)
    {
        if (pid->error[0] <  pid->DeadZone &&
            pid->error[0] > -pid->DeadZone)
        {
            pid->error[0] = 0.0f;
        }
    }

    if (pid->pidmode == PID_POSITION)
    {
        pid->Pout = pid->kp * pid->error[0];

        /* 积分项限幅,避免堵转时 sum_error 越积越大 */
        pid->sum_error += pid->error[0];
        pid->Iout = pid->ki * pid->sum_error;

        if (pid->MaxIout > 0.0f)
        {
            if (pid->Iout > pid->MaxIout)
            {
                pid->Iout = pid->MaxIout;
                if (pid->ki > 0.0f) pid->sum_error = pid->MaxIout / pid->ki;
            }
            else if (pid->Iout < -pid->MaxIout)
            {
                pid->Iout = -pid->MaxIout;
                if (pid->ki > 0.0f) pid->sum_error = -pid->MaxIout / pid->ki;
            }
        }

        /* 微分 */
        pid->Dout = pid->kd * (pid->error[0] - pid->error[1]);

        pid->Out = pid->Pout + pid->Iout + pid->Dout;
        pid->Out = pid_limit(pid->Out, -pid->MaxOut, pid->MaxOut);
    }
    else /* PID_DELTA */
    {
        pid->Pout = pid->kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->ki * pid->error[0];
        pid->Dout = pid->kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

        pid->Out += pid->Pout + pid->Iout + pid->Dout;
        pid->Out = pid_limit(pid->Out, -pid->MaxOut, pid->MaxOut);
    }

    return pid->Out;
}
