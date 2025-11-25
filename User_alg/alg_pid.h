#ifndef __ALG_PID_H
#define __ALG_PID_H

#include "main.h"

// 使用掩码格式进行模式设置
typedef enum PID_ModeTypeDef
{
    PID_MODE_POS = 0x00,        // 位置式
    PID_MODE_INC = 0x01,        // 增量式
    PID_MODE_DIFF_FIRST = 0x02,    // 微分先行
    PID_MODE_DIFF_NORMAL = 0x00,    // 不微分先行
    PID_MODE_INTEG_CHANGE = 0x04,    // 变速积分
    PID_MODE_INTEG_NORMAL = 0x00,    // 正常积分

} PID_ModeTypeDef;

typedef struct PID_HandleTypeDef
{
    PID_ModeTypeDef mode;       // 计算模式
    float target, actual, output;
    float act0, act1, act2;     // 微分先行专用
    float kp, ki, kd;
    float err0, err1, err2, err_int;
    float err_int_max, output_max, output_min; // 限幅专用
    float deadzone;     // 死区
    float compensation; // 补偿值

} PID_HandleTypeDef;


void PID_SetParam(PID_HandleTypeDef *pid, float kp, float ki, float kd);
void PID_Update(PID_HandleTypeDef *pid);
void PID_UpdateInc(PID_HandleTypeDef *pid);
HAL_StatusTypeDef PID_ParseSerialPack(PID_HandleTypeDef *pid, uint8_t *serial_pack);

#endif /*__PID_ALG_H*/
