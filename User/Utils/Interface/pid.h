/**
 * @file pid.h
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief PID 控制器接口文件
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>

/**
 * @brief PID 控制器（微分先行 + 抗饱和 + 角度环绕 + D 项滤波 + 前馈）
 *
 * 用法：
 *   pidCfg_t cfg = {.kp = .., .ki = .., .kd = ..,
 *                      .out_max = .., .out_min = ..,
 *                      .integral_max = .., .wrap = ..,
 *                      .d_filter_alpha = 1.0f};   // 1.0f = D 项不过滤
 *   pidController_t pid;
 *   pidInit(&pid, &cfg);
 *   float out = pidCalc(&pid, setpoint, measurement, dt);
 */

typedef struct
{
    float kp;               // 比例增益
    float ki;               // 积分增益
    float kd;               // 微分增益
    float kff;              // 前馈增益，置 0 关闭

    float out_max;          // 输出上限
    float out_min;          // 输出下限

    float integral_max;     // I 项幅值上限（与输出同单位），0 = 关闭此幅值限制
                            // 输出限幅仍参与抗饱和，防止积分继续加深饱和

    float wrap;             // 角度环绕半幅（弧度），>0 开启：误差卷回 [-wrap, wrap]
                            //   偏航角环填 PI，速度环等非角度环填 0；不得为负

    float d_filter_alpha;   // D 项低通滤波系数，取 (0,1) 开启，其余值关闭
                            //   值越小滤波越强；噪声大的反馈（如 M3508 转速）建议 0.1~0.3
} pidCfg_t;

typedef struct
{
    pidCfg_t cfg;

    float integral;          // 积分累加
    float last_measurement;  // 上次测量值（微分先行）
    float d_filtered;        // D 项滤波后的值
    float setpoint;          // 最近一次有效计算的目标值，供外部只读观测；复位归零
    bool  d_history_valid;   // 复位时 false，首次有效计算记录测量值后置 true
} pidController_t;

/* 必须先初始化再计算。配置中的数值应有限，out_min <= out_max，
 * integral_max >= 0，0 <= wrap <= FLT_MAX/2。NULL cfg 初始化为零配置。
 * setpoint 字段仅为观测快照；实际目标始终使用 pidCalc 的参数。
 * 同一实例由一个控制任务更新，对外并发读取应通过上层快照接口。 */
void  pidInit(pidController_t *pid, const pidCfg_t *cfg);
void  pidReset(pidController_t *pid);
/* 非法输入、配置或数值溢出返回 0，且不更新状态；正常返回已限幅的输出。
 * 复位后的第一帧只建立微分历史，D 项为 0，P/I/前馈仍正常计算。 */
float pidCalc(pidController_t *pid, float setpoint, float measurement, float dt);

#endif
