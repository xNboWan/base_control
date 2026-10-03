/**
 * @file pid.c
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief PID 控制器实现
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "pid.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

static bool pidCfgValid(const pidCfg_t *cfg)
{
    return isfinite(cfg->kp) && isfinite(cfg->ki) && isfinite(cfg->kd)
        && isfinite(cfg->kff) && isfinite(cfg->out_min) && isfinite(cfg->out_max)
        && isfinite(cfg->integral_max) && isfinite(cfg->wrap)
        && isfinite(cfg->d_filter_alpha)
        && cfg->out_min <= cfg->out_max
        && cfg->integral_max >= 0.0f
        && cfg->wrap >= 0.0f && cfg->wrap <= FLT_MAX / 2.0f;
}

static float pidClamp(float value, float lower, float upper)
{
    if (value > upper) return upper;
    if (value < lower) return lower;
    return value;
}

/* 输入有限且 2 * half_range 可表示；保留原来的 ±half_range 边界语义。
 * 用取余代替反复相减，避免大角度下浮点舍入使循环无法推进。 */
static float pidWrap(float value, float half_range)
{
    if (half_range > 0.0f && (value > half_range || value < -half_range))
    {
        float range = 2.0f * half_range;
        value = fmodf(value, range);
        if (value > half_range) value -= range;
        if (value < -half_range) value += range;
    }
    return value;
}

void pidInit(pidController_t *pid, const pidCfg_t *cfg)
{
    if (pid == NULL) return;
    pid->cfg = cfg != NULL ? *cfg : (pidCfg_t){0};
    pidReset(pid);
}

void pidReset(pidController_t *pid)
{
    if (pid == NULL) return;
    pid->integral = 0.0f;
    pid->last_measurement = 0.0f;
    pid->d_filtered = 0.0f;
    pid->setpoint = 0.0f;
    pid->d_history_valid = false;
}

float pidCalc(pidController_t *pid, float setpoint, float measurement, float dt)
{
    if (pid == NULL || !isfinite(setpoint) || !isfinite(measurement)
        || !isfinite(dt) || dt <= 0.0f || !pidCfgValid(&pid->cfg))
        return 0.0f;

    if (!isfinite(pid->integral)
        || (pid->d_history_valid && (!isfinite(pid->last_measurement)
                                    || !isfinite(pid->d_filtered))))
        return 0.0f;

    /* 先在局部变量中计算；异常输入或数值溢出不能污染控制器历史。 */
    float err = setpoint - measurement;
    if (!isfinite(err)) return 0.0f;
    err = pidWrap(err, pid->cfg.wrap);

    float d = 0.0f;
    if (pid->d_history_valid)
    {
        float delta = measurement - pid->last_measurement;
        if (!isfinite(delta)) return 0.0f;
        delta = pidWrap(delta, pid->cfg.wrap);
        d = -delta / dt;
        if (!isfinite(d)) return 0.0f;

        float alpha = pid->cfg.d_filter_alpha;
        if (alpha > 0.0f && alpha < 1.0f)
            d = alpha * d + (1.0f - alpha) * pid->d_filtered;
        if (!isfinite(d)) return 0.0f;
    }

    float p = pid->cfg.kp * err;
    float feedforward = pid->cfg.kff * setpoint;
    float d_term = pid->cfg.kd * d;
    float next_integral = pid->integral + pid->cfg.ki * err * dt;
    if (!isfinite(p) || !isfinite(feedforward) || !isfinite(d_term)
        || !isfinite(next_integral))
        return 0.0f;

    if (pid->cfg.integral_max > 0.0f)
        next_integral = pidClamp(next_integral, -pid->cfg.integral_max,
                                pid->cfg.integral_max);

    float base = p + d_term + feedforward;
    float candidate_out = base + next_integral;
    float previous_integral_out = base + pid->integral;
    if (!isfinite(base) || !isfinite(candidate_out) || !isfinite(previous_integral_out))
        return 0.0f;

    /* 输出感知抗饱和：禁止积分加深饱和，但允许向解除饱和的方向变化。
     * 单步首次跨越限幅时保留恰好到达边界的积分，避免纯 I 控制器卡在零。 */
    if (next_integral > pid->integral && candidate_out > pid->cfg.out_max)
    {
        next_integral = previous_integral_out >= pid->cfg.out_max
            ? pid->integral : pid->cfg.out_max - base;
    }
    else if (next_integral < pid->integral && candidate_out < pid->cfg.out_min)
    {
        next_integral = previous_integral_out <= pid->cfg.out_min
            ? pid->integral : pid->cfg.out_min - base;
    }

    if (pid->cfg.integral_max > 0.0f)
        next_integral = pidClamp(next_integral, -pid->cfg.integral_max,
                                pid->cfg.integral_max);

    float out = base + next_integral;
    if (!isfinite(next_integral) || !isfinite(out)) return 0.0f;
    out = pidClamp(out, pid->cfg.out_min, pid->cfg.out_max);

    pid->integral = next_integral;
    pid->last_measurement = measurement;
    pid->d_filtered = d;
    pid->setpoint = setpoint;
    pid->d_history_valid = true;
    return out;
}
