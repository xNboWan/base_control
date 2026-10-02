/**
 * @file generic_def.h
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief 该文件中存放了一些常用的宏
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef GENERIC_DEF_H
#define GENERIC_DEF_H

#define GRAVITY 9.792f
#define PI 3.1415926535f

typedef enum
{
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z
} axis_e;

typedef enum
{
    ROLL = 0,  // 横滚角
    PITCH,     // 俯仰角
    YAW        // 偏航角
} eular_e;

#define CHECK(func)  do { if (!(func)) return false; } while (0)


#endif