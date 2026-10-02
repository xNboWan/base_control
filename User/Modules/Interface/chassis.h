/**
 * @file chassis.h
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief 底盘模块接口文件
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef CHASSIS_H
#define CHASSIS_H

#include "stdbool.h"

bool chassisInit(void);
void chassisTask(void *arg);

/* 需要手动修改的参数 */
/*轮距(mm)*/
#define WHEELTRACK 383.0f
/*轴距(mm)*/
#define WHEELBASE 574.0f
/*轮组到中心距离(mm)*/
#define WHEEL_TO_CORE_DISTANCE 275.0f
/*轮子半径(mm) */
#define WHEEL_RADIUS (129.0f * 0.5f / 1000.0f)

/*半轮距m*/
#define Rx (WHEELTRACK/2.0f/1000.0f)
/*半轴距m*/
#define Ry (WHEELBASE/2.0f/1000.0f)
/*轮子周长，单位 mm*/
#define WHEEL_PERIMETER (WHEEL_RADIUS * 2 * PI)
/*轮子周长，单位 m*/
#define WHEEL_PERIMETER_M (WHEEL_PERIMETER*0.001f)
/*M3508电机减速比(修改了减速箱)，减速比为268/17*/
#define GEAR_RATIO 15.76470588f
/*M2006电机减速比*/
#define M2006_RATIO 36.00f
/*滚子倾角（麦克纳姆）*/
#define WHEEL_ROLLER_ANGLE (45.0f / 180.0f * PI)





#endif