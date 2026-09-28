#ifndef COMMON_H
#define COMMON_H

typedef enum
{
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z
} axis;

#define CHECK(func)  do { if (!(func)) return false; } while (0)


#endif