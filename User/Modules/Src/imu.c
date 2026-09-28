#include "imu.h"

bool imuInit(imu_t *imu)
{
    return imu->ops->init(imu->ctx);
}

bool imuRead(imu_t *imu, imuData_t *data)
{
    return imu->ops->read(imu->ctx, data);
}