#ifndef asm330lhh
#define asm330lhh

/* Include -------------------------------------------------------------------*/
#include "stdint.h"

/* Declare functions ---------------------------------------------------------*/
void configure_imu(int32_t rate);
void read_imu_measurements(float acceleration_mg[3], float angular_rate_mdps[3]);
void read_imu_timestamp(uint32_t *timestamp);

#endif
