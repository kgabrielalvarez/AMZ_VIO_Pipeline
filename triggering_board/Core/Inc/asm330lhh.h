#ifndef asm330lhh
#define asm330lhh

/* Declare functions ---------------------------------------------------------*/
void configure_imu(void);
void configure_imu_without_DEN(void);
void configure_imu_with_DEN(void);
void read_measurements(float acceleration_mg[3], float angular_rate_mdps[3]);

#endif
