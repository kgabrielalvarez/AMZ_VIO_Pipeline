/* Include -------------------------------------------------------------------*/
#include "asm330lhh.h"
#include "string.h"

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static uint8_t whoamI, rst;
static stmdev_ctx_t dev_ctx;

/* Define Private functions --------------------------------------------------*/

/*
 * @brief  Write generic device register
 *
 * @param  handle    sensor bus handler
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, TIMEOUT);
    HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, TIMEOUT);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    return 0;
}

/*
 * @brief  Read generic device register
 *
 * @param  handle    sensor bus handler
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    reg |= 0x80;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, TIMEOUT);
    HAL_SPI_Receive(handle, bufp, len, TIMEOUT);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    return 0;
}

/*
 * @brief  platform specific delay
 *
 * @param  ms        delay in ms
 *
 */
void platform_delay(uint32_t ms) {
  HAL_Delay(ms);
}

// @brief Configure IMU
void configure_imu(void) {

	// Configure INT1
	asm330lhhxg1_pin_int1_route_t int1_route;

    // Initialize mems driver interface
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &hspi2;

    // Wait sensor boot time
    HAL_Delay(BOOT_TIME);

    // Check device ID
    asm330lhhxg1_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != ASM330LHHXG1_ID) while(1);

    // Restore default configuration
    asm330lhhxg1_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
        asm330lhhxg1_reset_get(&dev_ctx, &rst);
    } while (rst);

    // Disable I3C interface
    asm330lhhxg1_i3c_disable_set(&dev_ctx, ASM330LHHXG1_I3C_DISABLE);

    // Enable Block Data Update
    asm330lhhxg1_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    // Set Output Data Rate
    asm330lhhxg1_xl_data_rate_set(&dev_ctx, ASM330LHHXG1_XL_ODR_833Hz);
    asm330lhhxg1_gy_data_rate_set(&dev_ctx, ASM330LHHXG1_GY_ODR_833Hz);

    // Set full scale
    asm330lhhxg1_xl_full_scale_set(&dev_ctx, ASM330LHHXG1_4g);
    asm330lhhxg1_gy_full_scale_set(&dev_ctx, ASM330LHHXG1_500dps);

    // Generate interrupt on INT1 when accelerometer data is ready
    asm330lhhxg1_pin_int1_route_get(&dev_ctx, &int1_route);
    int1_route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
    asm330lhhxg1_pin_int1_route_set(&dev_ctx, &int1_route);

    // ---------------------- UNCOMMENT FOR DEN ----------------------------------------
//    // Generate interrupt on INT1 when DEN signal is high (level-sensitive latched mode)
//    // Set INT1_CTRL register
//    asm330lhhxg1_pin_int1_route_get(&dev_ctx, &int1_route);
//	int1_route.int1_ctrl.den_drdy_flag = PROPERTY_ENABLE;
//	asm330lhhxg1_pin_int1_route_set(&dev_ctx, &int1_route);
//    // Set CTRL6_C register
//	asm330lhhxg1_den_mode_set(&dev_ctx, ASM330LHHXG1_LEVEL_LETCHED);
//	// Set CTRL9_XL register: DEN_X, DEN_Y, and DEN_Z bits
//	asm330lhhxg1_den_mark_axis_x_set(&dev_ctx, 0);
//	asm330lhhxg1_den_mark_axis_y_set(&dev_ctx, 0);
//	asm330lhhxg1_den_mark_axis_z_set(&dev_ctx, 1);
//	// Set CTRL9_XL register: DEN_XL_G bit
//	asm330lhhxg1_den_enable_set(&dev_ctx, ASM330LHHXG1_STAMP_IN_XL_DATA);
//	// Set CTRL9_XL register: DEN_XL_EN bit (need to do manually as there is no function to do this)
//	uint8_t buffer[8];
//	platform_read(&hspi2, 0x18, buffer, 1);
//	buffer[0] = buffer[0] | 0x08;
//	platform_write(&hspi2, 0x18, buffer, 1);
//	// NOTE: REMEMBER THAT THE DEN_LH LINE NEEDS TO BE CONFIGURED ACCORDING TO HOW THE
//	// DEN PIN IS TRIGGERED BY THE MICROCONTROLLER --> EITHER ACTIVE HIGH OR ACTIVE LOW

}

// @brief Read Measurements
void read_measurements(float_t acceleration_mg[3], float_t angular_rate_mdps[3]) {

	uint8_t reg;

	// NOTE: REMEMBER THAT YOU NEED TO CHECK THE LSB OF THE X-AXIS ACCELEROMETER
	// READING TO CONFIRM THAT THE DEN WAS PROPERLY TRIGGERED.

	// Read acceleration field data
	asm330lhhxg1_xl_flag_data_ready_get(&dev_ctx, &reg);
	if (reg) {
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		asm330lhhxg1_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
		acceleration_mg[0] = asm330lhhxg1_from_fs4g_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] = asm330lhhxg1_from_fs4g_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] = asm330lhhxg1_from_fs4g_to_mg(data_raw_acceleration[2]);
	}

    // Read angular rate field data
	asm330lhhxg1_gy_flag_data_ready_get(&dev_ctx, &reg);
	if (reg) {
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		asm330lhhxg1_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
		angular_rate_mdps[0] = asm330lhhxg1_from_fs500dps_to_mdps(data_raw_angular_rate[0]);
		angular_rate_mdps[1] = asm330lhhxg1_from_fs500dps_to_mdps(data_raw_angular_rate[1]);
		angular_rate_mdps[2] = asm330lhhxg1_from_fs500dps_to_mdps(data_raw_angular_rate[2]);
	}

}
