// Based on the example code from:
// https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/98b62f0e4f7cf79fb1464012f4ba13f3c42c959b/asm330lhhxg1_STdC/examples/asm330lhhxg1_read_data_interrupt.c#L225

/* Include -------------------------------------------------------------------*/
#include "asm330lhh.h"
#include "../../Drivers/asm330lhh/asm330lhhxg1_reg.h"
#include "main.h"
#include "state_machine.h"
#include "string.h"

/* Macros --------------------------------------------------------------------*/
#define BOOT_TIME 10 // [ms]
#define TIMEOUT 1000 // [ms]

/* Declare file-scope variables ----------------------------------------------*/

// Raw measurements
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];

// IMU configuration
static uint8_t whoamI, rst;
static stmdev_ctx_t dev_ctx;

// INT1 and INT2 configuration
static asm330lhhxg1_pin_int1_route_t int1_route;
static asm330lhhxg1_den_mode_t den_mode;
static asm330lhhxg1_den_lh_t den_lh;

/* Declare external variables ------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;

/* Declare file-scope functions ----------------------------------------------*/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);

/* Define functions ----------------------------------------------------------*/

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

void configure_imu(void) {

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

}

void configure_imu_without_DEN(void) {

	// Generate interrupt on INT1 when accelerometer data is ready
	asm330lhhxg1_pin_int1_route_get(&dev_ctx, &int1_route);
	int1_route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
	asm330lhhxg1_pin_int1_route_set(&dev_ctx, &int1_route);

}

void configure_imu_with_DEN(void) {

	// Configure DEN
	// DEN procedure:
	// 1. STM32 sends active high signal to INT2 pin (DEN)
	// 2. IMU stores measurements in output registers
	// 3. IMU sends active high signal on INT1 pin
	// 4. STM32 reads measurements

	// Set INT1_CTRL register to trigger data ready (DRDY) flag when DEN is triggered
	asm330lhhxg1_pin_int1_route_get(&dev_ctx, &int1_route);
	int1_route.int1_ctrl.den_drdy_flag = PROPERTY_ENABLE;
	asm330lhhxg1_pin_int1_route_set(&dev_ctx, &int1_route);

	// Set DEN to be in "level latched" mode in CTRL6_C register
	asm330lhhxg1_den_mode_get(&dev_ctx, &den_mode);
	den_mode = ASM330LHHXG1_LEVEL_LETCHED;
	asm330lhhxg1_den_mode_set(&dev_ctx, den_mode);

	// Set DEN to be "active high" in CTRL9_XL register
	asm330lhhxg1_den_polarity_get(&dev_ctx, &den_lh);
	den_lh = ASM330LHHXG1_DEN_ACT_HIGH;
	asm330lhhxg1_den_polarity_set(&dev_ctx, den_lh);

	// Stamp DEN value in z-axis accelerometer LSB (CTRL9_XL register)
	asm330lhhxg1_den_mark_axis_x_set(&dev_ctx, 0);
	asm330lhhxg1_den_mark_axis_y_set(&dev_ctx, 0);
	asm330lhhxg1_den_mark_axis_z_set(&dev_ctx, 1);
	// This last command does two things:
	// 1. Stamp in z-axis accelerometer
	// 2. Extends DEN functionality to accelerometer: DEN_XL_EN = 1
	// Source: https://github.com/STMicroelectronics/asm330lhhxg1-pid/issues/1#issuecomment-4864940543
	asm330lhhxg1_den_enable_set(&dev_ctx, ASM330LHHXG1_STAMP_IN_XL_DATA);
	// The "asm330lhhxg1_den_enable_set" function does not work correctly, it sets DEN_XL_G to 1 (as expected)
	// but it does not set DEN_XL_EN to 1, therefore, this has to be done manually.
	uint8_t buffer[8];
	platform_read(&hspi2, 0x18, buffer, 1);
	buffer[0] = buffer[0] | 0x08;
	platform_write(&hspi2, 0x18, buffer, 1);

}

void read_measurements(float acceleration_mg[3], float angular_rate_mdps[3]) {

	// Flag to check that data is ready
	uint8_t data_ready;

	// Read acceleration field data
	asm330lhhxg1_xl_flag_data_ready_get(&dev_ctx, &data_ready);

	// Convert acceleration to mg and store
	if (data_ready) {
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		asm330lhhxg1_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

		// Check that x-axis accelerometer reading is stamped
		if ((data_raw_acceleration[2] & 0x01) != 0x01) {
			error_state = DEN_MEASUREMENTS_NOT_STAMPED;
			Error_Handler();
		}

		acceleration_mg[0] = asm330lhhxg1_from_fs4g_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] = asm330lhhxg1_from_fs4g_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] = asm330lhhxg1_from_fs4g_to_mg(data_raw_acceleration[2]);
	}

    // Read angular rate field data
	asm330lhhxg1_gy_flag_data_ready_get(&dev_ctx, &data_ready);

	// Convert angular rate to mdps and store
	if (data_ready) {
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		asm330lhhxg1_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
		angular_rate_mdps[0] = asm330lhhxg1_from_fs500dps_to_mdps(data_raw_angular_rate[0]);
		angular_rate_mdps[1] = asm330lhhxg1_from_fs500dps_to_mdps(data_raw_angular_rate[1]);
		angular_rate_mdps[2] = asm330lhhxg1_from_fs500dps_to_mdps(data_raw_angular_rate[2]);
	}

}
