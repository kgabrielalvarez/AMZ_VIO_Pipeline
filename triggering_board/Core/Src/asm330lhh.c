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
    if (HAL_SPI_Transmit(handle, &reg, 1, TIMEOUT) != HAL_OK) {
    	error_state = SPI_TRANSMIT_FAILED;
    	Error_Handler();
    }
    if (HAL_SPI_Receive(handle, bufp, len, TIMEOUT) != HAL_OK) {
    	error_state = SPI_RECEIVE_FAILED;
    	Error_Handler();
    }
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

void configure_imu(int32_t rate) {

    // Initialize mems driver interface
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &hspi2;

    // Wait sensor boot time
    HAL_Delay(BOOT_TIME);

    // Check device ID
    if (asm330lhhxg1_device_id_get(&dev_ctx, &whoamI) != 0) {
    	error_state = FAILED_TO_READ_IMU_WHOAMI;
    	Error_Handler();
    }
    if (whoamI != ASM330LHHXG1_ID) {
    	error_state = WHOAMI_REGISTER_INCORRECT;
    	Error_Handler();
    }

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
    asm330lhhxg1_odr_xl_t accelerometer_rate;
    asm330lhhxg1_odr_g_t gyroscope_rate;
    switch (rate) {
    	case 26:
    		accelerometer_rate = ASM330LHHXG1_XL_ODR_26Hz;
			gyroscope_rate = ASM330LHHXG1_GY_ODR_26Hz;
			break;
		case 52:
			accelerometer_rate = ASM330LHHXG1_XL_ODR_52Hz;
			gyroscope_rate = ASM330LHHXG1_GY_ODR_52Hz;
			break;
		case 104:
			accelerometer_rate = ASM330LHHXG1_XL_ODR_104Hz;
			gyroscope_rate = ASM330LHHXG1_GY_ODR_104Hz;
			break;
		case 208:
			accelerometer_rate = ASM330LHHXG1_XL_ODR_208Hz;
			gyroscope_rate = ASM330LHHXG1_GY_ODR_208Hz;
			break;
		case 416:
			// NOTE: In the datasheet it says 416 Hz but in the enum that's not an option
			accelerometer_rate = ASM330LHHXG1_XL_ODR_417Hz;
			gyroscope_rate = ASM330LHHXG1_GY_ODR_417Hz;
			break;
		case 833:
			accelerometer_rate = ASM330LHHXG1_XL_ODR_833Hz;
			gyroscope_rate = ASM330LHHXG1_GY_ODR_833Hz;
			break;
		default:
			error_state = REQUESTED_IMU_RATE_NOT_AVAILABLE;
			Error_Handler();
    }
    asm330lhhxg1_xl_data_rate_set(&dev_ctx, accelerometer_rate);
    asm330lhhxg1_gy_data_rate_set(&dev_ctx, gyroscope_rate);

    // Set full scale
    asm330lhhxg1_xl_full_scale_set(&dev_ctx, ASM330LHHXG1_4g);
    asm330lhhxg1_gy_full_scale_set(&dev_ctx, ASM330LHHXG1_500dps);

	// Generate interrupt on INT1 when accelerometer data is ready
	asm330lhhxg1_pin_int1_route_get(&dev_ctx, &int1_route);
	int1_route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
	asm330lhhxg1_pin_int1_route_set(&dev_ctx, &int1_route);

	// Enable timestamps
	asm330lhhxg1_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

	// Configure DRDY flag to be latched
	asm330lhhxg1_data_ready_mode_set(&dev_ctx, ASM330LHHXG1_DRDY_LATCHED);

	// Read freq_fine register
	asm330lhhxg1_odr_cal_reg_get(&dev_ctx, &freq_fine);

}

void read_imu_measurements(float acceleration_mg[3], float angular_rate_mdps[3]) {

	// Flag to check that data is ready
	uint8_t data_ready;

	// Read acceleration field data
	if (asm330lhhxg1_xl_flag_data_ready_get(&dev_ctx, &data_ready) != 0) {
		error_state = FAILED_TO_GET_XL_DRDY_FLAG;
		Error_Handler();
	}

	// Convert acceleration to mg and store
	if (data_ready) {
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		if (asm330lhhxg1_acceleration_raw_get(&dev_ctx, data_raw_acceleration) != 0) {
			error_state = FAILED_TO_READ_XL_REGISTER;
			Error_Handler();
		}
		acceleration_mg[0] = asm330lhhxg1_from_fs4g_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] = asm330lhhxg1_from_fs4g_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] = asm330lhhxg1_from_fs4g_to_mg(data_raw_acceleration[2]);
	}

    // Read angular rate field data
	if (asm330lhhxg1_gy_flag_data_ready_get(&dev_ctx, &data_ready) != 0) {
		error_state = FAILED_TO_GET_GY_DRDY_FLAG;
		Error_Handler();
	}

	// Convert angular rate to mdps and store
	if (data_ready) {
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		if (asm330lhhxg1_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate) != 0) {
			error_state = FAILED_TO_READ_GY_REGISTER;
			Error_Handler();
		}
		angular_rate_mdps[0] = asm330lhhxg1_from_fs500dps_to_mdps(data_raw_angular_rate[0]);
		angular_rate_mdps[1] = asm330lhhxg1_from_fs500dps_to_mdps(data_raw_angular_rate[1]);
		angular_rate_mdps[2] = asm330lhhxg1_from_fs500dps_to_mdps(data_raw_angular_rate[2]);
	}

}

void read_imu_timestamp(uint32_t *timestamp) {
	if (asm330lhhxg1_timestamp_raw_get(&dev_ctx, timestamp) != 0) {
		error_state = FAILED_TO_READ_IMU_TIMESTAMP;
		Error_Handler();
	}
}
