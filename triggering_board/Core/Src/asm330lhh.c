#include "asm330lhh.h"

imu_spi_state_t write_imu_byte(uint8_t address, uint8_t data) {
	imu_spi_state_t state = IMU_SPI_OK;
    // STEP 1: Write Byte
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    write_buffer[0] = address;
    write_buffer[1] = data;
    if (HAL_SPI_Transmit(&hspi2, write_buffer, 2, TIMEOUT) != HAL_OK) {
    	state = TRANSMITTING_W_ADDR;
        return state;
    }
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    // STEP 2: Check that Byte was properly written
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    write_buffer[0] = 0x80 | address;
    if (HAL_SPI_Transmit(&hspi2, write_buffer, 1, TIMEOUT) != HAL_OK) {
    	state = TRANSMITTING_R_ADDR;
        return state;
    }
    if (HAL_SPI_Receive(&hspi2, read_buffer, 1, TIMEOUT) != HAL_OK) {
    	state = RECEIVING_BYTE;
        return state;
    }
    if (read_buffer[0] != data) {
    	state = WRITTEN_BYTE_DOESNT_MATCH;
    	return state;
    }
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    return state;
}

imu_status_t configure_imu(int32_t frequency, bool den_enabled) {
	imu_status_t status;
	status.state = IMU_OK;
	// STEP 1: Enable/Disable DEN and configure DRDY
    uint8_t CTRL6_C;
    uint8_t CTRL9_XL;
    uint8_t INT1_CTRL;
    if (den_enabled) {
        CTRL6_C = CTRL6_C_DEN;
        CTRL9_XL = CTRL9_XL_DEN;
        INT1_CTRL = INT1_CTRL_DEN;
    }
    else {
        CTRL6_C = CTRL6_C_STANDARD;
        CTRL9_XL = CTRL9_XL_STANDARD;
        INT1_CTRL = INT1_CTRL_STANDARD;
    }
    status.spi_state = write_imu_byte(CTRL6_C_ADDR, CTRL6_C);
    if (status.spi_state != IMU_SPI_OK) {
    	status.state = ERROR_WRITING_CTRL6_C;
        return status;
    }
    status.spi_state = write_imu_byte(CTRL9_XL_ADDR, CTRL9_XL);
    if (status.spi_state != IMU_SPI_OK) {
    	status.state = ERROR_WRITING_CTRL9_XL;
        return status;
    }
    status.spi_state = write_imu_byte(INT1_CTRL_ADDR, INT1_CTRL);
    if (status.spi_state != IMU_SPI_OK) {
    	status.state = ERROR_WRITING_INT1_CTRL;
        return status;
    }
    // STEP 2: Set operating frequency
    status.state = set_imu_frequency(frequency);
    if (status.state != IMU_OK) {
        return status;
    }
    // STEP 3: Disable FIFO
    status.spi_state = write_imu_byte(FIFO_CTRL4_ADDR, FIFO_CTRL4);
    if (status.spi_state != IMU_SPI_OK) {
        status.state = ERROR_WRITING_FIFO_CTRL4;
    	return status;
    }
    // STEP 4: Configure Accelerometer
    status.spi_state = write_imu_byte(CTRL1_XL_ADDR, CTRL1_XL);
    if (status.spi_state != IMU_SPI_OK) {
        status.state = ERROR_WRITING_CTRL1_XL;
    	return status;
    }
    status.spi_state = write_imu_byte(CTRL8_XL_ADDR, CTRL8_XL);
    if (status.spi_state != IMU_SPI_OK) {
        status.state = ERROR_WRITING_CTRL8_XL;
    	return status;
    }
    // STEP 5: Configure Gyroscope
    status.spi_state = write_imu_byte(CTRL2_G_ADDR, CTRL2_G);
    if (status.spi_state != IMU_SPI_OK) {
        status.state = ERROR_WRITING_CTRL2_G;
    	return status;
    }
    status.spi_state = write_imu_byte(CTRL7_G_ADDR, CTRL7_G);
    if (status.spi_state != IMU_SPI_OK) {
        status.state = ERROR_WRITING_CTRL7_G;
    	return status;
    }
    return status;
}

imu_state_t set_imu_frequency(int32_t frequency) {
	imu_state_t state = IMU_OK;
    switch (frequency) {
        case 833:
            CTRL1_XL = CTRL1_XL_833;
            CTRL2_G = CTRL2_G_833;
            return state;
        case 1667:
            CTRL1_XL = CTRL1_XL_1667;
            CTRL2_G = CTRL2_G_1667;
            return state;
        case 3333:
            CTRL1_XL = CTRL1_XL_3333;
            CTRL2_G = CTRL2_G_3333;
            return state;
        case 6667:
            CTRL1_XL = CTRL1_XL_6667;
            CTRL2_G = CTRL2_G_6667;
            return state;
        default:
        	state = ERROR_SETTING_FREQUENCY;
            return state;
    }
}

imu_status_t check_if_imu_measurements_ready(bool *measurements_ready) {
	imu_status_t status;
	status.state = IMU_OK;
	status.spi_state = IMU_SPI_OK;
    // STEP 1: Read STATUS_REG
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    write_buffer[0] = STATUS_REG_ADDR | 0x80;
    if (HAL_SPI_Transmit(&hspi2, write_buffer, 1, TIMEOUT) != HAL_OK) {
    	status.state = ERROR_CHECKING_IF_MEASUREMENTS_READY;
    	status.spi_state = TRANSMITTING_R_ADDR;
        return status;
    }
    if (HAL_SPI_Receive(&hspi2, read_buffer, 1, TIMEOUT) != HAL_OK) {
    	status.state = ERROR_CHECKING_IF_MEASUREMENTS_READY;
    	status.spi_state = RECEIVING_BYTE;
        return status;
    }
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    // STEP 2: Check if both the accelerometer and gyroscope measurements are ready
    if ((read_buffer[0] & STATUS_REG_MASK) == STATUS_REG_MASK) {
        *measurements_ready = true;
    }
    else {
        *measurements_ready = false;
    }
    return status;
}

imu_spi_state_t read_single_imu_measurement(uint8_t address, int16_t *measurement, bool check_den_stamp) {
	imu_spi_state_t state = IMU_SPI_OK;
    // STEP 1: Read MSB and LSB bytes
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    write_buffer[0] = address | 0x80;
    if (HAL_SPI_Transmit(&hspi2, write_buffer, 1, TIMEOUT) != HAL_OK) {
    	state = TRANSMITTING_R_ADDR;
        return state;
    }
    if (HAL_SPI_Receive(&hspi2, read_buffer, 2, TIMEOUT) != HAL_OK) {
    	state = RECEIVING_BYTE;
        return state;
    }
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    // STEP 2: Check den bit
    if (check_den_stamp) {
        if ((read_buffer[0] & 0x01) != 0x01) {
        	state = DEN_STAMP_MISSING;
            return state;
        }
    }
    // STEP 3: Convert measurement to decimal
    // Source: https://stackoverflow.com/questions/3180777/join-msb-and-lsb-of-a-16-bit-signed-integer-twos-complement
    uint16_t raw_data = ((uint16_t) read_buffer[1] << 8) | ((uint16_t) read_buffer[0]);
    int16_t data = (int16_t) raw_data;
    *measurement = data;
    return state;
}

imu_status_t read_imu_measurements(int16_t *x_accel, int16_t *y_accel, int16_t *z_accel,
								   int16_t *x_gyro, int16_t *y_gyro, int16_t *z_gyro,
								   bool den_enabled) {
	imu_status_t status;
	status.state = IMU_OK;
	// STEP 1: Read accelerometer measurements
	status.spi_state = read_single_imu_measurement(OUTX_L_A_ADDR, x_accel, false);
    if (status.spi_state != IMU_SPI_OK) {
    	status.state = ERROR_READING_X_ACCEL;
        return status;
    }
    status.spi_state = read_single_imu_measurement(OUTY_L_A_ADDR, y_accel, false);
    if (status.spi_state != IMU_SPI_OK) {
    	status.state = ERROR_READING_Y_ACCEL;
        return status;
    }
    status.spi_state = read_single_imu_measurement(OUTZ_L_A_ADDR, z_accel, den_enabled);
    if (status.spi_state != IMU_SPI_OK) {
        status.state = ERROR_READING_Z_ACCEL;
    	return status;
    }
    // STEP 2: Read gyroscope measurements
    status.spi_state = read_single_imu_measurement(OUTX_L_G_ADDR, x_gyro, false);
    if (status.spi_state != IMU_SPI_OK) {
        status.state = ERROR_READING_X_GYRO;
    	return status;
    }
    status.spi_state = read_single_imu_measurement(OUTY_L_G_ADDR, y_gyro, false);
    if (status.spi_state != IMU_SPI_OK) {
        status.state = ERROR_READING_Y_GYRO;
    	return status;
    }
    status.spi_state = read_single_imu_measurement(OUTZ_L_G_ADDR, z_gyro, false);
    if (status.spi_state != IMU_SPI_OK) {
    	status.state = ERROR_READING_Z_GYRO;
        return status;
    }
    return status;
}
