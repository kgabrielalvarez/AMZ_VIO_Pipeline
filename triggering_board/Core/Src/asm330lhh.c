#include "asm330lhh.h"

// Declare variables private to asm330lhh.c
static uint8_t write_buffer[BUFFER_SIZE];
static uint8_t read_buffer[BUFFER_SIZE];
static uint8_t FIFO_CTRL3;
static uint8_t CTRL1_XL;
static uint8_t CTRL2_G;

int write_byte(uint8_t address, uint8_t data) {
    // STEP 1: Write Byte
    if (HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET) != HAL_OK) {
        return 1;
    }
    write_buffer[0] = address;
    write_buffer[1] = data;
    if (HAL_SPI_Transmit(&hspi2, write_buffer, 2, TIMEOUT) != HAL_OK) {
        return 1;
    }
    if (HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET) != HAL_OK) {
        return 1;
    }
    // STEP 2: Check that Byte was properly written
    if (HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET) != HAL_OK) {
        return 1;
    }
    write_buffer[0] = 0x80 | address;
    if (HAL_SPI_Transmit(&hspi2, write_buffer, 1, TIMEOUT) != HAL_OK) {
        return 1;
    }
    if (HAL_SPI_Receive(&hspi2, read_buffer, 1, TIMEOUT) != HAL_OK) {
        return 1;
    }
    if (HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET) != HAL_OK) {
        return 1;
    }
    if (read_buffer[0] != data) {
        return 1;
    }
    return 0;
}

int configure_imu_den() {
    if (write_byte(CTRL6_C_ADDR, CTRL6_C_DEN) != 0) {
        return 1;
    }
    if (write_byte(CTRL9_XL_ADDR, CTRL9_XL_DEN) != 0) {
        return 1;
    }
    if (write_byte(INT1_CTRL_ADDR, INT1_CTRL_DEN) != 0) {
        return 1;
    }
    return 0;
}

int set_frequency(int32_t frequency) {
    switch (frequency) {
        case 833:
            FIFO_CTRL3 = FIFO_CTRL3_833;
            CTRL1_XL = CTRL1_XL_833;
            CTRL2_G = CTRL2_G_833;
            return 0;
        case 1667:
            FIFO_CTRL3 = FIFO_CTRL3_1667;
            CTRL1_XL = CTRL1_XL_1667;
            CTRL2_G = CTRL2_G_1667;
            return 0;
        case 3333:
            FIFO_CTRL3 = FIFO_CTRL3_3333;
            CTRL1_XL = CTRL1_XL_3333;
            CTRL2_G = CTRL2_G_3333;
            return 0;
        case 6667:
            FIFO_CTRL3 = FIFO_CTRL3_3333;
            CTRL1_XL = CTRL1_XL_6667;
            CTRL2_G = CTRL2_G_6667;
            return 0;
        default:
            return 1;
    }
}

int disable_fifo() {
    if (write_byte(FIFO_CTRL4_ADDR, FIFO_CTRL4) != 0) {
        return 1;
    }
    return 0;
}

int configure_accel() {
    if (write_byte(CTRL1_XL_ADDR, CTRL1_XL) != 0) {
        return 1;
    }
    if (write_byte(CTRL8_XL_ADDR, CTRL8_XL) != 0) {
        return 1;
    }
    return 0;
}

int configure_gyro() {
    if (write_byte(CTRL2_G_ADDR, CTRL2_G) != 0) {
        return 1;
    }
    if (write_byte(CTRL7_G_ADDR, CTRL7_G) != 0) {
        return 1;
    }
    return 0;
}

int read_single_measurement(uint8_t address, int16_t *measurement, bool check_den_stamp) {
    // STEP 1: Read MSB and LSB bytes
    if (HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET) != HAL_OK) {
        return 1;
    }
    write_buffer[0] = address | 0x80;
    if (HAL_SPI_Transmit(&hspi2, write_buffer, 1, TIMEOUT) != HAL_OK) {
        return 1;
    }
    if (HAL_SPI_Receive(&hspi2, read_buffer, 2, TIMEOUT) != HAL_OK) {
        return 1;
    }
    if (HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET) != HAL_OK) {
        return 1;
    }
    // STEP 2: Check den bit
    if (check_den_stamp) {
        if (read_buffer[0] & 0x01 != 0x01) {
            return 1;
        }
    }
    // STEP 3: Convert measurement to decimal
    // Source: https://stackoverflow.com/questions/3180777/join-msb-and-lsb-of-a-16-bit-signed-integer-twos-complement
    uint16_t raw_data = ((uint16_t) read_buffer[1] << 8) | ((uint16_t) read_buffer[0]);
    int16_t data = (int16_t) raw_data;
    *measurement = data;
    return 0;
}

int read_measurements_den(int16_t *x_accel,
                          int16_t *y_accel,
                          int16_t *z_accel,
                          int16_t *x_gyro,
                          int16_t *y_gyro,
                          int16_t *z_gyro) {
    // STEP 1: Read accelerometer measurements
    if (read_single_measurement(OUTX_L_A_ADDR, x_accel, false) != 0) {
        return 1;
    }
    if (read_single_measurement(OUTY_L_A_ADDR, y_accel, false) != 0) {
        return 1;
    }
    if (read_single_measurement(OUTZ_L_A_ADDR, z_accel, true) != 0) {
        return 1;
    }
    // STEP 2: Read gyroscope measurements
    if (read_single_measurement(OUTX_L_G_ADDR, x_gyro, false) != 0) {
        return 1;
    }
    if (read_single_measurement(OUTY_L_G_ADDR, y_gyro, false) != 0) {
        return 1;
    }
    if (read_single_measurement(OUTZ_L_G_ADDR, z_gyro, false) != 0) {
        return 1;
    }
    return 0;
}