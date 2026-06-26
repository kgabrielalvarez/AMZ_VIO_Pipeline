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

int configure_den() {
    if (write_byte(CTRL6_C_ADDR, CTRL6_C_DEN) != 0) {
        return 1;
    }
    if (write_byte(CTRL9_XL_ADDR, CTRL9_XL_DEN) != 0) {
        return 1;
    }
    return 0;
    // TO-DO --> section 4.8.1: SHOULD BE GOOD BUT DOUBLE CHECK!
    // 1. Gyro batch data rate and gyro ODR must be same
    // 2. Acc batch data rate and acc ODR must be same
    // 3. Gyro and acc data rate must be same
    // 4. ODRCHG_EN must be set to 0
    // 5. DEC_TS_BATCH must be set to 00
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

int configure_fifo() {
    if (write_byte(FIFO_CTRL1_ADDR, FIFO_CTRL1) != 0) {
        return 1;
    }
    if (write_byte(FIFO_CTRL2_ADDR, FIFO_CTRL2) != 0) {
        return 1;
    }
    if (FIFO_CTRL3 == 0x00) {
        return 1;
    }
    if (write_byte(FIFO_CTRL3_ADDR, FIFO_CTRL3) != 0) {
        return 1;
    }
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