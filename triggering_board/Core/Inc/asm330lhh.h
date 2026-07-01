// Based on the example code from:
// https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/98b62f0e4f7cf79fb1464012f4ba13f3c42c959b/asm330lhhxg1_STdC/examples/asm330lhhxg1_read_data_interrupt.c#L225

#ifndef asm330lhh
#define asm330lhh

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/asm330lhh/asm330lhhxg1_reg.h"

/* Private macro -------------------------------------------------------------*/
#define BOOT_TIME 10 // ms
#define TIMEOUT 1000 // ms

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;

/* Declare Private functions -------------------------------------------------*/
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void platform_delay(uint32_t ms);
void configure_imu(void);
void asm330lhhxg1_read_data_irq_handler(void);
void read_measurements();

#endif
