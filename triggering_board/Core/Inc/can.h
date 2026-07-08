// Based on code from:
// 1. FDCAN configuration: https://community.st.com/stm32-mcus-60/how-to-use-fdcan-to-create-a-simple-communication-with-a-basic-filter-132821
// 2. bxCAN part I: https://community.st.com/stm32-mcus-60/using-can-bxcan-in-normal-mode-with-stm32-microcontrollers-part-1-151183
// 3. bxCAN part II: https://community.st.com/stm32-mcus-60/using-can-bxcan-in-normal-mode-with-stm32-microcontrollers-part-2-152668
// 4. bxCAN bit time configuration: https://community.st.com/stm32-mcus-60/can-bxcan-bit-time-configuration-on-stm32-mcus-135466

#ifndef canh
#define canh

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"

/* Declare private variables -------------------------------------------------*/


/* External variables --------------------------------------------------------*/
extern FDCAN_TxHeaderTypeDef tx_header;
extern FDCAN_RxHeaderTypeDef rx_header;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/* Declare private functions -------------------------------------------------*/
void fdcan_config(void);

#endif
