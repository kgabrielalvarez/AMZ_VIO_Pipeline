#ifndef can
#define can

/* Include -------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Declare global variables --------------------------------------------------*/
// FDCAN controller 2
extern FDCAN_TxHeaderTypeDef TxHeader2;
extern FDCAN_RxHeaderTypeDef RxHeader2;
extern uint8_t TxData2[32];
extern uint8_t RxData2[32];

// FDCAN controller 3
extern FDCAN_TxHeaderTypeDef TxHeader3;
extern FDCAN_RxHeaderTypeDef RxHeader3;
extern uint8_t TxData3[32];
extern uint8_t RxData3[32];

/* Declare  functions --------------------------------------------------------*/

void configure_fdcan(void);

#endif
