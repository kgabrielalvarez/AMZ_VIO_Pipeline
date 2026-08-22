#ifndef can
#define can

/* Include -------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Macros --------------------------------------------------------------------*/
// IDs of CAN messages
#define STATE_CAN_ID 		0x001 // Highest priority
#define FINISHED_CAN_ID		0x002 // Second highest priority
#define TIMESTAMPS_CAN_ID	0x003 // Third highest priority
#define IMU_CAN_ID			0x004 // Lowest priority

// Buffer size
#define BUFFER_SIZE			   32 // [bytes]

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
