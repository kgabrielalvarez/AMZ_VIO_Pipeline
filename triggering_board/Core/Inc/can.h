#ifndef can
#define can

/* Include -------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Macros --------------------------------------------------------------------*/
// IDs of CAN messages
#define STATE_CAN_ID 		0x001 // Highest priority
#define FINISHED_CAN_ID		0x002
#define TIMESTAMPS_CAN_ID	0x003
#define CAM_CAN_ID			0x004
#define IMU_CAN_ID			0x005 // Lowest priority

// Buffer size
#define BUFFER_SIZE			   32 // [bytes]

// CAN message specifying that CAL_IMU phase is complete
#define FINISHED_CAN_MSG     0xFF

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
