// Based on code from:
// 1. FDCAN configuration: https://community.st.com/stm32-mcus-60/how-to-use-fdcan-to-create-a-simple-communication-with-a-basic-filter-132821
// 2. bxCAN part I: https://community.st.com/stm32-mcus-60/using-can-bxcan-in-normal-mode-with-stm32-microcontrollers-part-1-151183
// 3. bxCAN part II: https://community.st.com/stm32-mcus-60/using-can-bxcan-in-normal-mode-with-stm32-microcontrollers-part-2-152668
// 4. bxCAN bit time configuration: https://community.st.com/stm32-mcus-60/can-bxcan-bit-time-configuration-on-stm32-mcus-135466

/* Include -------------------------------------------------------------------*/
#include "can.h"
#include "main.h" // needed for Error_Handler()
#include "state_machine.h"

/* Declare global variables --------------------------------------------------*/
// FDCAN controller 2: left CAN transceiver on PCB and should be used to communicate with external IMU
FDCAN_TxHeaderTypeDef TxHeader2;
FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t TxData2[BUFFER_SIZE];
uint8_t RxData2[BUFFER_SIZE];

// FDCAN controller 3: right CAN transceiver on PCB and should be used to communicate with Jetson
FDCAN_TxHeaderTypeDef TxHeader3;
FDCAN_RxHeaderTypeDef RxHeader3;
uint8_t TxData3[BUFFER_SIZE];
uint8_t RxData3[BUFFER_SIZE];

/* Declare file-scope variables ----------------------------------------------*/
//CAN filter
static FDCAN_FilterTypeDef sFilterConfig;

/* Declare external variables ------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/* Define  functions ---------------------------------------------------------*/

void configure_fdcan(void) {

	// Configure fdcan2
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x22;
	sFilterConfig.FilterID2 = 0x22;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
		error_state = FDCAN_CONFIG_FILTER_FAILED;
		Error_Handler();
	}

	// Configure fdcan3
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 = 0x001;
	sFilterConfig.FilterID2 = 0x7FF;
	if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK) {
		error_state = FDCAN_CONFIG_FILTER_FAILED;
		Error_Handler();
	}

	// STart fdcan2
	if(HAL_FDCAN_Start(&hfdcan2)!= HAL_OK) {
		error_state = FDCAN_START_FAILED;
		Error_Handler();
	}

	// STart fdcan3
	if(HAL_FDCAN_Start(&hfdcan3)!= HAL_OK) {
		error_state = FDCAN_START_FAILED;
		Error_Handler();
	}

	// Activate the notification for new data in FIFO0 for fdcan2
	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		error_state = FDCAN_ACTIVATE_NOTIFICATION_FAILED;
		Error_Handler();
	}

	// Activate the notification for new data in FIFO1 for fdcan3
	if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
		error_state = FDCAN_ACTIVATE_NOTIFICATION_FAILED;
		Error_Handler();
	}

	// Configure TX Header for fdcan2
	TxHeader2.Identifier = STATE_CAN_ID;
	TxHeader2.IdType = FDCAN_STANDARD_ID;
	TxHeader2.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader2.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader2.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader2.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader2.MessageMarker = 0;

	// Configure TX Header for fdcan3
	TxHeader3.Identifier = STATE_CAN_ID;
	TxHeader3.IdType = FDCAN_STANDARD_ID;
	TxHeader3.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader3.DataLength = FDCAN_DLC_BYTES_32;
	TxHeader3.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader3.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader3.FDFormat = FDCAN_FD_CAN;
	TxHeader3.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader3.MessageMarker = 0;
    
}

// FDCAN2 callback
void HAL_FDCAN_RxFifo0Callba0k(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader2, RxData2) != HAL_OK) {
		error_state = FDCAN_GET_RX_MESSAGE_FAILED;
		Error_Handler();
	}

	// TO-DO

	if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
		error_state = FDCAN_ACTIVATE_NOTIFICATION_FAILED;
		Error_Handler();
	}
  }
}

// FDCAN3 callback
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{

  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader3, RxData3) != HAL_OK) {
    	error_state = FDCAN_GET_RX_MESSAGE_FAILED;
    	Error_Handler();
    }

    // Check message ID
    if (RxHeader3.Identifier != STATE_CAN_ID) {
    	error_state = INCORRECT_CAN_ID;
    	Error_Handler();
    }

    // Set flag to request state transition and check that message is valid
    switch (RxData3[0]) {
		case STOP:
		case CAL_IMU:
		case CAL_CAM:
		case RUN:
			state_transition_requested_flag = true;
			break;
		default:
			error_state = REQUESTED_TRANSITION_TO_INVALID_STATE,
			Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
    	error_state = FDCAN_ACTIVATE_NOTIFICATION_FAILED;
    	Error_Handler();
    }

  }

}
