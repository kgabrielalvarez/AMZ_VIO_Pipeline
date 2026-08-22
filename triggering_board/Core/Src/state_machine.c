/* Include -------------------------------------------------------------------*/
#include "state_machine.h"
#include "main.h" // Needed for Error_Handler()
#include "can.h"
#include "asm330lhh.h"
#include "string.h"

/* Define global variables ---------------------------------------------------*/
// Triggering board state
uint8_t triggering_board_state = STOP;

// Error state
error_code_t error_state = NO_ERRORS;

// Flags
volatile bool state_transition_requested = false;

/* Declare external variables ------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan3;

/* Declare file-scope variables ----------------------------------------------*/
// IMU and camera rates and number of calibration timestamps
static int32_t imu_rate; // [Hz]
static int32_t camera_rate; // [FPS]
static int32_t imu_calibration_timestamps;

// IMU measurements
static float acceleration_mg[3];
static float angular_rate_mdps[3];

// Timestamps
// Used during IMU calibration
static uint32_t imu_timestamp;
static uint32_t mcu_timestamp;
// Used during nominal operation
static uint32_t timestamp;

// Flags
static bool drdy_flag;

/* Declare file-scope functions ----------------------------------------------*/

// State execution functions
static void execute_STOP(void);
static void execute_CAL_IMU(void);
static void execute_CAL_CAM(void);
static void execute_RUN(void);

//State transition functions
static void transition_to_STOP(void);
static void transition_to_CAL_IMU(void);
static void transition_to_CAL_CAM(void);
static void transition_to_RUN(void);

/* Define state machine handler function -------------------------------------*/

void state_machine_handler(void) {

	// Check whether state transition needs to be performed
	if (state_transition_requested) {

		// Perform state transition
		switch (RxData3[0]) {
			case STOP:
				transition_to_STOP();
				break;
			case CAL_IMU:
				transition_to_CAL_IMU();
				break;
			case CAL_CAM:
				transition_to_CAL_CAM();
				break;
			case RUN:
				transition_to_RUN();
				break;
			default:
				error_state = REQUESTED_TRANSITION_TO_INVALID_STATE,
				Error_Handler();
		}

		// Reset transition flag
		state_transition_requested = false;

		// Send CAN message confirming that transition was performed
		TxData3[0] = triggering_board_state;
		memset(&TxData3[1], 0, (BUFFER_SIZE-1)*sizeof(uint8_t));
		TxHeader3.Identifier = STATE_CAN_ID;
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader3, TxData3) != HAL_OK) {
			error_state = FAILED_TO_SEND_CAN_STATE_MESSAGE;
			Error_Handler();
		}

	}

	switch (triggering_board_state) {
		case STOP:
			execute_STOP();
			break;
		case CAL_IMU:
			execute_CAL_IMU();
			break;
		case CAL_CAM:
			execute_CAL_CAM();
			break;
		case RUN:
			execute_RUN();
			break;
		default:
			error_state = TRIGGERING_BOARD_IN_UNKOWN_STATE;
			Error_Handler();
	}
}

/* Define state execution functions ------------------------------------------*/

void execute_STOP(void) {

}

void execute_CAL_IMU(void) {

}

void execute_CAL_CAM(void) {

}

void execute_RUN(void) {

}

/* Define state transition functions -----------------------------------------*/

void transition_to_STOP(void) {
	// Check if state transition is valid
	if ((triggering_board_state != CAL_IMU) &&
		(triggering_board_state != CAL_CAM) &&
		(triggering_board_state != RUN)) {
		error_state = TRANSITION_TO_STOP_IS_INVALID;
		Error_Handler();
	}

	// Perform state transition
	triggering_board_state = STOP;

}

void transition_to_CAL_IMU(void) {
	// Check if state transition is valid
	if (triggering_board_state != STOP) {
		error_state = TRANSITION_TO_CAL_IMU_IS_INVALID;
		Error_Handler();
	}

	// Perform state transition
	triggering_board_state = CAL_IMU;

	// Read relevant data
	memcpy(&imu_calibration_timestamps, &RxData3[1], sizeof(int32_t));

}

void transition_to_CAL_CAM(void) {
	// Check if state transition is valid
	if (triggering_board_state != CAL_IMU) {
		error_state = TRANSITION_TO_CAL_CAM_IS_INVALID;
		Error_Handler();
	}

	// Perform state transition
	triggering_board_state = CAL_CAM;

	// Read relevant data
	memcpy(&imu_rate, &RxData3[1], sizeof(int32_t));
	memcpy(&camera_rate, &RxData3[5], sizeof(int32_t));

}

void transition_to_RUN(void) {
	// Check if state transition is valid
	if (triggering_board_state != CAL_CAM) {
		error_state = TRANSITION_TO_RUN_IS_INVALID;
		Error_Handler();
	}

	// Perform state transition
	triggering_board_state = RUN;

	// Read relevant data
	memcpy(&imu_rate, &RxData3[1], sizeof(int32_t));
	memcpy(&camera_rate, &RxData3[5], sizeof(int32_t));

}

/* Define interrupt callbacks ------------------------------------------------*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

}
