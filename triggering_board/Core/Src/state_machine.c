/* Include -------------------------------------------------------------------*/
#include "state_machine.h"
#include "main.h" // Needed for Error_Handler()
#include "can.h"
#include "asm330lhh.h"
#include "string.h"

/* Define global variables ---------------------------------------------------*/
// Error state
error_code_t error_state = NO_ERRORS;

// Flags
volatile bool state_transition_requested_flag = false;

/* Declare external variables ------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan3;
extern TIM_HandleTypeDef htim2;

/* Declare file-scope variables ----------------------------------------------*/
// Triggering board state
static triggering_board_state_t triggering_board_state = STOP;

// IMU and camera rates and number of calibration timestamps
static int32_t imu_rate; // [Hz]
static int32_t camera_rate; // [FPS]
static int32_t imu_calibration_timestamps;

// IMU measurements
static float acceleration_mg[3];
static float angular_rate_mdps[3];

// Timestamps
// 1. Used during IMU calibration
static uint32_t imu_timestamp;
static uint32_t mcu_timestamp;
// 2. Used for IMU messages during nominal operation
static uint32_t timestamp;
// 3. Used for camera messages during nominal operation
static uint32_t cam_timestamp;

// Counter to keep track of the number of calibration timestamps that have been received
static volatile int32_t calibration_timestamp_counter = 0;

// Flags
static volatile bool drdy_flag = false;
static volatile bool tim2_started_flag = false;

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
	if (state_transition_requested_flag) {

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
		state_transition_requested_flag = false;

		// Send CAN message confirming that transition was performed
		TxData3[0] = triggering_board_state;
		memset(&TxData3[1], 0, BUFFER_SIZE-1);
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
	// Do nothing
}

void execute_CAL_IMU(void) {

	// Check that there is an IMU measurement ready
	if (drdy_flag == false) {
		return;
	}

	// Reset flag
	drdy_flag = false;

	// Check if we need more calibration timestamps
	if (calibration_timestamp_counter > imu_calibration_timestamps) {
		return;
	}

	// Check if we are done with the CAL_IMU phase
	if (calibration_timestamp_counter == imu_calibration_timestamps) {
		// Send CAN message notifying that CAL_IMU phase is complete
		TxData3[0] = FINISHED_CAN_MSG;
		memset(&TxData3[1], 0, BUFFER_SIZE-1);
		TxHeader3.Identifier = FINISHED_CAN_ID;
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader3, TxData3) != HAL_OK) {
			error_state = FAILED_TO_SEND_CAN_FINISHED_MESSAGE;
			Error_Handler();
		}
	}

	// Read IMU timestamp and measurement (to empty measurement register)
	read_imu_timestamp(&imu_timestamp);
	read_imu_measurements(acceleration_mg, angular_rate_mdps);

	// Send CAN message with IMU and MCU timestamps
	memcpy(&TxData3[0], &imu_timestamp, sizeof(uint32_t));
	memcpy(&TxData3[4], &mcu_timestamp, sizeof(uint32_t));
	memset(&TxData3[8], 0, BUFFER_SIZE-8);
	TxHeader3.Identifier = TIMESTAMPS_CAN_ID;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader3, TxData3) != HAL_OK) {
		error_state = FAILED_TO_SEND_CAN_TIMESTAMPS_MESSAGE;
		Error_Handler();
	}

	// Update counter
	calibration_timestamp_counter++;

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
	memcpy(&imu_rate, &RxData3[5], sizeof(int32_t));

	// Configure IMU
	configure_imu(imu_rate);

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
	memcpy(&camera_rate, &RxData3[1], sizeof(int32_t));

	// Reset counter
//	calibration_timestamp_counter = 0;

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
	memcpy(&camera_rate, &RxData3[1], sizeof(int32_t));

}

/* Define timer callbacks ----------------------------------------------------*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	switch (htim->Channel) {
		case HAL_TIM_ACTIVE_CHANNEL_3: // EXP_ACT triggered
			// TO-DO
			break;

		case HAL_TIM_ACTIVE_CHANNEL_4: // IMU_INT1 triggered
			drdy_flag = true;
			mcu_timestamp = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			break;

		default:
			error_state = INPUT_CAPTURE_TRIGGERED_ON_UNKNOWN_CHANNEL;
			Error_Handler();

	}
}
