#ifndef state_machine
#define state_machine

/* Include -------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"

/* Macros --------------------------------------------------------------------*/
// Duration of trigger pulse
#define TRIGGER_PULSE		   200 // [us]

// Conversion from s to us
#define S_TO_US		 pow(10.0, 6.0)

// Camera ID
#define CAM_1_ID			   0x01
#define CAM_2_ID			   0x02

/* Declare custom types ------------------------------------------------------*/
// Triggering board states
typedef enum {
	STOP = 0,		// Triggering board not active
	CAL_IMU = 1,	// IMU calibration mode
	CAL_CAM = 2,	// Camera calibration mode
	RUN = 3			// Nominal mode
} triggering_board_state_t;

typedef enum {

	// Nominal
	NO_ERRORS,

	// State machine
	TRIGGERING_BOARD_IN_UNKOWN_STATE,
	TRANSITION_TO_STOP_IS_INVALID,
	TRANSITION_TO_CAL_IMU_IS_INVALID,
	TRANSITION_TO_CAL_CAM_IS_INVALID,
	TRANSITION_TO_RUN_IS_INVALID,
	EXP_ACT_1_PIN_IN_UNKNOWN_STATE,
	EXP_ACT_2_PIN_IN_UNKNOWN_STATE,

	// CAN
	FDCAN_CONFIG_FILTER_FAILED,
	FDCAN_START_FAILED,
	FDCAN_ACTIVATE_NOTIFICATION_FAILED,
	FDCAN_GET_RX_MESSAGE_FAILED,
	INCORRECT_CAN_ID,
	REQUESTED_TRANSITION_TO_INVALID_STATE,
	FAILED_TO_SEND_CAN_STATE_MESSAGE,
	FAILED_TO_SEND_CAN_TIMESTAMPS_MESSAGE,
	FAILED_TO_SEND_CAN_FINISHED_MESSAGE,
	FAILED_TO_SEND_CAN_CAM_MESSAGE_1,
	FAILED_TO_SEND_CAN_CAM_MESSAGE_2,
	FAILED_TO_SEND_CAN_IMU_MESSAGE,

	// IMU
	DEN_MEASUREMENTS_NOT_STAMPED,
	REQUESTED_IMU_RATE_NOT_AVAILABLE,
	FAILED_TO_READ_XL_REGISTER,
	FAILED_TO_READ_GY_REGISTER,
	FAILED_TO_GET_XL_DRDY_FLAG,
	FAILED_TO_GET_GY_DRDY_FLAG,
	FAILED_TO_READ_IMU_TIMESTAMP,
	FAILED_TO_READ_IMU_WHOAMI,
	WHOAMI_REGISTER_INCORRECT,

	// SPI
	SPI_TRANSMIT_FAILED,
	SPI_RECEIVE_FAILED,

	// TIM
	UNKOWN_TIMER_TRIGGERED_CALLBACK,
	INPUT_CAPTURE_TRIGGERED_ON_UNKNOWN_CHANNEL,
	OUTPUT_CAPTURE_TRIGGERED_ON_UNKOWN_CHANNEL,
	TRIGGER_PIN_IN_UNKOWN_STATE,
	EXP_ACT_EDGES_DO_NOT_CORRESPOND_TO_SAME_PULSE_1,
	EXP_ACT_EDGES_DO_NOT_CORRESPOND_TO_SAME_PULSE_2,
	TRIGGER_PIN_STUCK_IN_LATCHED_HIGH,

} error_code_t;

/* Declare global variables ---------------------------------------------------*/

// Error state
extern error_code_t error_state;

// Flags
extern volatile bool state_transition_requested_flag;

/* Declare functions ---------------------------------------------------------*/

// Decides what code to execute based on the current state
void state_machine_handler(void);

#endif
