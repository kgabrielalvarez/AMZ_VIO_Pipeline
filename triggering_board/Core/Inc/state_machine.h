#ifndef state_machine
#define state_machine

/* Include -------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"

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

	// CAN
	FDCAN_CONFIG_FILTER_FAILED,
	FDCAN_START_FAILED,
	FDCAN_ACTIVATE_NOTIFICATION_FAILED,
	FDCAN_GET_RX_MESSAGE_FAILED,
	INCORRECT_CAN_ID,
	REQUESTED_TRANSITION_TO_INVALID_STATE,
	FAILED_TO_SEND_CAN_STATE_MESSAGE,

	// IMU
	DEN_MEASUREMENTS_NOT_STAMPED,

} error_code_t;

/* Declare global variables ---------------------------------------------------*/
// Triggering board state
extern uint8_t triggering_board_state;

// Error state
extern error_code_t error_state;

// Flags
extern volatile bool state_transition_requested;

/* Declare functions ---------------------------------------------------------*/

// Decides what code to execute based on the current state
void state_machine_handler(void);

#endif
