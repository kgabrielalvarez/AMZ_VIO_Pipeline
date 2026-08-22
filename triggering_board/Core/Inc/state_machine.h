#ifndef state_machine
#define state_machine

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

	// CAN
	FDCAN_CONFIG_FILTER_FAILED,
	FDCAN_START_FAILED,
	FDCAN_ACTIVATE_NOTIFICATION_FAILED,
	FDCAN_GET_RX_MESSAGE,


	// IMU
	DEN_MEASUREMENTS_NOT_STAMPED,

} error_code_t;

/* Declare global variables ---------------------------------------------------*/
// Triggering board state
extern triggering_board_state_t triggering_board_state;

// Error state
extern error_code_t error_state;

/* Declare functions ---------------------------------------------------------*/

// Decides what code to execute based on the current state
void state_machine_handler(void);

#endif
