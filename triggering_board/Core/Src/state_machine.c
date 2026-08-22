/* Include -------------------------------------------------------------------*/
#include "state_machine.h"
#include "main.h" // Needed for Error_Handler()
#include "can.h"
#include "asm330lhh.h"
#include "stdint.h"

/* Define global variables ---------------------------------------------------*/
// Triggering board state
triggering_board_state_t triggering_board_state = STOP;

// Error state
error_code_t error_state = NO_ERRORS;

/* Declare file-scope variables ----------------------------------------------*/
// IMU and camera rates
static int imu_rate; // [Hz]
static int camera_rate; // [FPS]

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
static uint8_t drdy_flag;

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

}

void transition_to_CAL_IMU(void) {

}

void transition_to_CAL_CAM(void) {

}

void transition_to_RUN(void) {

}

/* Define interrupt callbacks ------------------------------------------------*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

}
