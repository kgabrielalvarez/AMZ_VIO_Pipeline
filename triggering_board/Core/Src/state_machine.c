/* Include -------------------------------------------------------------------*/
#include "state_machine.h"
#include "main.h" // Needed for Error_Handler()
#include "can.h"
#include "asm330lhh.h"
#include "string.h"
#include "math.h"

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

// IMU and camera rates and number of calibration samples
static int32_t imu_rate; // [Hz]
static int32_t camera_rate; // [FPS]
static uint32_t camera_period; // [us]
static int32_t imu_calibration_samples;
static int32_t camera_calibration_samples;

// Camera exposure time
static int32_t camera_exposure_time;

// IMU measurements
static float acceleration_mg[3];
static float angular_rate_mdps[3];

// Timestamps
// 1. Used during IMU calibration
static uint32_t imu_timestamp;
static volatile uint32_t mcu_timestamp;
// 2. Used for IMU messages during nominal operation
static uint32_t timestamp;
// 3. Used for camera messages during nominal operation
static uint32_t cam_timestamp_1;
static uint32_t cam_timestamp_2;

// Counter to keep track of the number of calibration timestamps that have been received
static volatile int32_t imu_calibration_counter = 0;

// CCR value at which TRIGGER GPIO pin is fired
static volatile uint32_t trigger_CCR;
// Counter value at which frame exposure start and end
static volatile uint32_t exposure_start_1;
static volatile uint32_t exposure_end_1;
static volatile uint32_t exposure_start_2;
static volatile uint32_t exposure_end_2;
// Instant at which capture occurs
static volatile uint32_t exposure_capture_instant_1;
static volatile uint32_t exposure_capture_instant_2;
static volatile uint32_t trigger_capture_instant;
// Start of camera time
static volatile uint32_t start_of_camera_time;

// Counter to track the number of trigger pulses that have been fired
static volatile uint32_t trigger_counter = 0;
// Counter to track the number of camera frames that have been captured
static volatile uint32_t camera_counter_1 = 0;
static volatile uint32_t camera_counter_2 = 0;

// Exposure pin and trigger pin states
static volatile GPIO_PinState exposure_pin_capture_state_1;
static volatile GPIO_PinState exposure_pin_capture_state_2;
static volatile GPIO_PinState trigger_pin_capture_state;

// Flags
static volatile bool drdy_flag = false;
static volatile bool imu_cal_finished_flag = false;
static volatile bool exposure_act_1_flag = false;
static volatile bool exposure_act_2_flag = false;
static volatile bool tim2_started_flag = false;
static volatile bool trigger_flag = false;

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

	// Check if the IMU calibration process is complete
	if (imu_cal_finished_flag == true) {
		return;
	}

	// Check if we are ready to send the finished message to proceed to the next phase
	if (imu_calibration_counter == imu_calibration_samples) {
		TxData3[0] = FINISHED_IMU_CAL_MSG;
		memset(&TxData3[1], 0, BUFFER_SIZE-1);
		TxHeader3.Identifier = FINISHED_CAN_ID;
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader3, TxData3) != HAL_OK) {
			error_state = FAILED_TO_SEND_CAN_FINISHED_MESSAGE;
			Error_Handler();
		}
		imu_cal_finished_flag = true;
		return;
	}

	// Check that there is an IMU measurement ready
	if (drdy_flag == true) {

		// Reset flag
		drdy_flag = false;

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
		imu_calibration_counter++;

		return;
	}

}

void execute_CAL_CAM(void) {

	// Check to see if there was a falling edge on the TRIGGER pin
	if (trigger_flag == true) {
		// Reset flag
		trigger_flag = false;
		// Check if we should continue triggering
		if (camera_counter_2 >= camera_calibration_samples-1) {
			return;
		}
		// Configure CCR for next trigger:
		// 1. Exposure compensated mode cannot be used yet because a frame has not yet been captured by Pylon
		if (camera_counter_1 == 0) {
			trigger_CCR = start_of_camera_time + (trigger_counter+1)*camera_period;
		}
		// 2. Use exposure compensated mode
		else {
			trigger_CCR = start_of_camera_time + (trigger_counter+1)*camera_period -
						  (uint32_t)(((float)exposure_end_1 - (float)exposure_start_1)/2.0);
		}
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, trigger_CCR);
		return;
	}

	// Check whether there was a rising or falling edge on the EXP_ACT_1 pin
	if (exposure_act_1_flag == true) {
		// Reset flag
		exposure_act_1_flag = false;
		switch (exposure_pin_capture_state_1) {
			case (GPIO_PIN_SET):
				// Save capture instant
				exposure_start_1 = exposure_capture_instant_1;
				return;
			case (GPIO_PIN_RESET):
				// Check that falling edge corresponds to the same pulse as the previous rising edge
				if ((exposure_capture_instant_1 - exposure_start_1) > camera_period) {
					error_state = EXP_ACT_EDGES_DO_NOT_CORRESPOND_TO_SAME_PULSE_1;
					Error_Handler();
				}
				// Save capture instant
				exposure_end_1 = exposure_capture_instant_1;
				// Update counter
				camera_counter_1++;
				// Send CAN message with camera timestamps
				cam_timestamp_1 = (uint32_t)(((float)exposure_end_1 + (float)exposure_start_1)/2.0);
				memcpy(&TxData3[0], &cam_timestamp_1, sizeof(uint32_t));
				memcpy(&TxData3[4], &camera_counter_1, sizeof(uint32_t));
				TxData3[8] = CAM_1_ID;
				memset(&TxData3[9], 0, BUFFER_SIZE-9);
				TxHeader3.Identifier = CAM_CAN_ID;
				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader3, TxData3) != HAL_OK) {
					error_state = FAILED_TO_SEND_CAN_CAM_MESSAGE_1;
					Error_Handler();
				}
				return;
			default:
				error_state = EXP_ACT_1_PIN_IN_UNKNOWN_STATE;
				Error_Handler();
		}
	}

	// Check whether there was a rising or falling edge on the EXP_ACT_2 pin
	if (exposure_act_2_flag == true) {
		// Reset flag
		exposure_act_2_flag = false;
		switch (exposure_pin_capture_state_2) {
			case (GPIO_PIN_SET):
				// Save capture instant
				exposure_start_2 = exposure_capture_instant_2;
				return;
			case (GPIO_PIN_RESET):
				// Check that falling edge corresponds to the same pulse as the previous rising edge
				if ((exposure_capture_instant_2 - exposure_start_2) > camera_period) {
					error_state = EXP_ACT_EDGES_DO_NOT_CORRESPOND_TO_SAME_PULSE_2;
					Error_Handler();
				}
				// Save capture instant
				exposure_end_2 = exposure_capture_instant_2;
				// Update counter
				camera_counter_2++;
				// Send CAN message with camera timestamps
				cam_timestamp_2 = (uint32_t)(((float)exposure_end_2 + (float)exposure_start_2)/2.0);
				memcpy(&TxData3[0], &cam_timestamp_2, sizeof(uint32_t));
				memcpy(&TxData3[4], &camera_counter_2, sizeof(uint32_t));
				TxData3[8] = CAM_2_ID;
				memset(&TxData3[9], 0, BUFFER_SIZE-9);
				TxHeader3.Identifier = CAM_CAN_ID;
				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader3, TxData3) != HAL_OK) {
					error_state = FAILED_TO_SEND_CAN_CAM_MESSAGE_2;
					Error_Handler();
				}
				// If calibration phase is complete send finished message
				if (camera_counter_2 == camera_calibration_samples) {
					TxData3[0] = FINISHED_CAM_CAL_MSG;
					memcpy(&TxData3[1], &camera_counter_1, sizeof(uint32_t));
					memcpy(&TxData3[5], &camera_counter_2, sizeof(uint32_t));
					memset(&TxData3[9], 0, BUFFER_SIZE-9);
					TxHeader3.Identifier = FINISHED_CAN_ID;
					if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader3, TxData3) != HAL_OK) {
						error_state = FAILED_TO_SEND_CAN_FINISHED_MESSAGE;
						Error_Handler();
					}
				}
				return;
			default:
				error_state = EXP_ACT_2_PIN_IN_UNKNOWN_STATE;
				Error_Handler();
		}
	}

	// Check that there is an IMU measurement ready
	if (drdy_flag == true) {
		// Reset flag
		drdy_flag = false;
		// Read measurements
		read_imu_measurements(acceleration_mg, angular_rate_mdps);
		// Send CAN message
		memcpy(&TxData3[0], &acceleration_mg[0], sizeof(float));
		memcpy(&TxData3[4], &acceleration_mg[1], sizeof(float));
		memcpy(&TxData3[8], &acceleration_mg[2], sizeof(float));
		memcpy(&TxData3[12], &angular_rate_mdps[0], sizeof(float));
		memcpy(&TxData3[16], &angular_rate_mdps[1], sizeof(float));
		memcpy(&TxData3[20], &angular_rate_mdps[2], sizeof(float));
		memcpy(&TxData3[24], &mcu_timestamp, sizeof(uint32_t));
		memset(&TxData3[28], 0, BUFFER_SIZE-28);
		TxHeader3.Identifier = IMU_CAN_ID;
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader3, TxData3) != HAL_OK) {
			error_state = FAILED_TO_SEND_CAN_IMU_MESSAGE;
			Error_Handler();
		}
		return;
	}

}

void execute_RUN(void) {

	execute_CAL_CAM();

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

	// Reset counters
	imu_calibration_counter = 0;
	camera_counter_1 = 0;
	camera_counter_2 = 0;

	// Reset flags
	drdy_flag = false;
	imu_cal_finished_flag = false;
	exposure_act_1_flag = false;
	exposure_act_2_flag = false;
	tim2_started_flag = false;
	trigger_flag = false;

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
	memcpy(&imu_calibration_samples, &RxData3[1], sizeof(int32_t));
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
	memcpy(&camera_calibration_samples, &RxData3[1], sizeof(int32_t));
	memcpy(&camera_rate, &RxData3[5], sizeof(int32_t));
	camera_period = (uint32_t) (1.0/((float)camera_rate) * S_TO_US); // [us]

	// Set the CCR for CH1 for the first trigger
	start_of_camera_time = __HAL_TIM_GET_COUNTER(&htim2);
	trigger_CCR = start_of_camera_time + camera_period;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, trigger_CCR);

}

void transition_to_RUN(void) {
	// Check if state transition is valid
	if (triggering_board_state != CAL_CAM) {
		error_state = TRANSITION_TO_RUN_IS_INVALID;
		Error_Handler();
	}

	// Perform state transition
	triggering_board_state = RUN;

}

/* Define timer callbacks ----------------------------------------------------*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// Check that TIM2 triggered the callback
	if (htim->Instance != TIM2) {
		error_state = UNKOWN_TIMER_TRIGGERED_CALLBACK;
		Error_Handler();
	}

	// Check what callback was triggered
	switch (htim->Channel) {

		case HAL_TIM_ACTIVE_CHANNEL_2: // EXP_ACT_1 triggered
			exposure_capture_instant_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			exposure_pin_capture_state_1 = HAL_GPIO_ReadPin(EXP_ACT_1_GPIO_Port, EXP_ACT_1_Pin);
			exposure_act_1_flag = true;
			break;

		case HAL_TIM_ACTIVE_CHANNEL_3: // EXP_ACT_2 triggered
			exposure_capture_instant_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			exposure_pin_capture_state_2 = HAL_GPIO_ReadPin(EXP_ACT_2_GPIO_Port, EXP_ACT_2_Pin);
			exposure_act_2_flag = true;
			break;

		case HAL_TIM_ACTIVE_CHANNEL_4: // IMU_INT1 triggered
			mcu_timestamp = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			drdy_flag = true;
			break;

		default:
			error_state = INPUT_CAPTURE_TRIGGERED_ON_UNKNOWN_CHANNEL;
			Error_Handler();

	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Check that TIM2 triggered the callback
	if (htim->Instance != TIM2) {
		error_state = UNKOWN_TIMER_TRIGGERED_CALLBACK;
		Error_Handler();
	}

	// Check that CH1 triggered the callback
	if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) {
		error_state = OUTPUT_CAPTURE_TRIGGERED_ON_UNKOWN_CHANNEL;
		Error_Handler();
	}

	// Capture trigger instant and pin state
	trigger_pin_capture_state = HAL_GPIO_ReadPin(TRIGGER_GPIO_Port, TRIGGER_Pin);
	trigger_capture_instant = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

	// Check to see if we had a rising edge or falling edge
	switch (trigger_pin_capture_state) {
		// Set CCR value at which trigger pin should turn off
		case GPIO_PIN_SET:
			trigger_CCR = trigger_capture_instant + TRIGGER_PULSE;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, trigger_CCR);
			// Update trigger counter
			trigger_counter++;
			break;
		// Set flag so that next trigger pulse gets set in main loop
		case GPIO_PIN_RESET:
			trigger_flag = true;
			break;
		default:
			error_state = TRIGGER_PIN_IN_UNKOWN_STATE;
			Error_Handler();
	}

}
