/* Include -------------------------------------------------------------------*/
#include "can.h"

/* Define Private functions --------------------------------------------------*/

void fdcan_config(void) {

    // Define filter configuration
    FDCAN_FilterTypeDef filter_config;
    filter_config.IdType = FDCAN_STANDARD_ID;           	// Filter applies to standard (not extended) IDs
    filter_config.FilterIndex = 0;                      	// Specify filter
    filter_config.FilterType = FDCAN_FILTER_RANGE;			// Specify filter type
    filter_config.FilterID1 = 0x321;                    	// Lowest ID that gets accepted
    filter_config.FilterID2 = 0x7FF;                    	// Highest ID that gets accepted
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 	// Specify FIFO

    // Configure FDCAN2 filter
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter_config) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure FDCAN3 filter
    // if (HAL_FDCAN_ConfigFilter(&hfdcan3, &filter_config) != HAL_OK) {
    //     Error_Handler();
    // }

    // Start FDCAN2 module
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
        Error_Handler();
    }

    // Start FDCAN3 module
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
        Error_Handler();
    }

    // Configure interrupt 0 to be triggered when FDCAN2 receives a message
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }

    // Configure interrupt 1 to be triggered when FDCAN3 receives a message
    // if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
    //     Error_Handler();
    // }

    // Configure TX header
    tx_header.Identifier = 0x321;                        // Message (not device) ID
    tx_header.IdType = FDCAN_STANDARD_ID;                // TX uses standard (not extended) IDs
    tx_header.TxFrameType = FDCAN_DATA_FRAME;            // Message being sent is data (not a request for data)
    tx_header.DataLength = FDCAN_DLC_BYTES_2;            // Send 2 bytes of data
    tx_header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;   // ???
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;             // Use constant bit rate (classical CAN)
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;              // Configure classical CAN
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;   // Disable recording of transmitted messages
    tx_header.MessageMarker = 0;                         // Unused since trnasmitted messages aren't recorded
    
}
