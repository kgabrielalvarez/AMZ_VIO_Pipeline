// Includes
#include <string>
#include <stdexcept>

// Macros
// Publisher and subscriber buffer size
#define PUB_SUB_BUFFER_SIZE     10
// Camera IDs
#define LEFT_CAM_ID           0x01
#define RIGHT_CAM_ID          0x02

// Triggering board state
enum class triggering_board_state : uint8_t {
    STOP = 0,       // Triggering board not active
    CAL_IMU = 1,    // IMU calibration mode
    CAL_CAM = 2,    // Camera calibration mode
    RUN = 3         // Nominal mode
};

// Function to convert triggering_board_state enum to string
std::string state_to_string(triggering_board_state state);