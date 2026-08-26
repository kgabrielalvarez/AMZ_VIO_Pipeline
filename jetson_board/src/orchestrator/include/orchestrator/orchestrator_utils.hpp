// Includes
#include <string>
#include <stdexcept>

// Triggering board state
enum class triggering_board_state : uint8_t {
    STOP = 0,       // Triggering board not active
    CAL_IMU = 1,    // IMU calibration mode
    CAL_CAM = 2,    // Camera calibration mode
    RUN = 3         // Nominal mode
};

// Function to convert triggering_board_state enum to string
std::string state_to_string(triggering_board_state state);