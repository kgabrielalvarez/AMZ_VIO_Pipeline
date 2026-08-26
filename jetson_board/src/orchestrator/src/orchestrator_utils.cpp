#include "orchestrator/orchestrator_utils.hpp"

// Function to convert triggering_board_state enum to string
std::string state_to_string(triggering_board_state state) {

    switch (state) {

        case triggering_board_state::STOP:
            return "STOP";

        case triggering_board_state::CAL_IMU:
            return "CAL_IMU";

        case triggering_board_state::CAL_CAM:
            return "CAL_CAM";

        case triggering_board_state::RUN:
            return "RUN";

        default:
            throw std::runtime_error("Requested state_to_string conversion is not valid");

    }

}