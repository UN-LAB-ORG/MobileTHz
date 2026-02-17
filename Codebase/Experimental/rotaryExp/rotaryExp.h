#ifndef ROTARY_EXP_H
#define ROTARY_EXP_H

#include "Software/structDefinition.h"
#include "rotarylib/rotarylib.h" // Your provided rotarylib header
#include <memory>
#include <string>

class mobileTHzEngine; // Forward declaration

/**
 * @class rotaryExp
 * @brief Manages rotarylib instances for experimental nodes.
 *
 * This class acts as a high-level manager for all rotary controllers
 * used in an experimental setup. It initializes controllers based on the
 * configuration and provides a unified interface for the engine to issue
 * commands and retrieve timestamped data.
 */
class rotaryExp
{
public:
    rotaryExp();
    ~rotaryExp();

    /**
     * @brief Initializes rotary controllers for the specific node this engine instance represents.
     * @param config The main configuration file.
     */
    void initialize(const ConfigFile &config);

    /**
     * @brief Issues a command string to the node's rotary controller.
     * @param command The command string (e.g., "m 0 45.0") to be passed to rotarylib.
     */
    void issueCommand(const std::string &command);

    /**
     * @brief Retrieves the current state (position, moving status) of the rotary controller.
     *        This function polls the rotarylib and packages the data with a timestamp.
     * @return A TimestampedRotaryData struct with the latest s
     * tate.
     */
    TimestampedRotaryData getLatestData();

    // Simulation flag
    static bool simulationTesting;

private:
    // A single instance of rotarylib is needed per node.
    std::unique_ptr<rotarylib> rotary_controller_;

    // Track if new data is available since the last getLatestData call.
    // rotarylib updates its state in a background thread, so we poll it.
    std::atomic<bool> has_new_data_flag_{true}; // Start as true to get initial state

    // Simulation Mimic
    rotaryObject mimic_current_state_{};
    rotaryObject mimic_target_state_{};
    std::chrono::steady_clock::time_point mimic_move_end_time_;
    bool mimic_is_moving_ = false;
};

#endif // ROTARY_EXP_H
