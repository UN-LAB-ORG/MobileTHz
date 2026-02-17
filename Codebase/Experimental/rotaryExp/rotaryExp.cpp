#include "rotaryExp.h"
#include "Software/mobileTHzEngine/mobileTHzEngine.h"
#include <iostream>

// Default to false (real hardware)
bool rotaryExp::simulationTesting = true;
static bool debug = false;

rotaryExp::rotaryExp() {}

rotaryExp::~rotaryExp()
{
    // The unique_ptr will automatically call the rotarylib destructor,
    // which gracefully shuts down its worker thread and hardware.
}

void rotaryExp::initialize(const ConfigFile &config)
{
    const std::string &my_role = config.experimental.role;

    if (my_role == "OBSERVER")
    {
        return; // Observer has no rotary hardware or simulator
    }

    const engineUnit *my_unit_config = nullptr; // Find this Node's specific unit configuration
    for (const auto &unit : config.engine_units)
    {
        if (unit.label == my_role)
        {
            my_unit_config = &unit;
            break;
        }
    }

    if (!my_unit_config)
    {
        // This is a critical config error where the role doesn't match any unit.
        throw std::runtime_error("[rotaryExp] Role '" + my_role + "' not found in engine_units.");
    }
    if (!my_unit_config->Rotary_enabled)
    {
        // This is a valid configuration, just means we don't initialize this component.
        return;
    }

    // --- Check the simulation flag ---
    // If we are mimicking, there is nothing to initialize.
    if (simulationTesting)
    {
        if (debug)
        {
            std::cout << "[rotaryExp] WARNING: simulationTesting is TRUE. Mimicking hardware."
                      << std::endl;
        }
        mimic_current_state_ = rotaryObject{};
        mimic_target_state_ = rotaryObject{};
        mimic_is_moving_ = false;
        return;
    }

    // --- Hardware Initialization Logic ---
    for (const auto &unit : config.engine_units)
    {
        if (unit.label == my_role)
        {
            my_unit_config = &unit;
            break;
        }
    }

    if (!my_unit_config || !my_unit_config->Rotary_enabled)
    {
        std::cout << "[" << my_role << "] Rotary controller is disabled or not configured."
                  << std::endl;
        return;
    }

    std::cout << "[rotaryExp] Initializing rotary controller for node: " << my_role << std::endl;
    rotary_controller_ = std::make_unique<rotarylib>();

    const std::string &port_az = config.experimental.Rotary_port_azimuth;
    const std::string &port_alt = config.experimental.Rotary_port_altitude;

    if (port_az.empty())
    {
        throw std::runtime_error("[rotaryExp] Rotary is enabled for node " + my_role + " but rotary_port_azimuth is not specified.");
    }

    // DUAL PORT MODE: If an altitude port is also specified
    if (!port_alt.empty())
    {
        std::cout << "  - Dual-port mode. Azimuth: " << port_az << ", Altitude: " << port_alt
                  << std::endl;
        rotary_controller_->setPort(port_az, port_alt);
    }
    // SINGLE PORT MODE
    else
    {
        std::cout << "  - Single-port mode. Port: " << port_az << std::endl;
        rotary_controller_->setPort(port_az);
    }

    if (!rotary_controller_->getEnable())
    {
        throw std::runtime_error("[rotaryExp] Failed to initialize rotary controller for node " + my_role);
    }
    std::cout << "[rotaryExp] Rotary controller for " << my_role << " initialized successfully."
              << std::endl;
}

void rotaryExp::issueCommand(const std::string &command)
{
    if (simulationTesting)
    {
        // --- Primitive Mimic Command Parsing ---
        std::istringstream iss(command);
        std::string cmd_char;
        iss >> cmd_char;

        bool move_command_issued = false;

        if (cmd_char == "m")
        { // e.g., "m 0 45.0"
            int axis;
            double degrees;
            if (iss >> axis >> degrees)
            {
                if (axis == 0)
                { // Azimuth
                    mimic_target_state_.azimuth.angle = mimic_current_state_.azimuth.angle + degrees;
                }
                else if (axis == 1)
                { // Altitude
                    mimic_target_state_.altitude.angle = mimic_current_state_.altitude.angle + degrees;
                }
                move_command_issued = true;
                std::cout << "[rotaryExp Mimic] Moving axis " << axis << " by " << degrees
                          << " deg." << std::endl;
            }
        }
        // --- NEW: Added parsing for 'mdd' ---
        else if (cmd_char == "mdd")
        { // e.g., "mdd 0 45.0 1 10.0"
            int axis1, axis2;
            double degrees1, degrees2;
            if (iss >> axis1 >> degrees1 >> axis2 >> degrees2)
            {
                // Determine which value corresponds to azimuth and which to altitude
                double az_degrees = (axis1 == 0) ? degrees1 : degrees2;
                double alt_degrees = (axis1 == 1) ? degrees1 : degrees2;

                // Update the target state for both axes
                mimic_target_state_.azimuth.angle = mimic_current_state_.azimuth.angle + az_degrees;
                mimic_target_state_.altitude.angle = mimic_current_state_.altitude.angle + alt_degrees;

                move_command_issued = true;
                std::cout << "[rotaryExp Mimic] Moving dual axes. Azimuth by " << az_degrees
                          << " deg, Altitude by " << alt_degrees << " deg." << std::endl;
            }
        }

        // --- Common logic for any move command ---
        if (move_command_issued)
        {
            mimic_is_moving_ = true;
            // Set the move to finish in 1 second from now
            mimic_move_end_time_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        }

        return; // Always return after handling mimic logic
    }

    // --- Hardware Logic ---
    if (rotary_controller_)
    {
        rotary_controller_->inputFunction(command);
    }
    else
    {
        std::cerr << "[rotaryExp] Warning: Ignoring command, rotary controller not initialized."
                  << std::endl;
    }
}

TimestampedRotaryData rotaryExp::getLatestData()
{
    TimestampedRotaryData ts_data;

    if (simulationTesting)
    {
        // --- Primitive Mimic Data Generation ---
        if (mimic_is_moving_)
        {
            if (std::chrono::steady_clock::now() >= mimic_move_end_time_)
            {
                // Movement is finished
                mimic_current_state_ = mimic_target_state_;
                mimic_current_state_.isMoving = 0.0;
                mimic_is_moving_ = false;
            }
            else
            {
                // Movement is in progress (we can just say it's moving, no need to interpolate for a simple mimic)
                mimic_current_state_.isMoving = 1.0;
            }
        }

        ts_data.data = mimic_current_state_;
        ts_data.timestamp = std::chrono::high_resolution_clock::now();
        ts_data.has_new_data = true;
        return ts_data;
    }

    // --- Original Hardware Logic ---
    if (!rotary_controller_)
    {
        std::cerr << "[rotaryExp] Warning: Rotary controller not initialized." << std::endl;
        ts_data.has_new_data = false;
        return ts_data;
    }

    // Axis 0 -> Azimuth
    ts_data.data.azimuth.angle = rotary_controller_->getCurrentPosition(0);
    // NOTE: rotarylib doesn't expose velocity/acceleration directly.
    // For simplicity, we can leave them as 0, which is accurate when idle.

    // Axis 1 -> Altitude
    ts_data.data.altitude.angle = rotary_controller_->getCurrentPosition(1);

    // Check if either axis is moving
    ts_data.data.isMoving = (rotary_controller_->isAxisMoving(0) || rotary_controller_->isAxisMoving(1))
                                ? 1.0
                                : 0.0;

    ts_data.timestamp = std::chrono::high_resolution_clock::now();
    ts_data.has_new_data = true; // Always provide the latest polled state

    return ts_data;
}
