#ifndef ROTARYSIM_H
#define ROTARYSIM_H

#include "Software/mobileTHzEngine/mobileTHzEngine.h"
#include "Software/structDefinition.h"
#include <array>
#include <list>
#include <map>
#include <queue>
#include <string>
#include <vector>

// Forward declaration
class mobileTHzEngine;

class rotarySim
{
public:
    rotarySim();
    ~rotarySim();

    /**
     * @brief Initialize the rotary simulator with engine access.
     *
     * @param paramConfig The configuration parameters
     * @param engine Pointer to the mobileTHzEngine instance
     */
    void initialize(const ConfigFile &paramConfig, mobileTHzEngine *engine);

    /**
     * @brief Process the simulation for the current time slot.
     * This updates rotary states in the engine based on commands and physics.
     *
     * @param currentSlot The current time slot being processed
     */
    void processSlot(size_t currentSlot);

    // --- Command Interface (mirrors rotarylib) ---

    /**
+    * @brief Processes a command string, mimicking rotarylib::inputFunction.
+    * @param unitIndex The index of the unit to command.
+    * @param commandString The command string (e.g., "m 0 45", "halt 1").
+    */
    void inputFunction(size_t unitIndex, const std::string &commandString);

private:
    // IMPORTANT NOTES:
    // AXIS 0 Counts Per Degree = 2000  - AXIS 0 has 720,000 counts per revolution
    const double AXIS0_COUNTS_PER_DEGREE = 2000;
    // AXIS 1 Counts Per Degree = 778   - AXIS 1 has 280,000 counts per revolution
    const double AXIS1_COUNTS_PER_DEGREE = 777.7777777778;
    // ---------------------------------------------

    /**
     * @brief Set the velocity for a specific axis
     *
     * @param unitIndex Unit to control
     * @param axis 0 for azimuth, 1 for altitude
     * @param velocity Velocity value (in 0.1 counts/second)
     */
    void setVelocity(size_t unitIndex, int axis, double velocity);

    /**
     * @brief Set acceleration for a specific axis
     *
     * @param unitIndex Unit to control
     * @param axis 0 for azimuth, 1 for altitude
     * @param acceleration Acceleration value (in counts/second^2)
     */
    void setAcceleration(size_t unitIndex, int axis, double acceleration);

    /**
     * @brief Move axis by a specified number of degrees
     *
     * @param unitIndex Unit to control
     * @param axis 0 for azimuth, 1 for altitude
     * @param degrees Angle to move by (relative to current position)
     */
    void moveByDegrees(size_t unitIndex, int axis, double degrees);

    /**
     * @brief Move both axes simultaneously
     *
     * @param unitIndex Unit to control
     * @param azimuthDegrees Azimuth angle to move by
     * @param altitudeDegrees Altitude angle to move by
     */
    void moveDual(size_t unitIndex, double azimuthDegrees, double altitudeDegrees);

    /**
     * @brief Immediately halt movement on specified axis
     *
     * @param unitIndex Unit to control
     * @param axis 0 for azimuth, 1 for altitude
     */
    void halt(size_t unitIndex, int axis);

    /**
     * @brief Halts all axes for the specified unit
     *
     * @param unitIndex Unit to control
     */
    void haltAll(size_t unitIndex);

    // --- Engine access ---
    mobileTHzEngine *engine_;
    ConfigFile config_;
    double slotDuration_sec_; // Duration of one time slot in seconds
    size_t numUnits_;         // Number of units in simulation
    size_t numTimeSlots_;     // Total time slots in simulation

    // --- Command processing ---
    struct RotaryCommand
    {
        size_t unitIndex;
        int axis;             // 0 for azimuth, 1 for altitude
        std::string command;  // "move", "halt", "setVelocity", etc.
        double param1;        // Command-specific parameter (e.g., degrees)
        double param2;        // Additional parameter if needed
        size_t nextIssueSlot; // Time slot when command will be issued
    };

    // Queue of pending commands for each unit
    std::vector<std::queue<RotaryCommand>> pendingCommands_;

    // Input latency in slots
    size_t inputLatencySlots_;

    // --- Movement state tracking ---
    struct TrajectoryPoint
    {
        double position;     // Position in degrees
        double velocity;     // Velocity in degrees/second
        double acceleration; // Acceleration in degrees/second^2
    };

    struct MovementProfile
    {
        size_t startSlot;
        size_t endSlot;
        double startPosition;
        double targetPosition;

        // Kinematic parameters for calculation
        double timeAccel_s;
        double timeCruise_s;
        double accelerationSigned;
        double decelerationSigned;

        // Pre-calculated intermediate states for efficiency
        double posAtAccelEnd;
        double velAtAccelEnd;
        double posAtCruiseEnd;
        double velAtCruiseEnd;
    };

    struct AxisState
    {
        //  A vector sorted by startSlot is much faster for sequential access.
        std::vector<MovementProfile> movementQueue;

        // An index to the currently active or next upcoming profile.
        // This avoids O(log N) searches on every slot.
        size_t currentProfileIndex = 0;

        // Caches the last known state for fallback calculations
        TrajectoryPoint lastKnownState = {0.0, 0.0, 0.0};
    };

    // State for each unit's axes: [unitIndex][axis] -> state
    std::vector<std::array<AxisState, 2>> axisStates_;

    /**
     * @brief Process pending commands for the current slot
     *
     * @param currentSlot The current time slot
     */
    void processCommands(size_t currentSlot);

    /**
     * @brief Calculate and update trajectory for a move command
     *
     * @param cmd The move command to process
     * @param startSlot The slot to start the movement from
     * @return size_t The slot when movement will complete
     */
    size_t calculateTrajectory(const RotaryCommand &cmd, size_t startSlot);

    /**
     * @brief Calculate trajectory for a single axis
     *
     * @param unitIndex The unit index
     * @param axis The axis (0=azimuth, 1=altitude)
     * @param targetDegrees Target position (absolute) in degrees
     * @param startSlot Slot to start movement
     * @return size_t Slot when movement will complete
     */
    size_t calculateAxisTrajectory(size_t unitIndex, int axis, double targetDegrees, size_t startSlot);

    /**
     * @brief Get the trajectory point at a specific slot for an axis
     *
     * @param unitIndex The unit index
     * @param axis The axis (0=azimuth, 1=altitude)
     * @param slot The time slot
     * @return TrajectoryPoint The trajectory point at the specified slot
     */
    inline TrajectoryPoint getTrajectoryPointAt(size_t unitIndex, int axis, size_t slot);

    /**
     * @brief Get the current position of an axis
     *
     * @param unitIndex The unit index
     * @param axis The axis (0=azimuth, 1=altitude)
     * @param slot The time slot
     * @return double The current angle in degrees
     */
    double getCurrentPosition(size_t unitIndex, int axis, size_t slot);

    /**
     * @brief Get velocity for an axis from the configuration
     *
     * @param unitIndex The unit index
     * @param axis The axis (0=azimuth, 1=altitude)
     * @return double The configured velocity in degrees/second
     */
    double getConfigVelocity(size_t unitIndex, int axis);

    /**
     * @brief Get acceleration for an axis from the configuration
     *
     * @param unitIndex The unit index
     * @param axis The axis (0=azimuth, 1=altitude)
     * @return double The configured acceleration in degrees/second^2
     */
    double getConfigAcceleration(size_t unitIndex, int axis);

    /**
     * @brief Update engine with rotary state for a specific slot
     *
     * @param unitIndex The unit index
     * @param slot The time slot
     * @param rotaryData The rotary data to set
     */
    void updateEngineRotaryData(size_t unitIndex, size_t slot, const rotaryObject &rotaryData);

    /**
     * @brief Convert between degrees and counts
     *
     * @param degrees Input angle in degrees
     * @param axis The axis (0=azimuth, 1=altitude)
     * @param toCounts If true, convert degrees to counts; if false, convert counts to degrees
     * @return double The converted value
     */
    double convertUnits(double value, int axis, bool toCounts);

    // Parse Methods below
    void parseMoveDegree(size_t unitIndex, std::istringstream &iss);
    void parseMoveCount(size_t unitIndex, std::istringstream &iss);
    void parseMoveDualDegree(size_t unitIndex, std::istringstream &iss);
    void parseMoveDualCount(size_t unitIndex, std::istringstream &iss);
    void parseSetVelocity(size_t unitIndex, std::istringstream &iss);
    void parseSetAcceleration(size_t unitIndex, std::istringstream &iss);
    void parseHalt(size_t unitIndex, std::istringstream &iss);
};

#endif // ROTARYSIM_H
