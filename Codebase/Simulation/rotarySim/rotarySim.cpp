#include "rotarySim.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sstream>

static bool debug = false;

// Constructor
rotarySim::rotarySim()
    : engine_(nullptr), slotDuration_sec_(0.0), numUnits_(0), numTimeSlots_(0), inputLatencySlots_(0)
{
}

// Destructor
rotarySim::~rotarySim() {}

// Convert between degrees and counts
double rotarySim::convertUnits(double value, int axis, bool toCounts)
{
    double countsPerDegree = (axis == 0) ? AXIS0_COUNTS_PER_DEGREE : AXIS1_COUNTS_PER_DEGREE;

    if (toCounts)
    {
        // Convert degrees to counts
        return value * countsPerDegree;
    }
    else
    {
        // Convert counts to degrees
        return value / countsPerDegree;
    }
}

// Initialize the rotary simulator
void rotarySim::initialize(const ConfigFile &paramConfig, mobileTHzEngine *engine)
{
    if (!engine)
    {
        throw std::runtime_error("rotarySim::initialize called with null engine pointer.");
    }
    engine_ = engine;
    config_ = paramConfig;
    slotDuration_sec_ = config_.engine_slot_time_microsec / 1.0e6;
    if (slotDuration_sec_ <= 0.0)
    {
        throw std::runtime_error("Invalid slot duration (must be positive).");
    }
    numUnits_ = engine_->getNumUnits();
    numTimeSlots_ = engine_->getNumTimeSlots();

    pendingCommands_.clear();
    axisStates_.clear();

    pendingCommands_.resize(numUnits_);
    axisStates_.resize(numUnits_);

    double latencyMs = config_.engine_mode == "SIMULATION" ? 0.0 : 10.0;
    inputLatencySlots_ = static_cast<size_t>(
        std::ceil(latencyMs * 1000.0 / config_.engine_slot_time_microsec));

    if (debug)
        std::cout << "RotarySim initialized with " << numUnits_
                  << " units, latency: " << inputLatencySlots_ << " slots" << std::endl;
}

// Process the current simulation slot
void rotarySim::processSlot(size_t currentSlot)
{
    if (!engine_)
    {
        std::cerr << "Error: RotarySim not initialized" << std::endl;
        return;
    }

    // This might add new MovementProfiles to the queues for future slots
    processCommands(currentSlot);

    // Update rotary state for each unit
    for (size_t unitIdx = 0; unitIdx < numUnits_; ++unitIdx)
    {
        // If the unit's algorithm is PerfectAlignment, it controls the rotary
        // state directly. Skip all rotarySim processing for this unit to
        // avoid overwriting the algorithm's data.
        if (engine_->getEngineUnit(unitIdx).algorithm == "PerfectAlignment") {
            continue; // Skip to the next unit
        }

        rotaryObject rotaryData;
        bool isMoving = false;

        for (int axisIdx = 0; axisIdx < 2; ++axisIdx)
        {
            rotaryAxis &axis = (axisIdx == 0) ? rotaryData.azimuth : rotaryData.altitude;

            const TrajectoryPoint point = getTrajectoryPointAt(unitIdx, axisIdx, currentSlot);

            // Update the axis state with the calculated point
            axis.angle = point.position;
            axis.velocity = point.velocity;
            axis.acceleration = point.acceleration;

            // Cache the result for the next slot's calculation
            axisStates_[unitIdx][axisIdx].lastKnownState = point;

            if (std::abs(point.velocity) > 1e-6)
            {
                isMoving = true;
            }
        }

        rotaryData.isMoving = (isMoving ? 1.0 : 0.0);
        updateEngineRotaryData(unitIdx, currentSlot, rotaryData);
    }
}

// Process commands due for execution
void rotarySim::processCommands(size_t currentSlot)
{
    for (size_t unitIdx = 0; unitIdx < numUnits_; ++unitIdx)
    {
        auto &queue = pendingCommands_[unitIdx];

        while (!queue.empty())
        {
            const RotaryCommand &cmd = queue.front();
            size_t executionSlot = cmd.nextIssueSlot + inputLatencySlots_;
            if (executionSlot > currentSlot)
            {
                break;
            }

            // Time to execute this command
            if (cmd.command == "move")
            {
                // Calculate trajectory for this movement
                calculateTrajectory(cmd, currentSlot);
            }
            else if (cmd.command == "moveDual")
            {
                // Handle dual axis move - create separate commands for each axis
                RotaryCommand azCmd = cmd;
                azCmd.axis = 0; // Azimuth
                azCmd.command = "move";
                calculateTrajectory(azCmd, currentSlot);

                RotaryCommand altCmd = cmd;
                altCmd.axis = 1; // Altitude
                altCmd.command = "move";
                altCmd.param1 = altCmd.param2; // Move param2 to param1
                calculateTrajectory(altCmd, currentSlot);
            }
            else if (cmd.command == "halt")
            {
                int axis = cmd.axis;
                auto &moveQueue = axisStates_[unitIdx][axis].movementQueue;

                // Find the first future profile (one starting after the halt time).
                auto it = std::lower_bound(moveQueue.begin(),
                                           moveQueue.end(),
                                           currentSlot,
                                           [](const MovementProfile &p, size_t slot)
                                           {
                                               return p.startSlot < slot;
                                           });

                // Erase all future movements.
                moveQueue.erase(it, moveQueue.end());

                // If a movement is in progress, truncate it.
                // It must be the new `back()` of the vector, if any.
                if (!moveQueue.empty())
                {
                    auto &activeProfile = moveQueue.back();

                    // Check if the halt is issued during this profile's execution.
                    if (currentSlot >= activeProfile.startSlot && currentSlot < activeProfile.endSlot)
                    {
                        // Get the state at the moment of halt to correctly set the new target.
                        TrajectoryPoint haltPoint = getTrajectoryPointAt(unitIdx, axis, currentSlot);

                        // Modify the profile in-place to end now.
                        activeProfile.endSlot = currentSlot;
                        activeProfile.targetPosition = haltPoint.position;
                    }
                }
            }
            else if (cmd.command == "setVelocity")
            {
                // Update velocity settings - these will be used for future moves
                if (cmd.axis == 0)
                {
                    std::cout << "Set azimuth velocity for unit " << unitIdx << " to " << cmd.param1
                              << " (0.1 counts/sec)" << std::endl;
                }
                else
                {
                    std::cout << "Set altitude velocity for unit " << unitIdx << " to "
                              << cmd.param1 << " (0.1 counts/sec)" << std::endl;
                }
            }
            else if (cmd.command == "setAcceleration")
            {
                // Update acceleration settings
                if (cmd.axis == 0)
                {
                    std::cout << "Set azimuth acceleration for unit " << unitIdx << " to "
                              << cmd.param1 << " (counts/sec^2)" << std::endl;
                }
                else
                {
                    std::cout << "Set altitude acceleration for unit " << unitIdx << " to "
                              << cmd.param1 << " (counts/sec^2)" << std::endl;
                }
            }

            // Remove the executed command
            queue.pop();
        }
    }
}

// Calculate trajectory for a movement command
size_t rotarySim::calculateTrajectory(const RotaryCommand &cmd, size_t startSlot)
{
    if (cmd.command != "move")
    {
        return startSlot; // Not a movement command
    }

    double currentPosition = getCurrentPosition(cmd.unitIndex, cmd.axis, startSlot);
    double targetPosition = currentPosition + cmd.param1; // Relative movement

    return calculateAxisTrajectory(cmd.unitIndex, cmd.axis, targetPosition, startSlot);
}

// Calculate trajectory for a single axis with proper physics
size_t rotarySim::calculateAxisTrajectory(size_t unitIndex, int axis, double targetDegrees, size_t startSlot)
{
    // --- Phase 1: Physics calculation ---
    double currentDegrees = getCurrentPosition(unitIndex, axis, startSlot);
    double maxVelocityDegPerSec = getConfigVelocity(unitIndex, axis);
    double accelDegPerSecSq = getConfigAcceleration(unitIndex, axis);
    double distanceDegrees = targetDegrees - currentDegrees;

    if (std::abs(distanceDegrees) < 1e-9)
    {
        return startSlot;
    }

    double maxVelocitySigned = (distanceDegrees > 0) ? std::abs(maxVelocityDegPerSec) : -std::abs(maxVelocityDegPerSec);
    double accelerationSigned = (distanceDegrees > 0) ? std::abs(accelDegPerSecSq) : -std::abs(accelDegPerSecSq);
    double decelerationSigned = -accelerationSigned;

    double timeToMaxVel_s = std::abs(maxVelocitySigned / accelerationSigned);
    double distAccelPhaseFull = 0.5 * std::abs(accelerationSigned) * timeToMaxVel_s * timeToMaxVel_s;

    double timeAccel_s, timeCruise_s, timeDecel_s;
    if (distAccelPhaseFull * 2.0 <= std::abs(distanceDegrees))
    { /* Trapezoidal */
        double distCruise = std::abs(distanceDegrees) - (2.0 * distAccelPhaseFull);
        timeAccel_s = timeToMaxVel_s;
        timeDecel_s = timeToMaxVel_s;
        timeCruise_s = distCruise / std::abs(maxVelocitySigned);
    }
    else
    { /* Triangular */
        timeCruise_s = 0.0;
        timeAccel_s = std::sqrt(std::abs(distanceDegrees) / std::abs(accelerationSigned));
        timeDecel_s = timeAccel_s;
    }
    double totalTime = timeAccel_s + timeCruise_s + timeDecel_s;
    size_t totalSlots = static_cast<size_t>(std::ceil(totalTime / slotDuration_sec_));
    size_t endSlot = startSlot + totalSlots;
    endSlot = std::min(endSlot, numTimeSlots_ - 1);

    // --- Phase 2: Create the MovementProfile ---
    MovementProfile profile;
    profile.startSlot = startSlot;
    profile.endSlot = endSlot;
    profile.startPosition = currentDegrees;
    profile.targetPosition = targetDegrees;
    profile.accelerationSigned = accelerationSigned;
    profile.decelerationSigned = decelerationSigned;
    profile.timeAccel_s = timeAccel_s;
    profile.timeCruise_s = timeCruise_s;

    // Pre-calculate intermediate states to make on-demand calculation faster
    profile.velAtAccelEnd = accelerationSigned * timeAccel_s;
    profile.posAtAccelEnd = currentDegrees + 0.5 * accelerationSigned * timeAccel_s * timeAccel_s;
    profile.velAtCruiseEnd = profile.velAtAccelEnd;
    profile.posAtCruiseEnd = profile.posAtAccelEnd + profile.velAtCruiseEnd * timeCruise_s;

    // --- Phase 3: Manage the Queue ---
    auto &moveQueue = axisStates_[unitIndex][axis].movementQueue;

    // Find the first profile that does NOT start before 'startSlot'. O(log N).
    auto it = std::lower_bound(moveQueue.begin(),
                               moveQueue.end(),
                               startSlot,
                               [](const MovementProfile &p, size_t slot)
                               {
                                   return p.startSlot < slot;
                               });

    // Erase all profiles from this point onwards, as they are now superseded.
    moveQueue.erase(it, moveQueue.end());

    // Add the new profile to the end. Since new commands are issued at
    // increasing `startSlot`s, push_back maintains the sort order.
    moveQueue.push_back(profile);

    return endSlot;
}

// Calculates trajectory point per slot
// This function is now much faster due to O(log N) map lookup.
inline rotarySim::TrajectoryPoint rotarySim::getTrajectoryPointAt(size_t unitIndex,
                                                                  int axis,
                                                                  size_t slot)
{
    AxisState &state = axisStates_[unitIndex][axis];
    auto &moveQueue = state.movementQueue;
    auto &index = state.currentProfileIndex;

    if (moveQueue.empty())
    {
        return state.lastKnownState;
    }

    // Move the index forward as long as the current slot is past the end of the profile it points to.
    while (index < moveQueue.size() && moveQueue[index].endSlot < slot)
    {
        index++;
    }

    // After the loop, 'index' points to either:
    // 1. The first profile that has not yet ended (could be active or in the future).
    // 2. The end of the vector (size()).

    if (index >= moveQueue.size())
    {
        // All scheduled movements are in the past. Return the final state of the very last movement.
        const auto &lastProfile = moveQueue.back();
        state.lastKnownState = {lastProfile.targetPosition, 0.0, 0.0};
        return state.lastKnownState;
    }

    // Check the profile at the current index.
    const auto &profile = moveQueue[index];
    if (slot >= profile.startSlot)
    {
        // We are within this profile's duration (it's the active movement).
        // Perform the kinematic calculation
        double time_since_move_start_s = (slot - profile.startSlot + 1) * slotDuration_sec_;
        const double timeAccelEnd_s = profile.timeAccel_s;
        const double timeCruiseEnd_s = timeAccelEnd_s + profile.timeCruise_s;

        if (time_since_move_start_s <= timeAccelEnd_s)
        { /* Accel phase */
            return {profile.startPosition + 0.5 * profile.accelerationSigned * time_since_move_start_s * time_since_move_start_s,
                    profile.accelerationSigned * time_since_move_start_s,
                    profile.accelerationSigned};
        }
        else if (time_since_move_start_s <= timeCruiseEnd_s)
        { /* Cruise phase */
            double t_in_cruise = time_since_move_start_s - timeAccelEnd_s;
            return {profile.posAtAccelEnd + profile.velAtAccelEnd * t_in_cruise,
                    profile.velAtAccelEnd,
                    0.0};
        }
        else
        { /* Decel phase */
            double t_in_decel = time_since_move_start_s - timeCruiseEnd_s;
            return {profile.posAtCruiseEnd + profile.velAtCruiseEnd * t_in_decel + 0.5 * profile.decelerationSigned * t_in_decel * t_in_decel,
                    profile.velAtCruiseEnd + profile.decelerationSigned * t_in_decel,
                    profile.decelerationSigned};
        }
    }
    else
    {
        // The slot is in a gap: after the previous profile ended, but before this one started.
        if (index == 0)
        {
            // We are before the very first movement.
            return state.lastKnownState;
        }
        else
        {
            // We are after profile [index-1] finished. Hold its target position.
            const auto &prevProfile = moveQueue[index - 1];
            state.lastKnownState = {prevProfile.targetPosition, 0.0, 0.0};
            return state.lastKnownState;
        }
    }
}

// Get the current position of an axis
double rotarySim::getCurrentPosition(size_t unitIndex, int axis, size_t slot)
{
    // For a move starting at 'slot', its initial position is the final
    // position of the system at the end of 'slot - 1'.
    if (slot == 0)
    {
        return 0.0; // The simulation always starts at position 0.
    }
    // Calculate the position at the previous slot on demand.
    return getTrajectoryPointAt(unitIndex, axis, slot - 1).position;
}

// Get configured velocity for an axis (in degrees/second)
double rotarySim::getConfigVelocity(size_t unitIndex, int axis)
{
    // Get velocity in 0.1 counts/second
    double velDegreesPerSec = 0.0;

    try
    {
        const std::vector<engineUnit> &units = engine_->getAllEngineUnits();
        if (unitIndex < units.size())
        {
            velDegreesPerSec = (axis == 0) ? units[unitIndex].Rotary_azimuth_velocity_degpersec
                                           : units[unitIndex].Rotary_altitude_velocity_degpersec;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error accessing engine units: " << e.what() << std::endl;
        // Use default acceleration
        velDegreesPerSec = 0.0;
    }

    return velDegreesPerSec;
}

// Get configured acceleration for an axis (in degrees/second^2)
double rotarySim::getConfigAcceleration(size_t unitIndex, int axis)
{
    // Get acceleration in counts/second^2
    double accelDegreesPerSecSq = 0.0;

    try
    {
        const std::vector<engineUnit> &units = engine_->getAllEngineUnits();
        if (unitIndex < units.size())
        {
            accelDegreesPerSecSq = (axis == 0)
                                       ? units[unitIndex].Rotary_azimuth_acceleration_degpersecsq
                                       : units[unitIndex].Rotary_altitude_acceleration_degpersecsq;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error accessing engine units: " << e.what() << std::endl;
        // Use default acceleration
        accelDegreesPerSecSq = 0.0;
    }

    return accelDegreesPerSecSq;
}

// Update engine with rotary state
void rotarySim::updateEngineRotaryData(size_t unitIndex, size_t slot, const rotaryObject &rotaryData)
{
    try
    {
        engine_->setRotaryData(unitIndex, slot, rotaryData);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error updating engine rotary data: " << e.what() << std::endl;
    }
}

// --- Command Interface Methods ---
void rotarySim::setVelocity(size_t unitIndex, int axis, double velocity)
{
    if (unitIndex >= numUnits_ || (axis != 0 && axis != 1))
    {
        std::cerr << "Invalid unit index or axis in setVelocity" << std::endl;
        return;
    }

    // Create and queue the command
    RotaryCommand cmd;
    cmd.unitIndex = unitIndex;
    cmd.axis = axis;
    cmd.command = "setVelocity";
    cmd.param1 = velocity;
    cmd.nextIssueSlot = engine_->getCurrentSlot() < engine_->getNumTimeSlots()
                            ? engine_->getCurrentSlot() + 1
                            : engine_->getNumTimeSlots();

    pendingCommands_[unitIndex].push(cmd);
}

void rotarySim::setAcceleration(size_t unitIndex, int axis, double acceleration)
{
    if (unitIndex >= numUnits_ || (axis != 0 && axis != 1))
    {
        std::cerr << "Invalid unit index or axis in setAcceleration" << std::endl;
        return;
    }

    // Create and queue the command
    RotaryCommand cmd;
    cmd.unitIndex = unitIndex;
    cmd.axis = axis;
    cmd.command = "setAcceleration";
    cmd.param1 = acceleration;
    cmd.nextIssueSlot = engine_->getCurrentSlot() < engine_->getNumTimeSlots()
                            ? engine_->getCurrentSlot() + 1
                            : engine_->getNumTimeSlots();

    pendingCommands_[unitIndex].push(cmd);
}

void rotarySim::moveByDegrees(size_t unitIndex, int axis, double degrees)
{
    if (unitIndex >= numUnits_ || (axis != 0 && axis != 1))
    {
        std::cerr << "Invalid unit index or axis in moveByDegrees" << std::endl;
        return;
    }

    // Create and queue the command
    RotaryCommand cmd;
    cmd.unitIndex = unitIndex;
    cmd.axis = axis;
    cmd.command = "move";
    cmd.param1 = degrees;
    cmd.nextIssueSlot = engine_->getCurrentSlot() < engine_->getNumTimeSlots()
                            ? engine_->getCurrentSlot() + 1
                            : engine_->getNumTimeSlots();

    pendingCommands_[unitIndex].push(cmd);
}

void rotarySim::moveDual(size_t unitIndex, double azimuthDegrees, double altitudeDegrees)
{
    if (unitIndex >= numUnits_)
    {
        std::cerr << "Invalid unit index in moveDual" << std::endl;
        return;
    }

    // Create and queue the command
    RotaryCommand cmd;
    cmd.unitIndex = unitIndex;
    cmd.axis = -1;
    cmd.command = "moveDual";
    cmd.param1 = azimuthDegrees;
    cmd.param2 = altitudeDegrees;
    cmd.nextIssueSlot = engine_->getCurrentSlot() < engine_->getNumTimeSlots()
                            ? engine_->getCurrentSlot() + 1
                            : engine_->getNumTimeSlots();

    pendingCommands_[unitIndex].push(cmd);
}

void rotarySim::halt(size_t unitIndex, int axis)
{
    if (unitIndex >= numUnits_ || (axis != 0 && axis != 1))
    {
        std::cerr << "Invalid unit index or axis in halt" << std::endl;
        return;
    }

    // Create and queue the command
    RotaryCommand cmd;
    cmd.unitIndex = unitIndex;
    cmd.axis = axis;
    cmd.command = "halt";
    cmd.nextIssueSlot = engine_->getCurrentSlot() < engine_->getNumTimeSlots()
                            ? engine_->getCurrentSlot() + 1
                            : engine_->getNumTimeSlots();

    pendingCommands_[unitIndex].push(cmd);
}

void rotarySim::haltAll(size_t unitIndex)
{
    if (unitIndex >= numUnits_)
    {
        std::cerr << "Invalid unit index in haltAll" << std::endl;
        return;
    }

    // Create and queue the command
    RotaryCommand cmd;
    cmd.unitIndex = unitIndex;
    cmd.command = "haltAll";
    cmd.nextIssueSlot = engine_->getCurrentSlot() < engine_->getNumTimeSlots()
                            ? engine_->getCurrentSlot() + 1
                            : engine_->getNumTimeSlots();

    pendingCommands_[unitIndex].push(cmd);
}

// --- Input Function (Mimics rotarylib) ---
void rotarySim::inputFunction(size_t unitIndex, const std::string &commandString)
{
    if (unitIndex >= numUnits_ || !engine_)
    {
        std::cerr << "Invalid unit index or uninitialized engine in inputFunction" << std::endl;
        return;
    }
    if (!engine_->getEngineUnit(unitIndex).Rotary_enabled)
    {
        std::cerr << "Unit " << unitIndex << " is disabled. Skipping command." << std::endl;
        return;
    }

    std::istringstream iss(commandString);
    std::string command;
    iss >> command;

    try
    {
        // Call parsing helpers which now call queueing methods
        if (command == "m")
        {
            parseMoveDegree(unitIndex, iss);
        }
        else if (command == "mc")
        {
            parseMoveCount(unitIndex, iss);
        }
        else if (command == "mdd")
        {
            parseMoveDualDegree(unitIndex, iss);
        }
        else if (command == "mdc")
        {
            parseMoveDualCount(unitIndex, iss);
        }
        else if (command == "setvelocity")
        {
            parseSetVelocity(unitIndex, iss);
        }
        else if (command == "setacceleration")
        {
            parseSetAcceleration(unitIndex, iss);
        }
        else if (command == "halt")
        {
            parseHalt(unitIndex, iss);
        }
        else
        {
            std::cerr << "[RotarySim] Invalid command '" << commandString << "' for unit "
                      << unitIndex << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[RotarySim] Error parsing/queueing command '" << commandString
                  << "' for unit " << unitIndex << ": " << e.what() << std::endl;
    }
}

void rotarySim::parseMoveDegree(size_t unitIndex, std::istringstream &iss)
{
    int axis;
    double degrees;
    if (!(iss >> axis >> degrees))
        throw std::runtime_error("Invalid 'm' args.");
    if (axis != 0 && axis != 1)
        throw std::runtime_error("Invalid 'm' axis.");

    moveByDegrees(unitIndex, axis, degrees);
}

void rotarySim::parseMoveCount(size_t unitIndex, std::istringstream &iss)
{
    int axis;
    double counts;
    if (!(iss >> axis >> counts))
        throw std::runtime_error("Invalid 'mc' args.");
    if (axis != 0 && axis != 1)
        throw std::runtime_error("Invalid 'mc' axis.");
    double degrees = convertUnits(counts, axis, false);
    moveByDegrees(unitIndex, axis, degrees);
}

void rotarySim::parseMoveDualDegree(size_t unitIndex, std::istringstream &iss)
{
    int axis1, axis2;
    double degrees1, degrees2;
    if (!(iss >> axis1 >> degrees1 >> axis2 >> degrees2))
        throw std::runtime_error("Invalid 'mdd' args.");
    if (!((axis1 == 0 && axis2 == 1) || (axis1 == 1 && axis2 == 0)))
        throw std::runtime_error("Invalid 'mdd' axis combo.");
    moveDual(unitIndex,
             (axis1 == 0 ? degrees1 : degrees2),
             (axis1 == 1 ? degrees1 : degrees2));
}

void rotarySim::parseMoveDualCount(size_t unitIndex, std::istringstream &iss)
{
    int axis1, axis2;
    double counts1, counts2;
    if (!(iss >> axis1 >> counts1 >> axis2 >> counts2))
        throw std::runtime_error("Invalid 'mdc' args.");
    if (!((axis1 == 0 && axis2 == 1) || (axis1 == 1 && axis2 == 0)))
        throw std::runtime_error("Invalid 'mdc' axis combo.");
    double degrees1 = convertUnits(counts1, axis1, false);
    double degrees2 = convertUnits(counts2, axis2, false);
    moveDual(unitIndex,
             (axis1 == 0 ? degrees1 : degrees2),
             (axis1 == 1 ? degrees1 : degrees2));
}

void rotarySim::parseSetVelocity(size_t unitIndex, std::istringstream &iss)
{
    int axis;
    double velocity_counts_tenths;
    if (!(iss >> axis >> velocity_counts_tenths))
        throw std::runtime_error("Invalid 'setvelocity' args.");
    if (axis != 0 && axis != 1)
        throw std::runtime_error("Invalid 'setvelocity' axis.");
    setVelocity(unitIndex, axis, velocity_counts_tenths);
}

void rotarySim::parseSetAcceleration(size_t unitIndex, std::istringstream &iss)
{
    int axis;
    double accel_counts_sq;
    if (!(iss >> axis >> accel_counts_sq))
        throw std::runtime_error("Invalid 'setacceleration' args.");
    if (axis != 0 && axis != 1)
        throw std::runtime_error("Invalid 'setacceleration' axis.");
    setAcceleration(unitIndex, axis, accel_counts_sq);
}

void rotarySim::parseHalt(size_t unitIndex, std::istringstream &iss)
{
    int axis;
    if (!(iss >> axis))
    {
        haltAll(unitIndex);
    }
    else
    {
        if (axis != 0 && axis != 1)
            throw std::runtime_error("Invalid 'halt' axis.");
        halt(unitIndex, axis);
    }
}
