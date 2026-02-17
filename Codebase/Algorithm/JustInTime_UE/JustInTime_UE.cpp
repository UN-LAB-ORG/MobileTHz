#include "JustInTime_UE.h"
#include "Codebase/Software/mobileTHzEngine/mobileTHzEngine.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <qmath.h>
#include <sstream>
#include <string>
#include <vector>

static bool debug = false;

namespace
{
    constexpr double SPEED_OF_LIGHT = 299792458.0;
    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    constexpr double LN10_OVER_10 = 0.23025850929940457; // std::log(10.0) / 10.0
    constexpr double TEN_OVER_LN10 = 4.342944819032518;  // 10.0 / std::log(10.0)
    constexpr double MIN_WATT_THRESHOLD = 1e-30;
    constexpr double MIN_DBM_FLOOR = -300.0;
    constexpr double LN10 = 2.302585092994046;
    constexpr double MIN_LINEAR_GAIN_THRESHOLD = 1e-30;
    constexpr double MIN_DBI_FLOOR = -300.0;

    inline double dBmtoWatts(double dBm)
    {
        if (dBm <= MIN_DBM_FLOOR)
        {
            return 0.0;
        }
        return std::exp((dBm - 30.0) * LN10_OVER_10);
    }

    inline double wattsToDbm(double watts)
    {
        if (watts <= MIN_WATT_THRESHOLD)
        {
            return MIN_DBM_FLOOR;
        }
        return TEN_OVER_LN10 * std::log(watts) + 30.0;
    }

    inline double dbiToLinear(double dBi)
    {
        // For very small dBi values, the linear gain is effectively zero.
        if (dBi <= MIN_DBI_FLOOR)
        {
            return 0.0;
        }
        return std::exp(dBi * LN10_OVER_10);
    }

    inline double linearToDbi(double linearGain)
    {
        // If the gain is at or below the threshold, return the floor value
        // to avoid std::log(0) or std::log(negative).
        if (linearGain <= MIN_LINEAR_GAIN_THRESHOLD)
        {
            return MIN_DBI_FLOOR;
        }
        return TEN_OVER_LN10 * std::log(linearGain);
    }
    void vectorToAngles(const Position &v, double &azimuthDeg, double &altitudeDeg)
    {
        azimuthDeg = std::atan2(v.y, v.x) * RAD_TO_DEG;
        double xy_dist = std::sqrt(v.x * v.x + v.y * v.y);
        altitudeDeg = std::atan2(v.z, xy_dist) * RAD_TO_DEG;
    }

    Position toEulerAngles(const Quaternion &q)
    {
        Position angles; // x: roll, y: pitch, z: yaw

        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        angles.x = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            angles.y = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            angles.y = std::asin(sinp);

        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        angles.z = std::atan2(siny_cosp, cosy_cosp);

        // Convert radians to degrees
        angles.x *= RAD_TO_DEG;
        angles.y *= RAD_TO_DEG;
        angles.z *= RAD_TO_DEG;

        return angles;
    }

}

Quaternion JustInTime_UE::conjugate(const Quaternion &q) const
{
    return {q.w, -q.x, -q.y, -q.z};
}

Quaternion JustInTime_UE::multiplyQuaternions(const Quaternion &q1, const Quaternion &q2) const
{
    // Performs q1 * q2 (rotation q2 then rotation q1)
    Quaternion result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

Position JustInTime_UE::rotateVectorByQuaternion(const Position &v, const Quaternion &q) const
{
    Quaternion v_quat = {0, v.x, v.y, v.z};
    Quaternion q_conj = conjugate(q);
    Quaternion rotated_v_quat = multiplyQuaternions(multiplyQuaternions(q, v_quat), q_conj);
    return {rotated_v_quat.x, rotated_v_quat.y, rotated_v_quat.z};
}

// Constructor
JustInTime_UE::JustInTime_UE()
{
    setState(AlgoState::REFERENCE, AlgoAction::NONE);
}

// Configuration
void JustInTime_UE::configure(const ConfigFile &config, const engineUnit &unitConfig)
{
    slotTimeMicrosec = config.engine_slot_time_microsec;
    if (slotTimeMicrosec <= 0)
    {
        std::cerr << "[" << algorithmName << "] Error: Invalid engine_slot_time_microsec ("
                  << slotTimeMicrosec << "). Using default 1000 us." << std::endl;
        slotTimeMicrosec = 1000.0;
    }

    currentState = AlgoState::REFERENCE;
    currentAction = AlgoAction::NONE;
    algorithmName = unitConfig.algorithm + "_" + unitConfig.label;
    txPowerDbm = wattsToDbm(unitConfig.TxAntenna_transmitPower_watts);
    txGainDbi = linearToDbi(
        unitConfig.radiationPatterns_maxGain_linear.at(0)); // ASSUMING same pattern for Tx
    rxGainDbi = linearToDbi(
        unitConfig.radiationPatterns_maxGain_linear.at(0)); // ASSUMING same pattern for Rx
    frequencyHz = config.engine_carrier_frequency;
    configured = true;
    imuEnabledRuntime = unitConfig.IMU_enabled;
    last_transmitted_relative_position_m = {0.0f, 0.0f, 0.0f};
    last_transmitted_orientation_ = {0.0f, 0.0f, 0.0f, 1.0f};
    last_tracked_orientation_ = {0.0f, 0.0f, 0.0f, 1.0f};
    filterInitialized = false;
    baselineEstablished = false;
    warmupCounter = 0;
    currentReferencePower = -9999999;
    shortTermEMA = -9999999;
    longTermEMA = -9999999;
    lastRawPower = -9999999;
    lastProcessedRxSampleSlot = std::numeric_limits<size_t>::max();
    lastProcessedImuSampleSlot = std::numeric_limits<size_t>::max();
    localSpiralOffsets.clear();
    m_isPeakSeeking = false;
    m_peakSeekingBestPower = -9999999;
    m_peakSeekingBestIndex = -1;
    currentRelativePosition = {0, 0, 0};
    initialEulerAngles = {0, 0, 0};
    lastImuSampleTimeSec = -1.0;
    initialIMUTargetAz = 0.0f;
    initialIMUTargetAlt = 0.0f;
    IMUTargetAz = 0.0f;
    IMUTargetAlt = 0.0f;
    localSpiralBeamWidthDeg = unitConfig.radiationPatterns_hpbw.at(0) * 0.1;
    realignSuccessMarginIncreaseRateDbPerSec = linearToDbi(
                                                   unitConfig.radiationPatterns_maxGain_linear.at(0)) *
                                               0.5;
    localSpiralTriggerThresholdDb = 3;
    realignSuccessInitialMarginDb = 1;
    panicPacketRotationThresholdDeg = unitConfig.radiationPatterns_hpbw.at(0) * 0.1;

    // Ensure panicPacketMaxImuSamplesToStore_ is a valid positive value
    if (panicPacketMaxImuSamplesToStore_ <= 0)
    {
        if (debug)
            std::cerr << "[" << algorithmName
                      << "] Warning: panicPacketMaxImuSamplesToStore_ is invalid ("
                      << panicPacketMaxImuSamplesToStore_ << "). Defaulting to 1." << std::endl;
        panicPacketMaxImuSamplesToStore_ = 1; // Prevent zero-capacity buffer
    }

    // Reconfigure the circular buffer's capacity
    // This will resize the underlying vector and clear the buffer.
    imuSamples_.reconfigure_capacity(panicPacketMaxImuSamplesToStore_);

    // Precompute IMU weights for exponential decay
    precomputed_imu_weights_.resize(panicPacketMaxImuSamplesToStore_);
    for (int k = 0; k < panicPacketMaxImuSamplesToStore_; ++k)
    {
        const double normalized_k = (panicPacketMaxImuSamplesToStore_ > 1)
                                        ? static_cast<double>(k) / (panicPacketMaxImuSamplesToStore_ - 1)
                                        : 0.0;
        precomputed_imu_weights_[k] = std::exp(m_imuWeightDecayConstant * normalized_k);
    }

    // --- JIT Packet Specific Configuration & Reset ---
    if (slotTimeMicrosec > 0)
    {
        panicPacketCooldownSlotsConverted_ = static_cast<size_t>(
            std::ceil(panicPacketCooldownMicroseconds_ / slotTimeMicrosec));
    }
    else
    {
        panicPacketCooldownSlotsConverted_ = 100;
        if (debug)
            std::cerr << "[" << algorithmName
                      << "] Warning: Invalid slotTimeMicrosec for panic cooldown calculation."
                      << std::endl;
    }
    lastJITPacketSentSlot_ = 0;
    nextJITPacketId_ = 1;

    if (enableLocalSpiral && localSpiralMaxPoints > 0)
    {
        generateLocalSpiralOffsets();
        if (debug && localSpiralOffsets.empty())
        {
            std::cerr << "[" << algorithmName
                      << "] Warning: Failed to generate local spiral offsets. Disabling feature."
                      << std::endl;
            enableLocalSpiral = false;
        }
    }
    else
    {
        if (debug)
            std::cout << "[" << algorithmName << "] Local spiral fine-tuning disabled."
                      << std::endl;
        enableLocalSpiral = false;
    }

    setState(AlgoState::REFERENCE, AlgoAction::NONE);
    if (!imuEnabledRuntime)
    {
        std::cerr << "[" << algorithmName
                  << "] CRITICAL WARNING: IMU is disabled in config, this algorithm *requires* IMU."
                  << std::endl;
    }

    if (debug)
    {
        std::cout << "[" << algorithmName
                  << "] Configured. IMU Sig. Motion Thr: " << imuSignificantMotionThreshold_
                  << " m/s^2, IMU Panic Accum. Mag. Thr: " << imuPanicAccumulatedMagnitudeThreshold_
                  << " m/s^2, Cooldown: " << panicPacketCooldownMicroseconds_ << "us ("
                  << panicPacketCooldownSlotsConverted_
                  << " slots), IMU Samples: " << panicPacketMaxImuSamplesToStore_ << std::endl;
    }
}

// Set State Helpe
void JustInTime_UE::setState(AlgoState newStatus, AlgoAction newAction)
{
    currentState = newStatus;
    currentAction = newAction;
    if (isLocalSpiraling && (newStatus == AlgoState::REFERENCE || newStatus == AlgoState::ERROR_STATUS))
    {
        isLocalSpiraling = false;
        currentLocalSpiralIndex = -1;
    }
}

// Get Status
algorithmObject JustInTime_UE::getStatus() const
{
    algorithmObject currentStatus;
    currentStatus.status = static_cast<double>(currentState);
    currentStatus.action = static_cast<double>(currentAction);
    return currentStatus;
}

// Power Filter Update Function
bool JustInTime_UE::updatePowerFilter(double rawPower)
{
    if (!std::isfinite(rawPower) || rawPower <= -200)
        return false;
    bool isSpike = false;
    if (filterInitialized && std::isfinite(lastRawPower))
    {
        if (std::abs(rawPower - lastRawPower) > filterSpikeThresholdDb)
            isSpike = true;
    }

    if (!isSpike)
    {
        if (!filterInitialized)
        {
            shortTermEMA = rawPower;
            longTermEMA = rawPower;
            filterInitialized = true;
            warmupCounter = 0;
        }
        else
        {
            shortTermEMA = filterAlphaShort * rawPower + (1.0f - filterAlphaShort) * shortTermEMA;
            if (currentState == AlgoState::MONITORING || currentState == AlgoState::REFERENCE)
            {
                longTermEMA = filterAlphaLong * rawPower + (1.0f - filterAlphaLong) * longTermEMA;
            }
        }
        lastRawPower = rawPower;
    }

    if (!isSpike)
        return true;
    else
        return false;
}

// Shortest Angle Difference Helper
double JustInTime_UE::shortestAngleDiff(double targetDeg, double currentDeg) const
{
    double diff = targetDeg - currentDeg;
    diff = fmod(diff + 180.0, 360.0) - 180.0;
    // Handle cases where fmod returns -0.0
    return (diff < -180.0) ? diff + 360.0 : diff;
}

// Distance Estimation Helper
double JustInTime_UE::estimateDistanceFromPower(double rxPowerDbm) const
{
    if (!std::isfinite(rxPowerDbm) || frequencyHz <= 0)
    {
        return -1.0; // Invalid input
    }

    // Calculate total gain and convert power to linear watts
    double totalGainLinear = std::exp((txGainDbi + rxGainDbi) * LN10_OVER_10);
    double txPowerLinear = std::pow(10.0, (txPowerDbm - 30.0) / 10.0); // dBm to W
    double rxPowerLinear = std::pow(10.0, (rxPowerDbm - 30.0) / 10.0); // dBm to W

    if (txPowerLinear <= 0 || rxPowerLinear <= 0 || totalGainLinear <= 0)
    {
        return -1.0; // Invalid linear values
    }

    // --- Friis Transmission Equation ---
    double lambda_m = SPEED_OF_LIGHT / (frequencyHz); // frequency Ghz to Hz
    if (lambda_m <= 0)
    {
        return -1.0; // Invalid wavelength
    }
    double factor = txPowerLinear * totalGainLinear * std::pow(lambda_m / (4.0 * M_PI), 2.0);

    double distance = std::sqrt(factor / rxPowerLinear);

    return distance;
}

// Converts spherical coordinates (in the local frame) to a Cartesian vector.
Position JustInTime_UE::anglesToVector(double azDeg, double altDeg, double distance) const
{
    // Convert degrees to radians for trigonometric functions
    double azRad = azDeg * DEG_TO_RAD;
    double altRad = altDeg * DEG_TO_RAD;

    Position vec;
    vec.x = distance * std::cos(altRad) * std::cos(azRad);
    vec.y = distance * std::cos(altRad) * std::sin(azRad);
    vec.z = distance * std::sin(altRad);

    return vec;
}

void JustInTime_UE::updateKinematics(const imuObject &currentImu, double currentTimeSec)
{
    if (!baselineEstablished || lastImuSampleTimeSec < 0 || currentTimeSec <= lastImuSampleTimeSec)
    {
        lastImuSampleTimeSec = currentTimeSec;
        return;
    }

    if (!std::isfinite(currentImu.acceleration.x) || !std::isfinite(currentImu.acceleration.y) || !std::isfinite(currentImu.acceleration.z) || !std::isfinite(currentImu.quaternion.w))
    {
        std::cerr << "[" << algorithmName
                  << "] Warning: Invalid IMU data in updateKinematics. Skipping." << std::endl;
        lastImuSampleTimeSec = currentTimeSec;
        return;
    }

    double deltaTime = currentTimeSec - lastImuSampleTimeSec;
    if (deltaTime <= 0 || deltaTime > 1.0)
    {
        std::cerr << "[" << algorithmName << "] Warning: Invalid deltaTime (" << deltaTime
                  << ") in updateKinematics. Skipping." << std::endl;
        lastImuSampleTimeSec = currentTimeSec;
        return;
    }

    // 1. Get current acceleration reading (in local device frame).
    const Position &localAcceleration = currentImu.acceleration;

    // 2. Rotate the acceleration reading from the local frame into the fixed world frame.
    Position worldAcceleration = rotateVectorByQuaternion(localAcceleration,
                                                          conjugate(currentImu.quaternion));

    // 3. Subtract the constant world gravity vector to get true linear motion in the world frame.
    Position motionAccelerationWorld = {worldAcceleration.x - baselineAcceleration.x,
                                        worldAcceleration.y - baselineAcceleration.y,
                                        worldAcceleration.z - baselineAcceleration.z};

    // 4. Integrate acceleration in the WORLD FRAME to get velocity.
    currentVelocity.x += motionAccelerationWorld.x * deltaTime;
    currentVelocity.y += motionAccelerationWorld.y * deltaTime;
    currentVelocity.z += motionAccelerationWorld.z * deltaTime;

    // 5. Integrate velocity in the WORLD FRAME to get relative position.
    currentRelativePosition.x += currentVelocity.x * deltaTime;
    currentRelativePosition.y += currentVelocity.y * deltaTime;
    currentRelativePosition.z += currentVelocity.z * deltaTime;

    lastImuSampleTimeSec = currentTimeSec;
}

// Calculate Target Angles Helper

void JustInTime_UE::calculateTargetAngles(const imuObject &currentImu)
{
    if (!baselineEstablished || initialDistance <= 0)
    {
        return;
    }

    Position targetWorldPosition = rotateVectorByQuaternion(initialPointingVectorLocal_, conjugate(initialOrientation));

    // 2. Calculate the new required pointing vector in the world frame.
    Position newWorldVector = {targetWorldPosition.x - currentRelativePosition.x,
                               targetWorldPosition.y - currentRelativePosition.y,
                               targetWorldPosition.z - currentRelativePosition.z};

    // 3. Transform this new world-frame vector into the UE's current local frame.
    Position targetVectorInLocalFrame = rotateVectorByQuaternion(newWorldVector,
                                                                 conjugate(currentImu.quaternion));

    // 4. Convert the final local-frame vector into azimuth and altitude angles.
    double finalTargetAz, finalTargetAlt;
    vectorToAngles(targetVectorInLocalFrame, finalTargetAz, finalTargetAlt);

    // 5. Update the member variables.
    IMUTargetAz = finalTargetAz;
    IMUTargetAlt = finalTargetAlt;

    // --- Normalize/Clamp Target Angles ---
    IMUTargetAz = fmod(IMUTargetAz, 360.0);
    if (IMUTargetAz < 0)
    {
        IMUTargetAz += 360.0;
    }
    IMUTargetAlt = std::max(-90.0, std::min(90.0, IMUTargetAlt));
}

// Generate Local Spiral Offsets
void JustInTime_UE::generateLocalSpiralOffsets()
{
    localSpiralOffsets.clear();
    if (localSpiralMaxPoints <= 0 || !enableLocalSpiral || localSpiralBeamWidthDeg <= 0 || localSpiralStepScale <= 0)
        return;
    double initialDiagonalStep = localSpiralBeamWidthDeg * localSpiralStepScale;
    if (initialDiagonalStep <= 0.001f)
        initialDiagonalStep = 0.1f;
    double currentStep = initialDiagonalStep;
    int segmentsPerSide = 1;
    int totalPointsGenerated = 0;
    const double invSqrt2 = static_cast<double>(M_SQRT1_2);
    double currentOffsetAz = 0.0f, currentOffsetAlt = 0.0f;
    while (totalPointsGenerated < localSpiralMaxPoints)
    {
        double stepComponent = currentStep * invSqrt2;
        for (int i = 0; i < segmentsPerSide && totalPointsGenerated < localSpiralMaxPoints; ++i)
        {
            currentOffsetAz += stepComponent;
            currentOffsetAlt += stepComponent;
            localSpiralOffsets.push_back({currentOffsetAz, currentOffsetAlt});
            totalPointsGenerated++;
        }
        if (totalPointsGenerated >= localSpiralMaxPoints)
            break;
        for (int i = 0; i < segmentsPerSide && totalPointsGenerated < localSpiralMaxPoints; ++i)
        {
            currentOffsetAz -= stepComponent;
            currentOffsetAlt += stepComponent;
            localSpiralOffsets.push_back({currentOffsetAz, currentOffsetAlt});
            totalPointsGenerated++;
        }
        if (totalPointsGenerated >= localSpiralMaxPoints)
            break;
        segmentsPerSide++;
        for (int i = 0; i < segmentsPerSide && totalPointsGenerated < localSpiralMaxPoints; ++i)
        {
            currentOffsetAz -= stepComponent;
            currentOffsetAlt -= stepComponent;
            localSpiralOffsets.push_back({currentOffsetAz, currentOffsetAlt});
            totalPointsGenerated++;
        }
        if (totalPointsGenerated >= localSpiralMaxPoints)
            break;
        for (int i = 0; i < segmentsPerSide && totalPointsGenerated < localSpiralMaxPoints; ++i)
        {
            currentOffsetAz += stepComponent;
            currentOffsetAlt -= stepComponent;
            localSpiralOffsets.push_back({currentOffsetAz, currentOffsetAlt});
            totalPointsGenerated++;
        }
        if (totalPointsGenerated >= localSpiralMaxPoints)
            break;
        segmentsPerSide++;
    }
}

// Start Local Spiral Search
void JustInTime_UE::startAlignmentSpiral(mobileTHzEngine *engine,
                                         size_t unitIndex,
                                         size_t currentSlot)
{
    if (!enableLocalSpiral || localSpiralOffsets.empty())
    {
        isLocalSpiraling = false;
        setState(AlgoState::MONITORING, AlgoAction::NONE);
        return;
    }
    currentLocalSpiralIndex = 0;
    isLocalSpiraling = true;
    m_isPeakSeeking = false;
    m_peakSeekingBestPower = -9999999;
    m_peakSeekingBestIndex = -1;
    if (debug)
        std::cout << "[" << algorithmName << "] (" << currentSlot
                  << ") Power dip. Starting dynamic spiral around Target (Az:" << IMUTargetAz
                  << ", Alt:" << IMUTargetAlt << ")" << std::endl;
    try
    {
        rotaryObject currentRotaryState = engine->getRotaryData(unitIndex, currentSlot);
        if (currentRotaryState.isMoving == 0)
            issueLocalSpiralMoveCommand(engine, unitIndex, currentSlot, currentRotaryState);
        else
        {
            if (debug)
                std::cerr << "[" << algorithmName
                          << "] Warning: Tried to start spiral while rotary moving." << std::endl;
            isLocalSpiraling = false;
            setState(AlgoState::MONITORING, AlgoAction::MOVING);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[" << algorithmName << "] Error getting rotary data for spiral: " << e.what()
                  << std::endl;
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        isLocalSpiraling = false;
    }
}

// Issue Local Spiral Move Command
void JustInTime_UE::issueLocalSpiralMoveCommand(mobileTHzEngine *engine,
                                                size_t unitIndex,
                                                size_t currentSlot,
                                                const rotaryObject &currentRotaryState)
{
    if (!engine || !isLocalSpiraling || currentLocalSpiralIndex < 0 || currentLocalSpiralIndex >= static_cast<int>(localSpiralOffsets.size()))
    { // explicit cast for comparison
        if (debug)
            std::cerr << "[" << algorithmName << "] Error: Invalid state for dynamic spiral move."
                      << std::endl;
        isLocalSpiraling = false;
        setState(AlgoState::MONITORING, AlgoAction::NONE);
        return;
    }
    const auto &targetOffset = localSpiralOffsets[currentLocalSpiralIndex];
    double absoluteTargetAz = fmod(IMUTargetAz + targetOffset.deltaAz, 360.0);
    if (absoluteTargetAz < 0)
        absoluteTargetAz += 360.0;
    double absoluteTargetAlt = std::max(0.0, std::min(90.0, IMUTargetAlt + targetOffset.deltaAlt));
    double relativeAzCmd = shortestAngleDiff(absoluteTargetAz, currentRotaryState.azimuth.angle);
    double relativeAltCmd = absoluteTargetAlt - currentRotaryState.altitude.angle;
    std::stringstream ss_az, ss_alt;
    ss_az << std::fixed << std::setprecision(3) << relativeAzCmd;
    ss_alt << std::fixed << std::setprecision(3) << relativeAltCmd;
    std::string moveCommand = "mdd 0 " + ss_az.str() + " 1 " + ss_alt.str();
    try
    {
        engine->issueRotaryCommand(unitIndex, moveCommand);
        // if (debug)
        //     std::cout << "[" << algorithmName << "] (" << currentSlot << ", Spiral Step "
        //               << currentLocalSpiralIndex << ") Issued cmd: '" << moveCommand
        //               << "' -> Target Abs: (" << absoluteTargetAz << ", " << absoluteTargetAlt
        //               << ")" << std::endl;
        setState(AlgoState::ALIGNMENT, AlgoAction::STARTING);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[" << algorithmName << "] Error issuing spiral cmd '" << moveCommand
                  << "': " << e.what() << std::endl;
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        isLocalSpiraling = false;
    }
}

// Calculate Dynamic Success Margin
double JustInTime_UE::calculateDynamicSuccessMargin(size_t currentSlot) const
{
    double margin = realignSuccessMaxMarginDb;
    if (lastSuccessfulPointSlot > 0 && currentSlot > lastSuccessfulPointSlot && slotTimeMicrosec > 0)
    {
        double timeElapsedSeconds = static_cast<double>(currentSlot - lastSuccessfulPointSlot) * slotTimeMicrosec / 1000000.0;
        margin = realignSuccessInitialMarginDb + (static_cast<double>(timeElapsedSeconds) * realignSuccessMarginIncreaseRateDbPerSec);
        margin = std::max(realignSuccessInitialMarginDb,
                          std::min(realignSuccessMaxMarginDb, margin));
    }
    else if (lastSuccessfulPointSlot == 0 && currentState == AlgoState::MONITORING)
    {
        margin = realignSuccessInitialMarginDb;
    }
    // Ensure margin is not negative (can happen with very small configured values)
    if (margin < 0)
    {
        margin = 0.0;
    }

    return margin; // Return the calculated margin
}

// Check for and Send JIT Packet based on IMU Acceleration
void JustInTime_UE::checkForAndSendJITPacket(mobileTHzEngine *engine,
                                             size_t unitIndex,
                                             size_t currentSlot,
                                             const imuObject &currentImu,
                                             bool translationTrigger,
                                             bool rotationTrigger)
{
    if (!imuEnabledRuntime)
    {
        return;
    }

    bool cooldownPassed = (currentSlot >= lastJITPacketSentSlot_ + panicPacketCooldownSlotsConverted_) || (lastJITPacketSentSlot_ == 0 && panicPacketCooldownSlotsConverted_ == 0);

    if ((translationTrigger || rotationTrigger) && cooldownPassed && (currentState != AlgoState::ERROR_STATUS))
    {
        double imuSamplingDelay = engine->getEngineUnit(unitIndex).IMU_samplingDelay_microsec;

        // --- Calculate Translational Delta ---
        Position delta_since_last_tx_m;
        delta_since_last_tx_m.x = currentRelativePosition.x - last_transmitted_relative_position_m.x;
        delta_since_last_tx_m.y = currentRelativePosition.y - last_transmitted_relative_position_m.y;
        delta_since_last_tx_m.z = currentRelativePosition.z - last_transmitted_relative_position_m.z;

        // --- Calculate Rotational Delta ---
        Quaternion delta_rotation_integrated = multiplyQuaternions(currentImu.quaternion, conjugate(last_transmitted_orientation_));

        // Prediction
        auto predicted_motion = calculatePredictedMotion(imuSamples_,
                                                         imuSamplingDelay,
                                                         1 * panicPacketCooldownMicroseconds_,
                                                         m_imuWeightDecayConstant,
                                                         baselineAcceleration,
                                                         currentVelocity,
                                                         algorithmName);

        Position predicted_displacement = predicted_motion.first;
        Quaternion predicted_rotation = predicted_motion.second;

        JITPacket.delta_position = {delta_since_last_tx_m.x + predicted_displacement.x,
                                    delta_since_last_tx_m.y + predicted_displacement.y,
                                    delta_since_last_tx_m.z + predicted_displacement.z};

        // Final rotation is the product of the integrated rotation and the predicted rotation
        JITPacket.delta_rotation = multiplyQuaternions(predicted_rotation,
                                                       delta_rotation_integrated);

        JITPacket.unit_index = static_cast<double>(unitIndex);
        JITPacket.id = static_cast<double>(nextJITPacketId_++);
        JITPacket.transmit_start_slot = static_cast<double>(currentSlot);
        JITPacket.receive_end_slot = JITPacket.transmit_start_slot + engine->JIT_packet_reception_delay_slots;
        JITPacket.imu_data = imuSamples_.toVector(); // pass last 204 VALID IMU samples

        lastJITPacketSentSlot_ = currentSlot;

        // Update last_transmitted_relative_position_m after creating the packet
        last_transmitted_relative_position_m = currentRelativePosition;
        last_transmitted_orientation_ = currentImu.quaternion;

        if (debug)
        {
            std::string reason = translationTrigger ? (rotationTrigger ? "Trans+Rot" : "Trans")
                                                    : "Rot";
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2);
            ss << "[" << algorithmName << "] (" << currentSlot << ") PANIC ID "
               << static_cast<int>(JITPacket.id) << ". Trig: " << reason << " dPos:("
               << std::setprecision(3) << JITPacket.delta_position.x << ","
               << JITPacket.delta_position.y << "," << JITPacket.delta_position.z << ") [pred:("
               << predicted_displacement.x << "," << predicted_displacement.y << ","
               << predicted_displacement.z << ")]";
            std::cout << ss.str() << std::endl;
        }

        try
        {
            engine->addPacketToLayer(JITPacket);
        }
        catch (const std::exception &e)
        {
            std::cerr << "[" << algorithmName
                      << "] Error adding panic packet to layer: " << e.what()
                      << std::endl; // More specific error
            // Do not call setState here, let processSlot handle overall state management if error occurs higher up
        }
    }
}

// --- Main Process Slot Logic ---
void JustInTime_UE::processSlot(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot)
{
    if (!engine || !configured)
    {
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        if (engine)
            try
            {
                engine->setAlgorithmData(unitIndex, currentSlot, getStatus());
            }
            catch (...)
            {
            }
        return;
    }
    if (!imuEnabledRuntime && currentState != AlgoState::ERROR_STATUS)
    {
        std::cerr
            << "[" << algorithmName
            << "] Error: IMU required by this algorithm variant but is disabled in unit config."
            << std::endl;
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        try
        {
            engine->setAlgorithmData(unitIndex, currentSlot, getStatus());
        }
        catch (...)
        {
        }
        return;
    }

    double rawPower = -9999999;
    size_t currentRxSampleSlot = std::numeric_limits<size_t>::max();
    bool rxSourceEnabled = false;
    imuObject currentIMU;
    size_t currentImuSampleSlot_local = std::numeric_limits<size_t>::max();
    bool imuDataValidRaw_local = false;
    rotaryObject currentRotaryState;
    bool rotaryEnabled = false;
    bool isCurrentlyMoving = false;
    double currentTimeSec = static_cast<double>(currentSlot) * slotTimeMicrosec / 1000000.0;

    try
    {
        const auto &unitConf = engine->getEngineUnit(unitIndex);
        rotaryEnabled = unitConf.Rotary_enabled;
        if (unitConf.RxChain_enabled)
        {
            const auto &rxChainData = engine->getRxChainData(unitIndex, currentSlot);
            rawPower = wattsToDbm(rxChainData.power_watts);
            currentRxSampleSlot = rxChainData.sample_slot;
            rxSourceEnabled = true;
        }
        else
        {
            if (currentState != AlgoState::ERROR_STATUS)
                std::cerr << "[" << algorithmName << "] Error: RxChain not enabled." << std::endl;
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            try
            {
                engine->setAlgorithmData(unitIndex, currentSlot, getStatus());
            }
            catch (...)
            {
            }
            return;
        }
        if (unitConf.IMU_enabled)
        {
            currentIMU = engine->getIMUData(unitIndex, currentSlot);
            currentImuSampleSlot_local = currentSlot;
            imuDataValidRaw_local = std::isfinite(currentIMU.acceleration.x) && std::isfinite(currentIMU.acceleration.y) && std::isfinite(currentIMU.acceleration.z);
        }
        else
        {
            imuDataValidRaw_local = false;
        }
        if (rotaryEnabled)
        {
            currentRotaryState = engine->getRotaryData(unitIndex, currentSlot);
            isCurrentlyMoving = ((currentRotaryState.isMoving == 1) ? true : false);
        }
    }
    catch (const std::exception &e)
    {
        if (currentState != AlgoState::ERROR_STATUS)
            std::cerr << "[" << algorithmName << "] Error getting engine data: " << e.what()
                      << std::endl;
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        try
        {
            engine->setAlgorithmData(unitIndex, currentSlot, getStatus());
        }
        catch (...)
        {
        }
        return;
    }

    bool isNewRxData = (currentRxSampleSlot != lastProcessedRxSampleSlot && currentRxSampleSlot != std::numeric_limits<size_t>::max());
    bool readingIsValid = false;
    if (isNewRxData && rxSourceEnabled)
    {
        lastProcessedRxSampleSlot = currentRxSampleSlot;
        readingIsValid = updatePowerFilter(rawPower);
    }
    else
    {
        readingIsValid = false;
    }

    bool isNewImuData = (currentImuSampleSlot_local != lastProcessedImuSampleSlot && currentImuSampleSlot_local != std::numeric_limits<size_t>::max());
    bool imuDataUsableThisSlot = false;
    if (isNewImuData)
    {
        lastProcessedImuSampleSlot = currentImuSampleSlot_local;
        imuDataUsableThisSlot = imuDataValidRaw_local;

        if (imuDataUsableThisSlot)
        {
            imuSamples_.push(currentIMU);

            // Update additive IMU motion accumulator if baseline is established
            if (baselineEstablished)
            {
                // Correctly calculate motion in the world frame for the accumulator
                Position world_accel = rotateVectorByQuaternion(currentIMU.acceleration,
                                                                conjugate(currentIMU.quaternion));
                Position motionAcceleration = {world_accel.x - baselineAcceleration.x,
                                               world_accel.y - baselineAcceleration.y,
                                               world_accel.z - baselineAcceleration.z};

                if (std::abs(motionAcceleration.x) > imuSignificantMotionThreshold_)
                {
                    imuAccumulatedMotion_.x += motionAcceleration.x;
                }
                if (std::abs(motionAcceleration.y) > imuSignificantMotionThreshold_)
                {
                    imuAccumulatedMotion_.y += motionAcceleration.y;
                }
                if (std::abs(motionAcceleration.z) > imuSignificantMotionThreshold_)
                {
                    imuAccumulatedMotion_.z += motionAcceleration.z;
                }
            }
        }
    }
    else
    {
        imuDataUsableThisSlot = false;
    }

    if (imuDataUsableThisSlot)
    { // This check ensures currentIMU_data_for_slot is valid for kinematics
        updateKinematics(currentIMU, currentTimeSec);
        // calculateTargetAngles should only be called if baseline is established, as it uses initialDistance etc.
        if (baselineEstablished)
        {
            calculateTargetAngles(currentIMU);
        }
    }

    // --- Check for JIT Packet ---
    // --- Check for Translational Trigger ---
    double accumulatedTranslationMag = std::sqrt(imuAccumulatedMotion_.x * imuAccumulatedMotion_.x + imuAccumulatedMotion_.y * imuAccumulatedMotion_.y + imuAccumulatedMotion_.z * imuAccumulatedMotion_.z);
    bool translationTrigger = (accumulatedTranslationMag > imuPanicAccumulatedMagnitudeThreshold_);

    // --- Check for Rotational Trigger ---
    // Calculate the dot product between the current orientation and the last transmitted one.
    const auto &q_current = currentIMU.quaternion;
    double dotProduct_panicPacket = q_current.w * last_transmitted_orientation_.w + q_current.x * last_transmitted_orientation_.x + q_current.y * last_transmitted_orientation_.y + q_current.z * last_transmitted_orientation_.z;
    double dotProduct_imuTracking = q_current.w * last_tracked_orientation_.w + q_current.x * last_tracked_orientation_.x + q_current.y * last_tracked_orientation_.y + q_current.z * last_tracked_orientation_.z;

    // Clamp dot product to handle potential floating point errors outside [-1, 1]
    dotProduct_panicPacket = std::max(-1.0, std::min(1.0, dotProduct_panicPacket));
    dotProduct_imuTracking = std::max(-1.0, std::min(1.0, dotProduct_imuTracking));

    // Calculate the angular difference in degrees. Angle = 2 * acos(|dot|)
    // We use abs(dot) to get the shortest angle, which is what we care about.
    double angleDegJITPacket = (2.0 * std::acos(std::abs(dotProduct_panicPacket))) * RAD_TO_DEG;
    double angleDeg = (2.0 * std::acos(std::abs(dotProduct_imuTracking))) * RAD_TO_DEG;
    bool rotationJITPacketTrigger = (angleDegJITPacket > panicPacketRotationThresholdDeg);
    bool rotationIMUTrackingTigger = (angleDeg > panicPacketRotationThresholdDeg);

    checkForAndSendJITPacket(engine,
                             unitIndex,
                             currentSlot,
                             currentIMU,
                             translationTrigger,
                             rotationJITPacketTrigger);

    switch (currentState)
    {
    case AlgoState::REFERENCE:
    {
        if (readingIsValid && filterInitialized)
            warmupCounter++;
        else if (!filterInitialized && isNewRxData && rxSourceEnabled && std::isfinite(rawPower) && rawPower > -99.9f)
        { // Ensure rawPower is somewhat sensible
            updatePowerFilter(rawPower);
            if (filterInitialized)
                warmupCounter++;
        }

        // For baseline, we need valid IMU data for this specific slot, not just general usability
        bool currentSlotImuValidForBaseline = imuDataValidRaw_local && (currentImuSampleSlot_local == currentSlot);

        bool rotaryReady = rotaryEnabled;

        if (warmupCounter >= referenceWarmupSamples && filterInitialized && currentSlotImuValidForBaseline && rotaryReady)
        {
            currentReferencePower = longTermEMA;
            if (!std::isfinite(currentReferencePower))
            {
                if (debug)
                    std::cerr << "[" << algorithmName
                              << "] Error: Invalid LongTermEMA for baseline. Retrying."
                              << std::endl;
                warmupCounter = 0;
                filterInitialized = false; // Reset filter to re-initialize EMA
                break;
            }
            lastSuccessfulPointSlot = currentSlot;
            double newInitialDistance = estimateDistanceFromPower(currentReferencePower);

            // Chek imu approx distance
            double imuApproxDisplacement = std::sqrt(
                currentRelativePosition.x * currentRelativePosition.x + currentRelativePosition.y * currentRelativePosition.y + currentRelativePosition.z * currentRelativePosition.z);

            // if|new distance - current initialDistance| > 2* imuApproxDisplacement then new distance is probably wrong
            if (imuApproxDisplacement > 0 && std::abs(newInitialDistance - initialDistance) > 2 * imuApproxDisplacement)
            {
                if (debug)
                    std::cerr << "[" << algorithmName << "] Warning: New distance estimate ("
                              << newInitialDistance << "m) >> initial distance (" << initialDistance
                              << "m). IMU Displacement: " << imuApproxDisplacement << "m. "
                              << std::endl;
            }
            else
            {
                initialDistance = newInitialDistance; // Update to new estimate
            }

            if (initialDistance <= 0)
            {
                if (debug)
                    std::cerr << "[" << algorithmName
                              << "] Error: Failed to estimate initial distance (" << initialDistance
                              << " from power " << currentReferencePower
                              << " dBm). Cannot baseline." << std::endl;
                setState(AlgoState::ERROR_STATUS,
                         AlgoAction::ERROR_ACTION);
                warmupCounter = 0;
                filterInitialized = false;
                break;
            }

            // Using currentIMU data to establish a world gravity vector
            // This corrects the assumption that the device starts in a specific orientation.
            initialOrientation = currentIMU.quaternion;
            // The measured acceleration is gravity in the local frame. Rotate it to the world frame.
            baselineAcceleration = rotateVectorByQuaternion(currentIMU.acceleration,
                                                            conjugate(initialOrientation));
            initialEulerAngles = toEulerAngles(initialOrientation);

            try
            {
                rotaryObject currentRotary = engine->getRotaryData(unitIndex, currentSlot);
                initialIMUTargetAz = currentRotary.azimuth.angle;
                initialIMUTargetAlt = currentRotary.altitude.angle;
                initialPointingVectorLocal_ = anglesToVector(initialIMUTargetAz, initialIMUTargetAlt, initialDistance);
            }
            catch (const std::exception &e)
            {
                std::cerr << "[" << algorithmName
                          << "] Error getting rotary data for baseline: " << e.what() << std::endl;
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                break;
            }
            IMUTargetAz = initialIMUTargetAz;
            IMUTargetAlt = initialIMUTargetAlt;
            currentRelativePosition = {0, 0, 0};

            lastImuSampleTimeSec = currentTimeSec; // Initialize for delta calculations

            baselineEstablished = true;
            isLocalSpiraling = false;
            currentLocalSpiralIndex = -1;

            if (debug)
                std::cout << "[" << algorithmName << "] Baseline Established (" << currentSlot
                          << ") RefPwr=" << currentReferencePower << "dBm, Dist=" << initialDistance
                          << "m, Az/Alt=" << initialIMUTargetAz << "/" << initialIMUTargetAlt
                          << " Baseline Accel: (" << baselineAcceleration.x << ","
                          << baselineAcceleration.y << "," << baselineAcceleration.z << ")"
                          << std::endl;
            setState(AlgoState::MONITORING, AlgoAction::NONE);
        }
        else if (filterInitialized && !currentSlotImuValidForBaseline && warmupCounter >= referenceWarmupSamples)
        {
            // Filter is warm, but IMU isn't ready for baseline this slot. Wait.
            if (debug && warmupCounter % 100 == 0)
            { // Log occasionally
                std::cout << "[" << algorithmName
                          << "] Reference: Filter warm, waiting for valid IMU for baseline."
                          << std::endl;
            }
        }
    }
    break;

    case AlgoState::MONITORING:
    {
        if (!baselineEstablished || !filterInitialized)
        {
            if (debug && currentState != AlgoState::REFERENCE) // Avoid spam if already trying to go to REFERENCE
                std::cout
                    << "[" << algorithmName
                    << "] Warning: Monitoring requires baseline/filter. Re-entering REFERENCE."
                    << std::endl;
            setState(AlgoState::REFERENCE, AlgoAction::NONE);
            // Reset relevant states for re-baselining
            filterInitialized = false;
            warmupCounter = 0;
            baselineEstablished = false;

            break;
        }
        if (isCurrentlyMoving)
            setState(AlgoState::MONITORING, AlgoAction::MOVING);
        else if (currentAction == AlgoAction::MOVING || currentAction == AlgoAction::STARTING)
            setState(AlgoState::MONITORING, AlgoAction::NONE);

        bool powerIsGood = readingIsValid && std::isfinite(currentReferencePower) && std::isfinite(shortTermEMA) && (shortTermEMA >= (currentReferencePower - localSpiralTriggerThresholdDb));
        if (powerIsGood)
            lastSuccessfulPointSlot = currentSlot;

        bool shouldTriggerSpiral = false;
        if (enableLocalSpiral && rotaryEnabled && !isCurrentlyMoving && currentAction == AlgoAction::NONE && std::isfinite(currentReferencePower) && readingIsValid && std::isfinite(shortTermEMA))
        {
            if (shortTermEMA < (currentReferencePower - localSpiralTriggerThresholdDb))
                shouldTriggerSpiral = true;
        }

        if (rotaryEnabled && !isCurrentlyMoving && currentAction == AlgoAction::NONE && (rotationIMUTrackingTigger || translationTrigger))
        {
            last_tracked_orientation_ = currentIMU.quaternion;

            // 1. Calculate the predicted motion delta from historical IMU data
            double imuSamplingDelay = engine->getEngineUnit(unitIndex).IMU_samplingDelay_microsec;
            auto predicted_motion_delta = calculatePredictedMotion(imuSamples_,
                                                                   imuSamplingDelay,
                                                                   200000,
                                                                   m_imuWeightDecayConstant,
                                                                   baselineAcceleration,
                                                                   currentVelocity,
                                                                   algorithmName);

            Position predicted_displacement = predicted_motion_delta.first;
            Quaternion predicted_rotation_delta = predicted_motion_delta.second;

            // 2. Determine the UE's predicted future state (position and orientation)
            Position predicted_world_pos = {currentRelativePosition.x + predicted_displacement.x,
                                            currentRelativePosition.y + predicted_displacement.y,
                                            currentRelativePosition.z + predicted_displacement.z};

            Quaternion predicted_orientation = multiplyQuaternions(predicted_rotation_delta,
                                                                   currentIMU.quaternion);

            // 3. Determine the AP's static world position (same logic as in calculateTargetAngles)
            Position initialForwardVector = {initialDistance, 0.0, 0.0};
            Position targetWorldPosition = rotateVectorByQuaternion(initialPointingVectorLocal_, conjugate(initialOrientation));

            // 4. Calculate the required pointing vector in the WORLD frame (from future UE to static AP)
            Position future_world_vector_to_ap = {targetWorldPosition.x - predicted_world_pos.x,
                                                  targetWorldPosition.y - predicted_world_pos.y,
                                                  targetWorldPosition.z - predicted_world_pos.z};

            // 5. Transform this world-frame vector into the UE's PREDICTED LOCAL frame.
            //    This gives the vector the antenna needs to point along, relative to the future device orientation.
            Position future_target_vector_local = rotateVectorByQuaternion(
                future_world_vector_to_ap,
                conjugate(predicted_orientation));

            // 6. Convert the final local-frame vector into azimuth and altitude angles.
            double predictedTargetAz, predictedTargetAlt;
            vectorToAngles(future_target_vector_local, predictedTargetAz, predictedTargetAlt);

            // 7. Normalize and Clamp the final angles
            predictedTargetAz = fmod(predictedTargetAz, 360.0);
            if (predictedTargetAz < 0)
            {
                predictedTargetAz += 360.0;
            }
            predictedTargetAlt = std::max(-90.0, std::min(90.0, predictedTargetAlt)) * -1;

            // 8. Issue the move command with the correctly predicted target angles
            issueIMUTrackMoveCommand(engine,
                                     unitIndex,
                                     currentSlot,
                                     currentRotaryState,
                                     predictedTargetAz,
                                     predictedTargetAlt);
        }
        else if (shouldTriggerSpiral)
        {
            startAlignmentSpiral(engine, unitIndex, currentSlot);
        }
    }
    break;

    case AlgoState::ALIGNMENT:
    {
        if (!baselineEstablished || !filterInitialized || !std::isfinite(currentReferencePower))
        {
            if (debug)
                std::cerr << "[" << algorithmName << "] ALIGNMENT: Missing baseline/filter. Error."
                          << std::endl;
            if (currentState != AlgoState::ERROR_STATUS)
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            isLocalSpiraling = false; // Ensure spiral stops
            m_isPeakSeeking = false;
            break;
        }
        if (!rotaryEnabled || localSpiralOffsets.empty() || currentLocalSpiralIndex < 0)
        {
            if (debug)
                std::cerr << "[" << algorithmName
                          << "] ALIGNMENT: Rotary/spiral config issue. Error." << std::endl;
            if (currentState != AlgoState::ERROR_STATUS)
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            isLocalSpiraling = false;
            m_isPeakSeeking = false;
            break;
        }

        if (isCurrentlyMoving)
        {
            if (currentAction == AlgoAction::STARTING || currentAction == AlgoAction::NONE) // If it started moving
                setState(AlgoState::ALIGNMENT, AlgoAction::MOVING);
        }
        else
        {                                                                                     // Not moving
            if (currentAction == AlgoAction::MOVING || currentAction == AlgoAction::STARTING) // If it just stopped
                setState(AlgoState::ALIGNMENT, AlgoAction::NONE);
        }

        bool triggerHalt = false;

        if (readingIsValid && std::isfinite(shortTermEMA) && filterInitialized)
        {
            double currentSuccessMargin = calculateDynamicSuccessMargin(currentSlot);
            bool baseSuccessConditionMet = (shortTermEMA >= (currentReferencePower - currentSuccessMargin));

            if (baseSuccessConditionMet && !m_isPeakSeeking)
            {
                if (debug)
                    std::cout << "[" << algorithmName << "] (" << currentSlot
                              << ") Alignment threshold met. Peak Seeking. Power: " << shortTermEMA
                              << " Ref: " << currentReferencePower
                              << " Margin: " << currentSuccessMargin << std::endl;
                m_isPeakSeeking = true;
                m_peakSeekingBestPower = shortTermEMA;
                m_peakSeekingBestIndex = currentLocalSpiralIndex;
            }
            if (m_isPeakSeeking)
            {
                if (shortTermEMA > m_peakSeekingBestPower)
                {
                    if (debug && (shortTermEMA - m_peakSeekingBestPower > 0.05))
                        std::cout << "[" << algorithmName << "] (" << currentSlot
                                  << ") Peak Seeking: Power increased to " << shortTermEMA
                                  << std::endl;
                    m_peakSeekingBestPower = shortTermEMA;
                    m_peakSeekingBestIndex = currentLocalSpiralIndex;
                }
                else if (shortTermEMA < (m_peakSeekingBestPower - peakSeekingDecreaseThresholdDb))
                {
                    if (debug)
                        std::cout
                            << "[" << algorithmName << "] (" << currentSlot
                            << ") Peak Seeking: Power decreased below threshold. Triggering HALT."
                            << " Current: " << shortTermEMA << " Best: " << m_peakSeekingBestPower
                            << std::endl;
                    triggerHalt = true;
                }
            }
        }

        if (!isCurrentlyMoving && currentAction == AlgoAction::STOPPING)
        {
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Halt completed. Best Power: " << m_peakSeekingBestPower
                          << " at spiral index " << m_peakSeekingBestIndex << ". Re-baselining."
                          << std::endl;
            try
            {
                setState(AlgoState::REFERENCE, AlgoAction::NONE);
                filterInitialized = false; // Re-establish reference at the peak
                warmupCounter = 0;
                m_isPeakSeeking = false; // Reset flag
                // Break here as we are transitioning state
            }
            catch (const std::exception &e)
            {
                std::cerr << "[" << algorithmName << "] (" << currentSlot
                          << ") Error re-baselining post-spiral: " << e.what() << std::endl;
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            }
            isLocalSpiraling = false;
            m_isPeakSeeking = false;
            break;
        }

        if (triggerHalt && currentAction != AlgoAction::STOPPING)
        {
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Triggering HALT command." << std::endl;
            try
            {
                engine->issueRotaryCommand(unitIndex, "halt 0");
                engine->issueRotaryCommand(unitIndex, "halt 1");
                setState(AlgoState::ALIGNMENT, AlgoAction::STOPPING);
            }
            catch (const std::exception &e)
            {
                if (currentState != AlgoState::ERROR_STATUS)
                { // Avoid multiple error reports
                    std::cerr << "[" << algorithmName << "] Error issuing halt: " << e.what()
                              << std::endl;
                    setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                    isLocalSpiraling = false;
                    m_isPeakSeeking = false;
                }
            }
            break;
        }

        // If not moving, not already stopping, and not triggering halt yet, move to next point
        if (!isCurrentlyMoving && currentAction == AlgoAction::NONE && !triggerHalt)
        {
            int nextSpiralPointIndex = currentLocalSpiralIndex + 1;
            if (nextSpiralPointIndex >= static_cast<int>(localSpiralOffsets.size()) || nextSpiralPointIndex >= localSpiralMaxPoints)
            {
                if (m_isPeakSeeking)
                { // If at end of spiral & peak seeking, halt to lock in best
                    if (debug)
                        std::cout << "[" << algorithmName << "] (" << currentSlot
                                  << ") End of spiral while peak seeking. Triggering HALT."
                                  << std::endl;
                    triggerHalt = true; // This will be caught in the next iteration or by the halt block above
                    // For immediate action:
                    if (currentAction != AlgoAction::STOPPING)
                    { // Check to prevent re-issuing halt
                        try
                        {
                            engine->issueRotaryCommand(unitIndex, "halt 0");
                            engine->issueRotaryCommand(unitIndex, "halt 1");
                            setState(AlgoState::ALIGNMENT, AlgoAction::STOPPING);
                        }
                        catch (const std::exception &e)
                        { /* error handling as above */
                        }
                    }
                }
                else
                { // End of spiral, not peak seeking (e.g. power never met threshold) -> spiral failed
                    if (currentState != AlgoState::ERROR_STATUS)
                    {
                        if (debug)
                            std::cerr << "[" << algorithmName << "] (" << currentSlot
                                      << ") Error: Spiral FAILED (end of points, no peak found)."
                                      << std::endl;
                        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                        isLocalSpiraling = false;
                        m_isPeakSeeking = false;
                    }
                }
                break; // Exit switch case for this slot
            }

            // If halt was triggered by end of spiral logic above, and we are not already stopping
            if (triggerHalt && currentAction != AlgoAction::STOPPING)
            {
                // This block might be redundant if the main triggerHalt block handles it
            }
            else if (!triggerHalt && currentState != AlgoState::ERROR_STATUS)
            { // Proceed to next point
                currentLocalSpiralIndex = nextSpiralPointIndex;
                if (debug)
                    std::cout << "[" << algorithmName << "] (" << currentSlot
                              << ") Moving to next spiral idx: " << currentLocalSpiralIndex
                              << std::endl;
                try
                {
                    rotaryObject currentRotary = engine->getRotaryData(unitIndex, currentSlot);
                    issueLocalSpiralMoveCommand(engine, unitIndex, currentSlot, currentRotary);
                }
                catch (const std::exception &e)
                {
                    std::cerr << "[" << algorithmName
                              << "] Error getting rotary for spiral move: " << e.what()
                              << std::endl;
                    setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                    isLocalSpiraling = false;
                    m_isPeakSeeking = false;
                }
            }
            break; // Processed this non-moving state
        }
    }
    break;

    case AlgoState::ERROR_STATUS:
        if (rotaryEnabled && isCurrentlyMoving && currentAction != AlgoAction::STOPPING)
        {
            try
            {
                engine->issueRotaryCommand(unitIndex, "halt 0");
                engine->issueRotaryCommand(unitIndex, "halt 1");
                setState(AlgoState::ERROR_STATUS,
                         AlgoAction::STOPPING); // Indicate attempting to stop
            }
            catch (const std::exception &e)
            {
                std::cerr << "[" << algorithmName << "] Error HALT in ERROR state: " << e.what()
                          << std::endl;
                // Already in ERROR_STATUS, action might remain ERROR_ACTION or STOPPING
            }
        }
        else if (!isCurrentlyMoving && currentAction == AlgoAction::STOPPING)
        {
            setState(AlgoState::ERROR_STATUS,
                     AlgoAction::ERROR_ACTION); // Stopped, now truly in error
        }
        // If not moving and not trying to stop, it's already in a stable error state.
        // No specific action needed other than ensuring currentAction is ERROR_ACTION.
        if (currentAction != AlgoAction::ERROR_ACTION && currentAction != AlgoAction::STOPPING)
        {
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        }
        break;

    default:
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        break;
    }

    try
    {
        engine->setAlgorithmData(unitIndex, currentSlot, getStatus());
    }
    catch (const std::exception &e)
    {
        if (currentState != AlgoState::ERROR_STATUS) // Avoid logging storm if already in error
            std::cerr << "[" << algorithmName << "] Error setting algorithm data: " << e.what()
                      << std::endl;
        // Potentially set to ERROR_STATUS here if not already, but often the error originates elsewhere.
    }
}

// Issue Normal IMU Tracking Move Command
void JustInTime_UE::issueIMUTrackMoveCommand(mobileTHzEngine *engine,
                                             size_t unitIndex,
                                             size_t currentSlot,
                                             const rotaryObject &currentRotaryState,
                                             double targetAz,
                                             double targetAlt)
{
    if (!engine || currentState != AlgoState::MONITORING || !imuEnabledRuntime || !baselineEstablished) // Ensure baseline is established before tracking
        return;

    // Ensure targets are valid (can become NaN if initialDistance was bad)
    if (!std::isfinite(targetAz) || !std::isfinite(targetAlt))
    {
        if (debug)
            std::cerr << "[" << algorithmName
                      << "] IMU Track: Invalid target angles (Az=" << targetAz
                      << ", Alt=" << targetAlt << "). Skipping move." << std::endl;
        return;
    }

    double deltaAzCmd = shortestAngleDiff(targetAz, currentRotaryState.azimuth.angle);
    double deltaAltCmd = targetAlt - currentRotaryState.altitude.angle;

    if (std::abs(deltaAzCmd) > minAngleChangeThresholdDeg || std::abs(deltaAltCmd) > minAngleChangeThresholdDeg)
    {
        std::stringstream ss_cmd_imu;
        // Ensure precision for small movements
        ss_cmd_imu << "mdd 0 " << std::fixed << std::setprecision(4) << deltaAzCmd << " 1 "
                   << std::fixed << std::setprecision(4) << deltaAltCmd;
        std::string moveCommand = ss_cmd_imu.str();
        try
        {
            engine->issueRotaryCommand(unitIndex, moveCommand);
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Issued Predicted IMU Track Move: '" << moveCommand
                          << "' (Target Az/Alt: " << targetAz << "/" << targetAlt << ")"
                          << std::endl;
            setState(AlgoState::MONITORING, AlgoAction::STARTING); // Indicate movement initiated
        }
        catch (const std::exception &e)
        {
            if (currentState != AlgoState::ERROR_STATUS)
            { // Avoid error storms
                std::cerr << "[" << algorithmName << "] (" << currentSlot
                          << ") Error IMU tracking move: " << e.what() << std::endl;
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            }
        }
    }
    else
    {
        // If no significant change needed, ensure state is NONE if it was STARTING/MOVING
        if (currentAction == AlgoAction::STARTING || currentAction == AlgoAction::MOVING)
        {
            setState(AlgoState::MONITORING, AlgoAction::NONE);
        }
    }
}
std::pair<Position, Quaternion> JustInTime_UE::calculatePredictedMotion(
    const CircularBuffer<imuObject> &imuSamples,
    double imuSamplingDelayMicrosec,
    double predictionHorizonMicrosec,
    double imuWeightDecayConstant,
    const Position &worldGravity,
    const Position &currentVelocityAtPredictionStart,
    const std::string &algoName) const
{

    static const bool debug_prediction = true;
    static long long debug_counter = 0;

    // Initialize the return pair with zero displacement and no rotation (identity quaternion)
    std::pair<Position, Quaternion> prediction = {{0, 0, 0}, {1, 0, 0, 0}};

    if (imuSamples.empty() || predictionHorizonMicrosec <= 0)
    {
        return prediction;
    }

    size_t numSamples = imuSamples.size();
    Position weightedAccelTrend = {0, 0, 0};
    Position weightedGyroTrendRad = {0, 0, 0};
    double totalWeight = 0;
    bool logged_this_call = false; // Flag to check if we printed anything inside the loop

    for (size_t k = 0; k < numSamples; ++k)
    {
        const imuObject &sample = imuSamples[k];
        const double weight = precomputed_imu_weights_[k];

        // It transforms the acceleration measured in the device's local frame...
        const Position local_accel = sample.acceleration;
        // ...into the fixed world frame using the device's current orientation.
        const Position world_accel = rotateVectorByQuaternion(sample.acceleration,
                                                              conjugate(sample.quaternion));

        // This subtracts the constant downward pull of gravity to find the actual motion acceleration.
        const Position motionAccel = {world_accel.x - worldGravity.x,
                                      world_accel.y - worldGravity.y,
                                      world_accel.z - worldGravity.z};

        const Position gyroRad = sample.angular_velocity;
        weightedAccelTrend.x += motionAccel.x * weight;
        weightedAccelTrend.y += motionAccel.y * weight;
        weightedAccelTrend.z += motionAccel.z * weight;
        weightedGyroTrendRad.x += gyroRad.x * weight;
        weightedGyroTrendRad.y += gyroRad.y * weight;
        weightedGyroTrendRad.z += gyroRad.z * weight;
        totalWeight += weight;
    }

    if (totalWeight <= 1e-9)
    {
        return prediction;
    }

    // Normalize both trends
    weightedAccelTrend.x /= totalWeight;
    weightedAccelTrend.y /= totalWeight;
    weightedAccelTrend.z /= totalWeight;
    weightedGyroTrendRad.x /= totalWeight;
    weightedGyroTrendRad.y /= totalWeight;
    weightedGyroTrendRad.z /= totalWeight;

    double predictionTimeSec = predictionHorizonMicrosec / 1000000.0;

    prediction.first.x = currentVelocityAtPredictionStart.x * predictionTimeSec + 0.5 * weightedAccelTrend.x * predictionTimeSec * predictionTimeSec;
    prediction.first.y = currentVelocityAtPredictionStart.y * predictionTimeSec + 0.5 * weightedAccelTrend.y * predictionTimeSec * predictionTimeSec;
    prediction.first.z = currentVelocityAtPredictionStart.z * predictionTimeSec + 0.5 * weightedAccelTrend.z * predictionTimeSec * predictionTimeSec;

    double totalAngleRad = std::sqrt(std::pow(weightedGyroTrendRad.x * predictionTimeSec, 2) + std::pow(weightedGyroTrendRad.y * predictionTimeSec, 2) + std::pow(weightedGyroTrendRad.z * predictionTimeSec, 2));

    if (totalAngleRad > 1e-6)
    {
        double halfAngle = totalAngleRad / 2.0;
        double sinHalfAngle = std::sin(halfAngle);
        prediction.second.w = std::cos(halfAngle);
        prediction.second.x = (weightedGyroTrendRad.x * predictionTimeSec / totalAngleRad) * sinHalfAngle;
        prediction.second.y = (weightedGyroTrendRad.y * predictionTimeSec / totalAngleRad) * sinHalfAngle;
        prediction.second.z = (weightedGyroTrendRad.z * predictionTimeSec / totalAngleRad) * sinHalfAngle;
    }

    return prediction;
}
