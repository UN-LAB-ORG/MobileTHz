#include "IMUAssist.h"
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

static bool debug = true;

namespace
{
    constexpr double SPEED_OF_LIGHT = 299792458.0;
    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    constexpr double LN10_OVER_10 = 0.23025850929940457;
    constexpr double TEN_OVER_LN10 = 4.342944819032518;
    constexpr double MIN_WATT_THRESHOLD = 1e-30;
    constexpr double MIN_DBM_FLOOR = -300.0;

    inline double wattsToDbm(double watts)
    {
        if (watts <= MIN_WATT_THRESHOLD)
            return MIN_DBM_FLOOR;
        return TEN_OVER_LN10 * std::log(watts) + 30.0;
    }
    inline double linearToDbi(double linearGain)
    {
        if (linearGain <= 1e-30)
            return -300.0;
        return TEN_OVER_LN10 * std::log(linearGain);
    }
    void vectorToAngles(const Position &v, double &azimuthDeg, double &altitudeDeg)
    {
        azimuthDeg = std::atan2(v.y, v.x) * RAD_TO_DEG;
        double xy_dist = std::sqrt(v.x * v.x + v.y * v.y);
        altitudeDeg = std::atan2(v.z, xy_dist) * RAD_TO_DEG;
    }
}

Quaternion IMUAssist::conjugate(const Quaternion &q) const
{
    return {q.w, -q.x, -q.y, -q.z};
}

Quaternion IMUAssist::multiplyQuaternions(const Quaternion &q1, const Quaternion &q2) const
{
    Quaternion result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

Position IMUAssist::rotateVectorByQuaternion(const Position &v, const Quaternion &q) const
{
    Quaternion v_quat = {0, v.x, v.y, v.z};
    Quaternion q_conj = conjugate(q);
    Quaternion rotated_v_quat = multiplyQuaternions(multiplyQuaternions(q, v_quat), q_conj);
    return {rotated_v_quat.x, rotated_v_quat.y, rotated_v_quat.z};
}

IMUAssist::IMUAssist()
{
    setState(AlgoState::REFERENCE, AlgoAction::NONE);
}

void IMUAssist::configure(const ConfigFile &config, const engineUnit &unitConfig)
{
    slotTimeMicrosec = config.engine_slot_time_microsec;
    algorithmName = unitConfig.algorithm + "_" + unitConfig.label;
    txPowerDbm = wattsToDbm(unitConfig.TxAntenna_transmitPower_watts);
    txGainDbi = linearToDbi(unitConfig.radiationPatterns_maxGain_linear.at(0));
    rxGainDbi = linearToDbi(unitConfig.radiationPatterns_maxGain_linear.at(0));
    frequencyHz = config.engine_carrier_frequency;
    configured = true;
    imuEnabledRuntime = unitConfig.IMU_enabled;

    // Reset runtime data
    last_tracked_orientation_ = {1.0f, 0.0f, 0.0f, 0.0f};
    filterInitialized = false;
    baselineEstablished = false;
    warmupCounter = 0;
    currentReferencePower = -9999999;
    lastProcessedRxSampleSlot = std::numeric_limits<size_t>::max();
    lastProcessedImuSampleSlot = std::numeric_limits<size_t>::max();
    m_isPeakSeeking = false;
    currentRelativePosition = {0, 0, 0};
    lastImuSampleTimeSec = -1.0;
    initialIMUTargetAz = 0.0f;
    initialIMUTargetAlt = 0.0f;
    IMUTargetAz = 0.0f;
    IMUTargetAlt = 0.0f;
    localSpiralOffsets.clear();

    // Set parameters
    localSpiralBeamWidthDeg = unitConfig.radiationPatterns_hpbw.at(0) * 0.1;
    realignSuccessMarginIncreaseRateDbPerSec = linearToDbi(
                                                   unitConfig.radiationPatterns_maxGain_linear.at(0)) *
                                               0.5;
    localSpiralTriggerThresholdDb = 3;
    realignSuccessInitialMarginDb = 1;
    imuTrackingRotationThresholdDeg = unitConfig.radiationPatterns_hpbw.at(0) * 0.1;

    imuSamples_.reconfigure_capacity(m_maxImuSamplesForPrediction);
    precomputed_imu_weights_.resize(m_maxImuSamplesForPrediction);
    for (int k = 0; k < m_maxImuSamplesForPrediction; ++k)
    {
        const double normalized_k = (m_maxImuSamplesForPrediction > 1)
                                        ? static_cast<double>(k) / (m_maxImuSamplesForPrediction - 1)
                                        : 0.0;
        precomputed_imu_weights_[k] = std::exp(m_imuWeightDecayConstant * normalized_k);
    }

    if (enableLocalSpiral)
    {
        generateLocalSpiralOffsets();
    }

    setState(AlgoState::REFERENCE, AlgoAction::NONE);
    if (!imuEnabledRuntime)
    {
        std::cerr << "[" << algorithmName
                  << "] CRITICAL WARNING: IMU is disabled, this algorithm *requires* IMU."
                  << std::endl;
    }
}

void IMUAssist::setState(AlgoState newStatus, AlgoAction newAction)
{
    currentState = newStatus;
    currentAction = newAction;
    if (isLocalSpiraling && (newStatus == AlgoState::REFERENCE || newStatus == AlgoState::ERROR_STATUS))
    {
        isLocalSpiraling = false;
        currentLocalSpiralIndex = -1;
    }
}

algorithmObject IMUAssist::getStatus() const
{
    return {static_cast<double>(currentState), static_cast<double>(currentAction)};
}

bool IMUAssist::updatePowerFilter(double rawPower)
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
    return !isSpike;
}

double IMUAssist::shortestAngleDiff(double targetDeg, double currentDeg) const
{
    double diff = targetDeg - currentDeg;
    diff = fmod(diff + 180.0, 360.0) - 180.0;
    return (diff < -180.0) ? diff + 360.0 : diff;
}

double IMUAssist::estimateDistanceFromPower(double rxPowerDbm) const
{
    if (!std::isfinite(rxPowerDbm) || frequencyHz <= 0)
        return -1.0;

    double boundedRxPowerDbm = std::max(rxPowerDbm, -45.0);

    double totalGainLinear = std::pow(10.0, (txGainDbi + rxGainDbi) / 10.0);
    double txPowerLinear = std::pow(10.0, (txPowerDbm - 30.0) / 10.0);
    double rxPowerLinear = std::pow(10.0, (boundedRxPowerDbm - 30.0) / 10.0);
    if (txPowerLinear <= 0 || rxPowerLinear <= 0 || totalGainLinear <= 0)
        return -1.0;

    double lambda_m = SPEED_OF_LIGHT / (frequencyHz);
    if (lambda_m <= 0)
        return -1.0;

    double factor = txPowerLinear * totalGainLinear * std::pow(lambda_m / (4.0 * M_PI), 2.0);
    return std::sqrt(factor / rxPowerLinear);
}

Position IMUAssist::anglesToVector(double azDeg, double altDeg, double distance) const
{
    double azRad = azDeg * DEG_TO_RAD;
    double altRad = altDeg * DEG_TO_RAD;
    Position vec;
    vec.x = distance * std::cos(altRad) * std::cos(azRad);
    vec.y = distance * std::cos(altRad) * std::sin(azRad);
    vec.z = distance * std::sin(altRad);
    return vec;
}

void IMUAssist::updateKinematics(const imuObject &currentImu, double currentTimeSec)
{
    if (!baselineEstablished || lastImuSampleTimeSec < 0 || currentTimeSec <= lastImuSampleTimeSec)
    {
        lastImuSampleTimeSec = currentTimeSec;
        return;
    }
    double deltaTime = currentTimeSec - lastImuSampleTimeSec;
    if (deltaTime <= 0 || deltaTime > 1.0)
    {
        lastImuSampleTimeSec = currentTimeSec;
        return;
    }
    Position worldAcceleration = rotateVectorByQuaternion(currentImu.acceleration, conjugate(currentImu.quaternion));
    Position motionAccelerationWorld = {worldAcceleration.x - baselineAcceleration.x,
                                        worldAcceleration.y - baselineAcceleration.y,
                                        worldAcceleration.z - baselineAcceleration.z};
    currentVelocity.x += motionAccelerationWorld.x * deltaTime;
    currentVelocity.y += motionAccelerationWorld.y * deltaTime;
    currentVelocity.z += motionAccelerationWorld.z * deltaTime;
    currentRelativePosition.x += currentVelocity.x * deltaTime;
    currentRelativePosition.y += currentVelocity.y * deltaTime;
    currentRelativePosition.z += currentVelocity.z * deltaTime;
    lastImuSampleTimeSec = currentTimeSec;
}

void IMUAssist::calculateTargetAngles(const imuObject &currentImu)
{
    if (!baselineEstablished)
        return;

    Position targetWorldPosition = rotateVectorByQuaternion(initialPointingVectorLocal_, conjugate(initialOrientation));
    Position newWorldVector = {targetWorldPosition.x - currentRelativePosition.x,
                               targetWorldPosition.y - currentRelativePosition.y,
                               targetWorldPosition.z - currentRelativePosition.z};
    Position targetVectorInLocalFrame = rotateVectorByQuaternion(newWorldVector, conjugate(currentImu.quaternion));
    double finalTargetAz, finalTargetAlt;
    vectorToAngles(targetVectorInLocalFrame, finalTargetAz, finalTargetAlt);
    IMUTargetAz = fmod(finalTargetAz, 360.0);
    if (IMUTargetAz < 0)
        IMUTargetAz += 360.0;
    IMUTargetAlt = std::max(-90.0, std::min(90.0, finalTargetAlt));
}

void IMUAssist::generateLocalSpiralOffsets()
{
    localSpiralOffsets.clear();
    if (localSpiralMaxPoints <= 0 || !enableLocalSpiral || localSpiralBeamWidthDeg <= 0)
        return;
    double initialDiagonalStep = localSpiralBeamWidthDeg * localSpiralStepScale;
    if (initialDiagonalStep <= 0.001f)
        initialDiagonalStep = 0.1f;
    double currentStep = initialDiagonalStep;
    int segmentsPerSide = 1;
    int totalPointsGenerated = 0;
    const double invSqrt2 = M_SQRT1_2;
    double currentOffsetAz = 0.0f, currentOffsetAlt = 0.0f;
    while (totalPointsGenerated < localSpiralMaxPoints)
    {
        double step = currentStep * invSqrt2;
        for (int i = 0; i < segmentsPerSide && totalPointsGenerated < localSpiralMaxPoints; ++i)
        {
            currentOffsetAz += step;
            currentOffsetAlt += step;
            localSpiralOffsets.push_back({currentOffsetAz, currentOffsetAlt});
            totalPointsGenerated++;
        }
        if (totalPointsGenerated >= localSpiralMaxPoints)
            break;
        for (int i = 0; i < segmentsPerSide && totalPointsGenerated < localSpiralMaxPoints; ++i)
        {
            currentOffsetAz -= step;
            currentOffsetAlt += step;
            localSpiralOffsets.push_back({currentOffsetAz, currentOffsetAlt});
            totalPointsGenerated++;
        }
        if (totalPointsGenerated >= localSpiralMaxPoints)
            break;
        segmentsPerSide++;
        for (int i = 0; i < segmentsPerSide && totalPointsGenerated < localSpiralMaxPoints; ++i)
        {
            currentOffsetAz -= step;
            currentOffsetAlt -= step;
            localSpiralOffsets.push_back({currentOffsetAz, currentOffsetAlt});
            totalPointsGenerated++;
        }
        if (totalPointsGenerated >= localSpiralMaxPoints)
            break;
        for (int i = 0; i < segmentsPerSide && totalPointsGenerated < localSpiralMaxPoints; ++i)
        {
            currentOffsetAz += step;
            currentOffsetAlt -= step;
            localSpiralOffsets.push_back({currentOffsetAz, currentOffsetAlt});
            totalPointsGenerated++;
        }
        if (totalPointsGenerated >= localSpiralMaxPoints)
            break;
        segmentsPerSide++;
    }
}

void IMUAssist::startAlignmentSpiral(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot)
{
    if (!enableLocalSpiral || localSpiralOffsets.empty())
    {
        setState(AlgoState::MONITORING, AlgoAction::NONE);
        return;
    }
    currentLocalSpiralIndex = 0;
    isLocalSpiraling = true;
    m_isPeakSeeking = false;
    m_peakSeekingBestPower = -9999999;
    m_peakSeekingBestIndex = -1;

    if (debug)
        std::cout << "[" << algorithmName << "] (" << currentSlot << ") Power dip. Starting spiral." << std::endl;

    try
    {
        rotaryObject currentRotaryState = engine->getRotaryData(unitIndex, currentSlot);

        // CAPTURE THE PHYSICAL CENTER HERE (Inverting Alt to mathematical)
        localSpiralCenterAz = currentRotaryState.azimuth.angle;
        localSpiralCenterAlt = currentRotaryState.altitude.angle * -1.0;

        if (!currentRotaryState.isMoving)
            issueLocalSpiralMoveCommand(engine, unitIndex, currentSlot, currentRotaryState);
        else
            setState(AlgoState::MONITORING, AlgoAction::MOVING);
    }
    catch (const std::exception &e)
    {
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        isLocalSpiraling = false;
    }
}

void IMUAssist::issueLocalSpiralMoveCommand(mobileTHzEngine *engine,
                                            size_t unitIndex,
                                            size_t currentSlot,
                                            const rotaryObject &currentRotaryState)
{
    if (!isLocalSpiraling || currentLocalSpiralIndex < 0 || currentLocalSpiralIndex >= static_cast<int>(localSpiralOffsets.size()))
    {
        isLocalSpiraling = false;
        setState(AlgoState::MONITORING, AlgoAction::NONE);
        return;
    }
    const auto &offset = localSpiralOffsets[currentLocalSpiralIndex];

    // 1. Calculate dynamic mathematical targets
    double absTargetAz = fmod(IMUTargetAz + offset.deltaAz, 360.0);
    if (absTargetAz < 0)
        absTargetAz += 360.0;

    double mathematicalTargetAlt = std::max(-90.0, std::min(90.0, IMUTargetAlt + offset.deltaAlt));

    // 2. Convert to motor coordinates
    double rotaryTargetAlt = mathematicalTargetAlt * -1.0;

    // 3. Issue relative move
    double relAz = shortestAngleDiff(absTargetAz, currentRotaryState.azimuth.angle);
    double relAlt = rotaryTargetAlt - currentRotaryState.altitude.angle;

    std::stringstream cmd;
    cmd << "mdd 0 " << std::fixed << std::setprecision(6) << relAz << " 1 " << relAlt;
    try
    {
        engine->issueRotaryCommand(unitIndex, cmd.str());
        setState(AlgoState::ALIGNMENT, AlgoAction::STARTING);
    }
    catch (const std::exception &e)
    {
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        isLocalSpiraling = false;
    }
}

double IMUAssist::calculateDynamicSuccessMargin(size_t currentSlot) const
{
    double margin = realignSuccessMaxMarginDb;
    if (lastSuccessfulPointSlot > 0 && currentSlot > lastSuccessfulPointSlot && slotTimeMicrosec > 0)
    {
        double elapsed_s = static_cast<double>(currentSlot - lastSuccessfulPointSlot) * slotTimeMicrosec / 1e6;
        margin = realignSuccessInitialMarginDb + (elapsed_s * realignSuccessMarginIncreaseRateDbPerSec);
        margin = std::max(realignSuccessInitialMarginDb, std::min(realignSuccessMaxMarginDb, margin));
    }
    else if (lastSuccessfulPointSlot == 0)
    {
        margin = realignSuccessInitialMarginDb;
    }
    return std::max(0.0, margin);
}

void IMUAssist::processSlot(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot)
{
    if (!engine || !configured || !imuEnabledRuntime)
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

    double rawPower = -9999999;
    imuObject currentIMU;
    rotaryObject currentRotaryState;
    bool rxSourceEnabled = false, imuDataValid = false, rotaryEnabled = false, isMoving = false;
    size_t rxSlot = std::numeric_limits<size_t>::max(), imuSlot = std::numeric_limits<size_t>::max();
    double currentTimeSec = static_cast<double>(currentSlot) * slotTimeMicrosec / 1e6;

    try
    {
        const auto &unitConf = engine->getEngineUnit(unitIndex);
        rotaryEnabled = unitConf.Rotary_enabled;
        if (unitConf.RxChain_enabled)
        {
            const auto &rx = engine->getRxChainData(unitIndex, currentSlot);
            rawPower = wattsToDbm(rx.power_watts);
            rxSlot = rx.sample_slot;
            rxSourceEnabled = true;
        }
        else
        {
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            return;
        }

        currentIMU = engine->getIMUData(unitIndex, currentSlot);
        imuSlot = currentSlot;
        imuDataValid = std::isfinite(currentIMU.acceleration.x);

        if (rotaryEnabled)
        {
            currentRotaryState = engine->getRotaryData(unitIndex, currentSlot);
            isMoving = currentRotaryState.isMoving;
        }
    }
    catch (const std::exception &e)
    {
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        return;
    }

    bool newRx = (rxSlot != lastProcessedRxSampleSlot);
    bool validReading = newRx && rxSourceEnabled && updatePowerFilter(rawPower);
    if (newRx)
        lastProcessedRxSampleSlot = rxSlot;

    bool newImu = (imuSlot != lastProcessedImuSampleSlot);
    bool usableImu = newImu && imuDataValid;
    if (newImu)
    {
        lastProcessedImuSampleSlot = imuSlot;
        if (usableImu)
        {
            imuSamples_.push(currentIMU);
            if (baselineEstablished)
            {
                Position world_acc = rotateVectorByQuaternion(currentIMU.acceleration, conjugate(currentIMU.quaternion));
                Position motion = {world_acc.x - baselineAcceleration.x, world_acc.y - baselineAcceleration.y, world_acc.z - baselineAcceleration.z};
                if (std::abs(motion.x) > imuSignificantMotionThreshold_)
                    imuAccumulatedMotion_.x += motion.x;
                if (std::abs(motion.y) > imuSignificantMotionThreshold_)
                    imuAccumulatedMotion_.y += motion.y;
                if (std::abs(motion.z) > imuSignificantMotionThreshold_)
                    imuAccumulatedMotion_.z += motion.z;
            }
        }
    }

    if (usableImu)
    {
        updateKinematics(currentIMU, currentTimeSec);
        if (baselineEstablished)
            calculateTargetAngles(currentIMU);
    }

    switch (currentState)
    {
    case AlgoState::REFERENCE:
    {
        if (validReading && filterInitialized)
            warmupCounter++;
        else if (!filterInitialized && validReading)
            warmupCounter++;

        if (warmupCounter >= referenceWarmupSamples && filterInitialized && usableImu && rotaryEnabled)
        {
            currentReferencePower = longTermEMA;
            if (!std::isfinite(currentReferencePower))
            {
                warmupCounter = 0;
                filterInitialized = false;
                break;
            }

            lastSuccessfulPointSlot = currentSlot;
            initialDistance = estimateDistanceFromPower(currentReferencePower);
            if (initialDistance <= 0)
            {
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                break;
            }

            initialOrientation = currentIMU.quaternion;
            baselineAcceleration = rotateVectorByQuaternion(currentIMU.acceleration, conjugate(initialOrientation));

            try
            {
                rotaryObject currentRotary = engine->getRotaryData(unitIndex, currentSlot);
                initialIMUTargetAz = currentRotary.azimuth.angle;
                initialIMUTargetAlt = currentRotary.altitude.angle * -1.0;
                initialPointingVectorLocal_ = anglesToVector(initialIMUTargetAz, initialIMUTargetAlt, initialDistance);
            }
            catch (const std::exception &e)
            {
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                break;
            }

            IMUTargetAz = initialIMUTargetAz;
            IMUTargetAlt = initialIMUTargetAlt;
            currentRelativePosition = {0, 0, 0};
            lastImuSampleTimeSec = currentTimeSec;
            currentVelocity = {0, 0, 0};
            baselineEstablished = true;
            last_tracked_orientation_ = currentIMU.quaternion;
            imuAccumulatedMotion_ = {0, 0, 0};

            if (debug)
                std::cout << "[" << algorithmName << "] Baseline OK (" << currentSlot << ") RefPwr=" << currentReferencePower << "dBm, Dist=" << initialDistance << "m, Az/Alt=" << initialIMUTargetAz << "/" << initialIMUTargetAlt << std::endl;
            setState(AlgoState::MONITORING, AlgoAction::NONE);
        }
    }
    break;

    case AlgoState::MONITORING:
    {
        if (!baselineEstablished)
        {
            setState(AlgoState::REFERENCE, AlgoAction::NONE);
            break;
        }
        if (isMoving)
            setState(AlgoState::MONITORING, AlgoAction::MOVING);
        else if (currentAction == AlgoAction::MOVING || currentAction == AlgoAction::STARTING)
            setState(AlgoState::MONITORING, AlgoAction::NONE);

        if (validReading && (shortTermEMA >= (currentReferencePower - localSpiralTriggerThresholdDb)))
            lastSuccessfulPointSlot = currentSlot;

        bool triggerSpiral = enableLocalSpiral && !isMoving && validReading && (shortTermEMA < (currentReferencePower - localSpiralTriggerThresholdDb));

        double accumMag = std::sqrt(imuAccumulatedMotion_.x * imuAccumulatedMotion_.x + imuAccumulatedMotion_.y * imuAccumulatedMotion_.y + imuAccumulatedMotion_.z * imuAccumulatedMotion_.z);
        bool transTrigger = (accumMag > imuTrackingAccumulatedMagnitudeThreshold_);
        double dot = currentIMU.quaternion.w * last_tracked_orientation_.w + currentIMU.quaternion.x * last_tracked_orientation_.x + currentIMU.quaternion.y * last_tracked_orientation_.y + currentIMU.quaternion.z * last_tracked_orientation_.z;
        double angle = (2.0 * std::acos(std::max(-1.0, std::min(1.0, std::abs(dot))))) * RAD_TO_DEG;
        bool rotTrigger = (angle > imuTrackingRotationThresholdDeg);

        if (triggerSpiral)
        {
            startAlignmentSpiral(engine, unitIndex, currentSlot);
        }
        else if (rotaryEnabled && !isMoving && (rotTrigger || transTrigger))
        {
            auto predicted_motion_delta = calculatePredictedMotion(imuSamples_, 200000.0, baselineAcceleration, currentVelocity);
            Position predicted_displacement = predicted_motion_delta.first;
            Quaternion predicted_rotation_delta = predicted_motion_delta.second;

            Position predicted_world_pos = {currentRelativePosition.x + predicted_displacement.x, currentRelativePosition.y + predicted_displacement.y, currentRelativePosition.z + predicted_displacement.z};
            Quaternion predicted_orientation = multiplyQuaternions(predicted_rotation_delta, currentIMU.quaternion);

            Position targetWorldPosition = rotateVectorByQuaternion(initialPointingVectorLocal_, conjugate(initialOrientation));

            Position future_world_vector_to_ap = {targetWorldPosition.x - predicted_world_pos.x, targetWorldPosition.y - predicted_world_pos.y, targetWorldPosition.z - predicted_world_pos.z};

            Position future_target_vector_local = rotateVectorByQuaternion(future_world_vector_to_ap, conjugate(predicted_orientation));

            double predictedTargetAz, predictedTargetAlt;
            vectorToAngles(future_target_vector_local, predictedTargetAz, predictedTargetAlt);

            predictedTargetAz = fmod(predictedTargetAz, 360.0);
            if (predictedTargetAz < 0)
                predictedTargetAz += 360.0;
            predictedTargetAlt = std::max(-90.0, std::min(90.0, predictedTargetAlt)) * -1.0;

            issueIMUTrackMoveCommand(engine, unitIndex, currentSlot, currentRotaryState, predictedTargetAz, predictedTargetAlt);
            last_tracked_orientation_ = currentIMU.quaternion;
            imuAccumulatedMotion_ = {0, 0, 0};
        }
    }
    break;

    case AlgoState::ALIGNMENT:
    {
        if (!baselineEstablished || !isLocalSpiraling)
        {
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            break;
        }
        if (isMoving)
        {
            if (currentAction != AlgoAction::MOVING)
                setState(AlgoState::ALIGNMENT, AlgoAction::MOVING);
        }
        else
        {
            if (currentAction == AlgoAction::MOVING || currentAction == AlgoAction::STARTING)
                setState(AlgoState::ALIGNMENT, AlgoAction::NONE);
        }

        bool halt = false;
        if (validReading)
        {
            if ((shortTermEMA >= (currentReferencePower - calculateDynamicSuccessMargin(currentSlot))) && !m_isPeakSeeking)
            {
                m_isPeakSeeking = true;
                m_peakSeekingBestPower = shortTermEMA;
                m_peakSeekingBestIndex = currentLocalSpiralIndex;
            }
            if (m_isPeakSeeking)
            {
                if (shortTermEMA > m_peakSeekingBestPower)
                {
                    m_peakSeekingBestPower = shortTermEMA;
                    m_peakSeekingBestIndex = currentLocalSpiralIndex;
                }
                else if (shortTermEMA < (m_peakSeekingBestPower - peakSeekingDecreaseThresholdDb))
                {
                    halt = true;
                }
            }
        }

        if (!isMoving && currentAction == AlgoAction::STOPPING)
        {
            setState(AlgoState::REFERENCE, AlgoAction::NONE);
            filterInitialized = false;
            warmupCounter = 0;
            m_isPeakSeeking = false;
            isLocalSpiraling = false;
            break;
        }

        int nextIdx = currentLocalSpiralIndex + 1;
        if (nextIdx >= static_cast<int>(localSpiralOffsets.size()) && m_isPeakSeeking)
            halt = true;

        if (halt && currentAction != AlgoAction::STOPPING)
        {
            try
            {
                engine->issueRotaryCommand(unitIndex, "halt 0");
                engine->issueRotaryCommand(unitIndex, "halt 1");
                setState(AlgoState::ALIGNMENT, AlgoAction::STOPPING);
            }
            catch (const std::exception &e)
            {
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            }
            break;
        }

        if (!isMoving && currentAction == AlgoAction::NONE && !halt)
        {
            if (nextIdx >= static_cast<int>(localSpiralOffsets.size()))
            {
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                isLocalSpiraling = false;
                break;
            }
            currentLocalSpiralIndex = nextIdx;
            try
            {
                issueLocalSpiralMoveCommand(engine, unitIndex, currentSlot, engine->getRotaryData(unitIndex, currentSlot));
            }
            catch (const std::exception &e)
            {
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            }
        }
    }
    break;

    case AlgoState::ERROR_STATUS:
        if (rotaryEnabled && isMoving && currentAction != AlgoAction::STOPPING)
        {
            try
            {
                engine->issueRotaryCommand(unitIndex, "halt 0");
                engine->issueRotaryCommand(unitIndex, "halt 1");
                setState(AlgoState::ERROR_STATUS, AlgoAction::STOPPING);
            }
            catch (...)
            {
            }
        }
        else if (!isMoving && currentAction == AlgoAction::STOPPING)
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
    catch (...)
    {
    }
}

void IMUAssist::issueIMUTrackMoveCommand(mobileTHzEngine *engine,
                                         size_t unitIndex,
                                         size_t currentSlot,
                                         const rotaryObject &currentRotaryState,
                                         double targetAz,
                                         double targetAlt)
{
    if (!baselineEstablished || !std::isfinite(targetAz) || !std::isfinite(targetAlt))
        return;
    double deltaAz = shortestAngleDiff(targetAz, currentRotaryState.azimuth.angle);
    double deltaAlt = targetAlt - currentRotaryState.altitude.angle;
    if (std::abs(deltaAz) > minAngleChangeThresholdDeg || std::abs(deltaAlt) > minAngleChangeThresholdDeg)
    {
        std::stringstream cmd;
        cmd << "mdd 0 " << std::fixed << std::setprecision(6) << deltaAz << " 1 " << deltaAlt;
        try
        {
            engine->issueRotaryCommand(unitIndex, cmd.str());
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot << ") Issued Predicted IMU Track Move: '" << cmd.str() << "' (Target Az/Alt: " << targetAz << "/" << targetAlt << ")\n";
            setState(AlgoState::MONITORING, AlgoAction::STARTING);
        }
        catch (const std::exception &e)
        {
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        }
    }
}

std::pair<Position, Quaternion> IMUAssist::calculatePredictedMotion(
    const CircularBuffer<imuObject> &imuSamples,
    double predictionHorizonMicrosec,
    const Position &worldGravity,
    const Position &currentVelocityAtPredictionStart) const
{
    std::pair<Position, Quaternion> prediction = {{0, 0, 0}, {1, 0, 0, 0}};
    if (imuSamples.empty() || predictionHorizonMicrosec <= 0)
        return prediction;

    size_t numSamples = imuSamples.size();
    Position accelTrend = {0, 0, 0}, gyroTrend = {0, 0, 0};
    double totalWeight = 0;

    for (size_t k = 0; k < numSamples; ++k)
    {
        const imuObject &sample = imuSamples[k];
        double weight = precomputed_imu_weights_[k];
        Position world_accel = rotateVectorByQuaternion(sample.acceleration, conjugate(sample.quaternion));
        Position motion = {world_accel.x - worldGravity.x, world_accel.y - worldGravity.y, world_accel.z - worldGravity.z};
        accelTrend.x += motion.x * weight;
        accelTrend.y += motion.y * weight;
        accelTrend.z += motion.z * weight;
        gyroTrend.x += sample.angular_velocity.x * weight;
        gyroTrend.y += sample.angular_velocity.y * weight;
        gyroTrend.z += sample.angular_velocity.z * weight;
        totalWeight += weight;
    }

    if (totalWeight <= 1e-9)
        return prediction;

    accelTrend.x /= totalWeight;
    accelTrend.y /= totalWeight;
    accelTrend.z /= totalWeight;
    gyroTrend.x /= totalWeight;
    gyroTrend.y /= totalWeight;
    gyroTrend.z /= totalWeight;

    double time_s = predictionHorizonMicrosec / 1e6;
    prediction.first.x = currentVelocityAtPredictionStart.x * time_s + 0.5 * accelTrend.x * time_s * time_s;
    prediction.first.y = currentVelocityAtPredictionStart.y * time_s + 0.5 * accelTrend.y * time_s * time_s;
    prediction.first.z = currentVelocityAtPredictionStart.z * time_s + 0.5 * accelTrend.z * time_s * time_s;

    double angle = std::sqrt(pow(gyroTrend.x * time_s, 2) + pow(gyroTrend.y * time_s, 2) + pow(gyroTrend.z * time_s, 2));
    if (angle > 1e-6)
    {
        double half = angle / 2.0, sinHalf = std::sin(half);
        prediction.second.w = std::cos(half);
        prediction.second.x = (gyroTrend.x * time_s / angle) * sinHalf;
        prediction.second.y = (gyroTrend.y * time_s / angle) * sinHalf;
        prediction.second.z = (gyroTrend.z * time_s / angle) * sinHalf;
    }
    return prediction;
}
