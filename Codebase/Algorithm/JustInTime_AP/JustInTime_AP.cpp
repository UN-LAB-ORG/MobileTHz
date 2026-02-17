#include "JustInTime_AP.h"
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

    Quaternion conjugate(const Quaternion &q)
    {
        return {q.w, -q.x, -q.y, -q.z};
    }

    Quaternion multiply(const Quaternion &q1, const Quaternion &q2)
    {
        Quaternion result;
        result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
        result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
        result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
        result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
        return result;
    }

    Position rotateVectorByQuaternion(const Position &v, const Quaternion &q)
    {
        Quaternion v_quat = {0, v.x, v.y, v.z};
        Quaternion q_conj = conjugate(q);
        Quaternion rotated_v_quat = multiply(multiply(q, v_quat), q_conj);
        return {rotated_v_quat.x, rotated_v_quat.y, rotated_v_quat.z};
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
            angles.y = std::copysign(M_PI / 2, sinp);
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

double estimateDistanceFromPowerAP(double rxPowerDbmAtAP,
                                   double ueTxPowerDbm,
                                   double ueTxGainDbi,
                                   double apRxGainDbi,
                                   long long carrierFrequencyHz,
                                   const std::string &apAlgorithmName)
{
    if (!std::isfinite(rxPowerDbmAtAP) || carrierFrequencyHz <= 0)
        return -1.0;
    double totalGainLinear = std::pow(10.0, (ueTxGainDbi + apRxGainDbi) / 10.0);
    double ueTxPowerLinearWatts = std::pow(10.0, (ueTxPowerDbm - 30.0) / 10.0);
    double rxPowerAtApLinearWatts = std::pow(10.0, (rxPowerDbmAtAP - 30.0) / 10.0);
    if (ueTxPowerLinearWatts <= 0 || rxPowerAtApLinearWatts <= 0 || totalGainLinear <= 0)
        return -1.0;
    double lambda_m = SPEED_OF_LIGHT / static_cast<double>(carrierFrequencyHz);
    if (lambda_m <= 0)
        return -1.0;
    double factor = ueTxPowerLinearWatts * totalGainLinear * std::pow(lambda_m / (4.0 * M_PI), 2.0);
    return std::sqrt(factor / rxPowerAtApLinearWatts);
}

JustInTime_AP::JustInTime_AP()
{
    setState(AlgoState::REFERENCE, AlgoAction::NONE);
}

void JustInTime_AP::configure(const ConfigFile &config, const engineUnit &unitConfig)
{
    algorithmName = unitConfig.algorithm + "_" + unitConfig.label;
    if (debug)
        std::cout << "[" << algorithmName << "] Configuring AP..." << std::endl;

    slotTimeMicrosec = config.engine_slot_time_microsec;
    if (slotTimeMicrosec <= 0)
    {
        if (debug)
            std::cerr << "[" << algorithmName
                      << "] Error: Invalid slot_time_microsec. Using 1000us." << std::endl;
        slotTimeMicrosec = 1000.0;
    }
    panic_info_trust_duration_slots = static_cast<size_t>(
        std::ceil((panic_info_trust_duration_sec * 1.0e6) / slotTimeMicrosec));
    if (panic_info_trust_duration_slots == 0 && panic_info_trust_duration_sec > 0)
        panic_info_trust_duration_slots = 1;

    currentState = AlgoState::REFERENCE;
    currentAction = AlgoAction::NONE;
    configured = true;
    filterInitialized = false;
    warmupCounter = 0;
    currentReferencePower = -9999999;
    shortTermEMA = -9999999;
    longTermEMA = -9999999;
    lastRawPower = -9999999;
    lastProcessedRxSampleSlot = std::numeric_limits<size_t>::max();
    lastSuccessfulAlignmentSlot = 0;
    spiralOffsets.clear();
    currentSpiralPointIndex = -1;
    spiralCenterAz = 0.0f;
    spiralCenterAlt = 0.0f;
    m_isPeakSeeking = false;
    m_peakSeekingBestPower = -9999999;
    m_peakSeekingBestIndex = -1;
    m_panic_move_command_issued = false;
    ap_cumulative_ue_displacement_m = {0.0f, 0.0f, 0.0f};
    ap_cumulative_ue_rotation = {1.0, 0, 0, 0};
    ue_baseline_az_at_ap_reference = 0.0f;
    ue_baseline_alt_at_ap_reference = 0.0f;
    has_fresh_panic_info = false;
    last_panic_info_reception_slot = 0;
    panic_predicted_target_az = 0.0f;
    panic_predicted_target_alt = 0.0f;
    halfPowerBeamWidth = unitConfig.radiationPatterns_hpbw.at(0);
    realignSuccessMarginIncreaseRateDbPerSec = linearToDbi(
                                                   unitConfig.radiationPatterns_maxGain_linear.at(0)) *
                                               0.5;
    misalignThresholdAbsoluteDb = 3;
    realignSuccessInitialMarginDb = 1;

    // System Parameters for Distance Estimation
    txPowerDbm = wattsToDbm(unitConfig.TxAntenna_transmitPower_watts);
    txGainDbi = linearToDbi(
        unitConfig.radiationPatterns_maxGain_linear.at(0)); // ASSUMING same pattern for Tx
    rxGainDbi = linearToDbi(
        unitConfig.radiationPatterns_maxGain_linear.at(0)); // ASSUMING same pattern for Rx
    frequencyGhz = config.engine_carrier_frequency;

    if (debug)
    {
        std::cout << "[" << algorithmName
                  << "] AP Configured. Panic Info Trust: " << panic_info_trust_duration_sec << "s ("
                  << panic_info_trust_duration_slots << " slots)." << std::endl;
        std::cout << "  IMU Interp Window: " << panic_imu_interpretation_window_slots
                  << " samples, Min samples for pred: " << panic_min_imu_samples_for_prediction
                  << std::endl;
    }
    setState(AlgoState::REFERENCE, AlgoAction::NONE);
}

void JustInTime_AP::setState(AlgoState newStatus, AlgoAction newAction)
{
    if (currentState != newStatus || currentAction != newAction)
    {
        if ((currentState == AlgoState::ALIGNMENT || currentState == AlgoState::PANIC_ALIGNMENT) && (newStatus != AlgoState::ALIGNMENT && newStatus != AlgoState::PANIC_ALIGNMENT))
        {
            // Exiting any alignment state
            m_panic_move_command_issued = false;
            m_isPeakSeeking = false;
            currentSpiralPointIndex = -1; // Reset index when leaving alignment
        }
        if (newStatus == AlgoState::ERROR_STATUS)
        {
            m_panic_move_command_issued = false;
            m_isPeakSeeking = false;
        }
        currentState = newStatus;
        currentAction = newAction;
    }
}

algorithmObject JustInTime_AP::getStatus() const
{
    return {static_cast<double>(currentState), static_cast<double>(currentAction)};
}

bool JustInTime_AP::updatePowerFilter(double rawPower)
{
    if (!std::isfinite(rawPower) || rawPower <= -200.0f)
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
            // warmupCounter is managed by REFERENCE state
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
    {
        return true;
    }
    else
    {
        return false;
    }
}

double JustInTime_AP::shortestAngleDiff(double targetDeg, double currentDeg) const
{
    double diff = std::fmod(targetDeg - currentDeg, 360.0);
    if (diff > 180.0)
    {
        diff -= 360.0;
    }
    else if (diff <= -180.0)
    {
        diff += 360.0;
    }
    return diff;
}

void JustInTime_AP::processReceivedJITPackets(mobileTHzEngine *engine,
                                              size_t apUnitIndex,
                                              size_t currentSlot,
                                              const rotaryObject &currentAPRotary)
{
    const auto &allPacketsInLayer = engine->getPacketLayer();

    // Constant variable
    const long long thermalNoise = wattsToDbm(engine->receiverThermalNoise_watts);
    const double rxChainNoise = linearToDbi(
        engine->getEngineUnit(apUnitIndex).RxChain_noise_factor_linear);

    size_t i = m_nextPacketToProcessIndex;

    for (; i < allPacketsInLayer.size(); ++i)
    { // Loop using the pre-declared 'i'.
        const auto &rxPacket = allPacketsInLayer[i];
        if (currentSlot < static_cast<size_t>(rxPacket.receive_end_slot))
        {
            break; // Stop processing for this slot, will resume from here next time
        }

        if (rxPacket.unit_index == apUnitIndex)
            continue;

        size_t senderUnitIndex = static_cast<size_t>(rxPacket.unit_index);
        size_t packetTxActualSlot = static_cast<size_t>(rxPacket.transmit_start_slot);

        double signalPowerDbm = wattsToDbm(
            engine->getSpecificLinkPowerWatts(senderUnitIndex, apUnitIndex, packetTxActualSlot));
        if (!std::isfinite(signalPowerDbm))
        {
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") AP Could not get link power for packet " << rxPacket.id
                          << " from UE " << senderUnitIndex << std::endl;
            continue;
        }
        double snrDb = signalPowerDbm - (thermalNoise + rxChainNoise); // SNR in dB
        if (snrDb >= engine->JIT_packet_min_snr_db)
        {
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") AP DECODED JIT Packet ID " << rxPacket.id << " from UE "
                          << senderUnitIndex << " with SNR " << std::fixed << std::setprecision(2)
                          << snrDb << "dB. DeltaPos: X=" << rxPacket.delta_position.x
                          << " Y=" << rxPacket.delta_position.y
                          << " Z=" << rxPacket.delta_position.z
                          << ". IMU samples: " << rxPacket.imu_data.size() << "." << std::endl;
            const auto &ueConfig = engine->getEngineUnit(senderUnitIndex);

            // --- Apply Rotational Change First ---
            Position delta_pos_in_world_frame = rxPacket.delta_position;

            // Update the AP's understanding of the UE's total displacement
            ap_cumulative_ue_displacement_m.x += delta_pos_in_world_frame.x;
            ap_cumulative_ue_displacement_m.y += delta_pos_in_world_frame.y;
            ap_cumulative_ue_displacement_m.z += delta_pos_in_world_frame.z;

            // Update the AP's understanding of the UE's total rotation
            ap_cumulative_ue_rotation = multiply(rxPacket.delta_rotation, ap_cumulative_ue_rotation);

            // --- Check if the movement was rotation-dominant ---
            double translation_magnitude = std::sqrt(
                delta_pos_in_world_frame.x * delta_pos_in_world_frame.x + delta_pos_in_world_frame.y * delta_pos_in_world_frame.y + delta_pos_in_world_frame.z * delta_pos_in_world_frame.z);

            double currentDistanceToUE = estimateDistanceFromPowerAP(signalPowerDbm,
                                                                     wattsToDbm(ueConfig.TxAntenna_transmitPower_watts),
                                                                     txGainDbi,
                                                                     rxGainDbi,
                                                                     engine->getCarrierFrequency(),
                                                                     algorithmName);
            if (currentDistanceToUE <= 0.1)
            {
                if (debug)
                    std::cout << "[" << algorithmName
                              << "] AP Warning: Unreliable current distance to UE "
                              << senderUnitIndex << " for delta_pos conversion." << std::endl;
                continue;
            }

            const double ROTATION_ONLY_TRANSLATION_THRESHOLD_M = 0.01;
            if (translation_magnitude < ROTATION_ONLY_TRANSLATION_THRESHOLD_M)
            {
                m_is_awaiting_ue_rotation_outcome = true;
                size_t wait_slots = static_cast<size_t>(
                    std::ceil((panicPacketCooldownMicroseconds) / slotTimeMicrosec));
                m_ue_rotation_wait_end_slot = currentSlot + wait_slots;

                if (debug)
                    std::cout
                        << "[" << algorithmName << "] (" << currentSlot
                        << ") Detected rotation-dominant movement from UE. Waiting until slot: "
                        << m_ue_rotation_wait_end_slot << std::endl;
            }

            // The access point can now use its imu quaterion data to translate world frame UE
            imuObject imuData = engine->getIMUData(apUnitIndex, currentSlot);

            // now rotate the world frame displacement vector into AP's local frame
            Position ue_displacement_in_ap_frame = rotateVectorByQuaternion(ap_cumulative_ue_displacement_m, imuData.quaternion);

            // 4. Calculate angular deltas using the components from the AP's new local frame.
            double angular_delta_az_rad = std::atan2(ue_displacement_in_ap_frame.y,
                                                     currentDistanceToUE);
            double angular_delta_alt_rad = std::atan2(ue_displacement_in_ap_frame.z,
                                                      currentDistanceToUE);

            const double RAD_TO_DEG_AP = 180.0 / M_PI;
            double totalDeltaAzDeg = static_cast<double>(angular_delta_az_rad * RAD_TO_DEG_AP);
            double totalDeltaAltDeg = static_cast<double>(angular_delta_alt_rad * RAD_TO_DEG_AP);
            double total_target_az = ue_baseline_az_at_ap_reference + totalDeltaAzDeg;
            double total_target_alt = ue_baseline_alt_at_ap_reference + totalDeltaAltDeg;

            panic_predicted_target_az = total_target_az;
            panic_predicted_target_alt = total_target_alt;
            panic_predicted_target_az = std::fmod(panic_predicted_target_az, 360.0f);
            if (panic_predicted_target_az < 0)
                panic_predicted_target_az += 360.0f;
            panic_predicted_target_alt = std::max(0.0, std::min(90.0, panic_predicted_target_alt));
            has_fresh_panic_info = true;
            last_panic_info_reception_slot = rxPacket.receive_end_slot;

            if (debug)
            {
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Final Panic Target Az/Alt: " << panic_predicted_target_az << "/"
                          << panic_predicted_target_alt
                          << " | Received DeltaPos (m): UE_dY=" << rxPacket.delta_position.y
                          << " UE_dZ=" << rxPacket.delta_position.z
                          << " | AP Cumulative UE Disp (m): Y=" << ap_cumulative_ue_displacement_m.y
                          << " Z=" << ap_cumulative_ue_displacement_m.z
                          << " | Converted Angular Delta (deg): Az=" << totalDeltaAzDeg
                          << " Alt=" << totalDeltaAltDeg << std::endl;
            }
        }
        else
        {
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") AP DISCARDED JIT Packet ID " << rxPacket.id << " from UE "
                          << senderUnitIndex << ". SNR " << std::fixed << std::setprecision(2)
                          << snrDb << "dB < MinSNR " << engine->JIT_packet_min_snr_db << "dB."
                          << std::endl;
        }
    }

    m_nextPacketToProcessIndex = i;
}

void JustInTime_AP::processSlot(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot)
{
    // --- Initial Checks ---
    if (!engine)
    {
        if (currentState != AlgoState::ERROR_STATUS)
        {
            std::cerr << "[" << algorithmName << "] Error: Engine pointer null." << std::endl;
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        }
        return;
    }
    if (!configured)
    {
        if (currentState != AlgoState::ERROR_STATUS)
        {
            std::cerr << "[" << algorithmName << "] Error: Not configured." << std::endl;
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        }
        return;
    }

    // --- Get Engine Data ---
    double rawPower = -9999999;
    size_t currentRxSampleSlot = std::numeric_limits<size_t>::max();
    bool rotaryEnabled = false;
    rotaryObject currentRotaryState;
    bool isCurrentlyMoving = false;
    bool rxSourceEnabled = false;

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
            {
                std::cerr << "[" << algorithmName << "] Error: RxChain not enabled for unit "
                          << unitIndex << std::endl;
            }
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
        if (rotaryEnabled)
        {
            currentRotaryState = engine->getRotaryData(unitIndex, currentSlot);
            isCurrentlyMoving = ((currentRotaryState.isMoving == 1) ? true : false);
        }
    }
    catch (const std::out_of_range &oor)
    {
        if (currentState != AlgoState::ERROR_STATUS)
        {
            std::cerr << "[" << algorithmName
                      << "] Error accessing engine unit data: " << oor.what() << std::endl;
        }
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
    catch (const std::exception &e)
    {
        if (currentState != AlgoState::ERROR_STATUS)
        {
            std::cerr << "[" << algorithmName << "] Error getting engine data slot " << currentSlot
                      << ": " << e.what() << std::endl;
        }
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

    // --- Check if the received Rx data is NEW ---
    bool isNewRxData = (currentRxSampleSlot != lastProcessedRxSampleSlot) && (currentRxSampleSlot != std::numeric_limits<size_t>::max());

    // --- Update Internal Filter ---
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

    // --- Process Received JIT Packets ---
    processReceivedJITPackets(engine, unitIndex, currentSlot, currentRotaryState);

    // --- State Machine Logic ---
    switch (currentState)
    {
    case AlgoState::REFERENCE:
    {
        if (readingIsValid && filterInitialized)
        {
            warmupCounter++;
        }
        if (warmupCounter >= referenceWarmupSamples && filterInitialized)
        {
            currentReferencePower = shortTermEMA;
            longTermEMA = shortTermEMA;
            ue_baseline_az_at_ap_reference = currentRotaryState.azimuth.angle;
            ue_baseline_alt_at_ap_reference = currentRotaryState.altitude.angle;
            ap_cumulative_ue_displacement_m = {0.0f, 0.0f, 0.0f};
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Reference Set: " << shortTermEMA
                          << " dBm. AP Baseline Az/Alt: " << ue_baseline_az_at_ap_reference << "/"
                          << ue_baseline_alt_at_ap_reference << std::endl;
            lastSuccessfulAlignmentSlot = currentSlot;
            setState(AlgoState::MONITORING, AlgoAction::NONE);
        }
        else if (warmupCounter >= referenceWarmupSamples && !filterInitialized)
        {
            if (currentState != AlgoState::ERROR_STATUS)
            {
                std::cerr << "[" << algorithmName
                          << "] Error: Reference setting failed - no valid power readings."
                          << std::endl;
            }
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        }
    }
    break;

    case AlgoState::MONITORING:
    {
        if (!filterInitialized || !std::isfinite(longTermEMA))
        {
            if (currentState != AlgoState::REFERENCE)
            {
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Warning: Monitoring skipped - filter/reference invalid."
                          << std::endl;
            }
            setState(AlgoState::REFERENCE, AlgoAction::NONE);
            break;
        }

        // --- Handle UE rotation waiting period ---
        if (m_is_awaiting_ue_rotation_outcome)
        {
            if (currentSlot >= m_ue_rotation_wait_end_slot)
            {
                if (debug)
                    std::cout << "[" << algorithmName << "] (" << currentSlot
                              << ") UE rotation wait period ended." << std::endl;
                m_is_awaiting_ue_rotation_outcome = false; // Timer expired
            }
            else
            {
                // We do nothing while waiting for UE rotation outcome
                break;
            }
        }
        bool misaligned = (shortTermEMA < (currentReferencePower - misalignThresholdAbsoluteDb));

        // Check for fresh panic info first
        if (has_fresh_panic_info && rotaryEnabled)
        {
            // Check to make sure if panic has been received within trust duration
            if (currentSlot > (last_panic_info_reception_slot + panic_info_trust_duration_slots))
            {
                if (debug)
                    std::cout << "[" << algorithmName << "] (" << currentSlot
                              << ") Panic info is not valid. Skipping fresh panic check."
                              << std::endl;
                has_fresh_panic_info = false;
            }
            else
            {
                if (debug)
                    std::cout << "[" << algorithmName << "] (" << currentSlot
                              << ") Fresh panic info received. Predicted Az/Alt: "
                              << panic_predicted_target_az << "/" << panic_predicted_target_alt
                              << ". Initiating PANIC_ALIGNMENT." << std::endl;
                startPanicAlignment(engine,
                                    unitIndex,
                                    currentSlot,
                                    currentRotaryState); // Moves to PANIC_ALIGNMENT
                has_fresh_panic_info = false;            // Consume the flag
            }
        }
        // Then check for misalignment (reactive adjustment)
        else if (misaligned && !m_is_awaiting_ue_rotation_outcome)
        {
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Misalignment Detected. ShortEMA: " << shortTermEMA
                          << " < Ref: " << currentReferencePower << " - "
                          << misalignThresholdAbsoluteDb << " dB. Initiating STANDARD ALIGNMENT."
                          << std::endl;
            if (rotaryEnabled)
            {
                startStandardAlignment(engine,
                                       unitIndex,
                                       currentSlot,
                                       currentRotaryState); // Moves to ALIGNMENT
            }
            else
            {
                if (currentState != AlgoState::ERROR_STATUS)
                {
                    std::cerr << "[" << algorithmName
                              << "] Error: Misaligned but Rotary not enabled. Cannot realign."
                              << std::endl;
                }
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            }
        }
        else
        {
            lastSuccessfulAlignmentSlot = currentSlot;
            setState(AlgoState::MONITORING, AlgoAction::NONE);
        }
    }
    break;

    case AlgoState::ALIGNMENT:
    {
        // --- Pre-checks for Alignment State  ---
        if (!filterInitialized || !std::isfinite(currentReferencePower))
        { /* ... error handling ... */
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            break;
        }
        if (!rotaryEnabled || spiralOffsets.empty())
        { /* ... error handling ... */
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            break;
        }
        if (currentSpiralPointIndex < 0)
        { /* ... error handling ... */
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            break;
        }

        if (has_fresh_panic_info && rotaryEnabled)
        {
            if (debug)
            {
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") INTERRUPTING standard ALIGNMENT due to new JIT Packet."
                          << " Switching to PANIC_ALIGNMENT." << std::endl;
            }

            startPanicAlignment(engine, unitIndex, currentSlot, currentRotaryState);
            has_fresh_panic_info = false; // Consume the flag.

            break;
        }

        // --- Update Action based on Movement Status ---
        if (isCurrentlyMoving)
        {
            if (currentAction == AlgoAction::STARTING || currentAction == AlgoAction::MOVING)
            {
                setState(AlgoState::ALIGNMENT, AlgoAction::MOVING);
            }
        }
        else
        { // Not moving
            if (currentAction == AlgoAction::MOVING || currentAction == AlgoAction::STARTING)
            {
                setState(AlgoState::ALIGNMENT, AlgoAction::NONE);
            }
        }

        bool triggerHalt = false;
        if (readingIsValid && std::isfinite(shortTermEMA) && filterInitialized)
        {
            double currentSuccessMargin = calculateDynamicSuccessMargin(currentSlot);
            bool baseSuccessConditionMet = (shortTermEMA >= (currentReferencePower - currentSuccessMargin));

            if (debug && (currentAction == AlgoAction::NONE || isCurrentlyMoving))
            { // Log power if stopped or moving
              // std::cout << "[" << algorithmName << "] (" << currentSlot
              //           << ") ShortEMA: " << shortTermEMA << ", Ref: " << currentReferencePower
              //           << ", Margin: " << currentSuccessMargin
              //           << ", Success: " << baseSuccessConditionMet
              //           << ", PeakSeek: " << m_isPeakSeeking
              //           << ", Idx: " << currentSpiralPointIndex
              //           << ", Moving: " << isCurrentlyMoving << std::endl;
            }

            if (baseSuccessConditionMet && !m_isPeakSeeking)
            {
                if (debug)
                    std::cout << "[" << algorithmName << "] (" << currentSlot
                              << ") Threshold met. Entering Peak Seeking. Power: " << shortTermEMA
                              << ". Triggering HALT." << std::endl;
                m_isPeakSeeking = true;
                m_peakSeekingBestPower = shortTermEMA;
                m_peakSeekingBestIndex = currentSpiralPointIndex;
                triggerHalt = true; // Premature halt is desired
            }
        }

        // --- State Transitions & Action Decisions ---
        if (!isCurrentlyMoving && currentAction == AlgoAction::STOPPING)
        {
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Halt completed. Transitioning to REFERENCE." << std::endl;
            setState(AlgoState::REFERENCE, AlgoAction::NONE);
            filterInitialized = false;
            warmupCounter = 0;
            m_isPeakSeeking = false;
            break;
        }

        if (triggerHalt && currentAction != AlgoAction::STOPPING)
        {
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot << ") Issuing HALT."
                          << std::endl;
            try
            {
                engine->issueRotaryCommand(unitIndex, "halt 0");
                engine->issueRotaryCommand(unitIndex, "halt 1");
                setState(AlgoState::ALIGNMENT, AlgoAction::STOPPING);
            }
            catch (const std::exception &e)
            { /* ... error handling ... */
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                m_isPeakSeeking = false;
            }
            break;
        }

        if (!isCurrentlyMoving && currentAction == AlgoAction::NONE && !triggerHalt)
        {
            int nextSpiralPointIndex = currentSpiralPointIndex + 1;
            if (nextSpiralPointIndex >= spiralOffsets.size() || nextSpiralPointIndex >= maxSpiralPoints)
            {
                if (m_isPeakSeeking && std::isfinite(m_peakSeekingBestPower))
                { // Exhausted during peak seeking
                    if (debug)
                    {
                        std::cout << "[" << algorithmName << "] (" << currentSlot
                                  << ") Spiral exhausted during peak seek. Triggering HALT."
                                  << std::endl;
                    }

                    triggerHalt = true;
                    if (triggerHalt && currentAction != AlgoAction::STOPPING)
                    {
                        goto issue_halt_now_standard; // Use goto carefully
                    }
                }
                else
                { // Exhausted, never met success condition
                    if (currentState != AlgoState::ERROR_STATUS)
                    {
                        std::cerr << "[" << algorithmName << "] (" << currentSlot
                                  << ") Error: Spiral exhausted. Last power " << shortTermEMA
                                  << std::endl;
                        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                        m_isPeakSeeking = false;
                    }
                }
                break;
            }
            else
            {
                currentSpiralPointIndex = nextSpiralPointIndex;
                if (debug)
                {
                    std::cout << "[" << algorithmName << "] (" << currentSlot
                              << ") Moving to next spiral point index: " << currentSpiralPointIndex
                              << std::endl;
                }
                issueStandardSpiralMoveCommand(engine, unitIndex, currentSlot, currentRotaryState);
            }
        }
        else if (!isCurrentlyMoving && (currentAction == AlgoAction::MOVING || currentAction == AlgoAction::STARTING) && !triggerHalt)
        {
            if (debug)
            {
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Warning: Unexpected stop. Resetting action." << std::endl;
            }
            setState(AlgoState::ALIGNMENT, AlgoAction::NONE);
        }

    issue_halt_now_standard:; // Label for goto jump
        if (triggerHalt && currentAction != AlgoAction::STOPPING)
        { // Re-check after potential goto
            if (debug)
            {
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Issuing HALT (goto path)." << std::endl;
            }

            try
            {
                engine->issueRotaryCommand(unitIndex, "halt 0");
                engine->issueRotaryCommand(unitIndex, "halt 1");
                setState(AlgoState::ALIGNMENT, AlgoAction::STOPPING);
            }
            catch (const std::exception &e)
            { /* handle error */
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                m_isPeakSeeking = false;
            }
        }
    }
    break; // End ALIGNMENT

    case AlgoState::PANIC_ALIGNMENT:
    {
        if (!filterInitialized || !std::isfinite(currentReferencePower))
        {
            if (currentState != AlgoState::ERROR_STATUS && debug)
            {
                std::cerr << "[" << algorithmName << "] (" << currentSlot
                          << ") PANIC_ALIGNMENT Error: Invalid filter/reference." << std::endl;
            }
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            break;
        }
        if (!rotaryEnabled || spiralOffsets.empty())
        {
            if (currentState != AlgoState::ERROR_STATUS && debug)
            {
                std::cerr << "[" << algorithmName << "] (" << currentSlot
                          << ") PANIC_ALIGNMENT Error: Rotary disabled or no panic spiral offsets."
                          << std::endl;
            }
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            break;
        }
        if (m_panic_move_command_issued && currentSpiralPointIndex != 0)
        { // After initial command, index should be 0
            if (currentState != AlgoState::ERROR_STATUS && debug)
            {
                std::cerr << "[" << algorithmName << "] (" << currentSlot
                          << ") PANIC_ALIGNMENT Error: Inconsistent state for initial panic move "
                             "(idx!=0)."
                          << std::endl;
            }
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            break;
        }

        if (isCurrentlyMoving)
        {
            if (currentAction == AlgoAction::STARTING || currentAction == AlgoAction::MOVING)
            {
                setState(AlgoState::PANIC_ALIGNMENT, AlgoAction::MOVING);
            }
        }
        else
        {
            if (currentAction == AlgoAction::MOVING || currentAction == AlgoAction::STARTING)
            {
                setState(AlgoState::PANIC_ALIGNMENT, AlgoAction::NONE);
            }
        }

        bool triggerHaltForPanicPointSuccess = false;

        // --- Logic for the first stop at the panic-predicted point (index 0) ---
        if (m_panic_move_command_issued && currentSpiralPointIndex == 0 && !isCurrentlyMoving && currentAction == AlgoAction::NONE)
        { // We've arrived at the panic-predicted point.

            if (!readingIsValid || !std::isfinite(shortTermEMA) || !filterInitialized)
            {
                // No valid power reading.
                // Wait for next valid power reading.
            }
            else
            {
                // Valid power reading. Now decide.
                double currentSuccessMargin = calculateDynamicSuccessMargin(currentSlot);
                bool successAtPanicPredictedPoint = (shortTermEMA >= (currentReferencePower - currentSuccessMargin));
                if (debug)
                {
                    std::cout << "[" << algorithmName << "] (PANIC_ALIGNMENT " << currentSlot
                              << ") At panic predicted point. Power: " << shortTermEMA
                              << ", Ref: " << currentReferencePower
                              << ", Margin: " << currentSuccessMargin
                              << ", Success: " << successAtPanicPredictedPoint << std::endl;
                }

                m_panic_move_command_issued = false; // Decision is being made based on valid data, consume the flag.

                if (successAtPanicPredictedPoint)
                {
                    if (debug)
                    {
                        std::cout << "[" << algorithmName << "] (" << currentSlot
                                  << ") Good signal AT predicted point. Triggering HALT."
                                  << std::endl;
                    }
                    triggerHaltForPanicPointSuccess = true;
                    // Proceed to halt logic later in this state case.
                }
                else
                {
                    // Valid reading, but signal NOT good at predicted point.
                    if (debug)
                    {
                        std::cout << "[" << algorithmName << "] (" << currentSlot
                                  << ") Signal NOT good (but valid reading) at predicted point. "
                                  << "Switching to STANDARD ALIGNMENT centered on this panic point."
                                  << std::endl;
                    }

                    generateStandardSpiralOffsets();
                    if (spiralOffsets.empty())
                    {
                        if (debug)
                            std::cerr
                                << "[" << algorithmName << "] (" << currentSlot
                                << ") PANIC_ALIGNMENT Error: Failed to generate standard spiral "
                                   "for fallback."
                                << std::endl;
                        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                    }
                    else
                    {
                        currentSpiralPointIndex = 0;
                        m_isPeakSeeking = false;
                        m_peakSeekingBestPower = -9999999;
                        m_peakSeekingBestIndex = -1;
                        setState(AlgoState::ALIGNMENT, AlgoAction::NONE);
                    }
                    try
                    {
                        engine->setAlgorithmData(unitIndex, currentSlot, getStatus());
                    }
                    catch (const std::exception &e)
                    {
                        if (debug)
                            std::cerr
                                << "[" << algorithmName << "] (" << currentSlot
                                << ") PANIC_ALIGNMENT Error setting algo data during fallback: "
                                << e.what() << std::endl;
                    }
                    return; // Exit processSlot for this cycle. ALIGNMENT state (or ERROR) takes over next.
                }
            }
        }

        if (!isCurrentlyMoving && currentAction == AlgoAction::STOPPING)
        {
            if (debug)
            {
                std::cout
                    << "[" << algorithmName << "] (PANIC_ALIGNMENT " << currentSlot
                    << ") Halt completed (from panic point success). Transitioning to REFERENCE."
                    << std::endl;
            }

            setState(AlgoState::REFERENCE, AlgoAction::NONE);
            filterInitialized = false;
            warmupCounter = 0;
            m_isPeakSeeking = false;
            break;
        }

        if (triggerHaltForPanicPointSuccess && currentAction != AlgoAction::STOPPING)
        {
            if (debug)
            {
                std::cout << "[" << algorithmName << "] (PANIC_ALIGNMENT " << currentSlot
                          << ") Issuing HALT (from panic point success)." << std::endl;
            }

            try
            {
                engine->issueRotaryCommand(unitIndex, "halt 0");
                engine->issueRotaryCommand(unitIndex, "halt 1");
                setState(AlgoState::PANIC_ALIGNMENT, AlgoAction::STOPPING);
            }
            catch (const std::exception &e)
            {
                if (debug)
                {
                    std::cerr << "[" << algorithmName << "] (" << currentSlot
                              << ") PANIC_ALIGNMENT Error issuing halt: " << e.what() << std::endl;
                }
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            }
            break;
        }

        if (!m_panic_move_command_issued && !triggerHaltForPanicPointSuccess && !isCurrentlyMoving && currentAction == AlgoAction::NONE)
        {
            // This condition implies we are past the initial panic point check (m_panic_move_command_issued is false),
            // AND we didn't decide to halt from that point (triggerHaltForPanicPointSuccess is false).
            // This means the panic point was not good, and we should have transitioned to ALIGNMENT state already.
            // If we reach here, something is unexpected.
            if (debug)
            {
                std::cout
                    << "[" << algorithmName << "] (PANIC_ALIGNMENT " << currentSlot
                    << ") Unexpectedly idle in PANIC_ALIGNMENT after initial point decision. "
                       "This should have transitioned to ALIGNMENT or halted. Forcing REFERENCE."
                    << std::endl;
            }
            setState(AlgoState::REFERENCE, AlgoAction::NONE);
            filterInitialized = false;
            warmupCounter = 0;
            break;
        }
    }
    break;

    case AlgoState::ERROR_STATUS:
    {
        m_isPeakSeeking = false;
        m_panic_move_command_issued = false;
        if (rotaryEnabled && isCurrentlyMoving && currentAction != AlgoAction::STOPPING && currentAction != AlgoAction::ERROR_ACTION)
        {
            engine->issueRotaryCommand(unitIndex, "halt 0");
            engine->issueRotaryCommand(unitIndex, "halt 1");
            setState(AlgoState::ERROR_STATUS, AlgoAction::STOPPING);
        }
        else if (!isCurrentlyMoving && currentAction == AlgoAction::STOPPING)
        {
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        }
        else
        {
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        }
    }
    break;

    default:
        if (currentState != AlgoState::ERROR_STATUS)
        {
            m_isPeakSeeking = false;
            m_panic_move_command_issued = false;
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        }
        break;
    } // End switch

    try
    {
        engine->setAlgorithmData(unitIndex, currentSlot, getStatus());
    }
    catch (const std::exception &e)
    {
        if (currentState != AlgoState::ERROR_STATUS)
        {
            std::cerr << "[" << algorithmName << "] Error setting algorithm data in engine (Slot "
                      << currentSlot << "): " << e.what() << std::endl;
        }
    }
}

void JustInTime_AP::startStandardAlignment(mobileTHzEngine *engine,
                                           size_t unitIndex,
                                           size_t currentSlot,
                                           const rotaryObject &currentRotary)
{
    if (debug)
    {
        std::cout << "[" << algorithmName << "] (" << currentSlot
                  << ") Starting STANDARD ALIGNMENT." << std::endl;
    }
    generateStandardSpiralOffsets(); // Generate spiral without {0,0} at start
    if (spiralOffsets.empty())
    {
        if (debug)
        {
            std::cerr << "[" << algorithmName
                      << "] No standard spiral offsets, cannot start alignment." << std::endl;
        }
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        return;
    }

    currentSpiralPointIndex = 0; // Start from the first actual offset
    m_isPeakSeeking = false;
    m_peakSeekingBestPower = -9999999;
    m_peakSeekingBestIndex = -1;
    m_panic_move_command_issued = false; // Not a panic move

    spiralCenterAz = currentRotary.azimuth.angle; // Center on current position
    spiralCenterAlt = currentRotary.altitude.angle;

    setState(AlgoState::ALIGNMENT, AlgoAction::NONE);
    issueStandardSpiralMoveCommand(engine, unitIndex, currentSlot, currentRotary);
}

void JustInTime_AP::generateStandardSpiralOffsets()
{
    spiralOffsets.clear();
    if (maxSpiralPoints <= 0)
    { /* ... */
        return;
    }
    double initialDiagonalStep = halfPowerBeamWidth / 2.0f;
    if (initialDiagonalStep <= 0.001f)
        initialDiagonalStep = 0.1f;
    double currentStep = initialDiagonalStep;
    int segmentsPerSide = 1;
    int totalUnitStepsGenerated = 0;
    const double invSqrt2 = static_cast<double>(M_SQRT1_2);

    while (totalUnitStepsGenerated < maxSpiralPoints && currentStep > 0.0001f)
    {
        double stepComponent = currentStep * invSqrt2;
        // Side 1: Up-Right (+Az, +Alt)
        int unitsThisSide1 = std::min(segmentsPerSide, maxSpiralPoints - totalUnitStepsGenerated);
        if (unitsThisSide1 > 0)
        {
            spiralOffsets.push_back(
                {unitsThisSide1 * stepComponent, unitsThisSide1 * stepComponent});
            totalUnitStepsGenerated += unitsThisSide1;
            if (totalUnitStepsGenerated >= maxSpiralPoints)
                break;
        }
        else if (totalUnitStepsGenerated >= maxSpiralPoints)
            break;
        // Side 2: Up-Left (-Az, +Alt)
        int unitsThisSide2 = std::min(segmentsPerSide, maxSpiralPoints - totalUnitStepsGenerated);
        if (unitsThisSide2 > 0)
        {
            spiralOffsets.push_back(
                {unitsThisSide2 * -stepComponent, unitsThisSide2 * stepComponent});
            totalUnitStepsGenerated += unitsThisSide2;
            if (totalUnitStepsGenerated >= maxSpiralPoints)
                break;
        }
        else if (totalUnitStepsGenerated >= maxSpiralPoints)
            break;
        segmentsPerSide++;
        // Side 3: Down-Left (-Az, -Alt)
        int unitsThisSide3 = std::min(segmentsPerSide, maxSpiralPoints - totalUnitStepsGenerated);
        if (unitsThisSide3 > 0)
        {
            spiralOffsets.push_back(
                {unitsThisSide3 * -stepComponent, unitsThisSide3 * -stepComponent});
            totalUnitStepsGenerated += unitsThisSide3;
            if (totalUnitStepsGenerated >= maxSpiralPoints)
                break;
        }
        else if (totalUnitStepsGenerated >= maxSpiralPoints)
            break;
        // Side 4: Down-Right (+Az, -Alt)
        int unitsThisSide4 = std::min(segmentsPerSide, maxSpiralPoints - totalUnitStepsGenerated);
        if (unitsThisSide4 > 0)
        {
            spiralOffsets.push_back(
                {unitsThisSide4 * stepComponent, unitsThisSide4 * -stepComponent});
            totalUnitStepsGenerated += unitsThisSide4;
            if (totalUnitStepsGenerated >= maxSpiralPoints)
                break;
        }
        else if (totalUnitStepsGenerated >= maxSpiralPoints)
            break;
        segmentsPerSide++;
        if (spiralGrowthFactor >= 1.0f)
        {
            currentStep *= spiralGrowthFactor;
        }
        else if (totalUnitStepsGenerated > 0)
            break;
        if (currentStep < 0.0001f)
            break;
    }
}

void JustInTime_AP::issueStandardSpiralMoveCommand(mobileTHzEngine *engine,
                                                   size_t unitIndex,
                                                   size_t currentSlot,
                                                   const rotaryObject &currentRotaryState)
{
    if (!engine)
        return;
    if (currentSpiralPointIndex < 0 || static_cast<size_t>(currentSpiralPointIndex) >= spiralOffsets.size())
    {
        /* ... error handling ... */ setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        return;
    }
    const auto &relativeStep = spiralOffsets[currentSpiralPointIndex];

    double relativeAzCmd = relativeStep.deltaAz;
    double relativeAltCmd = relativeStep.deltaAlt;

    std::stringstream ss_cmd;
    ss_cmd << "mdd 0 " << std::fixed << std::setprecision(3) << relativeAzCmd << " 1 " << std::fixed
           << std::setprecision(3) << relativeAltCmd;
    std::string moveCommand = ss_cmd.str();

    try
    {
        engine->issueRotaryCommand(unitIndex, moveCommand);
        if (debug)
        {
            std::cout << "[" << algorithmName << "] (" << currentSlot << ", Step "
                      << currentSpiralPointIndex << ") Issued cmd: '" << moveCommand
                      << "'. Spiral Center (Az/Alt): (" << spiralCenterAz << "/" << spiralCenterAlt
                      << ")" << std::endl;
        }
        setState(AlgoState::ALIGNMENT, AlgoAction::STARTING);
    }
    catch (const std::exception &e)
    { /* ... error handling ... */
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
    }
}

// --- Panic Alignment (Panic-Packet Triggered) ---
void JustInTime_AP::startPanicAlignment(mobileTHzEngine *engine,
                                        size_t unitIndex,
                                        size_t currentSlot,
                                        const rotaryObject &currentRotary)
{
    if (debug)
    {
        std::cout << "[" << algorithmName << "] (" << currentSlot << ") Starting PANIC ALIGNMENT."
                  << std::endl;
    }

    generatePanicSpiralOffsets(); // Generate spiral WITH {0,0} at start
    if (spiralOffsets.empty())
    {
        if (debug)
        {
            std::cerr << "[" << algorithmName
                      << "] No panic spiral offsets, cannot start alignment." << std::endl;
        }
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        return;
    }

    currentSpiralPointIndex = 0; // Start at index 0 (which is {0,0} offset for panic spiral)
    m_isPeakSeeking = false;
    m_peakSeekingBestPower = -9999999;
    m_peakSeekingBestIndex = -1;
    m_panic_move_command_issued = true; // Flag that the first move is to the predicted point

    spiralCenterAz = panic_predicted_target_az; // Center on the predicted panic point
    spiralCenterAlt = panic_predicted_target_alt;
    if (debug)
    {
        std::cout << "[" << algorithmName << "] (" << currentSlot
                  << ") PANIC ALIGNMENT: Centering on PANIC HINT Az=" << spiralCenterAz
                  << ", Alt=" << spiralCenterAlt << ". Commanding move to this point." << std::endl;
    }

    setState(AlgoState::PANIC_ALIGNMENT, AlgoAction::NONE);
    issuePanicSpiralMoveCommand(engine, unitIndex, currentSlot, currentRotary);
}

void JustInTime_AP::generatePanicSpiralOffsets()
{
    // THIS IS THE SPIRAL WITH {0,0} AS THE FIRST POINT
    spiralOffsets.clear();
    if (maxSpiralPoints <= 0)
        return;

    // Add the {0,0} point first for the initial "snap" attempt
    spiralOffsets.push_back({0.0f, 0.0f});
    int pointsAdded = 1;
    if (pointsAdded >= maxSpiralPoints)
        return;

    // Parameters for the rest of the spiral (similar to standard, but starts *after* {0,0})
    double step = halfPowerBeamWidth / 2.0f;
    if (step < 0.1f)
        step = 0.1f;

    int dx = 1, dy = 0; // Start East
    int segment_len = 1;
    double current_x_offset = 0; // Offsets are relative to the panic center
    double current_y_offset = 0;

    if (debug)
    {
        std::cout << "[" << algorithmName
                  << "] Generating PANIC spiral (includes {0,0} at start). Initial step: " << step
                  << std::endl;
    }

    while (pointsAdded < maxSpiralPoints)
    {
        for (int i = 0; i < 2; ++i)
        {
            for (int j = 0; j < segment_len; ++j)
            {
                if (pointsAdded >= maxSpiralPoints)
                    break;
                current_x_offset += dx * step;
                current_y_offset += dy * step;
                spiralOffsets.push_back({current_x_offset, current_y_offset});
                pointsAdded++;
            }
            if (pointsAdded >= maxSpiralPoints)
                break;
            int temp_dx = dx;
            dx = -dy;
            dy = temp_dx; // Rotate
        }
        if (pointsAdded >= maxSpiralPoints)
            break;
        segment_len++;
        if (spiralGrowthFactor > 1.0f)
        {
            step *= spiralGrowthFactor;
        }
    }
    if (spiralOffsets.size() <= 1 && maxSpiralPoints > 0 && debug)
    {
        std::cerr << "[" << algorithmName
                  << "] Warning: Failed to generate sufficient PANIC spiral steps." << std::endl;
    }
    else if (debug)
    {
        std::cout << "[" << algorithmName << "] Generated " << spiralOffsets.size()
                  << " PANIC spiral steps (includes {0,0})." << std::endl;
    }
}

void JustInTime_AP::issuePanicSpiralMoveCommand(mobileTHzEngine *engine,
                                                size_t unitIndex,
                                                size_t currentSlot,
                                                const rotaryObject &currentRotaryState)
{
    if (!engine)
        return;
    if (currentSpiralPointIndex < 0 || static_cast<size_t>(currentSpiralPointIndex) >= spiralOffsets.size())
    {
        /* ... error handling ... */ setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        return;
    }
    const auto &offset = spiralOffsets[currentSpiralPointIndex];
    // Target is absolute: panic-predicted spiral center + offset
    double targetAz = spiralCenterAz + offset.deltaAz;
    double targetAlt = spiralCenterAlt + offset.deltaAlt;
    targetAz = std::fmod(targetAz, 360.0f);
    if (targetAz < 0)
        targetAz += 360.0f;
    targetAlt = std::max(0.0, std::min(90.0, targetAlt));

    double relativeAzCmd = shortestAngleDiff(targetAz, currentRotaryState.azimuth.angle);
    double relativeAltCmd = targetAlt - currentRotaryState.altitude.angle;

    std::stringstream ss_cmd;
    ss_cmd << "mdd 0 " << std::fixed << std::setprecision(3) << relativeAzCmd << " 1 " << std::fixed
           << std::setprecision(3) << relativeAltCmd;
    std::string moveCommand = ss_cmd.str();

    try
    {
        engine->issueRotaryCommand(unitIndex, moveCommand);
        if (debug)
        {
            std::cout << "[" << algorithmName << "] (PANIC_ALIGNMENT " << currentSlot << ", Step "
                      << currentSpiralPointIndex << ") Issued cmd: '" << moveCommand << "'"
                      << ". Target Abs (Az/Alt): (" << targetAz << "/" << targetAlt
                      << "). Spiral Center (Az/Alt): (" << spiralCenterAz << "/" << spiralCenterAlt
                      << ")" << std::endl;
        }
        setState(AlgoState::PANIC_ALIGNMENT, AlgoAction::STARTING);
    }
    catch (const std::exception &e)
    { /* ... error handling ... */
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
    }
}

double JustInTime_AP::calculateDynamicSuccessMargin(size_t currentSlot) const
{
    if (lastSuccessfulAlignmentSlot == 0)
        return realignSuccessInitialMarginDb;
    double timeElapsedSeconds = static_cast<double>(currentSlot - lastSuccessfulAlignmentSlot) * slotTimeMicrosec / 1.0e6;
    double margin = realignSuccessInitialMarginDb + (timeElapsedSeconds * realignSuccessMarginIncreaseRateDbPerSec);
    return std::max(realignSuccessInitialMarginDb, std::min(realignSuccessMaxMarginDb, margin));
}
