#include "kpiClassifier.h"
#include "Codebase/Software/mobileTHzEngine/mobileTHzEngine.h"
#include "kpiResultsWriter.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <set>
#include <stdexcept>
#include <vector>

static bool debug = false;

namespace
{
    constexpr double PI = 3.14159265358979323846;
    constexpr double SPEED_OF_LIGHT = 299792458.0;
    constexpr double RAD_TO_DEG = 180.0 / PI;
    constexpr double DEG_TO_RAD = PI / 180.0;
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
            angles.y = std::copysign(PI / 2, sinp); // use 90 degrees if out of range
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

    // Quaternion math helpers
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
} // namespace

kpiClassifier::kpiClassifier() {}

void kpiClassifier::initialize(mobileTHzEngine *engine)
{
    try
    {
        engine_ = engine;
        if (!engine_)
        {
            throw std::invalid_argument("kpiClassifier: mobileTHzEngine pointer cannot be null.");
        }
        engine_mode_ = engine_->getConfig().engine_mode;

        // --- Set up UE tracking structures ---
        size_t num_units = engine_->getNumUnits();

        // 1. Explicitly clear the vectors.
        ueIsAlignedHistory_.clear();
        per_slot_kpis_.clear();
        ueRuntimeStates_.clear();
        ueIsSelectedForClassification_.clear();
        ue_total_noise_watts_.clear();

        // 2. Now resize. This will create fresh inner vectors with the correct new size.
        ueIsAlignedHistory_.resize(num_units, std::vector<bool>(engine_->getNumTimeSlots(), false));
        per_slot_kpis_.resize(engine_->getNumTimeSlots());
        ueRuntimeStates_.resize(num_units);
        ueIsSelectedForClassification_.resize(num_units, false);
        ue_total_noise_watts_.resize(num_units, 0.0);

        // 3. Reset all other stateful member variables
        primaryUeIndex_ = -1;
        ap_idx_for_kpi_calc_ = -1;
        m_last_ue_moving_slot = 0;
        classificationHasBeenPerformed_ = false;
        kpi_window1_ = {}; // Resets the accumulator struct

        const auto &engine_units = engine_->getAllEngineUnits();
        double N_thermal_watts = engine_->receiverThermalNoise_watts;

        // Identifying the primary UE for KPI calculations
        primaryUeIndex_ = -1;
        for (size_t i = 0; i < num_units; ++i)
        {
            if (engine_units[i].label.find("UE") != std::string::npos)
            {
                ueIsSelectedForClassification_[i] = true;
                if (primaryUeIndex_ == -1)
                {
                    primaryUeIndex_ = static_cast<int>(i);
                }
                // This initializes our single source of truth for noise
                if (engine_units[i].RxChain_enabled)
                {
                    ue_total_noise_watts_[i] = engine_units[i].RxChain_noise_factor_linear * N_thermal_watts;
                }
                else
                {
                    ue_total_noise_watts_[i] = 1.0 * N_thermal_watts;
                }
            }
        }

        ap_idx_for_kpi_calc_ = -1;
        for (size_t i = 0; i < num_units; ++i)
        {
            if ((engine_units[i].label.find("AP") != std::string::npos || engine_units[i].label.find("BS") != std::string::npos) && i != static_cast<size_t>(primaryUeIndex_))
            {
                ap_idx_for_kpi_calc_ = static_cast<int>(i);
                break;
            }
        }
        if (ap_idx_for_kpi_calc_ == -1 && num_units == 2 && primaryUeIndex_ != -1)
        {
            ap_idx_for_kpi_calc_ = (primaryUeIndex_ == 0) ? 1 : 0;
        }

        if (ap_idx_for_kpi_calc_ != -1 && primaryUeIndex_ != -1)
        {
            const auto &ap_config = engine_->getEngineUnit(ap_idx_for_kpi_calc_);
            const auto &ue_config = engine_->getEngineUnit(primaryUeIndex_);
            ap_tx_power_watts_ = ap_config.TxAntenna_transmitPower_watts;

            if (ap_config.radiationPatterns_maxGain_linear.count(
                    0))
            { // DEFAULT TO RADIATIONPATTERNID: 0
                ap_max_gain_linear_ = ap_config.radiationPatterns_maxGain_linear.at(0);
            }

            if (ue_config.radiationPatterns_maxGain_linear.count(
                    0))
            { // DEFAULT TO RADIATIONPATTERNID: 0
                ue_max_gain_linear_ = ue_config.radiationPatterns_maxGain_linear.at(0);
            }
        }
        else
        {
            std::cerr << "[Classifier] Warning: Could not find primary AP/UE pair for KPI calcs."
                      << std::endl;
        }

        // --- Set remaining classification parameters from config ---
        paramMinSnrLinearForAvailability_ = std::pow(10.0,
                                                     engine_->getConfig().link_min_snr_db / 10.0);
        paramCarrierFrequencyHz_ = static_cast<double>(engine_->carrierFrequency);
        if (paramCarrierFrequencyHz_ <= 0)
        {
            paramCarrierFrequencyHz_ = 100e9; // Safe default
        }
        slotTimeMicrosec_ = engine_->getConfig().engine_slot_time_microsec;
        if (slotTimeMicrosec_ <= 0)
        {
            slotTimeMicrosec_ = 1000.0; // Safe default
        }

        for (auto &state : ueRuntimeStates_)
        {
            state.resetForNewUeProcessing();
        }

        kpi_window1_ = {};
        classificationHasBeenPerformed_ = false;
        m_last_ue_moving_slot = 0;

        // --- Pre-calculate invariants for processSlot KPI calculations ---
        paramCarrierFrequencyHz_ = static_cast<double>(engine_->carrierFrequency);
        if (paramCarrierFrequencyHz_ > 1e-6)
        {
            kpi_lambda_m_ = SPEED_OF_LIGHT / paramCarrierFrequencyHz_;

            // Pre-calculate the part of the path loss formula that doesn't change.
            const double term = kpi_lambda_m_ / (4.0 * PI);
            kpi_friis_wavelength_factor_ = term * term;
        }

        // Precalculate constants for Shannon-Hartley capacity calculations
        kpi_B_Hz_ = static_cast<double>(engine_->bandwidthFrequency);
        kpi_slot_duration_sec_ = engine_->getConfig().engine_slot_time_microsec / 1e6;
        kpi_inv_log2_ = 1.0 / std::log(2.0);
        kpi_shannon_hartley_factor_ = kpi_B_Hz_ * kpi_slot_duration_sec_ * kpi_inv_log2_;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Classifier] Initialization failed: " << e.what() << std::endl;
        throw;
    }
}

void kpiClassifier::processSlot(size_t unit_idx, size_t time_idx)
{
    if (!engine_ || unit_idx >= ueRuntimeStates_.size() || !isUnitSelected(unit_idx))
    {
        return;
    }

    size_t firstMotionSlot = (engine_->firstMotionSlot >= 0)
                                 ? static_cast<size_t>(engine_->firstMotionSlot)
                                 : 0;

    UeClassificationRuntimeState &ueRuntimeState = ueRuntimeStates_[unit_idx];

    // --- 1. Get Power Data and Classify State ---
    double current_power_watts = 0.0;
    if (time_idx >= firstMotionSlot)
    {
        try
        {
            if (engine_->getEngineUnit(unit_idx).RxChain_enabled)
            {
                current_power_watts = engine_->getRxChainData(unit_idx, time_idx).power_watts;
            }
        }
        catch (...)
        { // Errors logged by engine, proceed with no power
        }

        const double noise_watts = ue_total_noise_watts_[unit_idx];

        // Step 1: Estimate the signal power by subtracting the noise floor from the total power.
        // Use std::max to prevent negative signal power if total power dips below the noise floor.
        const double signal_power_watts = std::max(0.0, current_power_watts - noise_watts);

        // Step 2: Calculate the true Signal-to-Noise Ratio (S/N).
        const double current_snr_linear = signal_power_watts / noise_watts;

        // Step 3: Classify the link state based on this more accurate SNR.
        ueRuntimeState.currentClass = (current_snr_linear >= paramMinSnrLinearForAvailability_)
                                          ? classifierClass::ALIGNED
                                          : classifierClass::MISALIGNED;
    }
    else
    {
        ueRuntimeState.currentClass = classifierClass::INITIALIZING;
    }

    // --- 2. Finalize and Store Result ---
    ueIsAlignedHistory_[unit_idx][time_idx] = (ueRuntimeState.currentClass == classifierClass::ALIGNED);
    try
    {
        engine_->setClassifierData(unit_idx, time_idx, ueRuntimeState.currentClass);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error setting classifier data for unit " << unit_idx << ", slot " << time_idx
                  << ": " << e.what() << std::endl;
        return; // Exit if we can't store the classification
    }

    // --- 3. Incremental KPI Calculation (only for primary UE after motion starts) ---
    if (unit_idx == primaryUeIndex_ && time_idx >= firstMotionSlot)
    {
        per_slot_kpis_[time_idx] = calculateMetricsForSlot(time_idx);
    }

    // This check is cheap and ensures we only access rotary data when needed.
    if (engine_->getEngineUnit(unit_idx).Rotary_enabled)
    {
        // This memory access is sequential and cache-friendly.
        const rotaryObject rotary_data = engine_->getRotaryData(unit_idx, time_idx);
        if (rotary_data.isMoving != 0.0)
        {
            m_last_ue_moving_slot = time_idx;
        }
    }
}

SlotKpiMetrics kpiClassifier::calculateMetricsForSlot(size_t time_idx)
{
    SlotKpiMetrics metrics = {}; // Initialize to zero
    size_t ue_idx = static_cast<size_t>(primaryUeIndex_);
    const double primary_ue_noise_watts = ue_total_noise_watts_[ue_idx];

    // --- 1. Get Power Data (COMMON to both modes) ---
    const rxChainObject rx_power_info = engine_->getRxChainData(ue_idx, time_idx);

    metrics.actual_total_power_watts = rx_power_info.power_watts;
    const double actual_signal_power_watts = std::max(0.0,
                                                      metrics.actual_total_power_watts - primary_ue_noise_watts);
    metrics.snr_linear = (primary_ue_noise_watts > 0)
                             ? actual_signal_power_watts / primary_ue_noise_watts
                             : 0.0;
    metrics.actual_slot_bits = calculateCapacityBits(metrics.snr_linear);
    metrics.is_snr_good = (metrics.snr_linear >= paramMinSnrLinearForAvailability_);
    metrics.is_aligned = ueIsAlignedHistory_[ue_idx][time_idx];

    // --- 2. Get Simulation-Specific Data ---
    if (engine_mode_ == "SIMULATION")
    {
        const environmentObject ue_env = engine_->getEnvironmentData(ue_idx, time_idx);

        // Velocity Calculation
        metrics.trans_vel = std::sqrt(ue_env.velocity.x * ue_env.velocity.x + ue_env.velocity.y * ue_env.velocity.y + ue_env.velocity.z * ue_env.velocity.z);
        metrics.rot_vel = std::sqrt(ue_env.angular_velocity.x * ue_env.angular_velocity.x + ue_env.angular_velocity.y * ue_env.angular_velocity.y + ue_env.angular_velocity.z * ue_env.angular_velocity.z);
    }
    // In EXPERIMENTAL mode, trans_vel and rot_vel will remain 0, which is correct.

    // --- 3. Return the computed metrics ---
    return metrics;
}

kpiClassifier::~kpiClassifier() {}

bool kpiClassifier::isUnitSelected(size_t unit_idx) const
{
    if (unit_idx < ueIsSelectedForClassification_.size())
    {
        return ueIsSelectedForClassification_[unit_idx];
    }
    return false;
}

// --- calculateAndSaveKPIs ---
void kpiClassifier::calculateFinalKPIsAndSave(const std::string &name)
{
    try
    {
        if (!classificationHasBeenPerformed_)
        {
            std::cerr << "KPI Calculation Error: Classification has not been finalized."
                      << std::endl;
            return;
        }

        json resultsJson;
        resultsJson["kpi_report_name"] = name;

        const ConfigFile &config_ = engine_->getConfig();
        const double slot_duration_sec = static_cast<double>(config_.engine_slot_time_microsec) / 1e6;
        size_t num_total_slots = engine_->getNumTimeSlots();

        // Basic simulation parameters
        std::set<std::string> algo_labels_and_names;
        for (const auto &unit_conf : engine_->getAllEngineUnits())
        {
            if (!unit_conf.algorithm.empty() && unit_conf.algorithm != "none")
            {
                algo_labels_and_names.insert(unit_conf.label + "-" + unit_conf.algorithm);
            }
        }
        std::string algo_combo_name;
        for (const auto &an : algo_labels_and_names)
        {
            if (!algo_combo_name.empty())
                algo_combo_name += "_&_";
            algo_combo_name += an;
        }
        if (algo_combo_name.empty())
            algo_combo_name = "N/A_Algos";
        resultsJson["algorithm_combination"] = algo_combo_name;
        resultsJson["config_source_param_file"] = config_.paramConfig_name;
        resultsJson["config_source_position_file"] = config_.preset_position_enabled
                                                         ? config_.positionConfig_name
                                                         : "none";
        resultsJson["config_source_antenna_file"] = config_.antennaConfig_name.empty()
                                                        ? "none"
                                                        : config_.antennaConfig_name;

        // --- Overall Theoretical Max Capacity ---
        const double packet_overhead_us = 5.0;
        const size_t packet_opportunity_cost_slots = static_cast<size_t>(
            std::ceil(packet_overhead_us / config_.engine_slot_time_microsec));
        std::set<size_t> packet_busy_slots;
        const auto &packet_layer = engine_->getPacketLayer();
        for (const auto &packet : packet_layer)
        {
            size_t start_slot = static_cast<size_t>(packet.transmit_start_slot);
            size_t end_slot = start_slot + packet_opportunity_cost_slots;
            for (size_t s = start_slot; s < end_slot && s < num_total_slots; ++s)
            {
                packet_busy_slots.insert(s);
            }
        }
        resultsJson["packet_opportunity_cost_slots_per_packet"] = packet_opportunity_cost_slots;
        resultsJson["total_slots_blocked_by_packets"] = packet_busy_slots.size();

        // --- Primary UE KPIs ---
        if (primaryUeIndex_ == -1 || static_cast<size_t>(primaryUeIndex_) >= engine_->getNumUnits() || num_total_slots == 0)
        {
            resultsJson["primary_ue_kpis"] = "No primary UE identified, or no simulation slots.";
        }
        else
        {
            size_t ue_idx = static_cast<size_t>(primaryUeIndex_);
            json ue_kpi_data;
            ue_kpi_data["primary_ue_label"] = engine_->getEngineUnit(ue_idx).label;
            const double primary_ue_noise_watts = ue_total_noise_watts_[ue_idx];

            const engineUnit *ue_config_ptr = &engine_->getEngineUnit(ue_idx);

            // --- Define Core Time Markers ---
            int firstMotionSlot_int = engine_->firstMotionSlot;
            int lastMotionSlot_engine_int = engine_->lastMotionSlot;

            size_t firstMotionSlot = (firstMotionSlot_int >= 0)
                                         ? static_cast<size_t>(firstMotionSlot_int)
                                         : 0;
            size_t end_of_scenario_slot = (num_total_slots > 0) ? num_total_slots - 1 : 0;
            size_t lastMotionSlot = (lastMotionSlot_engine_int >= 0)
                                        ? static_cast<size_t>(lastMotionSlot_engine_int)
                                        : end_of_scenario_slot;
            if (lastMotionSlot < firstMotionSlot && num_total_slots > 0)
                lastMotionSlot = end_of_scenario_slot;
            lastMotionSlot = std::min(lastMotionSlot, end_of_scenario_slot);

            resultsJson["simulation_time_markers_sec"] = {{"first_motion_time_s", firstMotionSlot * slot_duration_sec},
                                                          {"last_motion_time_engine_s", lastMotionSlot * slot_duration_sec},
                                                          {"total_scenario_duration_s", end_of_scenario_slot * slot_duration_sec}};
            resultsJson["slot_duration_s"] = slot_duration_sec;

            // --- Link Parameters for Capacity ---
            double B_Hz = static_cast<double>(engine_->bandwidthFrequency);
            double N_thermal_watts = engine_->receiverThermalNoise_watts;
            double ue_noise_figure_db = linearToDbi(ue_config_ptr->RxChain_noise_factor_linear);

            resultsJson["link_parameters"] = {{"bandwidth_MHz", B_Hz / 1e6},
                                              {"engine_thermal_noise_dBm", wattsToDbm(N_thermal_watts)},
                                              {"ue_rx_chain_noise_figure_db",
                                               (primaryUeIndex_ != -1 && ue_config_ptr->RxChain_enabled)
                                                   ? json(ue_noise_figure_db)
                                                   : json("N/A or RxChain disabled")},
                                              {"total_effective_noise_power_for_ue_dBm", wattsToDbm(primary_ue_noise_watts)}};

            // --- Find the last successful realignment (rotary stopped AND link is good) ---
            size_t last_realign_transition_slot = end_of_scenario_slot + 1; // Default to "not found"

            // Step 1: Find the last slot the UE rotary was moving.
            size_t last_moving_slot = m_last_ue_moving_slot;

            // Step 2: Determine the candidate slot for when realignment completed.
            // This is the first slot *after* the last detected movement.
            size_t candidate_slot = 0;
            if (last_moving_slot == end_of_scenario_slot)
            {
                // Case A: Still moving at the end. No valid stop event occurred.
                candidate_slot = 0; // Mark as invalid
            }
            else if (last_moving_slot > 0)
            {
                // Case B: Moved and then stopped. The event is the slot immediately following.
                candidate_slot = last_moving_slot + 1;
            }
            else
            {
                // Case C: Never moved after the main mobility phase began.
                // The "realignment" attempt is at the very start of the phase.
                candidate_slot = firstMotionSlot;
            }

            // Step 3: Validate the candidate slot by checking if the link is above the threshold.
            if (candidate_slot > 0 && candidate_slot <= end_of_scenario_slot)
            {
                double p_watts_at_stop = engine_->getRxChainData(ue_idx, candidate_slot).power_watts;
                double snr_at_stop = (primary_ue_noise_watts > 0)
                                         ? p_watts_at_stop / primary_ue_noise_watts
                                         : 0.0;

                bool is_link_good_at_stop = snr_at_stop >= paramMinSnrLinearForAvailability_;

                if (is_link_good_at_stop)
                {
                    // SUCCESS: The rotary stopped and the link was good. This is our slot.
                    last_realign_transition_slot = candidate_slot;
                }
                // If the link is not good at this point, we consider it a failure,
                // and last_realign_transition_slot keeps its default "not found" value.
            }

            // --- Calculate and save the SNR at the successful realignment slot, if it was found ---
            double last_alignment_snr_db_value = -999.0;
            if (last_realign_transition_slot <= end_of_scenario_slot)
            {
                // Success: A valid stop slot with a good link was found.
                ue_kpi_data["last_alignment_transition_slot"] = last_realign_transition_slot;

                double p_watts = engine_->getRxChainData(ue_idx, last_realign_transition_slot)
                                     .power_watts;
                double snr_linear = (primary_ue_noise_watts > 0) ? p_watts / primary_ue_noise_watts
                                                                 : 0.0;
                if (snr_linear > 0)
                {
                    last_alignment_snr_db_value = 10.0 * std::log10(snr_linear);
                }
                ue_kpi_data["last_alignment_snr_db"] = last_alignment_snr_db_value;
            }
            else
            {
                // Failure: No successful realignment event (stop + good link) was found.
                ue_kpi_data["last_alignment_transition_slot"] = nullptr;
                ue_kpi_data["last_alignment_snr_db"] = nullptr;
            }

            size_t window4_start_slot = firstMotionSlot;
            size_t window4_end_slot = (last_realign_transition_slot <= end_of_scenario_slot && last_realign_transition_slot > firstMotionSlot)
                                          ? last_realign_transition_slot // End at successful realignment
                                          : end_of_scenario_slot;        // <-- Default to the end of the entire scenario

            // --- Populate JSON with finalized window stats ---
            ue_kpi_data["window1_full_scenario_post_init"] = finalizeWindowStats(firstMotionSlot, end_of_scenario_slot);
            ue_kpi_data["window4_realignment_phase"] = finalizeWindowStats(window4_start_slot,
                                                                           window4_end_slot);

            // FIND PEAK ACCELERATIONS FOR EXPERIMENTAL MODE
            if (engine_mode_ == "EXPERIMENTAL")
            {
                // Initialize peak trackers for this calculation run.
                // These can be local variables now, no need for member variables.
                Position peak_acceleration{};
                Position peak_angular_velocity{};
                Position peak_angular_acceleration{};

                // Keep track of the previous IMU state to calculate derivatives
                imuObject prev_imu_data = engine_->getIMUData(ue_idx, 0);

                size_t num_slots = engine_->getNumTimeSlots();
                const double slot_duration_sec = static_cast<double>(config_.engine_slot_time_microsec) / 1e6;

                for (size_t slot = 1; slot < num_slots; ++slot)
                { // Start from slot 1
                    imuObject current_imu_data = engine_->getIMUData(ue_idx, slot);

                    // --- 1. Calculate DERIVED Angular Acceleration ---
                    Position current_angular_acceleration{};
                    if (slot_duration_sec > 1e-9)
                    { // Avoid division by zero
                        current_angular_acceleration.x = (current_imu_data.angular_velocity.x - prev_imu_data.angular_velocity.x) / slot_duration_sec;
                        current_angular_acceleration.y = (current_imu_data.angular_velocity.y - prev_imu_data.angular_velocity.y) / slot_duration_sec;
                        current_angular_acceleration.z = (current_imu_data.angular_velocity.z - prev_imu_data.angular_velocity.z) / slot_duration_sec;
                    }

                    // --- 2. Check and update peak values based on magnitude ---

                    // Peak Translational Acceleration
                    double current_accel_mag_sq = current_imu_data.acceleration.x * current_imu_data.acceleration.x +
                                                  current_imu_data.acceleration.y * current_imu_data.acceleration.y +
                                                  current_imu_data.acceleration.z * current_imu_data.acceleration.z;
                    double peak_accel_mag_sq = peak_acceleration.x * peak_acceleration.x +
                                               peak_acceleration.y * peak_acceleration.y +
                                               peak_acceleration.z * peak_acceleration.z;
                    if (current_accel_mag_sq > peak_accel_mag_sq)
                    {
                        peak_acceleration = current_imu_data.acceleration;
                    }

                    // Peak Angular Velocity
                    double current_ang_vel_mag_sq = current_imu_data.angular_velocity.x * current_imu_data.angular_velocity.x +
                                                    current_imu_data.angular_velocity.y * current_imu_data.angular_velocity.y +
                                                    current_imu_data.angular_velocity.z * current_imu_data.angular_velocity.z;
                    double peak_ang_vel_mag_sq = peak_angular_velocity.x * peak_angular_velocity.x +
                                                 peak_angular_velocity.y * peak_angular_velocity.y +
                                                 peak_angular_velocity.z * peak_angular_velocity.z;
                    if (current_ang_vel_mag_sq > peak_ang_vel_mag_sq)
                    {
                        peak_angular_velocity = current_imu_data.angular_velocity;
                    }

                    // Peak Angular Acceleration
                    double current_ang_accel_mag_sq = current_angular_acceleration.x * current_angular_acceleration.x +
                                                      current_angular_acceleration.y * current_angular_acceleration.y +
                                                      current_angular_acceleration.z * current_angular_acceleration.z;
                    double peak_ang_accel_mag_sq = peak_angular_acceleration.x * peak_angular_acceleration.x +
                                                   peak_angular_acceleration.y * peak_angular_acceleration.y +
                                                   peak_angular_acceleration.z * peak_angular_acceleration.z;
                    if (current_ang_accel_mag_sq > peak_ang_accel_mag_sq)
                    {
                        peak_angular_acceleration = current_angular_acceleration;
                    }

                    // --- 3. Update previous state for the next iteration ---
                    prev_imu_data = current_imu_data;
                } // End of slot loop

                // Add the final peak results to the JSON report
                ue_kpi_data["peak_values"] = {
                    {"acceleration_mps2", {{"x", peak_acceleration.x}, {"y", peak_acceleration.y}, {"z", peak_acceleration.z}}},
                    {"angular_velocity_radps", {{"x", peak_angular_velocity.x}, {"y", peak_angular_velocity.y}, {"z", peak_angular_velocity.z}}},
                    {"angular_acceleration_radps2", {{"x", peak_angular_acceleration.x}, {"y", peak_angular_acceleration.y}, {"z", peak_angular_acceleration.z}}}};
            }

            resultsJson["primary_ue_kpis"] = ue_kpi_data;
        }

        // --- Save JSON to File ---
        try
        {
            std::filesystem::path resultsDirPath(RESULTS_DIR);

            // Construct filename from the base name passed to the function
            std::filesystem::path filePath = resultsDirPath / name;

            if (filePath.extension() != ".json")
            {
                filePath += ".json";
            }

            // Serialize the JSON object to a string with 4-space indentation
            std::string jsonContent = resultsJson.dump(4);

            // Add the write task to the non-blocking queue
            KPIResultsWriter::getInstance().addWriteTask(filePath.string(), jsonContent);

            if (debug)
            {
                std::cout << "KPIs queued for writing to: "
                          << std::filesystem::absolute(filePath).string() << std::endl;
            }
        }
        catch (const json::exception &e)
        {
            // Handle errors during JSON serialization
            std::cerr << "JSON error during KPI serialization: " << e.what() << std::endl;
        }
        catch (const std::exception &e)
        {
            // Handle other errors (e.g., from filesystem path construction)
            std::cerr << "Standard exception while preparing KPI data for writing: " << e.what()
                      << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "KPI Calculation Error: " << e.what() << std::endl;
        return;
    }
}

void kpiClassifier::finalizeClassification()
{
    const double packet_overhead_us = 5.0;
    const size_t packet_opportunity_cost_slots = static_cast<size_t>(
        std::ceil(packet_overhead_us / engine_->getConfig().engine_slot_time_microsec));

    size_t firstMotionSlot = (engine_->firstMotionSlot >= 0)
                                 ? static_cast<size_t>(engine_->firstMotionSlot)
                                 : 0;
    size_t lastMotionSlot = (engine_->lastMotionSlot >= 0)
                                ? static_cast<size_t>(engine_->lastMotionSlot)
                                : engine_->getNumTimeSlots() - 1;

    // Exit if there's no primary UE to process
    if (primaryUeIndex_ == -1)
    {
        classificationHasBeenPerformed_ = true;
        return;
    }

    // Get the noise from our single source of truth
    const double primary_ue_noise_watts = ue_total_noise_watts_[primaryUeIndex_];

    for (const auto &packet : engine_->getPacketLayer())
    {
        if (static_cast<int>(packet.unit_index) != primaryUeIndex_)
            continue;

        size_t start_slot = static_cast<size_t>(packet.transmit_start_slot);
        if (start_slot < firstMotionSlot)
            continue;

        double lost_bits = 0;
        if (packet_opportunity_cost_slots > 0)
        {
            for (size_t s = start_slot; s < start_slot + packet_opportunity_cost_slots; ++s)
            {
                if (s >= engine_->getNumTimeSlots())
                    break;

                double p_watts = engine_->getRxChainData(primaryUeIndex_, s).power_watts;
                double s_watts = std::max(0.0, p_watts - primary_ue_noise_watts);
                const double snr_linear = (primary_ue_noise_watts > 0)
                                              ? s_watts / primary_ue_noise_watts
                                              : 0.0;
                lost_bits += calculateCapacityBits(snr_linear);
            }
        }

        // Add lost bits to the correct windows
        kpi_window1_.overhead_bits += lost_bits;
    }

    classificationHasBeenPerformed_ = true;
}

inline double kpiClassifier::calculateCapacityBits(double snr_linear) const
{
    return (kpi_B_Hz_ * std::log1p(snr_linear) * kpi_inv_log2_) * kpi_slot_duration_sec_;
}

json kpiClassifier::finalizeWindowStats(size_t window_start_slot, size_t window_end_slot) const
{
    // --- Guard Clause: Return null if the window is invalid or empty ---
    if (window_end_slot < window_start_slot)
    {
        return json(nullptr);
    }

    // --- Step 1: Accumulate stats from the pre-calculated per-slot data ---
    KpiWindowAccumulator kpi_data = {}; // Temporary accumulator for this specific window

    for (size_t ts = window_start_slot; ts <= window_end_slot; ++ts)
    {
        const auto &metrics = per_slot_kpis_[ts];

        kpi_data.throughput_sample_count++;
        kpi_data.sum_of_bits += metrics.actual_slot_bits;
        kpi_data.sum_of_squares_of_bits += (metrics.actual_slot_bits * metrics.actual_slot_bits);
        kpi_data.total_bits += metrics.actual_slot_bits;
        kpi_data.theoretical_bits += metrics.theo_slot_bits;

        if (metrics.is_aligned)
        {
            kpi_data.aligned_slots++;
        }
        else
        {
            kpi_data.outage_slots++;
        }

        if (metrics.is_snr_good)
            kpi_data.good_snr_slots++;
        if (metrics.trans_vel > kpi_data.peak_translational_velocity)
            kpi_data.peak_translational_velocity = metrics.trans_vel;
        if (metrics.rot_vel > kpi_data.peak_rotational_velocity)
            kpi_data.peak_rotational_velocity = metrics.rot_vel;

        kpi_data.sum_of_power += metrics.actual_total_power_watts;
        kpi_data.sum_of_squares_of_power += (metrics.actual_total_power_watts * metrics.actual_total_power_watts);

        kpi_data.sum_of_trans_vel += metrics.trans_vel;
        kpi_data.sum_of_squares_of_trans_vel += (metrics.trans_vel * metrics.trans_vel);
        kpi_data.sum_of_rot_vel += metrics.rot_vel;
        kpi_data.sum_of_squares_of_rot_vel += (metrics.rot_vel * metrics.rot_vel);

        kpi_data.sum_of_theo_bits += metrics.theo_slot_bits;
        kpi_data.sum_of_squares_of_theo_bits += (metrics.theo_slot_bits * metrics.theo_slot_bits);
        kpi_data.sum_of_no_align_bits += metrics.no_align_slot_bits;
        kpi_data.sum_of_squares_of_no_align_bits += (metrics.no_align_slot_bits * metrics.no_align_slot_bits);
        kpi_data.sum_of_snr += metrics.snr_linear;
        kpi_data.sum_of_squares_of_snr += (metrics.snr_linear * metrics.snr_linear);
        kpi_data.snr_sample_count++;
    }

    // Account for overhead bits calculated during finalizeClassification()
    kpi_data.overhead_bits = kpi_window1_.overhead_bits;

    // --- Step 2: Finalize statistics and build the JSON object ---
    json stats;
    const long total_slots = (window_end_slot - window_start_slot + 1);
    const double slot_duration_sec = engine_->getConfig().engine_slot_time_microsec / 1e6;
    const long long sample_count = kpi_data.throughput_sample_count;

    // --- Helper lambda for calculating statistics ---
    auto calculate_stats = [](long long count, double sum, double sum_sq) -> json
    {
        if (count < 2)
        {
            return {{"mean", (count == 1) ? sum : 0.0},
                    {"std_dev", 0.0},
                    {"variance", 0.0},
                    {"coeff_of_variation", 0.0}};
        }
        json result;
        double d_count = static_cast<double>(count);
        double mean = sum / d_count;
        double variance = (sum_sq / d_count) - (mean * mean);
        variance = std::max(0.0, variance);
        double std_dev = std::sqrt(variance);
        result["mean"] = mean;
        result["std_dev"] = std_dev;
        result["variance"] = variance;
        if (std::abs(mean) > 1e-9)
        {
            result["coeff_of_variation"] = std_dev / mean;
        }
        else
        {
            result["coeff_of_variation"] = 0.0;
        }
        return result;
    };

    // --- Helper lambda to scale throughput stats to Gbps ---
    auto scale_throughput_stats = [&](const json &raw_stats) -> json
    {
        json scaled_stats;
        double mean_bits = raw_stats.value("mean", 0.0);
        scaled_stats["mean"] = (mean_bits / slot_duration_sec) / 1e9;
        scaled_stats["variance"] = (raw_stats.value("variance", 0.0) / (slot_duration_sec * slot_duration_sec)) / 1e18;
        scaled_stats["std_dev"] = (raw_stats.value("std_dev", 0.0) / slot_duration_sec) / 1e9;
        scaled_stats["coeff_of_variation"] = raw_stats.value("coeff_of_variation", 0.0);
        return scaled_stats;
    };

    // --- Results ---
    json exp;
    double net_actual_bits = kpi_data.total_bits - kpi_data.overhead_bits;
    exp["total_outage_time_s"] = static_cast<double>(kpi_data.outage_slots) * slot_duration_sec;
    exp["total_outage_percent"] = static_cast<double>(kpi_data.outage_slots) / total_slots * 100.0;
    json throughput_raw_stats = calculate_stats(sample_count,
                                                kpi_data.sum_of_bits,
                                                kpi_data.sum_of_squares_of_bits);
    exp["throughput_gbps_stats"] = scale_throughput_stats(throughput_raw_stats);

    double cv_bits = throughput_raw_stats.value("coeff_of_variation", 0.0);
    double mean_bits = throughput_raw_stats.value("mean", 0.0);
    if (cv_bits > 1e-9)
    {
        exp["throughput_stability_index"] = 1.0 / cv_bits;
    }
    else
    {
        exp["throughput_stability_index"] = (mean_bits > 1e-12) ? 1e9 : 0.0;
    }

    // Convert SNR linear stats to dB stats
    json snr_linear_stats = calculate_stats(kpi_data.snr_sample_count,
                                            kpi_data.sum_of_snr,
                                            kpi_data.sum_of_squares_of_snr);
    json snr_db_stats;
    double mean_snr_linear = snr_linear_stats["mean"].get<double>();

    // Convert the linear MEAN to dB
    snr_db_stats["mean"] = (mean_snr_linear > 0) ? (10.0 * std::log10(mean_snr_linear)) : 0.0;

    if (snr_linear_stats["std_dev"].get<double>() > 1e-30 && mean_snr_linear > 1e-30)
    {
        double std_dev_snr_linear = snr_linear_stats["std_dev"].get<double>();

        // Using error propagation to estimate std_dev in dB
        double std_dev_snr_db = (10.0 / std::log(10.0)) * (std_dev_snr_linear / mean_snr_linear);
        double mean_snr_db = snr_db_stats["mean"].get<double>();

        snr_db_stats["std_dev"] = std_dev_snr_db;
        snr_db_stats["variance"] = std_dev_snr_db * std_dev_snr_db;
        snr_db_stats["coeff_of_variation"] = (std::abs(mean_snr_db) > 1e-9)
                                                 ? (std_dev_snr_db / mean_snr_db)
                                                 : 0.0;
    }
    else
    {
        std::cerr << "SNR stats: std_dev or mean too small, setting std_dev and "
                     "coeff_of_variation to 0."
                  << std::endl;
        snr_db_stats["std_dev"] = 0.0;
        snr_db_stats["variance"] = 0.0;
        snr_db_stats["coeff_of_variation"] = 0.0;
    }
    exp["snr_db_stats"] = snr_db_stats;

    // --- POWER STATS CALCULATION ---
    json power_linear_stats = calculate_stats(sample_count,
                                              kpi_data.sum_of_power,
                                              kpi_data.sum_of_squares_of_power);
    json power_dbm_stats;
    double mean_power_watts = power_linear_stats["mean"].get<double>();

    // Convert the linear MEAN (Watts) to dBm
    power_dbm_stats["mean"] = wattsToDbm(mean_power_watts);

    if (power_linear_stats["std_dev"].get<double>() > 1e-30 && mean_power_watts > 1e-30)
    {
        double std_dev_power_linear = power_linear_stats["std_dev"].get<double>();

        // Using error propagation to estimate std_dev in dBm
        double std_dev_power_dbm = (10.0 / std::log(10.0)) * (std_dev_power_linear / mean_power_watts);
        double mean_power_dbm = power_dbm_stats["mean"].get<double>();

        power_dbm_stats["std_dev"] = std_dev_power_dbm;
        power_dbm_stats["variance"] = std_dev_power_dbm * std_dev_power_dbm;
        power_dbm_stats["coeff_of_variation"] = (std::abs(mean_power_dbm) > 1e-9)
                                                    ? (std_dev_power_dbm / mean_power_dbm)
                                                    : 0.0;
    }
    else
    {
        std::cerr << "Power stats: std_dev or mean too small, setting std_dev and "
                     "coeff_of_variation to 0."
                  << std::endl;
        power_dbm_stats["std_dev"] = 0.0;
        power_dbm_stats["variance"] = 0.0;
        power_dbm_stats["coeff_of_variation"] = 0.0;
    }
    exp["power_dbm_stats"] = power_dbm_stats;

    if (engine_mode_ == "SIMULATION")
    {
        exp["translational_velocity_mps_stats"] = calculate_stats(sample_count,
                                                                  kpi_data.sum_of_trans_vel,
                                                                  kpi_data.sum_of_squares_of_trans_vel);
        exp["rotational_velocity_radps_stats"] = calculate_stats(sample_count,
                                                                 kpi_data.sum_of_rot_vel,
                                                                 kpi_data.sum_of_squares_of_rot_vel);
        exp["throughput_efficiency_percent"] = (net_actual_bits > 0 && kpi_data.theoretical_bits > 0)
                                                   ? (net_actual_bits / kpi_data.theoretical_bits) * 100.0
                                                   : 0.0;
        stats["peak_velocity"] = {{"translational_mps", kpi_data.peak_translational_velocity},
                                  {"rotational_radps", kpi_data.peak_rotational_velocity}};
    }
    else
    {
        // Set these fields to 9999 to indicate N/A in EXPERIMENTAL mode
        exp["translational_velocity_mps_stats"] = {{"mean", 9999.0},
                                                   {"std_dev", 9999.0},
                                                   {"variance", 9999.0},
                                                   {"coeff_of_variation", 9999.0}};
        exp["rotational_velocity_radps_stats"] = {{"mean", 9999.0},
                                                  {"std_dev", 9999.0},
                                                  {"variance", 9999.0},
                                                  {"coeff_of_variation", 9999.0}};
        exp["throughput_efficiency_percent"] = 9999.0;
        stats["peak_velocity"] = {{"translational_mps", 9999.0}, {"rotational_radps", 9999.0}};
    }

    exp["link_availability_percent"] = (total_slots > 0)
                                           ? (static_cast<double>(kpi_data.good_snr_slots) / total_slots) * 100.0
                                           : 0.0;
    exp["overhead_gbits"] = kpi_data.overhead_bits / 1e9;
    stats["experimental"] = exp;

    // --- Overall Window Metrics ---
    stats["total_slots_in_window"] = total_slots;

    return stats;
}
