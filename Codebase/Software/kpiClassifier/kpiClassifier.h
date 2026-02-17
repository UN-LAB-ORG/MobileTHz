#ifndef KPICLASSIFIER_H
#define KPICLASSIFIER_H

#include <QString>
#include "Codebase/Software/jsonReader/jsonReader.hpp"
#include "Codebase/Software/structDefinition.h"
#include <limits>
#include <vector>

class mobileTHzEngine;
using json = nlohmann::json;

class kpiClassifier
{
private:
    mobileTHzEngine *engine_;
    std::string engine_mode_;

    // Store runtime state for each UE
    std::vector<UeClassificationRuntimeState> ueRuntimeStates_;
    std::vector<double> ue_total_noise_watts_;

    // Parameter for Link Availability KPI ---
    double paramMinSnrLinearForAvailability_;

    // --- Parameters for Distance Estimation ---
    double paramTxPowerDbmForDistanceEst_;
    double paramTotalAntennaGainDbiForDistanceEst_;

    double slotTimeMicrosec_;
    double paramCarrierFrequencyHz_;

    std::vector<std::vector<bool>> ueIsAlignedHistory_;
    std::vector<bool> ueIsSelectedForClassification_;
    int primaryUeIndex_ = -1;
    // Stores the last slot the primary UE was detected to be moving.
    size_t m_last_ue_moving_slot = 0;

    bool classificationHasBeenPerformed_ = false;

    KpiWindowAccumulator kpi_window1_; // full scenario post-init
    // KpiWindowAccumulator kpi_window4_; // full scenario post-init to final realignment
    SlotKpiMetrics calculateMetricsForSlot(size_t time_idx);
    std::vector<SlotKpiMetrics> per_slot_kpis_;
    json finalizeWindowStats(size_t window_start_slot, size_t window_end_slot) const;

    // We need to know the AP index during processing
    int ap_idx_for_kpi_calc_ = -1;
    double ap_tx_power_watts_ = 0.0;
    double ap_max_gain_linear_ = 0.0;
    double ue_max_gain_linear_ = 0.0;

    inline double calculateCapacityBits(double snr_linear) const;

    static constexpr double MIN_DBM = -200.0;
    // The watt value corresponding to -200 dBm (10*log10(1e-23 * 1000) = -200)
    static constexpr double MIN_WATTS = 1e-23;

    // Pre-calculated constant for dBm-to-Watts conversion
    // Instead of pow(10, dBm / 10.0), we use exp(dBm * (log(10.0) / 10.0))
    // This constant is log(10.0) / 10.0
    static constexpr double LN10_OVER_10 = 0.23025850929940456840;

    // Pre-calculated constant for Watts-to-dBm conversion
    // Instead of 10.0 * log10(watts), we use 10.0 * (log(watts) / log(10.0))
    // This constant is 10.0 / log(10.0)
    static constexpr double TEN_OVER_LN10 = 4.34294481903251827651;

    // --- Pre-calculated values for KPI calculations ---
    double kpi_B_Hz_ = 0.0;
    double kpi_slot_duration_sec_ = 0.0;
    double kpi_lambda_m_ = 0.0;
    double kpi_inv_log2_ = 1.4426950408889634;
    double kpi_shannon_hartley_factor_ = 0.0;
    double kpi_friis_wavelength_factor_ = 0.0;

public:
    kpiClassifier();
    ~kpiClassifier();

    /**
     * @brief Performs initial setup for the classifier.
     * Must be called after the engine is fully constructed but before run() starts.
     */
    void initialize(mobileTHzEngine *engine);

    /**
     * @brief Processes a single time slot for a single unit.
     * This is the new core function to be called from the engine's run loop.
     * @param unit_idx The index of the unit to process.
     * @param time_idx The current time slot index.
     */
    void processSlot(size_t unit_idx, size_t time_idx);

    /**
     * @brief Called by the engine after the simulation run is complete.
     */
    void finalizeClassification();

    /** @brief Checks if a given unit is selected for classification (i.e., is a UE). */
    bool isUnitSelected(size_t unit_idx) const;

    void calculateFinalKPIsAndSave(const std::string &name);
};

#endif // KPICLASSIFIER_H
