#ifndef JUSTINTIME_AP_H
#define JUSTINTIME_AP_H

#include "Codebase/Algorithm/AlgorithmInterface.h"
#include "Codebase/Software/structDefinition.h"
#include <cstddef>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

class mobileTHzEngine;

class JustInTime_AP : public AlgorithmInterface
{
public:
    JustInTime_AP();
    ~JustInTime_AP() override = default;

    void processSlot(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot) override;
    algorithmObject getStatus() const override;
    void configure(const ConfigFile &config, const engineUnit &unitConfig) override;

private:
    // --- Internal State ---
    AlgoState currentState = AlgoState::REFERENCE;
    AlgoAction currentAction = AlgoAction::NONE;

    // --- Configuration Parameters ---
    std::string algorithmName = "JITPacket_AP_DefaultName";
    bool configured = false;
    double slotTimeMicrosec;

    // Power thresholds
    double misalignThresholdAbsoluteDb;
    double realignSuccessInitialMarginDb;
    double realignSuccessMaxMarginDb = 40.0f;
    double peakSeekingDecreaseThresholdDb = 0.0f;
    double realignSuccessMarginIncreaseRateDbPerSec;
    size_t m_nextPacketToProcessIndex = 0;

    // Spiral Search Parameters
    int maxSpiralPoints = 10000;
    double halfPowerBeamWidth;
    double spiralGrowthFactor = 1.25;

    // Filter parameters
    int referenceWarmupSamples = 50;
    double filterAlphaShort = 0.90f;
    double filterAlphaLong = 0.001f;
    double filterSpikeThresholdDb = 7.0f;

    // Peak seeking during spiral
    bool m_isPeakSeeking = false;
    double m_peakSeekingBestPower = -9999999;
    int m_peakSeekingBestIndex = -1;

    // --- Runtime Data ---
    double currentReferencePower = -9999999;
    bool filterInitialized = false;
    double shortTermEMA = -9999999;
    double longTermEMA = -9999999;
    double lastRawPower = -9999999;
    int warmupCounter = 0;
    size_t lastProcessedRxSampleSlot = std::numeric_limits<size_t>::max();

    size_t lastSuccessfulAlignmentSlot = 0;
    double spiralCenterAz = 0.0f;
    double spiralCenterAlt = 0.0f;

    struct SpiralOffsetPoint
    {
        double deltaAz;
        double deltaAlt;
    };
    std::vector<SpiralOffsetPoint> spiralOffsets; // Used by both ALIGNMENT and PANIC_ALIGNMENT
    int currentSpiralPointIndex = -1;             // Index into spiralOffsets vector

    // --- JIT Packet Reception Logic Parameters & Runtime Data ---
    const double panicPacketCooldownMicroseconds = 50000.0;
    std::unordered_set<uint64_t> processedPacketIds;
    bool m_panic_move_command_issued;
    double ue_baseline_az_at_ap_reference = 0.0f;
    double ue_baseline_alt_at_ap_reference = 0.0f;
    Position ap_cumulative_ue_displacement_m = {0.0f, 0.0f, 0.0f}; // UE's total displacement in WORLD frame since reference
    Quaternion ap_cumulative_ue_rotation = {1.0, 0, 0, 0};         // UE's total rotation in WORLD frame since reference

    // Configuration for AP reacting to panic packets
    double panic_info_trust_duration_sec = 0.1f;
    int panic_imu_interpretation_window_slots = 200;
    int panic_min_imu_samples_for_prediction = 30;

    // Runtime state for AP's panic packet handling
    double panic_predicted_target_az = 0.0f;
    double panic_predicted_target_alt = 0.0f;
    size_t last_panic_info_reception_slot = 0;
    bool has_fresh_panic_info = false;
    bool m_is_awaiting_ue_rotation_outcome = false;
    size_t m_ue_rotation_wait_end_slot = 0;
    size_t panic_info_trust_duration_slots = 0;

    // --- Helper Functions ---
    void setState(AlgoState newStatus, AlgoAction newAction = AlgoAction::NONE);

    // For standard ALIGNMENT state (power-drop triggered)
    void startStandardAlignment(mobileTHzEngine *engine,
                                size_t unitIndex,
                                size_t currentSlot,
                                const rotaryObject &currentRotary);
    void generateStandardSpiralOffsets(); // Generates spiral identical to AlignWithoutIMU
    void issueStandardSpiralMoveCommand(mobileTHzEngine *engine,
                                        size_t unitIndex,
                                        size_t currentSlot,
                                        const rotaryObject &currentRotaryState);

    // For PANIC_ALIGNMENT state (panic-packet triggered)
    void startPanicAlignment(mobileTHzEngine *engine,
                             size_t unitIndex,
                             size_t currentSlot,
                             const rotaryObject &currentRotary);
    void generatePanicSpiralOffsets(); // Generates spiral with {0,0} as first point for panic-snap
    void issuePanicSpiralMoveCommand(mobileTHzEngine *engine,
                                     size_t unitIndex,
                                     size_t currentSlot,
                                     const rotaryObject &currentRotaryState);

    bool updatePowerFilter(double rawPower);
    double calculateDynamicSuccessMargin(size_t currentSlot) const;
    double shortestAngleDiff(double targetDeg, double currentDeg) const;

    void processReceivedJITPackets(mobileTHzEngine *engine,
                                   size_t apUnitIndex,
                                   size_t currentSlot,
                                   const rotaryObject &currentAPRotary);

    // System Parameters
    double txPowerDbm;
    double txGainDbi;
    double rxGainDbi;
    double frequencyGhz;

    enum class AxisDirection
    {
        X_POS,
        X_NEG,
        Y_POS,
        Y_NEG,
        Z_POS,
        Z_NEG,
        NONE
    };
    const std::map<std::string, AxisDirection> axisDirectionLookup = {{"X_POS", AxisDirection::X_POS},
                                                                      {"X_NEG", AxisDirection::X_NEG},
                                                                      {"Y_POS", AxisDirection::Y_POS},
                                                                      {"Y_NEG", AxisDirection::Y_NEG},
                                                                      {"Z_POS", AxisDirection::Z_POS},
                                                                      {"Z_NEG", AxisDirection::Z_NEG},
                                                                      {"NONE", AxisDirection::NONE}};
    AxisDirection mapHorizontal = AxisDirection::Y_NEG;
    AxisDirection mapVertical = AxisDirection::Z_POS;
};

#endif // JUSTINTIME_AP_H
