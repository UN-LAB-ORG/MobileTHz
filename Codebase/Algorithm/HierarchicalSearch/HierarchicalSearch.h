#ifndef HIERARCHICAL_SEARCH_H
#define HIERARCHICAL_SEARCH_H

#include "Codebase/Algorithm/AlgorithmInterface.h"
#include "Codebase/Software/structDefinition.h"
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

class mobileTHzEngine;

class HierarchicalSearch : public AlgorithmInterface
{
public:
    HierarchicalSearch();
    ~HierarchicalSearch() override = default;

    void processSlot(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot) override;
    algorithmObject getStatus() const override;
    void configure(const ConfigFile &config, const engineUnit &unitConfig) override;

private:
    // --- Internal State ---
    AlgoState currentState = AlgoState::REFERENCE;
    AlgoAction currentAction = AlgoAction::NONE;

    // --- Configuration Parameters (set via configure) ---
    std::string algorithmName = "HierarchicalSearch";
    bool configured = false;
    double slotTimeMicrosec;

    // Power thresholds
    double realignSuccessMaxMarginDb = 40.0f;
    double realignSuccessMarginIncreaseRateDbPerSec;
    double misalignThresholdAbsoluteDb;
    double realignSuccessInitialMarginDb;
    double peakSeekingDecreaseThresholdDb = 0.0f;

    // Spiral Search Parameters
    int maxSpiralPoints = 10000;      // Max points to check
    double halfPowerBeamWidth;        // HPBW in degrees (sets initial scale)
    double spiralGrowthFactor = 1.25; // Factor > 1 for "exploding" effect per layer

    // Filter parameters
    int referenceWarmupSamples = 50;
    double filterAlphaShort = 0.90f;
    double filterAlphaLong = 0.001f;
    double filterSpikeThresholdDb = 7.0f;

    // -- Peak seeking
    bool m_isPeakSeeking = false;             // Flag: True when success threshold met, looking for peak
    double m_peakSeekingBestPower = -9999999; // Best power found during peak seek
    int m_peakSeekingBestIndex = -1;          // Index where best power was found

    // --- Runtime Data ---
    double currentReferencePower = -9999999;
    bool filterInitialized = false;
    double shortTermEMA = -9999999;
    double longTermEMA = -9999999;
    double lastRawPower = -9999999;
    int warmupCounter = 0;
    size_t lastProcessedRxSampleSlot = std::numeric_limits<size_t>::max();

    // Realignment/Spiral Related
    size_t lastSuccessfulAlignmentSlot = 0;

    // Position where misalignment was detected (acts as center for applying offsets)
    double spiralCenterAz = 0.0f;
    double spiralCenterAlt = 0.0f;

    // Pre-calculated spiral OFFSET points (relative to 0,0)
    struct SpiralOffsetPoint
    {
        double deltaAz;  // Offset in Azimuth
        double deltaAlt; // Offset in Altitude
    };
    std::vector<SpiralOffsetPoint> spiralOffsets; // Store pre-calculated offsets
    int currentSpiralPointIndex = -1;             // Index into spiralOffsets vector

    // --- Helper Functions ---
    void setState(AlgoState newStatus, AlgoAction newAction = AlgoAction::NONE);
    void startAlignment(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot);
    void generateSpiralOffsets();
    void issueSpiralMoveCommand(mobileTHzEngine *engine,
                                size_t unitIndex,
                                size_t currentSlot); // Issues moveDual using offsets + center
    bool updatePowerFilter(double rawPower);
    double calculateDynamicSuccessMargin(size_t currentSlot) const;
};

#endif // ALIGN_WITHOUT_IMU_H
