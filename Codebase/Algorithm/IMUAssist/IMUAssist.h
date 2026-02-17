#ifndef IMU_ASSIST_H
#define IMU_ASSIST_H

#include "Codebase/Algorithm/AlgorithmInterface.h"
#include "Codebase/Software/circularBuffer/CircularBuffer.h"
#include "Codebase/Software/structDefinition.h"
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

class mobileTHzEngine;

class IMUAssist : public AlgorithmInterface
{
public:
    IMUAssist();
    ~IMUAssist() override = default;

    void processSlot(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot) override;
    algorithmObject getStatus() const override;
    void configure(const ConfigFile &config, const engineUnit &unitConfig) override;

private:
    // --- Internal State ---
    AlgoState currentState = AlgoState::REFERENCE;
    AlgoAction currentAction = AlgoAction::NONE;

    // --- Configuration Parameters ---
    std::string algorithmName = "IMUAssist_TrackAndSpiral";
    bool configured = false;
    double slotTimeMicrosec;

    // System Parameters
    double txPowerDbm;
    double txGainDbi;
    double rxGainDbi;
    double frequencyHz;

    // IMU Integration & Control Parameters
    double minAngleChangeThresholdDeg = 0.10f;
    std::vector<double> precomputed_imu_weights_;
    double m_imuWeightDecayConstant = 0.9;
    int m_maxImuSamplesForPrediction = 204;

    // --- Local Spiral Fine-Tuning Parameters ---
    bool enableLocalSpiral = true;
    double localSpiralTriggerThresholdDb;
    double realignSuccessInitialMarginDb;
    double realignSuccessMaxMarginDb = 40.0f;
    double realignSuccessMarginIncreaseRateDbPerSec;
    double peakSeekingDecreaseThresholdDb = 0.0f;
    int localSpiralMaxPoints = 10000;
    double localSpiralStepScale = 1.25;
    double localSpiralBeamWidthDeg;

    // Filter parameters
    int referenceWarmupSamples = 50;
    double filterAlphaShort = 0.90f;
    double filterAlphaLong = 0.001f;
    double filterSpikeThresholdDb = 7.0f;

    // --- Proactive IMU Tracking Parameters ---
    Quaternion last_tracked_orientation_;
    Position imuAccumulatedMotion_ = {0.0f, 0.0f, 0.0f};     // Accumulator for significant motion components (units: m/s^2)
    double imuSignificantMotionThreshold_ = 0.001f;          // m/s^2, threshold for a single axis of motion accel to be added to accumulator
    double imuTrackingAccumulatedMagnitudeThreshold_ = 0.5f; // m/s^2, threshold to trigger a corrective move
    double imuTrackingRotationThresholdDeg;

    // --- Runtime Data ---
    bool m_isPeakSeeking = false;
    double m_peakSeekingBestPower = -9999999;
    int m_peakSeekingBestIndex = -1;
    double currentReferencePower = -9999999;
    bool filterInitialized = false;
    double shortTermEMA = -9999999;
    double longTermEMA = -9999999;
    double lastRawPower = -9999999;
    int warmupCounter = 0;
    size_t lastProcessedRxSampleSlot = std::numeric_limits<size_t>::max();
    size_t lastProcessedImuSampleSlot = std::numeric_limits<size_t>::max();
    size_t lastSuccessfulPointSlot = 0;

    // IMU related
    bool imuEnabledRuntime = false;
    bool baselineEstablished = false;
    Position baselineAcceleration = {0, 0, 0}; // This is the baseline gravity/orientation vector
    Quaternion initialOrientation = {1.0f, 0.0f, 0.0f, 0.0f};
    CircularBuffer<imuObject> imuSamples_;

    // Kinematic State
    double initialDistance = 0.0;
    Position currentVelocity = {0, 0, 0};
    Position currentRelativePosition = {0, 0, 0};
    double lastImuSampleTimeSec = -1.0;
    Position initialPointingVectorLocal_; // The vector from UE to target in UE's local frame at baseline

    // Pointing State (current targets)
    double initialIMUTargetAz = 0.0f;
    double initialIMUTargetAlt = 0.0f;
    double IMUTargetAz = 0.0f;
    double IMUTargetAlt = 0.0f;

    // --- Local Spiral Runtime Data ---
    struct SpiralOffsetPoint
    {
        double deltaAz;
        double deltaAlt;
    };
    double localSpiralCenterAz = 0.0f;
    double localSpiralCenterAlt = 0.0f;
    std::vector<SpiralOffsetPoint> localSpiralOffsets;
    bool isLocalSpiraling = false;
    int currentLocalSpiralIndex = -1;

    // --- Prediction Function ---
    std::pair<Position, Quaternion> calculatePredictedMotion(
        const CircularBuffer<imuObject> &imuSamples,
        double predictionHorizonMicrosec,
        const Position &worldGravity,
        const Position &currentVelocityAtPredictionStart) const;

    // --- Helper Functions ---
    Position anglesToVector(double azDeg, double altDeg, double distance) const;
    void setState(AlgoState newStatus, AlgoAction newAction = AlgoAction::NONE);
    bool updatePowerFilter(double rawPower);
    double estimateDistanceFromPower(double rxPowerDbm) const;
    void calculateTargetAngles(const imuObject &currentImu);
    double shortestAngleDiff(double target, double current) const;
    void generateLocalSpiralOffsets();
    void startAlignmentSpiral(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot);
    void issueLocalSpiralMoveCommand(mobileTHzEngine *engine,
                                     size_t unitIndex,
                                     size_t currentSlot,
                                     const rotaryObject &currentRotaryState);
    void issueIMUTrackMoveCommand(mobileTHzEngine *engine,
                                  size_t unitIndex,
                                  size_t currentSlot,
                                  const rotaryObject &currentRotaryState,
                                  double targetAz,
                                  double targetAlt);
    double calculateDynamicSuccessMargin(size_t currentSlot) const;
    void updateKinematics(const imuObject &currentImu, double currentTimeSec);
    Quaternion multiplyQuaternions(const Quaternion &q1, const Quaternion &q2) const;
    Position rotateVectorByQuaternion(const Position &v, const Quaternion &q) const;
    Quaternion conjugate(const Quaternion &q) const;
};

#endif // ALIGN_WITH_IMU_H
