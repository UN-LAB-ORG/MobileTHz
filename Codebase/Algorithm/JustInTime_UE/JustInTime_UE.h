#ifndef JUSTINTIME_UE_H
#define JUSTINTIME_UE_H

#include "Codebase/Algorithm/AlgorithmInterface.h"
#include "Codebase/Software/circularBuffer/CircularBuffer.h"
#include "Codebase/Software/structDefinition.h"
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

// Forward Declaration
class mobileTHzEngine;

class JustInTime_UE : public AlgorithmInterface
{
public:
    JustInTime_UE();
    ~JustInTime_UE() override = default;

    void processSlot(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot) override;
    algorithmObject getStatus() const override;
    void configure(const ConfigFile &config, const engineUnit &unitConfig) override;

private:
    // --- Internal State ---
    AlgoState currentState = AlgoState::REFERENCE;
    AlgoAction currentAction = AlgoAction::NONE;

    // --- Configuration Parameters ---
    std::string algorithmName = "JustInTime_UE";
    bool configured = false;
    double slotTimeMicrosec;

    // System Parameters
    double txPowerDbm;
    double txGainDbi;
    double rxGainDbi;
    double frequencyHz;

    // IMU Integration & Control Parameters
    std::string imuAxisHorizontal = "Y_POS";
    std::string imuAxisVertical = "X_POS";
    double minAngleChangeThresholdDeg = 0.10f;
    std::vector<double> precomputed_imu_weights_;

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

    // --- JIT Packet Configuration ---
    double panicPacketCooldownMicroseconds_ = 50000.0f;
    int panicPacketMaxImuSamplesToStore_ = 204;
    Position last_transmitted_relative_position_m;
    Quaternion last_transmitted_orientation_;
    Quaternion last_tracked_orientation_;

    double m_imuWeightDecayConstant = 0.9;

    // --- Additive IMU Motion Detection Parameters ---
    Position imuAccumulatedMotion_ = {0.0f, 0.0f, 0.0f};  // Accumulator for significant motion components (units: m/s^2)
    double imuSignificantMotionThreshold_ = 0.001f;       // m/s^2, threshold for a single axis of motion accel to be added to accumulator
    double imuPanicAccumulatedMagnitudeThreshold_ = 2.5f; // m/s^2, threshold for magnitude of imuAccumulatedMotion_ to trigger panic

    // --- Direct Rotation Change Threshold ---
    double panicPacketRotationThresholdDeg;
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
    Position initialEulerAngles = {0, 0, 0};      // Roll, Pitch, Yaw at baseline
    Position lastImuAccelerationRaw_ = {0, 0, 0}; // To store the raw IMU accel for delta checks
    CircularBuffer<imuObject> imuSamples_;        // store rolling IMU samples for panic packet

    // Kinematic State
    double initialDistance = 0.0;
    Position currentVelocity = {0, 0, 0};
    Position currentRelativePosition = {0, 0, 0};
    double lastImuSampleTimeSec = -1.0;
    Position initialPointingVectorLocal_;

    // Pointing State
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
    std::vector<SpiralOffsetPoint> localSpiralOffsets;
    bool isLocalSpiraling = false;
    int currentLocalSpiralIndex = -1;

    double debug_last_logged_rot_alt = -999.0;
    const double debug_log_threshold_deg = 0.1;

    // --- JIT Packet Runtime Data ---
    packetObject JITPacket;
    size_t panicPacketCooldownSlotsConverted_ = 0;
    size_t lastJITPacketSentSlot_ = 0;
    uint64_t nextJITPacketId_ = 1;
    std::pair<Position, Quaternion> calculatePredictedMotion(
        const CircularBuffer<imuObject> &imuSamples,
        double imuSamplingDelayMicrosec,
        double predictionHorizonMicrosec,
        double imuWeightDecayConstant,
        const Position &worldGravity,
        const Position &currentVelocityAtPredictionStart,
        const std::string &algoName) const;

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

    void checkForAndSendJITPacket(mobileTHzEngine *engine,
                                  size_t unitIndex,
                                  size_t currentSlot,
                                  const imuObject &currentImu,
                                  bool translationTrigger,
                                  bool rotationTrigger);
    void updateKinematics(const imuObject &currentImu, double currentTimeSec);

    Quaternion multiplyQuaternions(const Quaternion &q1, const Quaternion &q2) const;
    Position rotateVectorByQuaternion(const Position &v, const Quaternion &q) const;
    Quaternion conjugate(const Quaternion &q) const;

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

#endif // JUSTINTIME_UE
