#ifndef STRUCT_DEFINITION_H
#define STRUCT_DEFINITION_H

#include "Software/jsonReader/jsonReader.hpp"
#include <map>
#include <string>
#include <vector>
#include <chrono>

struct Quaternion
{
    double w;
    double x;
    double y;
    double z;
};

struct Position
{
    double x;
    double y;
    double z;
};

enum class InterpolationMode
{
    Spherical,
    Linear
};

struct environmentObject
{
    Position position = {-99, -99, -99};
    Position velocity = {-99, -99, -99};
    Position acceleration = {-99, -99, -99};
    Quaternion quaternion = {1, 0, 0, 0};
    Position angular_velocity = {-99, -99, -99};
    Position angular_acceleration = {-99, -99, -99};
};

struct imuObject
{
    Position acceleration = {-99, -99, -99};
    Position angular_velocity = {-99, -99, -99};
    Quaternion quaternion = {1, 0, 0, 0};
    double sample_slot = -99;
};

struct rxChainObject
{
    double power_watts = -99;
    double sample_slot = -99;
};

struct antennaObject
{
    double radiationPatternID = 0;
    double gain_linear = -99;
    double power_watts = -99;
};

struct radiationPattern
{
    std::vector<double> thetaValues;
    std::vector<double> gainValues_linear;
};

struct rotaryAxis
{
    double angle = 0;
    double velocity = 0;
    double acceleration = 0;
};

struct rotaryObject
{
    rotaryAxis azimuth;
    rotaryAxis altitude;
    double isMoving = 0; // 0 = not moving, 1 = moving
};

enum class AlgoState : int
{
    ZUPT = 0,            // Zero Velocity Update
    REFERENCE = 1,       // Setting/re-establishing baseline/reference point
    MONITORING = 2,      // Watching for deviations from reference
    ALIGNMENT = 3,       // Actively moving to meet alignment objective
    PANIC_ALIGNMENT = 4, // Special state for panic alignment scenarios
    ERROR_STATUS = 5     // Error state, typically when algo cannot proceed
};

enum class AlgoAction : int
{
    NONE = 0,        // No specific action being performed (typical in MONITORING)
    STARTING = 1,    // Command sent to initiate movement/process, awaiting physical start
    MOVING = 2,      // Rotary is confirmed to be physically moving
    STOPPING = 3,    // Command sent to halt movement, awaiting physical stop
    ERROR_ACTION = 4 // Action associated with ERROR_STATUS state
};

struct PowerStats
{
    double averagePowerDbm = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> powerValuesDbm;
    long count = 0;
};

struct KpiWindowAccumulator
{
    double total_bits = 0.0;
    double overhead_bits = 0.0;
    double theoretical_bits = 0.0;
    double no_alignment_bits = 0.0;
    long aligned_slots = 0;
    long outage_slots = 0;
    long good_snr_slots = 0;
    long good_snr_slots_theoretical_max = 0;
    long good_snr_slots_theoretical_min = 0;
    long long throughput_sample_count = 0;
    double sum_of_bits = 0.0;
    double sum_of_squares_of_bits = 0.0;
    double sum_of_snr = 0.0;
    double sum_of_squares_of_snr = 0.0;
    long long snr_sample_count = 0;
    double sum_of_power = 0.0;
    double sum_of_squares_of_power = 0.0;
    double sum_of_trans_vel = 0.0;
    double sum_of_squares_of_trans_vel = 0.0;
    double sum_of_rot_vel = 0.0;
    double sum_of_squares_of_rot_vel = 0.0;
    double peak_translational_velocity = 0.0;
    double peak_rotational_velocity = 0.0;
    double sum_of_theo_bits = 0.0;
    double sum_of_squares_of_theo_bits = 0.0;
    double sum_of_no_align_bits = 0.0;
    double sum_of_squares_of_no_align_bits = 0.0;
    double sum_of_theo_max_snr = 0.0;
    double sum_of_squares_of_theo_max_snr = 0.0;
    long long theo_max_snr_sample_count = 0;
    double sum_of_theo_min_snr = 0.0;
    double sum_of_squares_of_theo_min_snr = 0.0;
    long long theo_min_snr_sample_count = 0;
};

struct packetObject
{
    double unit_index = -1;           // Index of the engine unit that sent this packet
    double id = -99;                  // Unique identifier for the packet
    double transmit_start_slot = -99; // Time slot when the packet starts to transmit
    double receive_end_slot = -99;    // Time slot when the packet is fully received

    // array to hold imuObject data, max 204 size
    std::vector<imuObject> imu_data; // IMU data captured during the packet transmission
    Position delta_position = {0,
                               0,
                               0}; // Relative position UE has observed since beginning of scenario
    Quaternion delta_rotation = {1, 0, 0, 0};
};

struct algorithmObject
{
    double status = static_cast<double>(AlgoState::REFERENCE);
    double action = static_cast<double>(AlgoAction::NONE);
};

enum class classifierClass : int
{
    INITIALIZING = 0, // Initial state
    ALIGNED = 1,      // Successfully aligned
    MISALIGNED = 2,   // Misalignment detected
};

// --- Position Sim Data Structures ---
struct KeyframeData
{
    int slot;               // Time slot index for this keyframe
    Position position;      // Position (x, y, z) at this keyframe
    Quaternion orientation; // Orientation (w, x, y, z) at this keyframe
    InterpolationMode mode; // Interpolation mode *starting* from this keyframe
};

struct InterpolatedState
{
    Position position;
    Quaternion quaternion;
};

struct PrecomputedPatternLUT
{
    std::vector<double> fineGrainedGains;
    // The LUT spans from cos(theta) = 1.0 (0 deg) down to cos(theta_max).
    // It is indexed linearly by cos(theta).
    double cosThetaMin;  // e.g., cos(180 deg) = -1.0 or cos(90 deg) = 0.0
    double cosThetaMax;  // Always 1.0 (for theta = 0)
    double cosThetaStep; // The step size in the cos(theta) domain
    double invCosThetaStep = 0.0;
    size_t numSteps;
};

struct engineUnit
{
    std::string label;
    std::string algorithm;
    std::string antenna;

    // Components Parts
    bool IMU_enabled;
    bool RxChain_enabled;
    bool RxAntenna_enabled;
    bool TxAntenna_enabled;
    bool Rotary_enabled;
    double Rotary_azimuth_velocity_degpersec;
    double Rotary_azimuth_acceleration_degpersecsq;
    double Rotary_altitude_velocity_degpersec;
    double Rotary_altitude_acceleration_degpersecsq;

    double IMU_accel_noise_density;        // [m/s^2 / sqrt(Hz)] (sigma_a)
    double IMU_gyro_noise_density;         // [rad/s / sqrt(Hz)]  (sigma_g)
    double IMU_accel_bias_random_walk = 0; // [m/s^3 / sqrt(Hz)] (sigma_ba)
    double IMU_gyro_bias_random_walk = 0;  // [rad/s^2 / sqrt(Hz)] (sigma_bg)
    double IMU_accel_initial_bias_mean;    // [m/s^2]
    double IMU_accel_initial_bias_stddev;  // [m/s^2]
    double IMU_gyro_initial_bias_mean;     // [rad/s]
    double IMU_gyro_initial_bias_stddev;   // [rad/s]
    double IMU_samplingDelay_microsec;
    double IMU_samplingDelay_slots;

    double RxChain_noise_factor_linear; // The only RECEIVEr noise (thermal calculated automatically)
    double RxChain_samplingDelay_microsec;
    double RxChain_samplingDelay_slots;

    double TxAntenna_transmitPower_watts;

    // Each unit has its own antenna radiation patterns
    std::map<int, radiationPattern> radiationPatterns;      // Key: radiationPatternID, Value: Pattern data
    std::map<int, double> radiationPatterns_maxGain_linear; // Maps patternId to its max gain.
    std::map<int, double> radiationPatterns_hpbw;           // Maps patternId to its HPBW in degrees

    // Simulation supports multiple files/algorithms, but experimental only one.
    bool preset_antenna_enabled;
    bool preset_algorithm_enabled;
    std::vector<std::string> preset_antenna_files;
    std::vector<std::string> preset_algorithm_names;
};

struct experimentalConfig
{
    std::string role; // "observer" or name of engine_unit

    std::string observer_tcp_ip;      // Nodes connect to this IP of the observer
    int observer_tcp_port;            // Nodes connect to this PORT of the observer, observer listens here
    std::string Rotary_port_azimuth;  // Serial port for azimuth rotary control
    std::string Rotary_port_altitude; // Serial port for altitude rotary control
    double DSO_measurement_bandwidth; // Bandwidth in MHz for DSO measurements
    double DSO_IF_frequency;          // Intermediate Frequency in Hz for DSO measurements
};

struct ConfigFile
{
    // GUI or CONSOLE mode
    std::string display_mode;

    // MobileTHz Engine Configuration
    long double engine_slot_time_microsec;
    long double engine_max_time_sec;
    std::string engine_mode; // "simulation" or "experimental"
    std::vector<engineUnit> engine_units;
    int64_t engine_carrier_frequency;   // Frequency in hertz
    int64_t engine_bandwidth_frequency; // Bandwidth in hertz
    int64_t engine_temperature_kelvin;  // Temperature in Kelvin
    // We can calculate receiver noise from bandwidth and temperature
    double engine_receiver_thermal_noise_watts; // Receiver thermal noise in watts/Hz

    // Simulation-Related
    int randomgen_seed; /// Seed for random number generator
    bool preset_position_enabled;
    // This large structure is read-only during the combination runs.
    // Wrap it in a shared_ptr to a const vector to make copies cheap.
    std::shared_ptr<const std::vector<std::vector<std::string>>> preset_position_sets;

    // JIT Packet Reception Parameters
    double JIT_packet_min_snr_db;         // Minimum SNR in dB to successfully receive JIT packet
    double JIT_packet_reception_delay_us; // Nus DAC + Mus ADC.

    // KPI Link Classification Parameters
    double link_min_snr_db; // Minimum SNR in dB to be considered a valid link

    /// X/Y Visualizationnn Conversion
    int pixelsPerMeter = 10;

    // Global Configuration Names
    std::string paramConfig_name;
    std::string positionConfig_name;
    std::string antennaConfig_name;

    // Experimental Configuration (only used in experimental mode)
    experimentalConfig experimental;
};

struct PreprocessedData
{
    // The fully configured 'ConfigFile' for a specific run.
    ConfigFile runConfig;

    // The fully parsed 'positionConfig' for a specific run.
    nlohmann::json positionConfig;

    // We can even store the unique KPI name here.
    std::string kpiFileName;

    // Batching keys
    size_t num_time_slots;
    size_t num_units;
};

struct SlotKpiMetrics
{
    // Throughput
    double actual_slot_bits = 0.0;
    double theo_slot_bits = 0.0;
    double no_align_slot_bits = 0.0;

    // Link State
    bool is_aligned = false;
    bool is_snr_good = false;
    bool is_theo_max_snr_good = false;
    bool is_theo_min_snr_good = false;

    // Power & SNR
    double actual_total_power_watts = 0.0;
    double snr_linear = 0.0;
    double theo_max_snr_linear = 0.0;
    double theo_min_snr_linear = 0.0;

    // Velocity
    double trans_vel = 0.0;
    double rot_vel = 0.0;
};

// Experimental structs below
using Timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>;

struct TimestampedImuData
{
    imuObject data;
    Timestamp timestamp;
    bool has_new_data = false;
};

struct TimestampedPowerData
{
    double power_watts = -99.0;
    Timestamp timestamp;
    bool has_new_data = false;
};

struct TimestampedRotaryData
{
    rotaryObject data;
    Timestamp timestamp;
    bool has_new_data = false;
};

struct UeClassificationRuntimeState
{
    classifierClass currentClass = classifierClass::INITIALIZING;
    double shortTermEMA = -9999999;
    double lastRawPower = -9999999;
    bool filterInitialized = false;
    int warmupCounter = 0;
    size_t lastProcessedRxSampleSlot = std::numeric_limits<size_t>::max();

    double initialPeakRxPower = -9999999;
    double estimatedInitialDistanceM = -1.0;

    UeClassificationRuntimeState() = default;
    void resetForNewUeProcessing()
    {
        currentClass = classifierClass::INITIALIZING;
        shortTermEMA = -9999999;
        lastRawPower = -9999999;
        filterInitialized = false;
        warmupCounter = 0;
        lastProcessedRxSampleSlot = std::numeric_limits<size_t>::max();
        initialPeakRxPower = -9999999;
        estimatedInitialDistanceM = -1.0;
    }
};

#endif
