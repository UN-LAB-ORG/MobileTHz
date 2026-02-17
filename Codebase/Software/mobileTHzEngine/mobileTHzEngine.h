// mobileTHzEngine.h
#ifndef MOBILETHZENGINE_H
#define MOBILETHZENGINE_H

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "Algorithm/AlgorithmInterface.h"
#include "Codebase/Software/structDefinition.h"

class powerSim;
class powerExp;
class rotarySim;
class rotaryExp;
class imuExp;
class kpiClassifier;
class plotSaver;
class tcpExp;
struct SharedPointers;

// --- Enum for Data Field Indexing ---
enum class DataField : size_t
{
    ENV_POS,
    ENV_VEL,
    ENV_ACC,
    ENV_QUAT,
    ENV_ANG_VEL,
    ENV_ANG_ACC,
    IMU_ACC,
    IMU_ANG_VEL,
    IMU_QUAT,
    IMU_SAMPLE_SLOT,
    RXCHAIN_POWER,
    RXCHAIN_SAMPLE_SLOT,
    RXANT_GAIN,
    RXANT_POWER,
    RXANT_RADIATIONPATTERN,
    TXANT_GAIN,
    TXANT_POWER,
    TXANT_RADIATIONPATTERN,
    ROTARY_AZIMUTH,
    ROTARY_ALTITUDE,
    ROTARY_ISMOVING,
    ALG_STATUS,
    ALG_ACTION,
    CLASSIFIER_OUTPUT,
    COUNT // Represents the total number of fields
};

/**
 * @brief Main simulation engine for Mobile THz scenarios.
 *
 * Manages the simulation time, units (nodes), their states (environment, sensors,
 * actuators, algorithms), and orchestrates the simulation loop. Data is stored
 * in a contiguous memory block (time_grid_) for efficient access.
 */
class mobileTHzEngine
{
private:
    // --- Core Data Storage ---
    double *time_grid_;     ///< Pointer to the main data grid (contiguous memory). Layout: [Unit][TimeSlot][DataField]
    size_t num_units_;      ///< Number of simulated units (nodes).
    size_t num_time_slots_; ///< Total number of simulation time steps.

    const size_t
        allocated_num_units_; ///< Allocated size for num_units_ (for potential future resizing).
    const size_t
        allocated_num_time_slots_; ///< Allocated size for num_time_slots_ (for potential future resizing).

    size_t num_data_elements_;           ///< Number of double values per unit per time slot (calculated based on DataField enum).
    const ConfigFile *config_ = nullptr; // From ConfigFile config_;
    std::vector<size_t>
        data_offsets_;                     ///< Pre-calculated byte offsets for each DataField within a unit/timeslot block.
    std::vector<std::string> unit_labels_; ///< Cached list of unit labels from the configuration.

    // --- Simulation State ---
    size_t current_slot_ = 0; ///< Index of the currently processing time slot during the run() method.

    // --- Unit Configuration Cache ---
    std::vector<engineUnit> engine_units_; ///< Copy of the static configuration for each unit.

    // --- Pointers to Simulators ---
    powerSim *powerSimulator = nullptr;   ///< Pointer to the power simulation module.
    rotarySim *rotarySimulator = nullptr; ///< Pointer to the rotary stage simulation module.

    // --- Pointers to Experimental ---
    powerExp *powerExperimental = nullptr;   ///< Pointer to the experimental power measurement module.
    rotaryExp *rotaryExperimental = nullptr; ///< Pointer to the experimental rotary stage module.
    imuExp *imuExperimental = nullptr;       ///< Pointer to the experimental IMU module.
    tcpExp *tcpExperimental = nullptr;       ///< Pointer to the TCP module.
    std::atomic<bool> stop_flag_{false};

    // --- Algorithm Management ---
    std::vector<std::unique_ptr<AlgorithmInterface>>
        unitAlgorithms_; ///< Stores algorithm instances, one per unit.

    // --- KPI Classifier ---
    kpiClassifier *kpiClassifier_ = nullptr; ///< Pointer to the KPI classifier module.

    // --- Tikz Power Saver ---
    plotSaver *plotSaver_ = nullptr; ///< Pointer to the Plot Saver module.

    // --- Packet Layer ---
    std::vector<packetObject> packetLayer_; ///< Stores packets received during the simulation.

    // --- Private Helper Functions ---
    /** @brief Initializes data_offsets_ based on DataField enum sizes. */
    void initializeDataLayout();
    /** @brief Creates an algorithm instance based on its registered name. */
    std::unique_ptr<AlgorithmInterface> createAlgorithmByName(const std::string &name);
    /** @brief Checks if a specific component is enabled for a unit (Helper, potentially unused). */
    bool isEnabled(size_t unit_index,
                   size_t time_index,
                   const std::string &component) const; // Marked as potentially unused

public:
    // ========================================================================
    // === Constructor & Destructor ===========================================
    // ========================================================================

    /**
     * @brief Constructor. Allocates memory for the time grid based on configuration.
     * @param config The configuration settings for the simulation.
     */
    mobileTHzEngine(const ConfigFile &config);

    /**
     * @brief Initializes the engine with the provided configuration.
     * This method sets up internal structures and prepares the simulation.
     * Must be called after construction and before run().
     * @param runConfig The configuration settings for initialization.
     */
    void initialize(const ConfigFile &runConfig);

    /**
     * @brief Destructor. Releases the allocated memory for the time grid.
     */
    ~mobileTHzEngine();

    // ========================================================================
    // === Core Simulation & Control ==========================================
    // ========================================================================

    /**
     * @brief Runs the main simulation loop.
     * Iterates through time slots, updates simulator states (power, rotary),
     * and executes unit algorithms.
     * @param kpi_classifier Pointer to the KPI classifier module.
     * @param power_sim Pointer to a power simulator instance (for SIMULATION mode).
     * @param rotary_sim Pointer to a rotary simulator instance (for SIMULATION mode).
     * @param power_exp Pointer to an experimental power measurement instance (for EXPERIMENTAL mode).
     * @param rotary_exp Pointer to an experimental rotary stage instance (for EXPERIMENTAL mode).
     * @param imu_exp Pointer to an experimental IMU instance (for EXPERIMENTAL mode).
     */
    void run(kpiClassifier *kpi_classifier,
             plotSaver *plot_saver,
             powerSim *power_sim = nullptr,
             rotarySim *rotary_sim = nullptr,
             powerExp *power_exp = nullptr,
             rotaryExp *rotary_exp = nullptr,
             imuExp *imu_exp = nullptr,
             tcpExp *tcp_exp = nullptr);

    /**
     * @brief Issues a command string to the rotary simulator (or real hardware if implemented).
     * @param unitIndex The index of the unit whose rotary stage should receive the command.
     * @param commandString The command to be parsed and executed by the rotary module (e.g., "m 0 45.0").
     */
    void issueRotaryCommand(size_t unitIndex, const std::string &commandString);

    /**
     * @brief Pauses execution for the next time slot (primarily for EXPERIMENTAL mode).
     * In SIMULATION mode, this function does nothing.
     * In EXPERIMENTAL mode, it waits until the next slot based on real time.
     */
    void waitForNextSlot(Timestamp slot_start_time = Timestamp());

    // ========================================================================
    // === Data Access (Getters & Setters for Time Grid Data) =================
    // ========================================================================

    /**
     * @brief Gets a direct pointer to a specific data field.
     * @note Inlined for maximum performance.
     */
    inline double *getDataPointer(size_t unit_index, size_t time_index, DataField field) const
    {
        // Grid: [TimeSlot][Unit][DataField]
        size_t base_offset = (time_index * num_units_ + unit_index) * num_data_elements_;
        return &time_grid_[base_offset + data_offsets_[static_cast<size_t>(field)]];
    }

    // --- Environment Object Access ---
    inline void setEnvironmentData(size_t unit_index,
                                   size_t time_index,
                                   const environmentObject &env_data)
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::ENV_POS);
        std::memcpy(data_ptr, &env_data, sizeof(environmentObject));
    }
    inline const environmentObject &getEnvironmentData(size_t unit_index, size_t time_index) const
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::ENV_POS);
        return *reinterpret_cast<const environmentObject *>(data_ptr);
    }

    // --- IMU Object Access ---
    inline void setIMUData(size_t unit_index, size_t time_index, const imuObject &imu_data)
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::IMU_ACC);
        std::memcpy(data_ptr, &imu_data, sizeof(imuObject));
    }
    inline const imuObject &getIMUData(size_t unit_index, size_t time_index) const
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::IMU_ACC);
        return *reinterpret_cast<const imuObject *>(data_ptr);
    }

    // --- RxChain Object Access ---
    inline void setRxChainData(size_t unit_index,
                               size_t time_index,
                               const rxChainObject &rxChain_data)
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::RXCHAIN_POWER);
        std::memcpy(data_ptr, &rxChain_data, sizeof(rxChainObject));
    }
    inline const rxChainObject &getRxChainData(size_t unit_index, size_t time_index) const
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::RXCHAIN_POWER);
        return *reinterpret_cast<const rxChainObject *>(data_ptr);
    }

    // --- TX Antenna Object Access ---
    inline void setTxAntennaData(size_t unit_index,
                                 size_t time_index,
                                 const antennaObject &antenna_data)
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::TXANT_GAIN);
        std::memcpy(data_ptr, &antenna_data, sizeof(antennaObject));
    }
    inline const antennaObject &getTxAntennaData(size_t unit_index, size_t time_index) const
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::TXANT_GAIN);
        return *reinterpret_cast<const antennaObject *>(data_ptr);
    }

    // --- RX Antenna Object Access ---
    inline void setRxAntennaData(size_t unit_index,
                                 size_t time_index,
                                 const antennaObject &antenna_data)
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::RXANT_GAIN);
        std::memcpy(data_ptr, &antenna_data, sizeof(antennaObject));
    }
    inline const antennaObject &getRxAntennaData(size_t unit_index, size_t time_index) const
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::RXANT_GAIN);
        return *reinterpret_cast<const antennaObject *>(data_ptr);
    }

    // --- Rotary Object Access ---
    inline void setRotaryData(size_t unit_index, size_t time_index, const rotaryObject &rotary_data)
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::ROTARY_AZIMUTH);
        std::memcpy(data_ptr, &rotary_data, sizeof(rotaryObject));
    }
    inline const rotaryObject &getRotaryData(size_t unit_index, size_t time_index) const
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::ROTARY_AZIMUTH);
        return *reinterpret_cast<const rotaryObject *>(data_ptr);
    }

    // --- Algorithm Object Access ---
    inline void setAlgorithmData(size_t unit_index,
                                 size_t time_index,
                                 const algorithmObject &algo_data)
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::ALG_STATUS);
        std::memcpy(data_ptr, &algo_data, sizeof(algorithmObject));
    }
    inline const algorithmObject &getAlgorithmData(size_t unit_index, size_t time_index) const
    {
        double *data_ptr = getDataPointer(unit_index, time_index, DataField::ALG_STATUS);
        return *reinterpret_cast<const algorithmObject *>(data_ptr);
    }

    // --- Classifier Object Access ---
    inline void setClassifierData(size_t unit_index, size_t time_index, classifierClass state_class)
    {
        double val = static_cast<double>(state_class);
        std::memcpy(getDataPointer(unit_index, time_index, DataField::CLASSIFIER_OUTPUT),
                    &val,
                    sizeof(double));
    }
    inline classifierClass getClassifierData(size_t unit_index, size_t time_index) const
    {
        double val;
        std::memcpy(&val,
                    getDataPointer(unit_index, time_index, DataField::CLASSIFIER_OUTPUT),
                    sizeof(double));
        return static_cast<classifierClass>(static_cast<int>(std::round(val)));
    }

    // ========================================================================
    // === Packet Layer (Getters & Setters for Packet Layer)   =================
    // ========================================================================

    void addPacketToLayer(const packetObject &packet);
    const std::vector<packetObject> &getPacketLayer() const;
    double getSpecificLinkPowerWatts(size_t tx_unit_index,
                                     size_t rx_unit_index,
                                     size_t slot_to_evaluate) const;

    // ========================================================================
    // === General Getters & Configuration Access =============================
    // ========================================================================

    /** @brief Gets the numerical index of a unit given its label. */
    size_t getUnitIndex(const std::string &unit_label) const;

    /** @brief Gets a list of all unit labels. */
    std::vector<std::string> getUnits() const;

    /** @brief Gets the total number of simulated units. */
    size_t getNumUnits() const;

    /** @brief Gets the total number of simulation time slots. */
    size_t getNumTimeSlots() const;

    /** @brief Gets the number of data elements (doubles) stored per unit per time slot. */
    size_t getNumDataElements() const;

    /** @brief Gets a const reference to the engine's configuration object. */
    const ConfigFile &getConfig() const;

    /** @brief Gets the simulation carrier frequency in Hz. */
    long long getCarrierFrequency() const { return carrierFrequency; }

    /** @brief Gets the index of the current simulation slot (valid during run()). */
    size_t getCurrentSlot() const;

    /** @brief Gets a pointer to the KPI classifier instance. */
    kpiClassifier *getClassifier() const;

    /** @brief Gets a pointer to the power simulator instance. */
    powerSim *getPowerSimulator() const;

    // ========================================================================
    // === Engine Unit Configuration Access ===================================
    // ========================================================================

    /** @brief Gets a const reference to the vector of all engine unit configurations. */
    const std::vector<engineUnit> &getAllEngineUnits() const;

    /** @brief Gets a const reference to the configuration of a specific engine unit by index. */
    const engineUnit &getEngineUnit(size_t unit_index) const;

    /** @brief Sets/updates the configuration of a specific engine unit by index. */
    void setEngineUnit(size_t unit_index, const engineUnit &unit);

    // ========================================================================
    // === Public Members ==
    // ========================================================================
    long long carrierFrequency;        ///< Simulation carrier frequency (Hz). Initialized from config.
    long long bandwidthFrequency;      ///< Simulation bandwidth frequency (Hz). Initialized from config.
    long long temperatureKelvin;       ///< Simulation temperature in Kelvin. Initialized from config.
    double receiverThermalNoise_watts; ///< NEW: Receiver thermal noise in Watts.

    int lastMotionSlot = 0;    ///< Stores the last time motion is made (in simulation slots).
    int firstMotionSlot = 0;   ///< Stores the first time motion is made (in simulation slots).
    int lastAlgorithmSlot = 0; ///< Stores the last time algorithm is in monitoring (after last motion slot).

    // ========================================================================
    // === JIT Packet Parameters ==
    // ========================================================================
    double JIT_packet_min_snr_db;
    int JIT_packet_reception_delay_slots;

    // =======================================================================
    // Experimental Components
    // =======================================================================

    Timestamp scenario_start_time_; // To be set when the run starts
    size_t findSlotForTimestamp(Timestamp timestamp) const;
    std::string serialize_node_data(const std::string &role, size_t unit_idx,
                                    const TimestampedImuData &imu,
                                    const TimestampedPowerData &power,
                                    const TimestampedRotaryData &rotary);
    void deserialize_and_inject_node_data(const std::string &packet_str);
    void requestStop();

}; // End class mobileTHzEngine

#endif // MOBILETHZENGINE_H
