#include "mobileTHzEngine.h"
#include <QThread>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <qfuture.h>
#include <stdexcept>
#include <vector>

// Software Includes
#include "Software/kpiClassifier/kpiClassifier.h"
#include "Software/structDefinition.h"

// Algorithm Includes
#include "Algorithm/HierarchicalSearch/HierarchicalSearch.h"
#include "Algorithm/IMUAssist/IMUAssist.h"
#include "Algorithm/JustInTime_AP/JustInTime_AP.h"
#include "Algorithm/JustInTime_UE/JustInTime_UE.h"
#include "Algorithm/PerfectAlignment/PerfectAlignment.h"
#include "Algorithm/Static/Static.h"

// Simulation Includes
#include "Simulation/powerSim/powerSim.h"
#include "Simulation/rotarySim/rotarySim.h"

// Experimental Includes
#include "Experimental/imuExp/imuExp.h"
#include "Experimental/powerExp/powerExp.h"
#include "Experimental/rotaryExp/rotaryExp.h"
#include "Experimental/tcpExp/tcpExp.h"

static bool debug = false;

// --- Constants for Math Operations ---
namespace PowerConstants
{
    // For dBm <-> Watts conversion
    // Used in: dBmToWatts -> 0.001 * pow(10, dBm/10) = 0.001 * exp(dBm * (log(10)/10))
    constexpr double LN10_OVER_10 = 0.23025850929940457; // std::log(10.0) / 10.0

    // Used in: wattsToDbm -> 10*log10(W*1000) = 10*log10(W)+30 = (10/log(10))*log(W)+30
    constexpr double TEN_OVER_LN10 = 4.342944819032518; // 10.0 / std::log(10.0)

    constexpr double MIN_DBM_THRESHOLD = -120.0;
    constexpr double MIN_WATTS_THRESHOLD = 1e-15;
} // namespace PowerConstants

mobileTHzEngine::mobileTHzEngine(const ConfigFile &batchConfig)
    : allocated_num_units_(batchConfig.engine_units.size()), allocated_num_time_slots_(static_cast<size_t>(std::ceil(
                                                                 (batchConfig.engine_max_time_sec * 1e6) / batchConfig.engine_slot_time_microsec))),
      time_grid_(nullptr), num_units_(0), num_time_slots_(0), current_slot_(0)
{
    if (allocated_num_units_ == 0 || allocated_num_time_slots_ == 0)
    {
        throw std::invalid_argument("Batch config for engine results in zero units or time slots.");
    }

    initializeDataLayout();

    // ALLOCATE MEMORY based on the maximum possible size
    size_t total_elements = allocated_num_units_ * allocated_num_time_slots_ * num_data_elements_;
    if (total_elements == 0)
    {
        throw std::runtime_error("Calculated total elements for time grid is zero.");
    }

    try
    {
        time_grid_ = new double[total_elements];
    }
    catch (const std::bad_alloc &ba)
    {
        std::cerr << "Memory allocation failed for time_grid_. Requested elements: "
                  << total_elements << " (" << (total_elements * sizeof(double)) / (1024.0 * 1024.0)
                  << " MiB)."
                  << " Error: " << ba.what() << std::endl;
        throw; // Re-throw exception
    }

    // --- Initialize Algorithm Storage & Unit-Specific Data (SERIAL SECTION) ---
    unitAlgorithms_.resize(num_units_);                    // Pre-size the vector
    std::vector<antennaObject> unitTxAntennas(num_units_); // Store unit-specific antennas
}

void mobileTHzEngine::initialize(const ConfigFile &runConfig)
{
    // --- 1. Compatibility Check ---
    size_t run_num_units = runConfig.engine_units.size();
    size_t run_num_time_slots = static_cast<size_t>(
        std::ceil((runConfig.engine_max_time_sec * 1e6) / runConfig.engine_slot_time_microsec));

    if (run_num_units > allocated_num_units_)
    {
        throw std::runtime_error(
            "Engine initialization failed: run config requires more units than allocated.");
    }
    if (run_num_time_slots > allocated_num_time_slots_)
    {
        throw std::runtime_error(
            "Engine initialization failed: run config requires more time slots than allocated.");
    }

    // --- 2. Reset and Configure State for the New Run ---
    // Store the config and set the active dimensions for this run
    config_ = &runConfig;
    num_units_ = run_num_units;
    num_time_slots_ = run_num_time_slots;

    // Set all engine parameters from the new config
    carrierFrequency = config_->engine_carrier_frequency;
    bandwidthFrequency = config_->engine_bandwidth_frequency;
    temperatureKelvin = config_->engine_temperature_kelvin;
    receiverThermalNoise_watts = config_->engine_receiver_thermal_noise_watts;
    JIT_packet_min_snr_db = config_->JIT_packet_min_snr_db;
    JIT_packet_reception_delay_slots = static_cast<int>(config_->JIT_packet_reception_delay_us
                                                        / config_->engine_slot_time_microsec);
    if (JIT_packet_reception_delay_slots < 1)
        JIT_packet_reception_delay_slots = 1;

    // Clear and repopulate unit-specific data
    unit_labels_.clear();
    unit_labels_.reserve(num_units_);
    for (const auto &unit : config_->engine_units)
    {
        unit_labels_.push_back(unit.label);
    }
    engine_units_ = config_->engine_units;

    // Clear out any state from a previous run
    unitAlgorithms_.clear();
    unitAlgorithms_.resize(num_units_);
    packetLayer_.clear();
    current_slot_ = 0;
    firstMotionSlot = 0;
    lastMotionSlot = 0;
    lastAlgorithmSlot = 0;
}

mobileTHzEngine::~mobileTHzEngine()
{
    delete[] time_grid_;
    time_grid_ = nullptr;
}

// Calculates the offsets
void mobileTHzEngine::initializeDataLayout()
{
    // Resize vector to hold offsets for all enum fields + COUNT
    data_offsets_.resize(static_cast<size_t>(DataField::COUNT));

    size_t offset = 0;

    // Helper lambda to add fields and update offset
    auto add_field = [&](DataField field, size_t size)
    {
        // Store the starting offset for this field
        data_offsets_[static_cast<size_t>(field)] = offset;
        // Increment the global offset by the size of this field
        offset += size;
    };

    // Define the layout using the enum and sizes (number of doubles)

    // EnvironmentObject Fields
    add_field(DataField::ENV_POS, 3);     // Position (x, y, z)
    add_field(DataField::ENV_VEL, 3);     // Velocity (x, y, z)
    add_field(DataField::ENV_ACC, 3);     // Acceleration (x, y, z)
    add_field(DataField::ENV_QUAT, 4);    // Quaternion (w, x, y, z)
    add_field(DataField::ENV_ANG_VEL, 3); // Angular Velocity (x, y, z)
    add_field(DataField::ENV_ANG_ACC, 3); // Angular Acceleration (x, y, z)

    // imuObject Fields
    add_field(DataField::IMU_ACC, 3);         // Acceleration (x, y, z)
    add_field(DataField::IMU_ANG_VEL, 3);     // Angular Velocity (x, y, z)
    add_field(DataField::IMU_QUAT, 4);        // Quaternion (w, x, y, z)
    add_field(DataField::IMU_SAMPLE_SLOT, 1); // Last Sampled Slot (double)

    // rxChainObject Fields
    add_field(DataField::RXCHAIN_POWER, 1);       // Power (double)
    add_field(DataField::RXCHAIN_SAMPLE_SLOT, 1); // Last Sampled Slot (double)

    // rxAntennaObject Fields
    add_field(DataField::RXANT_GAIN, 1);             // Gain (double)
    add_field(DataField::RXANT_POWER, 1);            // Power (double)
    add_field(DataField::RXANT_RADIATIONPATTERN, 1); // Pattern ID (stored as double)

    // txAntennaObject Fields
    add_field(DataField::TXANT_GAIN, 1);             // Gain (double)
    add_field(DataField::TXANT_POWER, 1);            // Power (double)
    add_field(DataField::TXANT_RADIATIONPATTERN, 1); // Pattern ID (stored as double)

    // rotaryObject Fields
    add_field(DataField::ROTARY_AZIMUTH, 3);  // Azimuth Axis (pos, vel, acc)
    add_field(DataField::ROTARY_ALTITUDE, 3); // Altitude Axis (pos, vel, acc)
    add_field(DataField::ROTARY_ISMOVING, 1); // Is Moving (0.0 or 1.0)

    // algorithmObject Fields
    add_field(DataField::ALG_STATUS, 1); // Status (double)
    add_field(DataField::ALG_ACTION, 1); // Action (double)

    // classifierObject Fields
    add_field(DataField::CLASSIFIER_OUTPUT, 1); // Classifier Output (double)

    // Total size is the final offset
    num_data_elements_ = offset;

    // Ensure COUNT matches the last used index + 1
    if (data_offsets_.size() != static_cast<size_t>(DataField::COUNT))
    {
        // This should not happen if resize was correct
        throw std::logic_error("Internal error: data_offsets_ size mismatch after initialization.");
    }
    if (num_data_elements_ == 0)
    {
        throw std::logic_error("Data layout initialization resulted in zero size.");
    }
}

size_t mobileTHzEngine::getCurrentSlot() const
{
    return current_slot_;
}

// --- Rotary Command Interface ---
void mobileTHzEngine::issueRotaryCommand(size_t unitIndex, const std::string &commandString)
{
    if (unitIndex >= num_units_)
    {
        std::cerr << "Warning: Rotary command ignored, unit index " << unitIndex << " out of range."
                  << std::endl;
        return;
    }
    if (!engine_units_[unitIndex].Rotary_enabled)
    {
        // Log warning if command issued to disabled rotary
        std::cout << "Warning: Rotary command ignored for disabled unit " << unitIndex << std::endl;
        return;
    }

    if (config_->engine_mode == "SIMULATION")
    {
        if (rotarySimulator)
        { // Use the pointer set in run()
            rotarySimulator->inputFunction(unitIndex, commandString);
        }
        else
        {
            std::cerr << "Warning: Rotary command issued in SIMULATION mode, but rotarySim pointer "
                         "is null (was run() called?)."
                      << std::endl;
        }
    }
    else // EXPERIMENTAL mode
    {
        if (config_->experimental.role == "OBSERVER")
        {
            // I am the Observer, so I need to SEND this command to the correct Node.
            const std::string &target_role = engine_units_[unitIndex].label;

            // --- Build the Standardized Command Packet ---
            json command_packet;
            command_packet["type"] = "COMMAND"; // Use a consistent type name
            command_packet["role"] = "OBSERVER";

            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::high_resolution_clock::now().time_since_epoch())
                                    .count();
            command_packet["packet_timestamp_ns"] = static_cast<int64_t>(now_ns);

            // Create the payload
            json payload;

            // Nest the command inside a category for future flexibility
            // The payload *is* the command itself
            payload["rotary_command"] = commandString;

            command_packet["payload"] = payload;

            if (tcpExperimental)
            {
                tcpExperimental->sendDataToRole(target_role, command_packet.dump() + "\n");
            }
        }
        else // I am a Node
        {
            // A command received by a Node is always for itself.
            // Find my own index to confirm the command is for me.
            const size_t my_unit_idx = getUnitIndex(config_->experimental.role);

            if (unitIndex == my_unit_idx)
            {
                if (rotaryExperimental)
                {
                    rotaryExperimental->issueCommand(commandString);
                }
            }
            else
            {
                std::cerr << "Warning: Node '" << config_->experimental.role
                          << "' received a rotary command intended for unit index " << unitIndex
                          << ", but its own index is " << my_unit_idx << ". Command ignored."
                          << std::endl;
            }
        }
    }
}

// --- Algorithm Creation Helper ---
std::unique_ptr<AlgorithmInterface> mobileTHzEngine::createAlgorithmByName(const std::string &name)
{
    if (name == "HierarchicalSearch") {
        return std::make_unique<HierarchicalSearch>();
    } else if (name == "IMUAssist") {
        return std::make_unique<IMUAssist>();
    } else if (name == "JustInTime_AP") {
        return std::make_unique<JustInTime_AP>();
    } else if (name == "JustInTime_UE") {
        return std::make_unique<JustInTime_UE>();
    } else if (name == "Static") {
        return std::make_unique<Static>();
    } else if (name == "PerfectAlignment") {
        return std::make_unique<PerfectAlignment>();
    } else {
        std::cerr << "Warning: Unknown algorithm name '" << name << "' in createAlgorithmByName."
                  << std::endl;
        return nullptr; // Indicate unknown name
    }
}

// --- Packet Layer ---
void mobileTHzEngine::addPacketToLayer(const packetObject &packet)
{
    if (packet.unit_index < 0 || packet.unit_index >= num_units_)
    {
        throw std::out_of_range("Invalid unit index in addPacketToLayer: " + std::to_string(packet.unit_index));
    }
    if (packet.id < 0)
    {
        throw std::invalid_argument("Invalid packet ID in addPacketToLayer: " + std::to_string(packet.id));
    }
    if (packet.transmit_start_slot < 0 || packet.transmit_start_slot >= num_time_slots_)
    {
        throw std::out_of_range("Invalid transmit start slot in addPacketToLayer: " + std::to_string(packet.transmit_start_slot));
    }
    if (packet.receive_end_slot < 0 || packet.receive_end_slot >= num_time_slots_)
    {
        throw std::out_of_range("Invalid receive end slot in addPacketToLayer: " + std::to_string(packet.receive_end_slot));
    }
    if (packet.receive_end_slot <= packet.transmit_start_slot)
    {
        throw std::invalid_argument(
            "Receive end slot must be after transmit start slot in addPacketToLayer: " + std::to_string(packet.receive_end_slot) + " <= " + std::to_string(packet.transmit_start_slot));
    }
    if (packet.imu_data.size() > 204)
    {
        throw std::length_error("IMU data exceeds maximum size in addPacketToLayer: " + std::to_string(packet.imu_data.size()));
    }

    packetLayer_.push_back(packet);
}

const std::vector<packetObject> &mobileTHzEngine::getPacketLayer() const
{
    return packetLayer_;
}

double mobileTHzEngine::getSpecificLinkPowerWatts(size_t tx_unit_index,
                                                  size_t rx_unit_index,
                                                  size_t slot_to_evaluate) const
{
    if (!powerSimulator)
    {
        return 0.0; // Return 0 watts on error
    }
    if (tx_unit_index >= num_units_ || rx_unit_index >= num_units_)
    {
        std::cerr << "mobileTHzEngine::getSpecificLinkPowerWatts: Invalid unit index." << std::endl;
        return 0.0;
    }
    if (slot_to_evaluate >= num_time_slots_)
    {
        std::cerr << "mobileTHzEngine::getSpecificLinkPowerWatts: Invalid slot index ("
                  << slot_to_evaluate << " vs max " << num_time_slots_ - 1 << ")." << std::endl;
        return 0.0;
    }

    // This now correctly returns watts
    return powerSimulator->calculateReceivedPower(tx_unit_index,
                                                  rx_unit_index,
                                                  engine_units_[tx_unit_index],
                                                  engine_units_[rx_unit_index],
                                                  slot_to_evaluate);
}

// --- Getters ---
size_t mobileTHzEngine::getUnitIndex(const std::string &unit_label) const
{
    auto it = std::find(unit_labels_.begin(), unit_labels_.end(), unit_label);
    if (it != unit_labels_.end())
    {
        return std::distance(unit_labels_.begin(), it);
    }
    std::cerr << "Warning: Unit label '" << unit_label << "' not found in getUnitIndex."
              << std::endl;
    return static_cast<size_t>(-1);
}
std::vector<std::string> mobileTHzEngine::getUnits() const
{
    return unit_labels_;
}
size_t mobileTHzEngine::getNumUnits() const
{
    return num_units_;
}
size_t mobileTHzEngine::getNumTimeSlots() const
{
    return num_time_slots_;
}
size_t mobileTHzEngine::getNumDataElements() const
{
    return num_data_elements_;
}

// --- Engine Unit Functions ---
const ConfigFile &mobileTHzEngine::getConfig() const
{
    if (!config_)
    {
        throw std::runtime_error("getConfig() called before engine was initialized.");
    }
    return *config_;
}

const std::vector<engineUnit> &mobileTHzEngine::getAllEngineUnits() const
{
    return engine_units_;
}

const engineUnit &mobileTHzEngine::getEngineUnit(size_t unit_index) const
{
    if (unit_index >= num_units_)
    {
        throw std::out_of_range("Unit index out of range in getEngineUnit");
    }
    return engine_units_[unit_index];
}
void mobileTHzEngine::setEngineUnit(size_t unit_index, const engineUnit &unit)
{
    if (unit_index >= num_units_)
    {
        throw std::out_of_range("Unit index out of range in setEngineUnit");
    }
    engine_units_[unit_index] = unit;
}

kpiClassifier *mobileTHzEngine::getClassifier() const
{
    return kpiClassifier_;
}

powerSim *mobileTHzEngine::getPowerSimulator() const
{
    return powerSimulator;
}

// --- run() method ---
void mobileTHzEngine::run(kpiClassifier *kpi_classifier,
                          plotSaver *plot_saver,
                          powerSim *power_sim,
                          rotarySim *rotary_sim,
                          powerExp *power_exp,
                          rotaryExp *rotary_exp,
                          imuExp *imu_exp,
                          tcpExp *tcp_exp)
{
    try
    {
        // Check for simulaiton
        if (config_->engine_mode == "SIMULATION")
        {
            // --- Assign Algorithm 'last' ---
            for (size_t unit_index = 0; unit_index < num_units_; ++unit_index)
            {
                std::string algoName = engine_units_[unit_index].algorithm;
                if (algoName != "none" && !algoName.empty())
                {
                    unitAlgorithms_[unit_index] = createAlgorithmByName(algoName);
                    if (unitAlgorithms_[unit_index])
                    {
                        unitAlgorithms_[unit_index]->configure(*config_, engine_units_[unit_index]);
                        if (debug)
                            std::cout << "  Unit " << unit_index << " ("
                                      << engine_units_[unit_index].label
                                      << "): Configured Algorithm '" << algoName << "'"
                                      << std::endl;
                    }
                    else
                    {
                        std::cerr << "  Unit " << unit_index << " ("
                                  << engine_units_[unit_index].label
                                  << "): Warning - Unknown algorithm name '" << algoName
                                  << "'. No algorithm assigned." << std::endl;
                        unitAlgorithms_[unit_index] = nullptr; // Set to null if creation failed
                    }
                }
                else
                {
                    std::cerr << "  Unit " << unit_index << " (" << engine_units_[unit_index].label
                              << "): No algorithm assigned ('" << algoName << "')." << std::endl;
                    unitAlgorithms_[unit_index] = nullptr; // Set to null if none specified
                }
            }

            if (!power_sim)
            {
                std::cerr << "Error: power_sim pointer is null in run() method." << std::endl;
                return;
            }
            else
            {
                powerSimulator = power_sim;
            }
            if (!rotary_sim)
            {
                std::cerr << "Error: rotary_sim pointer is null in run() method." << std::endl;
                return;
            }
            else
            {
                rotarySimulator = rotary_sim;
            }
            if (!kpi_classifier)
            {
                std::cerr << "Error: kpi_classifier pointer is null in run() method." << std::endl;
                return;
            }
            else
            {
                kpiClassifier_ = kpi_classifier;
            }
            if (!plot_saver)
            {
                std::cerr << "Error: tikz_saver pointer is null in run() method." << std::endl;
                return;
            }
            else
            {
                plotSaver_ = plot_saver;
            }
            if (debug)
                std::cout << "Starting simulation run for " << num_time_slots_ << " time slots..."
                          << std::endl;
        }
        else if (config_->engine_mode == "EXPERIMENTAL")
        {
            // Only the observer node runs algorithms
            if (config_->experimental.role == "OBSERVER")
            {
                // --- Assign Algorithm 'last' ---
                for (size_t unit_index = 0; unit_index < num_units_; ++unit_index)
                {
                    std::string algoName = engine_units_[unit_index].algorithm;
                    if (algoName != "none" && !algoName.empty())
                    {
                        unitAlgorithms_[unit_index] = createAlgorithmByName(algoName);
                        if (unitAlgorithms_[unit_index])
                        {
                            unitAlgorithms_[unit_index]->configure(*config_,
                                                                   engine_units_[unit_index]);
                            if (debug)
                                std::cout << "  Unit " << unit_index << " ("
                                          << engine_units_[unit_index].label
                                          << "): Configured Algorithm '" << algoName << "'"
                                          << std::endl;
                        }
                        else
                        {
                            std::cerr << "  Unit " << unit_index << " ("
                                      << engine_units_[unit_index].label
                                      << "): Warning - Unknown algorithm name '" << algoName
                                      << "'. No algorithm assigned." << std::endl;
                            unitAlgorithms_[unit_index] = nullptr; // Set to null if creation failed
                        }
                    }
                    else
                    {
                        std::cerr << "  Unit " << unit_index << " ("
                                  << engine_units_[unit_index].label
                                  << "): No algorithm assigned ('" << algoName << "')."
                                  << std::endl;
                        unitAlgorithms_[unit_index] = nullptr; // Set to null if none specified
                    }
                }
            }

            if (!power_exp)
            {
                std::cerr << "Error: power_exp pointer is null in run() method." << std::endl;
                return;
            }
            else
            {
                powerExperimental = power_exp;
            }
            if (!rotary_exp)
            {
                std::cerr << "Error: rotary_exp pointer is null in run() method." << std::endl;
                return;
            }
            else
            {
                rotaryExperimental = rotary_exp;
            }
            if (!imu_exp)
            {
                std::cerr << "Error: imu_exp pointer is null in run() method." << std::endl;
                return;
            }
            else
            {
                imuExperimental = imu_exp;
            }
            if (!tcp_exp)
            {
                std::cerr << "Error: tcp_exp pointer is null in run() method." << std::endl;
                return;
            }
            else
            {
                tcpExperimental = tcp_exp;
            }
            if (!kpi_classifier)
            {
                std::cerr << "Error: kpi_classifier pointer is null in run() method." << std::endl;
                return;
            }
            else
            {
                kpiClassifier_ = kpi_classifier;
            }
            if (!plot_saver)
            {
                std::cerr << "Error: tikz_saver pointer is null in run() method." << std::endl;
                return;
            }
            else
            {
                plotSaver_ = plot_saver;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error during run() initialization: " << e.what() << std::endl;
        return; // Exit if initialization fails
    }

    // Direct-function vector for algorithms
    std::vector<std::function<void(mobileTHzEngine *, size_t, size_t)>> algorithm_processors;
    algorithm_processors.resize(num_units_);
    std::vector<std::function<algorithmObject()>> algorithm_status_getters;
    algorithm_status_getters.resize(num_units_);

    // Does not run if in experimental mode and role is NOT OBSERVER
    if (config_->engine_mode == "EXPERIMENTAL" && config_->experimental.role != "OBSERVER")
    {
        // Create empty functions for all units
        for (size_t i = 0; i < num_units_; ++i)
        {
            algorithm_processors[i] = [](mobileTHzEngine *, size_t, size_t) { /* Do nothing */ };
            algorithm_status_getters[i] = []() -> algorithmObject
            {
                return {static_cast<double>(AlgoState::ERROR_STATUS),
                        static_cast<double>(AlgoAction::NONE)};
            };
        }
    }
    else
    {
        for (size_t i = 0; i < num_units_; ++i)
        {
            if (unitAlgorithms_[i])
            {
                // Get the raw pointer to the concrete algorithm object.
                AlgorithmInterface *const algo = unitAlgorithms_[i].get();

                // Create the wrapper for processSlot.
                algorithm_processors[i] =
                    [algo](mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot)
                {
                    algo->processSlot(engine, unitIndex, currentSlot);
                };

                algorithm_status_getters[i] = [algo]() -> algorithmObject
                {
                    return algo->getStatus();
                };
            }
            else
            {
                // Create empty functions for units with no algorithm.
                algorithm_processors[i] = [](mobileTHzEngine *, size_t, size_t) { /* Do nothing */ };

                // Create a getter that returns a default state ---
                algorithm_status_getters[i] = []() -> algorithmObject
                {
                    return {static_cast<double>(AlgoState::ERROR_STATUS),
                            static_cast<double>(AlgoAction::NONE)};
                };
            }
        }
    }

    // ========================================================================
    // --- MODE: SIMULATION ---
    // ========================================================================

    if (config_->engine_mode == "SIMULATION")
    {
        // Loop through each time slot
        for (current_slot_ = 0; current_slot_ < num_time_slots_; ++current_slot_)
        {
            // --------------------------------------------------------------
            // 1. Process rotary simulator first, as its state might affect antenna pointing/power
            rotarySimulator->processSlot(current_slot_);
            // --------------------------------------------------------------

            // --------------------------------------------------------------
            // 2. Loop through each unit acting as a potential RECEIVER
            // --------------------------------------------------------------
            for (size_t rx_unit_index = 0; rx_unit_index < num_units_; ++rx_unit_index)
            {
                try
                {
                    // --------------------------------------------------------------
                    // 1. Get RX Antenna state & Calculate Received Power
                    // --------------------------------------------------------------

                    double total_received_power_watts = 0.0; // Initialize power sum in Watts
                    bool current_rx_antenna_enabled = engine_units_[rx_unit_index].RxAntenna_enabled;

                    if (current_rx_antenna_enabled)
                    {
                        // Loop through all other units acting as potential TRANSMITTERS
                        for (size_t tx_unit_index = 0; tx_unit_index < num_units_; ++tx_unit_index)
                        {
                            if (tx_unit_index == rx_unit_index)
                                continue; // Unit cannot transmit to itself

                            // Check if the potential TX unit has an enabled TX antenna
                            if (engine_units_[tx_unit_index].TxAntenna_enabled)
                            {
                                // Calculate power received from this specific TX unit
                                double received_power_watts = powerSimulator->calculateReceivedPower(
                                    tx_unit_index,                // Pass TX index
                                    rx_unit_index,                // Pass RX index
                                    engine_units_[tx_unit_index], // Pass units directly
                                    engine_units_[rx_unit_index],
                                    current_slot_); // Pass time index

                                // Convert dBm to Watts before summing
                                total_received_power_watts += received_power_watts;
                            }
                        } // End TX unit loop
                    } // End if current_rx_antenna.enabled

                    // Get the potentially updated RX antenna data for the current slot
                    antennaObject current_rx_antenna = getRxAntennaData(rx_unit_index,
                                                                        current_slot_);

                    // Update the RX antenna power field (convert total Watts back to dBm)
                    // Only update if the antenna was enabled, otherwise keep its potentially default value
                    if (current_rx_antenna_enabled)
                    {
                        current_rx_antenna.power_watts = total_received_power_watts;
                    }

                    // Write the updated antenna object back to the engine's grid
                    setRxAntennaData(rx_unit_index, current_slot_, current_rx_antenna);

                    // --------------------------------------------------------------
                    // 2. Update RxChain State (Sampling based on Antenna Power)
                    // --------------------------------------------------------------
                    rxChainObject current_rxChain = getRxChainData(rx_unit_index,
                                                                   current_slot_);

                    if (engine_units_[rx_unit_index].RxChain_enabled)
                    {
                        // Get the unit-specific block duration/interval
                        size_t interval = engine_units_[rx_unit_index].RxChain_samplingDelay_slots;
                        size_t source_slot = 0;

                        if (interval > 0)
                        {
                            // Calculate the index of the block the current slot is in
                            size_t current_block_index = current_slot_ / interval;

                            // The source data comes from the start of the previous block
                            // (or block 0 if current_block_index is 0)
                            size_t source_block_index = (current_block_index > 0)
                                                            ? (current_block_index - 1)
                                                            : 0;

                            // Calculate the actual slot index where the source data originates
                            source_slot = source_block_index * interval;
                        }
                        else
                        {
                            // Interval is 0, means instantaneous sampling (no delay/blocking)
                            source_slot = current_slot_;
                        }

                        // --- Fetch data from the calculated source_slot ---
                        // Get the state of the ANTENNA at the calculated source_slot
                        const antennaObject &source_antenna = current_rx_antenna;

                        // Update the current RxChain state with data from the source slot
                        current_rxChain.power_watts = source_antenna.power_watts;
                        current_rxChain.sample_slot = source_slot; // Store the slot the data actually came from

                        // Write the updated RxChain object back
                        setRxChainData(rx_unit_index, current_slot_, current_rxChain);
                    }
                    else
                    { // RxChain not enabled
                        current_rxChain.power_watts = -9999999;
                        current_rxChain.sample_slot = 0; // Or SIZE_MAX
                        setRxChainData(rx_unit_index, current_slot_, current_rxChain);
                    }
                }
                catch (const std::exception &e)
                {
                    std::cerr << "Error during simulation loop for RX unit " << rx_unit_index
                              << ", slot " << current_slot_ << ": " << e.what() << std::endl;
                    // Consider setting error states for the affected unit/slot if needed
                }
            }

            // --------------------------------------------------------------
            // 3. Process Algorithms AFTER Rotary and Power/RxChain updates + Classifier
            // --------------------------------------------------------------
            for (size_t unit_index = 0; unit_index < num_units_; ++unit_index)
            {
                // Call the processSlot wrapper (direct, inlinable call)
                algorithm_processors[unit_index](this,
                                                 unit_index,
                                                 current_slot_); // Calls processSlot
                setAlgorithmData(
                    unit_index,
                    current_slot_,
                    algorithm_status_getters[unit_index]()); // Calls getStatus and updates the grid

                if (kpiClassifier_ && kpiClassifier_->isUnitSelected(unit_index))
                {
                    kpiClassifier_->processSlot(unit_index, current_slot_);
                }
            } // End algorithm/classifier loop
        }
    }
    else if (config_->engine_mode == "EXPERIMENTAL")
    { // Real-World Mode

        // =============================================================
        // ROLE: OBSERVER
        // =============================================================
        if (config_->experimental.role == "OBSERVER")
        {
            // The number of nodes to wait for is simply the size of the engine_unit array.
            const size_t expected_nodes = config_->engine_units.size();

            while (tcpExperimental->getConnectedClientCount() < expected_nodes && !stop_flag_.load())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // If we broke the loop because of the stop flag, exit early.
            if (stop_flag_.load())
            {
                return;
            }
            // Pause briefly to ensure all nodes are ready
            std::this_thread::sleep_for(std::chrono::seconds(1));
            scenario_start_time_ = std::chrono::high_resolution_clock::now();

            bool rotary_command_sent = false;
            const double target_time_sec = 0.5;
            const double target_time2_sec = 3;
            const double slot_duration_sec = config_->engine_slot_time_microsec / 1e6;
            const size_t target_slot_for_command = static_cast<size_t>(target_time_sec / slot_duration_sec);
            const size_t target_slot_for_command2 = static_cast<size_t>(target_time2_sec / slot_duration_sec);
            const std::string command_to_send = "mdd 0 10 1 10";
            const std::string command_to_send2 = "mdd 0 -10 1 -10";

            for (current_slot_ = 0; current_slot_ < num_time_slots_; ++current_slot_)
            {
                auto slot_start_time = std::chrono::high_resolution_clock::now();

                // 1. Broadcast UPDATE message and RECORD the time.
                json update_msg;
                update_msg["type"] = "UPDATE";
                tcpExperimental->broadcastData(update_msg.dump() + "\n");
                auto update_sent_time = std::chrono::high_resolution_clock::now();

                // 2. Wait for SENSOR_DATA from ALL nodes.
                std::set<std::string> received_from_roles;
                std::map<std::string, Timestamp> arrival_times;

                while (received_from_roles.size() < expected_nodes && !stop_flag_.load())
                {
                    if (tcpExperimental->hasIncomingData())
                    {
                        auto [client_id, packet] = tcpExperimental->getNextPacket();
                        std::string sender_role = tcpExperimental->getRoleForClientId(client_id);
                        if (!sender_role.empty())
                        {
                            // RECORD an arrival time only the first time we hear from this role in this slot
                            if (received_from_roles.find(sender_role) == received_from_roles.end())
                            {
                                arrival_times[sender_role] = std::chrono::high_resolution_clock::now();
                            }
                            deserialize_and_inject_node_data(packet);
                            received_from_roles.insert(sender_role);
                        }
                    }
                    else
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                }

                if (received_from_roles.find("UE") != received_from_roles.end() && received_from_roles.find("AP") != received_from_roles.end())
                {
                    antennaObject ue_rx_antenna = getRxAntennaData(getUnitIndex("UE"),
                                                                   current_slot_);
                    setRxAntennaData(getUnitIndex("AP"), current_slot_, ue_rx_antenna);
                    rxChainObject ue_rx_chain = getRxChainData(getUnitIndex("UE"), current_slot_);
                    setRxChainData(getUnitIndex("AP"), current_slot_, ue_rx_chain);
                }

                // If we broke this inner loop because of the stop flag, also break the outer loop.
                if (stop_flag_.load())
                    break;

                // CALCULATE AND LOG LATENCIES
                for (const auto &[role, arrival_time] : arrival_times)
                {
                    auto latency = std::chrono::duration_cast<std::chrono::microseconds>(
                        arrival_time - update_sent_time);
                    double latency_ms = static_cast<double>(latency.count()) / 1000.0;

                    std::string log_msg = "Latency for " + role + ": " + std::to_string(latency_ms) + " ms";
                    tcpExperimental->addLatencyLogEntry(log_msg);
                }

                // 2. Check if it's time to send the rotary command
                if (current_slot_ == target_slot_for_command && !rotary_command_sent)
                {
                    // Loop through all units configured in the engine
                    for (size_t unit_idx = 0; unit_idx < num_units_; ++unit_idx)
                    {
                        // Only send the command if the unit is configured to have a rotary motor
                        if (getEngineUnit(unit_idx).Rotary_enabled)
                        {
                            issueRotaryCommand(unit_idx, command_to_send);
                        }
                    }
                } // End first command send check

                // 2b. Check if it's time to send the second rotary command
                if (current_slot_ == target_slot_for_command2 && !rotary_command_sent)
                {
                    // Loop through all units configured in the engine
                    for (size_t unit_idx = 0; unit_idx < num_units_; ++unit_idx)
                    {
                        // Only send the command if the unit is configured to have a rotary motor
                        if (getEngineUnit(unit_idx).Rotary_enabled)
                        {
                            issueRotaryCommand(unit_idx, command_to_send2);
                        }
                    }
                    rotary_command_sent = true; // Ensure we only send once
                }

                // --------------------------------------------------------------
                // 3. Process Algorithms AFTER Rotary and Power/RxChain updates + Classifier
                // --------------------------------------------------------------
                for (size_t unit_index = 0; unit_index < num_units_; ++unit_index)
                {
                    // Call the processSlot wrapper (direct, inlinable call)
                    algorithm_processors[unit_index](this,
                                                     unit_index,
                                                     current_slot_); // Calls processSlot
                    setAlgorithmData(unit_index,
                                     current_slot_,
                                     algorithm_status_getters
                                         [unit_index]()); // Calls getStatus and updates the grid

                    if (kpiClassifier_ && kpiClassifier_->isUnitSelected(unit_index))
                    {
                        kpiClassifier_->processSlot(unit_index, current_slot_);
                    }
                } // End algorithm/classifier loop

                waitForNextSlot(slot_start_time);
            }
        }
        // =============================================================
        // ROLE: NODE (any role that is not "OBSERVER")
        // =============================================================
        else
        {
            // A Node must find its own definition within the engine_unit array.
            const std::string my_role = config_->experimental.role;
            const size_t my_unit_idx = getUnitIndex(my_role);

            if (my_unit_idx == static_cast<size_t>(-1))
            {
                throw std::runtime_error("Node role '" + my_role + "' defined in 'experimental' block was not found in the "
                                                                   "'engine_unit' array.");
            }

            std::cout << "[" << my_role << "] Identified as Unit Index " << my_unit_idx
                      << ". Waiting for messages from Observer..." << std::endl;

            while (tcpExperimental->getConnectionStatus() == tcpExp::ConnectionStatus::Connected && !stop_flag_.load())
            {
                std::string packet = tcpExperimental->getNextPacketFromServer();
                if (packet.empty())
                {
                    std::cerr << "[" << my_role << "] Connection to Observer lost. Shutting down."
                              << std::endl;
                    break;
                }

                json msg = json::parse(packet, nullptr, false);
                if (msg.is_discarded() || !msg.contains("type"))
                    continue;
                std::string msg_type = msg["type"];

                if (msg_type == "UPDATE")
                {
                    // Gather all available local sensor data using the correct index.
                    TimestampedImuData imu;
                    if (engine_units_[my_unit_idx].IMU_enabled)
                    {
                        imu = imuExperimental->getLatestData();
                    }

                    TimestampedPowerData power;
                    if (engine_units_[my_unit_idx].RxChain_enabled)
                    {
                        power = powerExperimental->getLatestData();
                    }

                    TimestampedRotaryData rotary;
                    if (engine_units_[my_unit_idx].Rotary_enabled)
                    {
                        rotary = rotaryExperimental->getLatestData();
                    }

                    // Serialize and send the SENSOR_DATA bundle to the Observer.
                    std::string sensor_packet = serialize_node_data(my_role,
                                                                    my_unit_idx,
                                                                    imu,
                                                                    power,
                                                                    rotary);
                    tcpExperimental->sendDataToServer(sensor_packet + "\n");
                }
                else if (msg_type == "COMMAND")
                {
                    // 1. Check for the payload object
                    if (msg.contains("payload") && msg["payload"].is_object())
                    {
                        const auto &payload = msg["payload"];

                        // 2. Check for a rotary command within the payload
                        if (payload.contains("rotary_command"))
                        {
                            // 3. Safely get the command string
                            std::string command_str = payload["rotary_command"].get<std::string>();

                            if (!command_str.empty())
                            {
                                // 4. We have the command! Execute it using the existing function.
                                issueRotaryCommand(my_unit_idx, command_str);
                            }
                        }
                    }
                }
            }
        }
    }

    // --- Final Actions for SIMULATION and OBSERVER roles ---
    if (config_->engine_mode == "EXPERIMENTAL" && config_->experimental.role != "OBSERVER")
    {
        // In simulation, only one finalizeClassification() call is needed.
        // In experimental, there are 3 runs of the MobileTHz program. Only the Observer needs to call this.
    }
    else
    {
        if (kpiClassifier_)
        {
            kpiClassifier_->finalizeClassification();
        }
    }
    if (debug)
    {
        std::cout << std::endl
                  << "MobileTHz Run complete." << std::endl;
    }
}

// --- waitForNextSlot() method  ---
void mobileTHzEngine::waitForNextSlot(Timestamp slot_start_time)
{
    if (config_->engine_mode == "SIMULATION")
    {
        return; // No delay in simulation
    }

    // Calculate the target end time for the current slot
    auto target_duration = std::chrono::microseconds(
        static_cast<long long>(config_->engine_slot_time_microsec));
    auto target_end_time = slot_start_time + target_duration;

    // If we are already past the target end time, skip waiting but log a warning
    auto now = std::chrono::high_resolution_clock::now();
    if (now >= target_end_time)
    {
        auto delay = std::chrono::duration_cast<std::chrono::microseconds>(now - target_end_time)
                         .count();
        std::cerr << "Warning: Slot processing overran by " << delay << " microseconds."
                  << std::endl;
        return;
    }

    // wait loop.
    while (std::chrono::high_resolution_clock::now() < target_end_time)
    {
        std::this_thread::yield();
    }
}

size_t mobileTHzEngine::findSlotForTimestamp(Timestamp timestamp) const
{
    if (config_->engine_slot_time_microsec <= 0)
        return 0;

    auto duration_since_start = std::chrono::duration_cast<std::chrono::microseconds>(timestamp - scenario_start_time_);
    return static_cast<size_t>(duration_since_start.count() / config_->engine_slot_time_microsec);
}

std::string mobileTHzEngine::serialize_node_data(const std::string &role, size_t unit_idx,
                                                 const TimestampedImuData &imu,
                                                 const TimestampedPowerData &power,
                                                 const TimestampedRotaryData &rotary)
{
    json root;
    root["type"] = "SENSOR_DATA";
    root["role"] = role;
    root["unit_index"] = unit_idx;
    const auto &unit_config = getEngineUnit(unit_idx);

    // Optional packet-level timestamp (ns since epoch)
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::high_resolution_clock::now().time_since_epoch())
                            .count();
    root["packet_timestamp_ns"] = static_cast<int64_t>(now_ns);

    json payload = json::object();

    // IMU
    if (unit_config.IMU_enabled)
    {
        json imu_j;
        imu_j["timestamp"] = static_cast<int64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(imu.timestamp.time_since_epoch()).count());

        json imu_d = json::object();
        imu_d["acceleration"] = {
            {"x", imu.data.acceleration.x},
            {"y", imu.data.acceleration.y},
            {"z", imu.data.acceleration.z}};
        imu_d["angular_velocity"] = {
            {"x", imu.data.angular_velocity.x},
            {"y", imu.data.angular_velocity.y},
            {"z", imu.data.angular_velocity.z}};
        imu_d["quaternion"] = {
            {"w", imu.data.quaternion.w},
            {"x", imu.data.quaternion.x},
            {"y", imu.data.quaternion.y},
            {"z", imu.data.quaternion.z}};

        imu_j["data"] = std::move(imu_d);
        payload["imu"] = std::move(imu_j);
    }
    else
    {
        payload["imu"] = nullptr;
    }

    // Power
    if (unit_config.RxChain_enabled)
    {
        // * If label is 'AP' then don't send power data
        if (unit_config.label == "AP")
        {
            payload["power"] = nullptr;
        }
        else
        {
            json pow_j;
            pow_j["timestamp"] = static_cast<int64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(power.timestamp.time_since_epoch()).count());

            json pow_d = json::object();
            pow_d["power_watts"] = power.power_watts;

            pow_j["data"] = std::move(pow_d);
            payload["power"] = std::move(pow_j);
        }
    }
    else
    {
        payload["power"] = nullptr;
    }

    // Rotary
    if (unit_config.Rotary_enabled)
    {
        json rot_j;
        rot_j["timestamp"] = static_cast<int64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(rotary.timestamp.time_since_epoch()).count());

        json rot_d = json::object();
        rot_d["azimuth"] = {{"angle", rotary.data.azimuth.angle},
                            {"velocity", rotary.data.azimuth.velocity},
                            {"acceleration", rotary.data.azimuth.acceleration}};
        rot_d["altitude"] = {
            {"angle", rotary.data.altitude.angle},
            {"velocity", rotary.data.altitude.velocity},
            {"acceleration", rotary.data.altitude.acceleration}};
        rot_d["isMoving"] = rotary.data.isMoving;

        rot_j["data"] = std::move(rot_d);
        payload["rotary"] = std::move(rot_j);
    }
    else
    {
        payload["rotary"] = nullptr;
    }

    root["payload"] = std::move(payload);
    return root.dump();
}

void mobileTHzEngine::deserialize_and_inject_node_data(const std::string &packet_str)
{

    try
    {
        json packet = json::parse(packet_str, nullptr, true, true);
        if (!packet.contains("unit_index") || !packet.contains("payload"))
            return;

        size_t unit_idx = static_cast<size_t>(packet["unit_index"].get<size_t>());
        if (unit_idx >= getNumUnits())
        {
            std::cerr << "Warning: Received data for invalid unit index " << unit_idx << "." << std::endl;
            return;
        }

        // Make sure packet timestamp is valid from other nodes by comparing with current time and delta
        if (packet.contains("packet_timestamp_ns") && packet["packet_timestamp_ns"].is_number_integer())
        {
            int64_t pkt_ts_ns = packet["packet_timestamp_ns"].get<int64_t>();
            Timestamp pkt_ts((std::chrono::nanoseconds(pkt_ts_ns)));
            Timestamp now = std::chrono::high_resolution_clock::now();
            auto delta = std::chrono::duration_cast<std::chrono::seconds>(now - pkt_ts).count();
            if (std::abs(delta) > 1) // 1 second tolerance
            {
                std::cerr << "Warning: Packet timestamp from unit " << unit_idx
                          << " differs from current time by " << delta
                          << " seconds. Possible clock sync issue." << std::endl;
            }
        }

        const json &payload = packet["payload"];

        // IMU
        if (payload.contains("imu") && !payload["imu"].is_null())
        {
            const json &imu_j = payload["imu"];
            int64_t ts_ns = imu_j.contains("timestamp") ? imu_j["timestamp"].get<int64_t>() : 0;
            Timestamp ts((std::chrono::nanoseconds(ts_ns)));
            size_t data_slot = findSlotForTimestamp(ts);
            size_t engine_slot = getCurrentSlot();

            imuObject imu_data{};
            if (imu_j.contains("data") && imu_j["data"].is_object())
            {
                const json &imu_d = imu_j["data"];
                if (imu_d.contains("acceleration") && imu_d["acceleration"].is_object())
                {
                    const json &acc = imu_d["acceleration"];
                    imu_data.acceleration.x = acc["x"];
                    imu_data.acceleration.y = acc["y"];
                    imu_data.acceleration.z = acc["z"];
                }
                if (imu_d.contains("angular_velocity") && imu_d["angular_velocity"].is_object())
                {
                    const json &ang = imu_d["angular_velocity"];
                    imu_data.angular_velocity.x = ang["x"];
                    imu_data.angular_velocity.y = ang["y"];
                    imu_data.angular_velocity.z = ang["z"];
                }
                if (imu_d.contains("quaternion") && imu_d["quaternion"].is_object())
                {
                    const json &q = imu_d["quaternion"];
                    imu_data.quaternion.w = q["w"];
                    imu_data.quaternion.x = q["x"];
                    imu_data.quaternion.y = q["y"];
                    imu_data.quaternion.z = q["z"];
                }
            }
            imu_data.sample_slot = static_cast<double>(data_slot);
            setIMUData(unit_idx, engine_slot, imu_data);
        }

        // Power
        if (payload.contains("power") && !payload["power"].is_null())
        {
            const json &pow_j = payload["power"];
            int64_t ts_ns = pow_j.contains("timestamp") ? pow_j["timestamp"].get<int64_t>() : 0;
            Timestamp ts((std::chrono::nanoseconds(ts_ns)));
            size_t data_slot = findSlotForTimestamp(ts);
            size_t engine_slot = getCurrentSlot();

            double power_watts = 0.0;
            if (pow_j.contains("data") && pow_j["data"].is_object())
            {
                const json &pd = pow_j["data"];
                if (pd.contains("power_watts") && pd["power_watts"].is_number())
                    power_watts = pd["power_watts"].get<double>();
            }

            // Update RxAntenna
            antennaObject rx_ant = getRxAntennaData(unit_idx, engine_slot);
            rx_ant.power_watts = power_watts;
            setRxAntennaData(unit_idx, engine_slot, rx_ant);

            // Update RxChain
            rxChainObject rx_chain = getRxChainData(unit_idx, engine_slot);
            rx_chain.power_watts = power_watts;
            rx_chain.sample_slot = static_cast<double>(data_slot);
            setRxChainData(unit_idx, engine_slot, rx_chain);
        }

        // Rotary
        if (payload.contains("rotary") && !payload["rotary"].is_null())
        {
            const json &rot_j = payload["rotary"];
            int64_t ts_ns = rot_j.contains("timestamp") ? rot_j["timestamp"].get<int64_t>() : 0;
            Timestamp ts((std::chrono::nanoseconds(ts_ns)));
            size_t data_slot = findSlotForTimestamp(ts);
            size_t engine_slot = getCurrentSlot();

            rotaryObject rot{};
            if (rot_j.contains("data") && rot_j["data"].is_object())
            {
                const json &rd = rot_j["data"];
                if (rd.contains("azimuth") && rd["azimuth"].is_object())
                {
                    const json &az = rd["azimuth"];
                    rot.azimuth.angle = az["angle"];
                    rot.azimuth.velocity = az["velocity"];
                    rot.azimuth.acceleration = az["acceleration"];
                }
                if (rd.contains("altitude") && rd["altitude"].is_object())
                {
                    const json &al = rd["altitude"];
                    rot.altitude.angle = al["angle"];
                    rot.altitude.velocity = al["velocity"];
                    rot.altitude.acceleration = al["acceleration"];
                }
                rot.isMoving = rd.contains("isMoving") && rd["isMoving"].is_number()
                                   ? rd["isMoving"].get<double>()
                                   : 0.0;
            }
            setRotaryData(unit_idx, engine_slot, rot);
        }
    }
    catch (const json::parse_error &e)
    {
        std::cerr << "JSON Error: " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Deserialize Error: " << e.what() << std::endl;
    }
}

void mobileTHzEngine::requestStop()
{
    stop_flag_.store(true);
}
