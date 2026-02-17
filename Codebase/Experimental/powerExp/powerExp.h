#ifndef POWER_EXP_H
#define POWER_EXP_H

#include <QDateTime>
#include <QObject>
#include <atomic>
#include <mutex>
#include <thread>

#include "Software/structDefinition.h"

#if ENABLE_DSO
#include <visa.h>
#endif

class mobileTHzEngine;
struct Quaternion;
struct Position;
struct radiationPattern;
struct rotaryObject;

/**
 * @class powerExp
 * @brief Real-world power measurement component with an interface compatible with the engine.
 *
 * This class encapsulates all logic for interacting with a hardware power measurement device (DSO).
 * It runs an internal polling thread to continuously fetch power readings and makes the latest
 * value available through a `calculateReceivedPower` method, mirroring the `powerSim` interface.
 */
class powerExp : public QObject
{
    Q_OBJECT

public:
    powerExp(QObject *parent = nullptr);
    ~powerExp() override;

    static bool simulationTesting;

    /**
     * @brief Initializes the hardware connection and starts the internal polling thread.
     * @param paramConfig The main configuration file containing DSO settings.
     * @param engine_ptr Pointer to the engine (for interface compatibility, not used internally).
     */
    void initialize(const ConfigFile &paramConfig, mobileTHzEngine *engine_ptr);

    /**
     * @brief Returns the most recent power measurement in dBm.
     * @return Latest power measurement in dBm.
     */
    TimestampedPowerData getLatestData();

private:
    // --- Internal Polling Loop ---
    void pollingLoop();

// --- DSO Specific Functions ---
#if ENABLE_DSO
    void initializeDSO(const std::string &resource_string, const char *channel);
    float measureChannelPower();
    void handleError(const std::string &errorMessage, ViStatus errorStatus);
    float NR3ToFloat(const std::string &nr3_str);

    // DSO VISA Session Handles
    ViSession dso_session_ = VI_NULL;
    ViSession rm_session_ = VI_NULL;
    const ViUInt32 RESPONSE_BUFFER_SIZE = 256;
#endif

    // --- Threading and Data ---
    std::thread pollingThread_;
    std::atomic<bool> stopFlag_{false};
    mutable std::mutex data_mutex_;
    double latestPower_watts_{-99.0}; // Store power directly in watts
    Timestamp last_update_timestamp_;
    std::atomic<bool> has_new_data_flag_{false};

    // --- Cached Configuration ---
    std::atomic<int64_t> IF_frequency_{0};
    std::atomic<int64_t> meas_bw_{0};
    std::atomic<int> delay_us{0};

    const double ERROR_POWER_DBM = -9999.0;
};

#endif // POWER_EXP_H
