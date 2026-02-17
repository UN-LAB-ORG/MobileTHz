#ifndef IMU_EXP_H
#define IMU_EXP_H

#include <QObject>
#include <QDateTime>
#include <future>
#include <simpleble/SimpleBLE.h>
#include <memory>
#include <mutex>
#include <atomic>

#include "Software/structDefinition.h"

/**
 * @class imuExp
 * @brief Real-world IMU hardware component that provides data on demand.
 *
 * This class manages the connection to a BLE IMU sensor, handling all asynchronous
 * operations internally. It exposes a simple, thread-safe `getLatestData` method
 * for a consumer (like the mobileTHzEngine) to pull the most recent sensor reading.
 */
class imuExp : public QObject
{
    Q_OBJECT

public:
    explicit imuExp(QObject *parent = nullptr);
    ~imuExp() override;

    static bool simulationTesting;

    // Disable copy and move semantics
    imuExp(const imuExp &) = delete;
    imuExp &operator=(const imuExp &) = delete;
    imuExp(imuExp &&) = delete;
    imuExp &operator=(imuExp &&) = delete;

    /**
     * @brief Scans for, connects to, and starts listening to the IMU device.
     * @param config The main configuration file.
     */
    void initialize(const ConfigFile &config);

    /**
     * @brief Retrieves the most recently received IMU data in a thread-safe manner.
     * @return An imuObject struct containing the latest sensor readings.
     */
    TimestampedImuData getLatestData();

signals:
    /**
     * @brief Emitted whenever new data is received from the IMU hardware.
     */
    void imuDataUpdated(const imuObject &data, const QDateTime &timestamp);

private:
    // --- BLE Asynchronous Callback ---
    void onNotification(SimpleBLE::ByteArray data);

    // --- Helper Functions ---
    Quaternion eulerToQuaternion(float roll, float pitch, float yaw) const;

    // --- BLE and State Management ---
    std::unique_ptr<SimpleBLE::Peripheral> peripheral_;
    std::unique_ptr<SimpleBLE::Adapter> adapter_;
    SimpleBLE::BluetoothUUID serviceUUID_;
    SimpleBLE::BluetoothUUID characteristicUUID_;

    // --- Thread-Safe Data Handling ---
    mutable std::mutex dataMutex_; // Mutable to allow locking in a const method
    imuObject latestImuData_;
    Timestamp last_update_timestamp_;            // Store the timestamp
    std::atomic<bool> has_new_data_flag_{false}; // Track if data is fresh

    // --- For one-shot frequency measurement during initialization ---
    std::promise<void> init_freq_promise_;
    std::atomic<bool> is_measuring_init_freq_{false};
    std::vector<std::chrono::steady_clock::time_point> init_timestamps_;
    const int NUM_PACKETS_TO_MEASURE = 10; // Collect 10 packets for an average
    double measureCurrentRate();
};

#endif // IMU_EXP_H
