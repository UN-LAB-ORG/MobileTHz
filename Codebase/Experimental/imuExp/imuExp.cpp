#include "imuExp.h"
#include <cmath>
#include <future>
#include <iostream>
#include <random>
#include <stdexcept>
#include <iomanip>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define and initialize the static flag
bool imuExp::simulationTesting = false;
static bool debug = false;

namespace
{
    int16_t parseS16(const uint8_t *data)
    {
        return static_cast<int16_t>((data[1] << 8) | data[0]);
    }
    // Helper function to convert degrees to radians
    inline double degreesToRadians(double degrees)
    {
        return degrees * M_PI / 180.0;
    }
} // namespace

imuExp::imuExp(QObject *parent)
    : QObject(parent)
{
    // Initialize the shared data with default values
    latestImuData_ = imuObject{};
}

imuExp::~imuExp()
{
    if (peripheral_ && peripheral_->is_connected())
    {
        try
        {
            if (!serviceUUID_.empty() && !characteristicUUID_.empty())
            {
                peripheral_->unsubscribe(serviceUUID_, characteristicUUID_);
            }
            peripheral_->disconnect();
            std::cout << "[IMU] Disconnected from device." << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[IMU] Error during disconnection: " << e.what() << std::endl;
        }
    }
}

void imuExp::initialize(const ConfigFile &config)
{
    const std::string &my_role = config.experimental.role;

    if (my_role == "OBSERVER")
    {
        return; // Observer has no IMU hardware or simulator
    }

    const engineUnit *my_unit_config = nullptr; // Find this Node's specific unit configuration
    for (const auto &unit : config.engine_units)
    {
        if (unit.label == my_role)
        {
            my_unit_config = &unit;
            break;
        }
    }

    if (!my_unit_config)
    {
        // This is a critical config error where the role doesn't match any unit.
        throw std::runtime_error("[rotaryExp] Role '" + my_role + "' not found in engine_units.");
    }
    if (!my_unit_config->IMU_enabled)
    {
        // This is a valid configuration, just means we don't initialize this component.
        return;
    }

    // --- Check the simulation flag ---
    // If we are mimicking, there is nothing to initialize.
    if (simulationTesting)
    {
        if (debug)
        {
            std::cout << "[imuExp] WARNING: simulationTesting is TRUE. Mimicking hardware."
                      << std::endl;
        }
        return;
    }

    auto adapters = SimpleBLE::Adapter::get_adapters();
    if (adapters.empty())
    {
        throw std::runtime_error("[IMU] No Bluetooth adapters found.");
    }

    adapter_ = std::make_unique<SimpleBLE::Adapter>(adapters[0]);
    std::cout << "[IMU] Using adapter: " << adapter_->identifier() << std::endl;

    adapter_->scan_for(5000); // Scan for 5 seconds

    auto peripherals = adapter_->scan_get_results();
    bool deviceFound = false;

    for (auto &p : peripherals)
    {
        if (p.identifier() == "WT901BLE68")
        {
            std::cout << "[IMU] Found device: " << p.identifier() << " (" << p.address() << ")"
                      << std::endl;
            try
            {
                p.connect();
                peripheral_ = std::make_unique<SimpleBLE::Peripheral>(p);
                deviceFound = true;
                break;
            }
            catch (const std::exception &e)
            {
                throw std::runtime_error("[IMU] Failed to connect: " + std::string(e.what()));
            }
        }
    }

    if (!deviceFound || !peripheral_)
    {
        throw std::runtime_error("[IMU] Target device 'WT901BLE68' not found after scan.");
    }
    std::cout << "[IMU] Connected successfully." << std::endl;

    // --- DISCOVER NOTIFY AND WRITE CHARACTERISTICS ---
    auto services = peripheral_->services();
    std::string notifyServiceUUID;
    std::string notifyCharacteristicUUID;
    std::string writeServiceUUID;
    std::string writeCharacteristicUUID;

    std::cout << "[IMU] Discovering characteristics..." << std::endl;
    for (auto &service : services)
    {
        for (auto &characteristic : service.characteristics())
        {
            if (characteristic.can_notify() && notifyCharacteristicUUID.empty())
            {
                notifyServiceUUID = service.uuid();
                notifyCharacteristicUUID = characteristic.uuid();
                std::cout << "[IMU] Found Notify Characteristic: " << notifyCharacteristicUUID << std::endl;
            }
            // WitMotion sensors often use the same characteristic for notify and write
            if ((characteristic.can_write_request() || characteristic.can_write_command()) && writeCharacteristicUUID.empty())
            {
                writeServiceUUID = service.uuid();
                writeCharacteristicUUID = characteristic.uuid();
                std::cout << "[IMU] Found Write Characteristic: " << writeCharacteristicUUID << std::endl;
            }
        }
    }

    // Assign the found UUIDs to your class members
    serviceUUID_ = notifyServiceUUID;
    characteristicUUID_ = notifyCharacteristicUUID;

    if (notifyCharacteristicUUID.empty())
    {
        throw std::runtime_error("[IMU] Could not find a notifiable characteristic.");
    }
    if (writeCharacteristicUUID.empty())
    {
        throw std::runtime_error("[IMU] Could not find a writable characteristic to set the rate.");
    }

    try
    {
        // --- FREQUENCY MEASUREMENT LOGIC ---
        peripheral_->notify(serviceUUID_, characteristicUUID_, [this](SimpleBLE::ByteArray data)
                            { this->onNotification(data); });
        std::cout << "[IMU] Subscribed to notifications successfully." << std::endl;

        // --- STEP 1: MEASURE INITIAL RATE ---
        double initial_rate = measureCurrentRate();
        std::cout << "[IMU] Initial Data Rate: " << std::fixed << std::setprecision(2)
                  << initial_rate << " Hz" << std::endl;

        // --- STEP 2: SEND COMMANDS TO CHANGE SENSOR SETTINGS ---
        std::cout << "[IMU] Configuring sensor for 256Hz Bandwidth and 50Hz Return Rate..." << std::endl;

        try
        {
            // Helper lambda to simplify sending command bytes
            auto send_command = [&](const std::string &name, const uint8_t *cmd_data, size_t size)
            {
                std::cout << "[IMU] Sending command: " << name << "..." << std::endl;
                SimpleBLE::ByteArray payload(reinterpret_cast<const char *>(cmd_data), size);
                peripheral_->write_request(writeServiceUUID, writeCharacteristicUUID, payload);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            };

            // Define the command sequences based on the C# examples
            const uint8_t unlock_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
            const uint8_t bandwidth_256hz_cmd[] = {0xFF, 0xAA, 0x1F, 0x00, 0x00};  // SetBandWidth(0x00) -> 256Hz
            const uint8_t return_rate_50hz_cmd[] = {0xFF, 0xAA, 0x03, 0x08, 0x00}; // SetReturnRate(0x08) -> 50Hz
            const uint8_t save_config_cmd[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};      // Save settings command

            // Send the sequence of commands
            send_command("Unlock Register", unlock_cmd, sizeof(unlock_cmd));
            send_command("Set Bandwidth to 256Hz", bandwidth_256hz_cmd, sizeof(bandwidth_256hz_cmd));
            send_command("Set Return Rate to 50Hz", return_rate_50hz_cmd, sizeof(return_rate_50hz_cmd));
            send_command("Save Configuration", save_config_cmd, sizeof(save_config_cmd));

            std::cout << "[IMU] Configuration commands sent successfully." << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[IMU] ERROR: Failed to send configuration commands: " << e.what() << std::endl;
            // Depending on requirements, you might want to re-throw or handle this error
        }

        // Give the sensor a moment to apply the new rate
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // --- STEP 3: MEASURE NEW RATE ---
        double new_rate = measureCurrentRate();
        std::cout << "-------------------------------------------" << std::endl;
        std::cout << "[IMU] New Data Rate:     " << std::fixed << std::setprecision(2)
                  << new_rate << " Hz" << std::endl;
        std::cout << "-------------------------------------------" << std::endl;
    }
    catch (const std::exception &e)
    {
        is_measuring_init_freq_.store(false);
        throw std::runtime_error("[IMU] Failed during setup: " + std::string(e.what()));
    }
}

TimestampedImuData imuExp::getLatestData()
{
    if (simulationTesting)
    {
        // --- Primitive Mimic Data Generation ---
        TimestampedImuData ts_data; // Create the return object
        imuObject data;
        data.acceleration = {0.0, 0.0, 9.81}; // Gravity
        data.angular_velocity = {0.0, 0.0, 0.0};
        data.quaternion = {1.0, 0.0, 0.0, 0.0}; // No rotation

        // Add some noise to make it look dynamic
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> distr(-0.05, 0.05);

        data.acceleration.x += distr(gen);
        data.acceleration.y += distr(gen);
        data.angular_velocity.z += distr(gen) * 0.1; // Gyro noise is usually smaller

        ts_data.data = data;
        ts_data.timestamp = std::chrono::high_resolution_clock::now();
        ts_data.has_new_data = true; // Sim always provides "new" data
        return ts_data;
    }

    // --- Real Hardware Logic ---
    std::lock_guard<std::mutex> lock(dataMutex_);
    TimestampedImuData ts_data;

    // ALWAYS populate the return struct with the most recent valid data we have.
    ts_data.data = latestImuData_;
    ts_data.timestamp = last_update_timestamp_;

    // Use the flag ONLY to indicate if this data is "new" since the last call.
    if (has_new_data_flag_.load())
    {
        ts_data.has_new_data = true;
        has_new_data_flag_.store(false); // Consume the "new" status
    }
    else
    {
        ts_data.has_new_data = false;
    }

    return ts_data;
}

void imuExp::onNotification(SimpleBLE::ByteArray data)
{
    // --- Check if we are in the initial measurement phase ---
    if (is_measuring_init_freq_.load())
    {
        init_timestamps_.push_back(std::chrono::steady_clock::now());
        if (init_timestamps_.size() >= NUM_PACKETS_TO_MEASURE)
        {
            is_measuring_init_freq_.store(false); // Stop measuring
            try
            {
                init_freq_promise_.set_value(); // Fulfill the promise to unblock initialize()
            }
            catch (const std::future_error &e)
            {
                // This can happen if the promise is already satisfied or abandoned. Safe to ignore.
            }
        }
    }

    // The data packet is 20 bytes long for the default 0x61 message
    if (data.size() < 20 || data[0] != 0x55 || data[1] != 0x61)
    {
        // Optional: Add a log message for invalid packets
        // std::cerr << "[IMU] Received invalid or incomplete packet." << std::endl;
        return; // Invalid packet
    }

    imuObject new_data;

    const float G_TO_MS2 = 9.80665f;
    const float ACCEL_SCALE = 16.0f;          // Range is ±16g
    const float GYRO_SCALE = 2000.0f;         // Range is ±2000 °/s
    const float ANGLE_SCALE = 180.0f;         // Range is ±180°
    const float SENSOR_RESOLUTION = 32768.0f; // 2^15 for a signed 16-bit value

    // Cast the raw byte array pointer to a more type-safe uint8_t pointer
    const uint8_t *raw_bytes = reinterpret_cast<const uint8_t *>(data.data());

    // --- Correctly parse signed 16-bit values ---

    // Parse acceleration and convert to m/s^2
    // Data starts at index 2 (after 0x55, 0x61)
    new_data.acceleration.x = static_cast<float>(parseS16(raw_bytes + 2)) / SENSOR_RESOLUTION * ACCEL_SCALE * G_TO_MS2;
    new_data.acceleration.y = static_cast<float>(parseS16(raw_bytes + 4)) / SENSOR_RESOLUTION * ACCEL_SCALE * G_TO_MS2;
    new_data.acceleration.z = static_cast<float>(parseS16(raw_bytes + 6)) / SENSOR_RESOLUTION * ACCEL_SCALE * G_TO_MS2;

    // Parse angular velocity and convert to rad/s
    new_data.angular_velocity.x = degreesToRadians(static_cast<float>(parseS16(raw_bytes + 8)) / SENSOR_RESOLUTION * GYRO_SCALE);
    new_data.angular_velocity.y = degreesToRadians(static_cast<float>(parseS16(raw_bytes + 10)) / SENSOR_RESOLUTION * GYRO_SCALE);
    new_data.angular_velocity.z = degreesToRadians(static_cast<float>(parseS16(raw_bytes + 12)) / SENSOR_RESOLUTION * GYRO_SCALE);

    // Parse Euler angles (in degrees)
    float roll_deg = static_cast<float>(parseS16(raw_bytes + 14)) / SENSOR_RESOLUTION * ANGLE_SCALE;
    float pitch_deg = static_cast<float>(parseS16(raw_bytes + 16)) / SENSOR_RESOLUTION * ANGLE_SCALE;
    float yaw_deg = static_cast<float>(parseS16(raw_bytes + 18)) / SENSOR_RESOLUTION * ANGLE_SCALE;

    // Convert Euler angles to quaternion
    new_data.quaternion = eulerToQuaternion(roll_deg, pitch_deg, yaw_deg);

    // Lock the mutex to safely update the shared data buffer
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        latestImuData_ = new_data;
        last_update_timestamp_ = std::chrono::high_resolution_clock::now();
        has_new_data_flag_.store(true);
    }
    emit imuDataUpdated(new_data, QDateTime::currentDateTime());
}

Quaternion imuExp::eulerToQuaternion(float roll, float pitch, float yaw) const
{
    double roll_rad = degreesToRadians(roll);
    double pitch_rad = degreesToRadians(pitch);
    double yaw_rad = degreesToRadians(yaw);

    double cy = cos(yaw_rad * 0.5);
    double sy = sin(yaw_rad * 0.5);
    double cp = cos(pitch_rad * 0.5);
    double sp = sin(pitch_rad * 0.5);
    double cr = cos(roll_rad * 0.5);
    double sr = sin(roll_rad * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

double imuExp::measureCurrentRate()
{
    // 1. Prepare for measurement
    is_measuring_init_freq_.store(true);
    init_timestamps_.clear();
    init_freq_promise_ = std::promise<void>(); // Reset the promise
    std::future<void> freq_future = init_freq_promise_.get_future();

    std::cout << "[IMU] Measuring data rate..." << std::endl;

    // 2. Wait for the onNotification handler to collect enough packets
    if (freq_future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
    {
        std::cerr << "[IMU] WARNING: Timed out waiting for packets. No data received during measurement." << std::endl;
        is_measuring_init_freq_.store(false); // Make sure to stop measurement on timeout
        return 0.0;
    }

    // 3. Calculate the frequency
    double avg_frequency = 0.0;
    if (init_timestamps_.size() > 1)
    {
        auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                                  init_timestamps_.back() - init_timestamps_.front())
                                  .count();
        if (total_duration > 0)
        {
            avg_frequency = (static_cast<double>(init_timestamps_.size() - 1) / total_duration) * 1'000'000.0;
        }
    }

    return avg_frequency;
}