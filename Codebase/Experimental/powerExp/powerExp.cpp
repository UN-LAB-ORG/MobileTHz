#include "powerExp.h"
#include <QDateTime>
#include <cmath>
#include <iostream>
#include <numbers>
#include <random>

bool powerExp::simulationTesting = true; // Default to false (real hardware)
static bool debug = false;

namespace
{
    constexpr double ERROR_POWER_WATTS = 0.0;            // Value to return on error/disabled
    constexpr double LN10_OVER_10 = 0.23025850929940457; // std::log(10.0) / 10.0
    constexpr double TEN_OVER_LN10 = 4.342944819032518;  // 10.0 / std::log(10.0)

    // For path loss dB calculation (20 * log10(x))
    constexpr double TWENTY_OVER_LN10 = 8.685889638065037; // 20.0 / std::log(10.0)

    // For angle conversion
    constexpr double DEG_TO_RAD = std::numbers::pi / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / std::numbers::pi;

    // Thresholds
    constexpr double MIN_WATT_THRESHOLD = 1e-30;
    constexpr double MIN_DBM_FLOOR = -300.0;

    inline double degreesToRadians(double degrees)
    {
        return degrees * DEG_TO_RAD;
    }

    inline double dBmtoWatts(double dBm)
    {
        if (dBm <= MIN_DBM_FLOOR)
        {
            return 0.0;
        }
        // Replaces std::pow(10.0, (dBm - 30.0) / 10.0)
        return std::exp((dBm - 30.0) * LN10_OVER_10);
    }

    inline double wattsToDbm(double watts)
    {
        if (watts <= MIN_WATT_THRESHOLD)
        {
            return MIN_DBM_FLOOR;
        }
        // Replaces 10.0 * std::log10(mW)
        return TEN_OVER_LN10 * std::log(watts) + 30.0;
    }

} // namespace

powerExp::powerExp(QObject *parent)
    : QObject(parent)
{
}

powerExp::~powerExp()
{
    // Signal the thread to stop and wait for it to finish
    stopFlag_.store(true);
    if (pollingThread_.joinable())
    {
        pollingThread_.join();
    }

#if ENABLE_DSO
    // Clean up VISA resources
    if (dso_session_ != VI_NULL)
    {
        viClose(dso_session_);
    }
    if (rm_session_ != VI_NULL)
    {
        viClose(rm_session_);
    }
#endif
}

void powerExp::initialize(const ConfigFile &paramConfig, mobileTHzEngine *engine_ptr)
{
    const std::string &my_role = paramConfig.experimental.role;

    if (my_role == "OBSERVER")
    {
        return; // Observer has no IMU hardware or simulator
    }

    const engineUnit *my_unit_config = nullptr; // Find this Node's specific unit configuration
    for (const auto &unit : paramConfig.engine_units)
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
    if (!my_unit_config->RxAntenna_enabled && !my_unit_config->RxChain_enabled)
    {
        // This is a valid configuration, just means we don't initialize this component.
        return;
    }

    // Store configuration parameters
    this->IF_frequency_.store(paramConfig.experimental.DSO_IF_frequency);
    this->meas_bw_.store(paramConfig.experimental.DSO_measurement_bandwidth);
    this->delay_us.store(my_unit_config->IMU_samplingDelay_microsec);

    if (meas_bw_ == 0 || IF_frequency_ == 0)
    {
        throw std::runtime_error("[powerExp] Initialization failed: DSO frequency or bandwidth is zero.");
    }

    // --- Check the simulation flag ---
    if (simulationTesting)
    {
        if (debug)
        {
            std::cout << "[powerExp] WARNING: simulationTesting is TRUE. Mimicking hardware."
                      << std::endl;
        }
        // No hardware to initialize, so we just start the mimic polling loop.
        stopFlag_.store(false);
        pollingThread_ = std::thread(&powerExp::pollingLoop, this);
        return;
    }
    else
    {
#if ENABLE_DSO
        try
        {
            // Initialize the DSO hardware
            initializeDSO("TCPIP0::192.168.1.10::INSTR", "CHAN1"); // Example resource string
        }
        catch (const std::exception &e)
        {
            std::cerr << "[powerExp] FATAL: Failed to initialize DSO. " << e.what() << std::endl;
            throw; // Re-throw to stop the application
        }
#endif

        // Start the internal polling thread
        stopFlag_.store(false);
        pollingThread_ = std::thread(&powerExp::pollingLoop, this);

        std::cout << "[powerExp] Initialized and polling thread started." << std::endl;
    }
}

TimestampedPowerData powerExp::getLatestData()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    TimestampedPowerData ts_data;

    // ALWAYS populate the return struct with the most recent valid data we have.
    ts_data.power_watts = latestPower_watts_;
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

void powerExp::pollingLoop()
{
    while (!stopFlag_.load())
    {
        double currentPower_watts = ERROR_POWER_WATTS;
        double currentPower_dBm = ERROR_POWER_DBM;

        try
        {
            if (simulationTesting)
            {
                // In mimic mode, simulate a plausible power level with noise.
                std::this_thread::sleep_for(std::chrono::microseconds(delay_us.load()));

                // Base power level in dBm
                double base_power_dbm = -0;
                // Add some noise
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<> distr(-0.01, 0.01);
                currentPower_dBm = base_power_dbm + distr(gen);

                // Convert to watts for internal storage
                currentPower_watts = dBmtoWatts(currentPower_dBm);
            }
            else // --- Original Hardware Logic ---
            {
#if ENABLE_DSO
                currentPower_dBm = measureChannelPower();
                currentPower_watts = dBmtoWatts(currentPower_dBm);
#else
                // Fallback for non-DSO builds (but not in mimic mode)
                std::this_thread::sleep_for(std::chrono::microseconds(delay_us.load()));
                currentPower_dBm = -9988.0;
                currentPower_watts = dBmtoWatts(currentPower_dBm);
#endif
            }

            // Store the new reading and timestamp (common to both modes)
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latestPower_watts_ = currentPower_watts;
                last_update_timestamp_ = std::chrono::high_resolution_clock::now();
                has_new_data_flag_.store(true);
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "[powerExp Polling Thread] Error: " << e.what() << ". Continuing..."
                      << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    std::cout << "[powerExp] Polling thread stopped." << std::endl;
}

#if ENABLE_DSO
void powerExp::handleError(const std::string &errorMessage, ViStatus errorStatus)
{
    // This is now a member function, no need to pass sessions
    if (dso_session_)
    {
        viClose(dso_session_);
        dso_session_ = VI_NULL;
    }
    if (rm_session_)
    {
        viClose(rm_session_);
        rm_session_ = VI_NULL;
    }
    throw std::runtime_error(errorMessage + " (ViStatus: " + std::to_string(errorStatus) + ")");
}

float powerExp::NR3ToFloat(const std::string &nr3_str)
{
    try
    {
        return std::stof(nr3_str);
    }
    catch (...)
    {
        return ERROR_POWER_DBM;
    }
}

float powerExp::measureChannelPower()
{
    ViStatus status;
    char response_buffer[RESPONSE_BUFFER_SIZE];

    std::string command_str = ":MEASure:FFT:CPOWer? FUNC1," + std::to_string(IF_frequency_.load()) + "," + std::to_string(meas_bw_.load());

    // Apply the configured delay
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms_.load()));

    status = viQueryf(dso_session_, command_str.c_str(), "%s", response_buffer);
    if (status < VI_SUCCESS)
    {
        // Throw an exception that can be caught in the polling loop
        throw std::runtime_error("[DSO] Error querying channel power (ViStatus: " + std::to_string(status) + ")");
    }

    std::string response_str(response_buffer);
    response_str.erase(std::remove(response_str.begin(), response_str.end(), '\n'), response_str.end());

    return NR3ToFloat(response_str);
}

void powerExp::initializeDSO(const std::string &resource_string, const char *channel)
{
    ViStatus status;
    char response_buffer[RESPONSE_BUFFER_SIZE];
    ViUInt32 bytes_read;

    // Open Resource Manager
    status = viOpenDefaultRM(&rm_session_);
    if (status < VI_SUCCESS)
    {
        handleError("[DSO] Could not open VISA resource manager", status);
    }

    // Open Session with DSO
    status = viOpen(rm_session_, const_cast<char *>(resource_string.c_str()), VI_NULL, VI_NULL, &dso_session_);
    if (status < VI_SUCCESS)
    {
        handleError("[DSO] Could not open a session with the DSO", status);
    }
    std::cout << "[DSO] Opened DSO session... success" << std::endl;

    // Set a timeout (e.g., 5 seconds) to prevent infinite waits
    viSetAttribute(dso_session_, VI_ATTR_TMO_VALUE, 5000);

    // Identify device
    status = viQueryf(dso_session_, (ViString) "*IDN?", (ViString) "%s", response_buffer);
    if (status < VI_SUCCESS)
    {
        handleError("[DSO] Could not query device identification", status);
    }
    std::cout << "[DSO] DSO response: " << response_buffer;

    // --- Configure DSO for power measurement ---
    const std::vector<std::string> commands = {
        std::string(":MEASure:SOURce ") + channel,
        ":TIMebase:SCALe 1e-9",
        ":FUNCtion1:FFT:CENTer " + std::to_string(IF_frequency_.load()),
        ":FUNCtion1:FFT:SPAN 1E9",
        ":FUNCtion1:FFT:VUNits DBM",
        ":FUNCtion1:OPERation FFT",
        ":FUNCtion1:SOURce CHANnel1",
        ":FUNCtion1:DISPlay ON",
        ":SYSTem:GUI OFF",
        ":ACQuire:AVERage:COUNt 2",
        ":ACQuire:AVERage ON",
        ":SYSTem:HEADer OFF",
    };

    for (const auto &cmd : commands)
    {
        status = viWrite(dso_session_, (ViBuf)cmd.c_str(), (ViUInt32)cmd.length(), &bytes_read);
        if (status < VI_SUCCESS)
        {
            handleError("[DSO] Failed to send command: '" + cmd + "'", status);
        }
    }
    std::cout << "[DSO] Configuration complete." << std::endl;
}

#endif // ENABLE_DSO
