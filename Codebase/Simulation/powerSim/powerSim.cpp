#include "powerSim.h"
#include <QRandomGenerator>
#include "Software/mobileTHzEngine/mobileTHzEngine.h"
#include "Software/structDefinition.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>
#include <vector>

#ifndef M_PI
#ifdef _USE_MATH_DEFINES
#else
#define M_PI 3.14159265358979323846
#endif
#endif

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
        return std::exp((dBm - 30.0) * LN10_OVER_10);
    }

    inline double wattsToDbm(double watts)
    {
        if (watts <= MIN_WATT_THRESHOLD)
        {
            return MIN_DBM_FLOOR;
        }
        return TEN_OVER_LN10 * std::log(watts) + 30.0;
    }

} // namespace

PrecomputedPatternLUT powerSim::buildFineGrainedCosLut(const radiationPattern &originalPattern,
                                                       double desiredAngleStepDegrees) const
{
    PrecomputedPatternLUT lut;

    if (originalPattern.thetaValues.empty() || originalPattern.thetaValues.size() != originalPattern.gainValues_linear.size())
    {
        std::cerr << "powerSim::buildFineGrainedCosLut: Invalid original pattern data."
                  << std::endl;
        return lut;
    }

    const double maxAngleDeg = originalPattern.thetaValues.back();
    // The LUT will cover from theta=0 to theta=maxAngleDeg.
    // In the cosine domain, this is from cos(0)=1.0 down to cos(maxAngleDeg).
    lut.cosThetaMax = 1.0;
    lut.cosThetaMin = std::cos(maxAngleDeg * M_PI / 180.0);

    if (maxAngleDeg <= 0.0 || desiredAngleStepDegrees <= 0.0)
    {
        std::cerr << "powerSim::buildFineGrainedCosLut: Invalid angle range or step." << std::endl;
        return lut;
    }

    lut.numSteps = static_cast<size_t>(std::ceil(maxAngleDeg / desiredAngleStepDegrees)) + 1;
    // The step size is in the cosine domain, so it's not linear. We calculate it.
    if (lut.numSteps > 1)
    {
        lut.cosThetaStep = (lut.cosThetaMax - lut.cosThetaMin) / (lut.numSteps - 1);
        if (std::abs(lut.cosThetaStep) > 1e-9)
        {
            lut.invCosThetaStep = 1.0 / lut.cosThetaStep; // <--- PRE-CALCULATE INVERSE
        }
        else
        {
            lut.invCosThetaStep = 0.0; // Avoid division by zero
        }
    }
    else
    {
        lut.cosThetaStep = 0.0;
        lut.invCosThetaStep = 0.0;
    }

    lut.fineGrainedGains.resize(lut.numSteps);

    for (size_t i = 0; i < lut.numSteps; ++i)
    {
        // Find the cos(theta) value for this LUT index
        double currentCosTheta = lut.cosThetaMax - static_cast<double>(i) * lut.cosThetaStep;
        // Convert it back to an angle to look up in the original pattern
        double currentAngleDeg = std::acos(std::clamp(currentCosTheta, -1.0, 1.0)) * 180.0 / M_PI;

        // Interpolate on the original (angle, gain) data
        double gain_linear = powerSim::linearInterpolate(currentAngleDeg,
                                                         originalPattern.thetaValues,
                                                         originalPattern.gainValues_linear);
        lut.fineGrainedGains[i] = std::max(0.0, gain_linear);
    }
    return lut;
}

powerSim::powerSim() {} // Constructor

powerSim::~powerSim() {} // Destructor

void powerSim::initialize(const ConfigFile &paramConfig, mobileTHzEngine *engine_ptr)
{
    if (!engine_ptr)
    {
        throw std::runtime_error("powerSim::initialize called with null engine pointer.");
    }
    engine_ = engine_ptr;

    seed_ = paramConfig.randomgen_seed;
    // Seed the member engine once.
    // This provides reproducibility for the entire simulation run.
    rand_engine_.seed(seed_);

    // Initialize the distribution with a dummy parameter.
    // The actual rate will be set right before use in calculateReceivedPower.
    exponential_dist_ = std::exponential_distribution<double>(1.0); // 1.0 is just a placeholder rate

    double frequency_hz = static_cast<double>(engine_->carrierFrequency);
    if (frequency_hz > 1e-6)
    {
        double lambda = c_ / frequency_hz;
        double term = lambda / (4.0 * M_PI);
        path_loss_wavelength_factor_ = term * term;
    }

    pattern_lut_cache_.clear();

    for (const auto &unitConfig : engine_ptr->getAllEngineUnits())
    {
        for (const auto &pattern_pair : unitConfig.radiationPatterns)
        {
            const radiationPattern *pattern_key = &pattern_pair.second;
            // Check if this pattern is already in the cache to avoid redundant work
            if (pattern_lut_cache_.find(pattern_key) == pattern_lut_cache_.end())
            {
                // Not found, so build it. Use a high resolution for accuracy.
                double desired_resolution_deg = 0.1;
                PrecomputedPatternLUT new_lut = buildFineGrainedCosLut(*pattern_key,
                                                                       desired_resolution_deg);
                if (!new_lut.fineGrainedGains.empty())
                {
                    pattern_lut_cache_.emplace(pattern_key, std::move(new_lut));
                }
            }
        }
    }
}

Quaternion powerSim::conjugate(const Quaternion &q) const
{
    return {q.w, -q.x, -q.y, -q.z};
}

Quaternion powerSim::normalizeQuaternion(const Quaternion &q) const
{
    // Calculate the squared magnitude of the quaternion.
    double mag_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

    // Fast path for nearly-normalized quaternions.
    if (std::abs(1.0 - mag_sq) < 1e-14)
    {
        return q;
    }

    // Safety check for zero-magnitude quaternions.
    if (mag_sq <= 1e-14)
    {
        return {1.0, 0.0, 0.0, 0.0};
    }

    // Calculate the inverse magnitude.
    const double inv_mag = 1.0 / std::sqrt(mag_sq);

    // Create a new Quaternion object to store the normalized result.
    Quaternion result;

    // Multiply each component by the inverse magnitude to normalize.
    result.w = q.w * inv_mag;
    result.x = q.x * inv_mag;
    result.y = q.y * inv_mag;
    result.z = q.z * inv_mag;

    return result;
}

Quaternion powerSim::multiplyQuaternions(const Quaternion &q1, const Quaternion &q2) const
{
    // Performs q1 * q2 (rotation q2 then rotation q1)
    Quaternion result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

Position powerSim::rotateVectorByQuaternion(const Position &v, const Quaternion &q_normalized) const
{
    // cheaper expansion of v' = q * v * q_conj.
    // This formula avoids intermediate quaternion objects and two full quaternion multiplications.
    // It uses the vector part of the quaternion 'u' and the scalar part 'w'.
    // v' = v + 2w * (u x v) + 2 * (u x (u x v))

    // Extract the vector part of the quaternion
    Position u = {q_normalized.x, q_normalized.y, q_normalized.z};

    // Calculate the cross product u x v
    Position cross_uv;
    cross_uv.x = u.y * v.z - u.z * v.y;
    cross_uv.y = u.z * v.x - u.x * v.z;
    cross_uv.z = u.x * v.y - u.y * v.x;

    // Calculate the cross product u x (u x v)
    Position cross_u_uv;
    cross_u_uv.x = u.y * cross_uv.z - u.z * cross_uv.y;
    cross_u_uv.y = u.z * cross_uv.x - u.x * cross_uv.z;
    cross_u_uv.z = u.x * cross_uv.y - u.y * cross_uv.x;

    // Scale the cross products by 2w and 2
    const double w = q_normalized.w;

    // Combine to get the final rotated vector
    Position rotated_v;
    rotated_v.x = v.x + 2.0 * (w * cross_uv.x + cross_u_uv.x);
    rotated_v.y = v.y + 2.0 * (w * cross_uv.y + cross_u_uv.y);
    rotated_v.z = v.z + 2.0 * (w * cross_uv.z + cross_u_uv.z);

    return rotated_v;
}

double powerSim::linearInterpolate(double x,
                                   const std::vector<double> &x_data,
                                   const std::vector<double> &y_data)
{
    if (x_data.empty() || x_data.size() != y_data.size())
    {
        return -9999999;
    }
    if (x_data.size() == 1)
    {
        return y_data[0];
    }
    if (x <= x_data.front())
    {
        return y_data.front();
    }
    if (x >= x_data.back())
    {
        return y_data.back();
    }

    auto it = std::lower_bound(x_data.begin(), x_data.end(), x);
    // Handle exact match
    if (it != x_data.end() && *it == x)
    { // Check *it == x for exact match
        return y_data[std::distance(x_data.begin(), it)];
    }
    // If not exact match, it points to the first element > x
    // So, the element before it (i_lower) and it (i_upper) form the interpolation interval
    size_t i_upper = std::distance(x_data.begin(), it);
    if (i_upper == 0)
    { // Should be caught by x <= x_data.front(), but defensive
        return y_data.front();
    }
    size_t i_lower = i_upper - 1;

    double x0 = x_data[i_lower];
    double x1 = x_data[i_upper];
    double y0 = y_data[i_lower];
    double y1 = y_data[i_upper];

    if (std::abs(x1 - x0) < std::numeric_limits<double>::epsilon())
    {
        return y0; // Avoid division by zero if x0 and x1 are too close
    }
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

inline double powerSim::getGainFromPattern(const engineUnit &inputEngineUnit,
                                           int patternId,
                                           double cos_theta_off_boresight) const
{
    if (!engine_)
    {
        throw std::runtime_error("powerSim::getGainFromPattern called with null engine.");
    }

    auto it_original_pattern_map = inputEngineUnit.radiationPatterns.find(patternId);
    if (it_original_pattern_map == inputEngineUnit.radiationPatterns.end())
    {
        throw std::runtime_error("powerSim::getGainFromPattern: Radiation pattern ID: " + std::to_string(patternId) + " not found.");
    }

    const radiationPattern &original_pattern_ref = it_original_pattern_map->second;
    const radiationPattern *pattern_key = &original_pattern_ref;

    // With the pre-built cache, we expect a cache hit every time.
    auto cache_it = pattern_lut_cache_.find(pattern_key);
    if (cache_it != pattern_lut_cache_.end())
    {
        const PrecomputedPatternLUT &lut = cache_it->second;

        double lookup_cos_theta = std::clamp(cos_theta_off_boresight,
                                             lut.cosThetaMin,
                                             lut.cosThetaMax);

        double exact_idx = (lut.cosThetaMax - lookup_cos_theta) * lut.invCosThetaStep;

        size_t idx0 = static_cast<size_t>(exact_idx);

        if (idx0 >= lut.numSteps - 1)
        {
            return lut.fineGrainedGains.back();
        }

        const double y0 = lut.fineGrainedGains[idx0];
        const double y1 = lut.fineGrainedGains[idx0 + 1];
        const double t = exact_idx - static_cast<double>(idx0);
        return y0 + t * (y1 - y0);
    }

    // This fallback should now ideally never be hit during a simulation run.
    double safe_cos_theta = std::clamp(cos_theta_off_boresight, -1.0, 1.0);
    double angle_deg = std::acos(safe_cos_theta) * 180.0 / M_PI;
    double gain_linear = linearInterpolate(angle_deg,
                                           original_pattern_ref.thetaValues,
                                           original_pattern_ref.gainValues_linear);
    return std::max(0.0, gain_linear);
}

Quaternion powerSim::quaternionFromAzAltDegrees(double azimuthDegrees, double altitudeDegrees) const
{
    // Convert angles to radians
    double azRad = degreesToRadians(azimuthDegrees);
    double altRad = degreesToRadians(altitudeDegrees);

    // Calculate half angles
    double halfAz = azRad * 0.5;
    double halfAlt = altRad * 0.5;

    // Calculate sin/cos of half angles
    double c_az = std::cos(halfAz);
    double s_az = std::sin(halfAz);
    double c_alt = std::cos(halfAlt);
    double s_alt = std::sin(halfAlt);

    // Create Quaternion for Azimuth rotation (around Z-axis)
    Quaternion azQuat = {static_cast<double>(c_az),
                         0.0,
                         0.0,
                         static_cast<double>(s_az)}; // w, x, y, z

    // Create Quaternion for Altitude rotation (around Y-axis)
    Quaternion altQuat = {static_cast<double>(c_alt),
                          0.0,
                          static_cast<double>(s_alt),
                          0.0}; // w, x, y, z

    // Combine rotations: Azimuth first, then Altitude around the Y-axis
    // This corresponds to multiplication order: azQuat * altQuat
    return multiplyQuaternions(azQuat, altQuat);
}

double powerSim::calculateReceivedPower(const size_t &txUnitIdx,
                                        const size_t &rxUnitIdx,
                                        const engineUnit &txEngineUnit,
                                        const engineUnit &rxEngineUnit,
                                        const size_t &timeIdx) const
{
    if (!engine_ || txUnitIdx == rxUnitIdx)
    {
        return ERROR_POWER_WATTS;
    }

    try
    {
        // Engine calls remain as per the constraint
        const environmentObject txEnv = engine_->getEnvironmentData(txUnitIdx, timeIdx);
        const environmentObject rxEnv = engine_->getEnvironmentData(rxUnitIdx, timeIdx);
        const rotaryObject txRotary = engine_->getRotaryData(txUnitIdx, timeIdx);
        const rotaryObject rxRotary = engine_->getRotaryData(rxUnitIdx, timeIdx);
        const antennaObject txAnt = engine_->getTxAntennaData(txUnitIdx, timeIdx);
        const antennaObject rxAnt = engine_->getRxAntennaData(rxUnitIdx, timeIdx);

        Position vec_tx_to_rx = {rxEnv.position.x - txEnv.position.x,
                                 rxEnv.position.y - txEnv.position.y,
                                 rxEnv.position.z - txEnv.position.z};
        double distance_sq = vec_tx_to_rx.x * vec_tx_to_rx.x + vec_tx_to_rx.y * vec_tx_to_rx.y + vec_tx_to_rx.z * vec_tx_to_rx.z;
        if (distance_sq < 1e-12)
        {
            return ERROR_POWER_WATTS;
        }
        double distance = std::sqrt(distance_sq);
        double inv_distance = 1.0 / distance;
        Position dir_tx_to_rx = {vec_tx_to_rx.x * inv_distance,
                                 vec_tx_to_rx.y * inv_distance,
                                 vec_tx_to_rx.z * inv_distance};
        Position dir_rx_to_tx = {-dir_tx_to_rx.x, -dir_tx_to_rx.y, -dir_tx_to_rx.z};

        // 1. Calculate Path Loss directly in linear scale
        // pathLoss_linear = (lambda / (4*pi*d))^2 = path_loss_wavelength_factor_ / d^2
        double pathLoss_linear = (distance_sq > 1e-12) ? path_loss_wavelength_factor_ / distance_sq
                                                       : 0.0;

        // 2. Get Effective Orientations
        const Quaternion &txBaseQuat = txEnv.quaternion;
        Quaternion txRotaryQuat = quaternionFromAzAltDegrees(txRotary.azimuth.angle,
                                                             txRotary.altitude.angle);
        Quaternion effectiveTxQuat = multiplyQuaternions(txBaseQuat, txRotaryQuat);

        const Quaternion &rxBaseQuat = rxEnv.quaternion;
        Quaternion rxRotaryQuat = quaternionFromAzAltDegrees(rxRotary.azimuth.angle,
                                                             rxRotary.altitude.angle);
        Quaternion effectiveRxQuat = multiplyQuaternions(rxBaseQuat, rxRotaryQuat);

        // 3. Get Antenna Gains
        Position local_dir_at_tx = rotateVectorByQuaternion(dir_tx_to_rx,
                                                            conjugate(effectiveTxQuat));
        double cos_theta_tx = local_dir_at_tx.x;
        double gainTx_linear = getGainFromPattern(txEngineUnit,
                                                  0, // DEFAULT TO RADIATIONPATTERNID: 0
                                                  cos_theta_tx);

        Position local_dir_at_rx = rotateVectorByQuaternion(dir_rx_to_tx,
                                                            conjugate(effectiveRxQuat));
        double cos_theta_rx = local_dir_at_rx.x;
        double gainRx_linear = getGainFromPattern(rxEngineUnit,
                                                  0, // DEFAULT TO RADIATIONPATTERNID: 0
                                                  cos_theta_rx);

        // 4. Calculate Ideal Received Power in WATTS
        double idealReceivedPower_watts = txEngineUnit.TxAntenna_transmitPower_watts * gainTx_linear * gainRx_linear * pathLoss_linear;

        // 5. Add Noise in WATTS
        const double averageNoisePower_watts = engine_->receiverThermalNoise_watts * rxEngineUnit.RxChain_noise_factor_linear;
        exponential_dist_.param(
            std::exponential_distribution<double>::param_type(1.0 / averageNoisePower_watts));
        double instantaneousNoisePower_watts = exponential_dist_(rand_engine_);
        double noisyReceivedPower_watts = idealReceivedPower_watts + instantaneousNoisePower_watts;

        // 6. Update Antenna Objects with LINEAR values
        antennaObject updatedTxAnt = txAnt;
        updatedTxAnt.gain_linear = gainTx_linear;
        updatedTxAnt.power_watts = txEngineUnit.TxAntenna_transmitPower_watts;
        engine_->setTxAntennaData(txUnitIdx, timeIdx, updatedTxAnt);

        antennaObject updatedRxAnt = rxAnt;
        updatedRxAnt.gain_linear = gainRx_linear;
        engine_->setRxAntennaData(rxUnitIdx, timeIdx, updatedRxAnt);

        // 7. Return final power in WATTS
        return std::max(0.0, noisyReceivedPower_watts);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error during calculateReceivedPower for TX:" << txUnitIdx
                  << ", RX:" << rxUnitIdx << ", Slot:" << timeIdx << ": " << e.what() << std::endl;
        // Error handling
        try
        {
            antennaObject errorAnt = engine_->getTxAntennaData(txUnitIdx, timeIdx);
            errorAnt.gain_linear = -999;
            engine_->setTxAntennaData(txUnitIdx, timeIdx, errorAnt);
            errorAnt = engine_->getRxAntennaData(rxUnitIdx, timeIdx);
            errorAnt.gain_linear = -999;
            engine_->setRxAntennaData(rxUnitIdx, timeIdx, errorAnt);
        }
        catch (...)
        { // Ignore exceptions during error reporting
        }
        return ERROR_POWER_WATTS;
    }
}

double powerSim::calculateReceivedPower(const engineUnit &txEngineUnit,
                                        const engineUnit &rxEngineUnit,
                                        const environmentObject &txEnv,
                                        const environmentObject &rxEnv,
                                        const rotaryObject &txRotary,
                                        const rotaryObject &rxRotary,
                                        const antennaObject &txAnt,
                                        const antennaObject &rxAnt) const
{
    // This stateless method calculates received power in WATTS.
    // It has NO side effects on the engine.
    const double ERROR_POWER_WATTS = 0.0;

    try
    {
        // 1. --- Vector Math & Distance ---
        Position vec_tx_to_rx = {rxEnv.position.x - txEnv.position.x,
                                 rxEnv.position.y - txEnv.position.y,
                                 rxEnv.position.z - txEnv.position.z};

        double distance_sq = vec_tx_to_rx.x * vec_tx_to_rx.x + vec_tx_to_rx.y * vec_tx_to_rx.y + vec_tx_to_rx.z * vec_tx_to_rx.z;

        if (distance_sq < 1e-12)
        {
            return ERROR_POWER_WATTS; // Return 0 watts if distance is negligible
        }
        double distance = std::sqrt(distance_sq);
        double inv_distance = 1.0 / distance;
        Position dir_tx_to_rx = {vec_tx_to_rx.x * inv_distance,
                                 vec_tx_to_rx.y * inv_distance,
                                 vec_tx_to_rx.z * inv_distance};
        Position dir_rx_to_tx = {-dir_tx_to_rx.x, -dir_tx_to_rx.y, -dir_tx_to_rx.z};

        // 2. --- Path Loss Calculation ---
        // pathLoss_linear = (lambda / (4*pi*d))^2 = path_loss_wavelength_factor_ / d^2
        double pathLoss_linear = path_loss_wavelength_factor_ / distance_sq;

        // 3. --- Effective Orientation  ---
        const Quaternion &txBaseQuat = txEnv.quaternion;
        Quaternion txRotaryQuat = quaternionFromAzAltDegrees(txRotary.azimuth.angle,
                                                             txRotary.altitude.angle);
        Quaternion effectiveTxQuat = multiplyQuaternions(txBaseQuat, txRotaryQuat);

        const Quaternion &rxBaseQuat = rxEnv.quaternion;
        Quaternion rxRotaryQuat = quaternionFromAzAltDegrees(rxRotary.azimuth.angle,
                                                             rxRotary.altitude.angle);
        Quaternion effectiveRxQuat = multiplyQuaternions(rxBaseQuat, rxRotaryQuat);

        // 4. --- Antenna Gain Calculation ---
        Position local_dir_at_tx = rotateVectorByQuaternion(dir_tx_to_rx,
                                                            conjugate(effectiveTxQuat));
        double cos_theta_tx = local_dir_at_tx.x;
        double gainTx_linear = getGainFromPattern(txEngineUnit,
                                                  0, // DEFAULT TO RADIATIONPATTERNID: 0
                                                  cos_theta_tx);

        Position local_dir_at_rx = rotateVectorByQuaternion(dir_rx_to_tx,
                                                            conjugate(effectiveRxQuat));
        double cos_theta_rx = local_dir_at_rx.x;
        double gainRx_linear = getGainFromPattern(rxEngineUnit,
                                                  0, // DEFAULT TO RADIATIONPATTERNID: 0
                                                  cos_theta_rx);

        // 5. --- Ideal Power Calculation (Friis Equation in Linear) ---
        double idealReceivedPower_watts = txEngineUnit.TxAntenna_transmitPower_watts * gainTx_linear * gainRx_linear * pathLoss_linear;

        // 6. --- Noise Calculation (Using Pre-Linearized Values) ---
        double noisyReceivedPower_watts = idealReceivedPower_watts;

        const double averageNoisePower_watts = engine_->receiverThermalNoise_watts * rxEngineUnit.RxChain_noise_factor_linear;

        if (averageNoisePower_watts > 0)
        {
            // Update the distribution parameter for the current noise level.
            exponential_dist_.param(
                std::exponential_distribution<double>::param_type(1.0 / averageNoisePower_watts));

            // Draw a sample for the instantaneous noise power.
            double instantaneousNoisePower_watts = exponential_dist_(rand_engine_);

            // Add the noise power to the signal power.
            noisyReceivedPower_watts += instantaneousNoisePower_watts;
        }

        // 7. --- Return Final Power in WATTS ---
        return std::max(0.0, noisyReceivedPower_watts);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error during stateless calculateReceivedPower: " << e.what() << std::endl;
        return ERROR_POWER_WATTS;
    }
}
