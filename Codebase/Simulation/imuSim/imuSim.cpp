// imuSim.cpp
#include "imuSim.h"
#include <QDebug>
#include <QList>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include "Software/mobileTHzEngine/mobileTHzEngine.h"

// --- Static Constants ---
static const double GRAVITY_MAGNITUDE = 9.80665; // m/s^2
static const Position GRAVITY_WORLD = {0.0, 0.0, -GRAVITY_MAGNITUDE};
static const double EPSILON = std::numeric_limits<double>::epsilon();
static bool debug = false;

Quaternion imuSim::conjugate(const Quaternion &q)
{
    return {q.w, -q.x, -q.y, -q.z};
}

Position imuSim::rotateVectorByQuaternion(const Position &v, const Quaternion &q)
{
    // Using Hamilton product: q * p * q_conj where p = (0, v.x, v.y, v.z)
    double w = q.w, x = q.x, y = q.y, z = q.z;
    double vx = v.x, vy = v.y, vz = v.z;

    double tx = w * vx + y * vz - z * vy;
    double ty = w * vy + z * vx - x * vz;
    double tz = w * vz + x * vy - y * vx;
    double tw = -x * vx - y * vy - z * vz;

    Position result;
    result.x = tw * (-x) + tx * w + ty * (-z) - tz * (-y);
    result.y = tw * (-y) + ty * w + tz * (-x) - tx * (-z);
    result.z = tw * (-z) + tz * w + tx * (-y) - ty * (-x);

    return result;
}

// Static helper: Update bias using random walk model
void imuSim::updateBiasRandomWalk(
    Position &bias,
    double bias_random_walk_density, // sigma_b [unit/s^2/sqrt(Hz)] or [unit/s/sqrt(Hz)]
    double delta_t,
    std::mt19937 &generator)
{
    // Basic validation
    if (delta_t <= EPSILON || bias_random_walk_density < 0.0)
    {
        return; // No update if parameters are invalid or density is zero
    }

    // Calculate the standard deviation for the bias increment in this discrete step
    double bias_increment_std_dev = bias_random_walk_density * std::sqrt(delta_t);

    if (bias_increment_std_dev > EPSILON)
    {
        std::normal_distribution<double> standard_normal_dist(0.0, 1.0);

        // Apply the random walk step using the provided generator
        bias.x += bias_increment_std_dev * standard_normal_dist(generator);
        bias.y += bias_increment_std_dev * standard_normal_dist(generator);
        bias.z += bias_increment_std_dev * standard_normal_dist(generator);
    }
}

void imuSim::initialize(const ConfigFile &paramConfig, mobileTHzEngine *engine_ptr)
{
    if (!engine_ptr)
    {
        std::cerr << "Error: imuSim::initialize called with null engine pointer." << std::endl;
        return;
    }
    engine = engine_ptr;

    const double local_dt_sec = paramConfig.engine_slot_time_microsec / 1e6;
    if (local_dt_sec <= EPSILON)
    {
        std::cerr << "Error: Invalid or non-positive engine_slot_time_microsec in imuSim."
                  << std::endl;
        return;
    }
    dt_sec = local_dt_sec;

    const double inv_sqrt_dt_sec = (dt_sec > EPSILON) ? 1.0 / std::sqrt(dt_sec)
                                                      : 0.0;
    const size_t num_units = engine->getNumUnits();
    if (num_units == 0)
    {
        qWarning() << "imuSim::initialize: No units found in engine. Skipping IMU simulation.";
        return;
    }

    // --- Initialize Bias Vectors ---
    try
    {
        accel_bias.resize(num_units);
        gyro_bias.resize(num_units);
    }
    catch (const std::bad_alloc &e)
    {
        std::cerr << "Error: Failed to allocate memory for bias vectors: " << e.what() << std::endl;
        return;
    }

    const std::vector<engineUnit> &engineUnits = engine_ptr->getAllEngineUnits();
    if (engineUnits.size() != num_units)
    {
        std::cerr << "Error: Mismatch between engine->getNumUnits() and engineUnits size."
                  << std::endl;
        return;
    }

    std::mt19937 master_generator(paramConfig.randomgen_seed);

    // --- Set Initial Bias (using global randomgen_seed) ---
    std::mt19937 initial_bias_generator(paramConfig.randomgen_seed);

    for (size_t i = 0; i < num_units; ++i)
    {
        const engineUnit &unit = engineUnits[i];

        // Helper lambda to generate initial bias for one axis
        auto generate_initial = [&](double mean, double stddev)
        {
            if (stddev > EPSILON)
            {
                // Use the deterministically seeded generator
                std::normal_distribution<double> dist(mean, stddev);
                return dist(initial_bias_generator);
            }
            else
            {
                return mean; // If stddev is zero, use mean directly
            }
        };

        // Accel Initial Bias
        // Assuming mean/stddev are now per-axis in ConfigFile, adjust if they are single values
        accel_bias[i].x = generate_initial(unit.IMU_accel_initial_bias_mean,
                                           unit.IMU_accel_initial_bias_stddev);
        accel_bias[i].y = generate_initial(unit.IMU_accel_initial_bias_mean,
                                           unit.IMU_accel_initial_bias_stddev);
        accel_bias[i].z = generate_initial(unit.IMU_accel_initial_bias_mean,
                                           unit.IMU_accel_initial_bias_stddev);

        // Gyro Initial Bias
        gyro_bias[i].x = generate_initial(unit.IMU_gyro_initial_bias_mean,
                                          unit.IMU_gyro_initial_bias_stddev);
        gyro_bias[i].y = generate_initial(unit.IMU_gyro_initial_bias_mean,
                                          unit.IMU_gyro_initial_bias_stddev);
        gyro_bias[i].z = generate_initial(unit.IMU_gyro_initial_bias_mean,
                                          unit.IMU_gyro_initial_bias_stddev);

        if (debug && unit.IMU_enabled)
        {
            std::cout << "Unit " << i << " Initial Accel Bias: (" << accel_bias[i].x << ", "
                      << accel_bias[i].y << ", " << accel_bias[i].z << ")" << std::endl;
            std::cout << "Unit " << i << " Initial Gyro Bias: (" << gyro_bias[i].x << ", "
                      << gyro_bias[i].y << ", " << gyro_bias[i].z << ")" << std::endl;
        }
    }

    const size_t totalSlots = engine->getNumTimeSlots();
    if (totalSlots == 0)
    {
        qWarning() << "imuSim::initialize: No time slots found in engine. Skipping IMU simulation.";
        return;
    }

    std::vector<size_t> unitIndices;
    unitIndices.reserve(num_units);
    for (size_t i = 0; i < num_units; ++i)
    {
        unitIndices.push_back(i);
    }

    if (debug)
        std::cout << "Starting IMU simulation for " << num_units << " units over " << totalSlots
                  << " slots (Seed: " << paramConfig.randomgen_seed << ")..."
                  << std::endl; // Log the seed

    std::vector<Position> lastSampledAccels(num_units, {0.0, 0.0, 0.0});
    std::vector<Position> lastSampledGyros(num_units, {0.0, 0.0, 0.0});
    std::vector<Quaternion> lastSampledOrientations(num_units, {1.0, 0.0, 0.0, 0.0});
    std::vector<size_t> lastSampleSourceSlots(num_units, 0);

    // 2. Main loop: iterate through Units
    for (size_t unitIndex = 0; unitIndex < num_units; ++unitIndex)
    {
        // Then Iterate through Time Slots
        for (size_t currentSlot = 0; currentSlot < totalSlots; ++currentSlot)
        {
            const engineUnit &currentUnit = engineUnits[unitIndex];
            if (!currentUnit.IMU_enabled)
            {
                continue; // Skip this unit if its IMU is disabled
            }

            // --- Fetch necessary parameters for this unit ---
            std::normal_distribution<double> standard_normal_dist(0.0, 1.0);
            const size_t unitSpecificDelaySlots = currentUnit.IMU_samplingDelay_slots;

            // --- Update Bias State for this unit at this time slot ---
            updateBiasRandomWalk(accel_bias[unitIndex],
                                 currentUnit.IMU_accel_bias_random_walk,
                                 dt_sec,
                                 master_generator);
            updateBiasRandomWalk(gyro_bias[unitIndex],
                                 currentUnit.IMU_gyro_bias_random_walk,
                                 dt_sec,
                                 master_generator);

            // --- Check for Sampling Instant ---
            bool isSamplingInstant = (unitSpecificDelaySlots == 0) || (currentSlot % unitSpecificDelaySlots == 0);

            if (isSamplingInstant)
            {
                try
                {
                    size_t sourceSlotIndex = (currentSlot >= unitSpecificDelaySlots)
                                                 ? (currentSlot - unitSpecificDelaySlots)
                                                 : 0;
                    sourceSlotIndex = std::min(sourceSlotIndex, totalSlots > 0 ? totalSlots - 1 : 0);
                    lastSampleSourceSlots[unitIndex] = sourceSlotIndex;

                    environmentObject trueEnvData = engine->getEnvironmentData(unitIndex,
                                                                               sourceSlotIndex);
                    Quaternion trueOrientationAtSource = trueEnvData.quaternion;

                    Position gravity_body = rotateVectorByQuaternion(GRAVITY_WORLD,
                                                                     trueOrientationAtSource);
                    Position true_accel_body = rotateVectorByQuaternion(trueEnvData.acceleration,
                                                                        trueOrientationAtSource);
                    Position specific_force = {true_accel_body.x - gravity_body.x,
                                               true_accel_body.y - gravity_body.y,
                                               true_accel_body.z - gravity_body.z};

                    Position true_ang_vel_body = rotateVectorByQuaternion(trueEnvData.angular_velocity,
                                                                          trueOrientationAtSource);

                    // --- Apply Bias & White Noise ---
                    Position biased_accel = {specific_force.x + accel_bias[unitIndex].x,
                                             specific_force.y + accel_bias[unitIndex].y,
                                             specific_force.z + accel_bias[unitIndex].z};
                    Position biased_gyro = {true_ang_vel_body.x + gyro_bias[unitIndex].x,
                                            true_ang_vel_body.y + gyro_bias[unitIndex].y,
                                            true_ang_vel_body.z + gyro_bias[unitIndex].z};

                    const double accel_wn_discrete_std = currentUnit.IMU_accel_noise_density * inv_sqrt_dt_sec;
                    const double gyro_wn_discrete_std = currentUnit.IMU_gyro_noise_density * inv_sqrt_dt_sec;

                    if (accel_wn_discrete_std > EPSILON)
                    {
                        biased_accel.x += accel_wn_discrete_std * standard_normal_dist(master_generator);
                        biased_accel.y += accel_wn_discrete_std * standard_normal_dist(master_generator);
                        biased_accel.z += accel_wn_discrete_std * standard_normal_dist(master_generator);
                    }
                    if (gyro_wn_discrete_std > EPSILON)
                    {
                        biased_gyro.x += gyro_wn_discrete_std * standard_normal_dist(master_generator);
                        biased_gyro.y += gyro_wn_discrete_std * standard_normal_dist(master_generator);
                        biased_gyro.z += gyro_wn_discrete_std * standard_normal_dist(master_generator);
                    }

                    // Update the stored "last sample" for this unit
                    lastSampledAccels[unitIndex] = biased_accel;
                    lastSampledGyros[unitIndex] = biased_gyro;
                    lastSampledOrientations[unitIndex] = trueOrientationAtSource;
                }
                catch (const std::exception &e)
                {
                    qWarning() << "IMU sim error for unit" << unitIndex << "at slot" << currentSlot
                               << ":" << e.what();
                }
            } // End if(isSamplingInstant)

            // Get a pointer to the start of the IMU data block for this unit and slot.
            imuObject *imu_data_ptr = reinterpret_cast<imuObject *>(
                engine->getDataPointer(unitIndex, currentSlot, DataField::IMU_ACC));

            // Write the data directly to the final destination.
            imu_data_ptr->acceleration = lastSampledAccels[unitIndex];
            imu_data_ptr->angular_velocity = lastSampledGyros[unitIndex];
            imu_data_ptr->quaternion = lastSampledOrientations[unitIndex];
            imu_data_ptr->sample_slot = lastSampleSourceSlots[unitIndex];

        } // End inner unit loop
    } // End outer time slot loop

    if (debug)
        std::cout << "IMU simulation completed for all units." << std::endl;
}
