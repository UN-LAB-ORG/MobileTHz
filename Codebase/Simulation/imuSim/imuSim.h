// imuSim.h

#ifndef IMUSIM_H
#define IMUSIM_H

#include "Software/structDefinition.h"
#include <random>
#include <vector>

class mobileTHzEngine;
class QRandomGenerator;

class imuSim
{
private:
    mobileTHzEngine *engine = nullptr;
    double dt_sec = 0.0;

    // Store current bias values for each unit [Units: accel(m/s^2), gyro(rad/s)]
    std::vector<Position> accel_bias;
    std::vector<Position> gyro_bias;

    static Quaternion conjugate(const Quaternion &q);
    static Position rotateVectorByQuaternion(const Position &v, const Quaternion &q);

    /**
     * @brief Updates the bias estimate using a discrete-time random walk model,
     *        derived from the continuous-time random walk density parameter.
     * @param bias Reference to the bias vector (Position) to update [m/s^2 or rad/s].
     * @param bias_random_walk_density Standard deviation density of the underlying
     *        continuous-time white noise driving the bias random walk
     *        (e.g., [m/s^3/sqrt(Hz)] for accel, [rad/s^2/sqrt(Hz)] for gyro). This is sigma_b.
     * @param delta_t Time step duration in seconds (dt_sec).
     * @param local_generator Pointer to a thread-local random number generator.
     */
    static void updateBiasRandomWalk(Position &bias,
                                     double bias_random_walk_density,
                                     double delta_t,
                                     std::mt19937 &generator);

public:
    imuSim() = default;
    ~imuSim() = default;

    /**
     * @brief Initializes and runs the IMU simulation.
     *        - Sets initial bias based on mean and stddev parameters.
     *        - Iterates through time slots for each unit:
     *          - Updates bias using the random walk model.
     *          - Determines sampling instants based on delay.
     *          - Calculates noisy measurements (accel, gyro) based on:
     *            - Delayed ground truth motion.
     *            - Body-frame gravity projection.
     *            - Current bias estimate.
     *            - White noise (derived from density parameter).
     *          - Stores simulated IMU data back into the engine.
     * @param paramConfig Simulation configuration containing time step, units etc.
     * @param engine_ptr Pointer to the mobileTHzEngine.
     */
    void initialize(const ConfigFile &paramConfig, mobileTHzEngine *engine_ptr);
};

#endif // IMUSIM_H
