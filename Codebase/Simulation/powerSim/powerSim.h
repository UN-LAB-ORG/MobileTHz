// powerSim.h
#ifndef POWERSIM_H
#define POWERSIM_H

#include "structDefinition.h"
#include <random>
#include <vector>

class mobileTHzEngine;
struct Quaternion;
struct Position;
struct radiationPattern;
struct rotaryObject;

class powerSim
{
private:
    // Configuration & Engine Access
    mobileTHzEngine *engine_ = nullptr;

    // Cached Parameters
    double c_ = 299792458.0; // Speed of light (m/s)
    double path_loss_wavelength_factor_ = 0.0;
    uint64_t seed_ = 0;

    mutable std::mt19937 rand_engine_;
    mutable std::exponential_distribution<double> exponential_dist_;

    // Helper Functions
    inline Quaternion conjugate(const Quaternion &q) const;
    inline Position rotateVectorByQuaternion(const Position &v,
                                             const Quaternion &q_normalized) const;
    inline Quaternion normalizeQuaternion(const Quaternion &q) const;
    inline double getGainFromPattern(const engineUnit &inputEngineUnit,
                                     int patternId,
                                     double cos_theta_off_boresight) const;
    static double linearInterpolate(double x,
                                    const std::vector<double> &x_data,
                                    const std::vector<double> &y_data);
    inline Quaternion multiplyQuaternions(const Quaternion &q1, const Quaternion &q2) const;

    /**
     * @brief Converts Azimuth and Altitude angles (in degrees) to a rotation quaternion.
     * Assumes Azimuth is rotation around the base Z-axis, followed by
     * Altitude rotation around the new Y-axis.
     * @param azimuthDegrees Rotation around Z-axis (degrees).
     * @param altitudeDegrees Rotation around the intermediate Y-axis (degrees).
     * @return The corresponding rotation quaternion relative to the base frame.
     */
    Quaternion quaternionFromAzAltDegrees(double azimuthDegrees, double altitudeDegrees) const;

    double lut_angle_step_degrees_ = 0.1; // Step size for fine-grained LUT (degrees)

    PrecomputedPatternLUT buildFineGrainedCosLut(const radiationPattern &originalPattern,
                                                 double desiredAngleStepDegrees) const;

    // Cache: Key is the address of the original radiationPattern, Value is the LUT
    // Marked mutable because getGainFromPattern is const but needs to populate the cache on first access.
    mutable std::unordered_map<const radiationPattern *, PrecomputedPatternLUT> pattern_lut_cache_;

public:
    powerSim();
    ~powerSim();
    void initialize(const ConfigFile &paramConfig, mobileTHzEngine *engine_ptr);

    /**
     * @brief Calculates the received power at the RX unit from the TX unit at a specific time.
     *        Considers positions, orientations (base + rotary from Az/Alt),
     *        antenna patterns, Tx power, and path loss.
     * @param txUnitIdx Index of the transmitting unit.
     * @param rxUnitIdx Index of the receiving unit.
     * @param txUnit Reference to the transmitting unit.
     * @param rxUnit Reference to the receiving unit.
     * @param timeIdx Time slot index.
     * @return Received power in dBm. Returns a very low value (e.g., -99.0) on error or if disabled.
     */
    double calculateReceivedPower(const size_t &txUnitIdx,
                                  const size_t &rxUnitIdx,
                                  const engineUnit &txEngineUnit,
                                  const engineUnit &rxEngineUnit,
                                  const size_t &timeIdx) const;

    // "Stateless" version for what-if scenarios
    double calculateReceivedPower(const engineUnit &txEngineUnit,
                                  const engineUnit &rxEngineUnit,
                                  const environmentObject &txEnv,
                                  const environmentObject &rxEnv,
                                  const rotaryObject &txRotary,
                                  const rotaryObject &rxRotary,
                                  const antennaObject &txAnt,
                                  const antennaObject &rxAnt) const;
};

#endif // POWERSIM_H
