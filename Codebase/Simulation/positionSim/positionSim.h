#ifndef POSITIONSIM_H
#define POSITIONSIM_H

#include "Software/jsonReader/jsonReader.hpp"
#include "Software/structDefinition.h"

#include <utility>
#include <vector>

class mobileTHzEngine;

class positionSim
{
public:
  // Constructor and Destructor
  positionSim();
  ~positionSim();

  /**
   * @brief Initializes the position simulation, calculating position, orientation,
   *        and their derivatives (velocity, acceleration, jerk) for all units
   *        over all time slots based on sparse keyframes.
   *
   * @param positionConfig The JSON configuration containing the keyframe position data.
   * @param paramConfig The main simulation configuration file (ConfigFile struct).
   * @param engine Pointer to the mobileTHzEngine instance to populate with data.
   */
  void initialize(const nlohmann::json &positionConfig,
                  const ConfigFile &paramConfig,
                  mobileTHzEngine *engine);

private:
  /**
   * @brief Spherical Linear Interpolation between two quaternions.
   * @param q1 Start quaternion (will be normalized inside).
   * @param q2 End quaternion (will be normalized inside).
   * @param t Interpolation factor [0, 1].
   * @return Interpolated normalized quaternion using SLERP.
   */
  inline Quaternion slerp(const Quaternion &q1, const Quaternion &q2, double t) const;

  /**
   * @brief Normalized Linear Interpolation between two quaternions.
   * @param q1 Start quaternion (will be normalized inside).
   * @param q2 End quaternion (will be normalized inside).
   * @param t Interpolation factor [0, 1].
   * @return Interpolated normalized quaternion using NLERP.
   */
  inline Quaternion nlerp(const Quaternion &q1,
                          const Quaternion &q2,
                          double t) const; // Added NLERP

  /**
   * @brief Generates a smooth S-curve interpolation factor (using Perlin's smoother step).
   * @param t Input factor [0, 1].
   * @return Smoothed output factor [0, 1] with C2 continuity.
   */
  double sigmoidPosition(double t) const;

  /**
   * @brief Linear Interpolation between two positions.
   * @param p1 Start position.
   * @param p2 End position.
   * @param t Interpolation factor [0, 1].
   * @return Interpolated position.
   */
  inline Position lerpPosition(const Position &p1, const Position &p2, double t) const;

  /**
   * @brief S-Curve (smoothstep) interpolation between two positions.
   * @param p1 Start position.
   * @param p2 End position.
   * @param t Interpolation factor [0, 1]. Uses sigmoidPosition(t) as the effective t.
   * @return Interpolated position (smooth start and end).
   */
  inline Position sCurvePosition(const Position &p1, const Position &p2, double t) const;

  /**
   * @brief Calculates the average angular velocity required to rotate from q1 to q2 over dt.
   * @param q1 Start orientation (normalized internally).
   * @param q2 End orientation (normalized internally).
   * @param dt Time duration (in seconds). Should be > 0.
   * @return Angular velocity vector (rad/s).
   */
  inline Position quaternionToAngularVelocity(const Quaternion &q1,
                                              const Quaternion &q2,
                                              double dt) const;

  /**
   * @brief Calculates angular acceleration based on change in angular velocity over dt (finite difference).
   * @param ang_vel1 Angular velocity at time t1.
   * @param ang_vel2 Angular velocity at time t2.
   * @param dt Time difference (t2 - t1) in seconds. Should be > 0.
   * @return Angular acceleration vector (rad/s^2).
   */
  inline Position calculateAngularAcceleration(const Position &ang_vel1,
                                               const Position &ang_vel2,
                                               double dt) const;

  /**
   * @brief Calculates the distance squared between two positions.
   * @param p1 First position.
   * @param p2 Second position.
   * @return Squared distance between the two positions.
   */
  double positionDistanceSq(const Position &p1, const Position &p2);

  /**
   * @brief Checks if two quaternions are different within a specified tolerance.
   * @param q1_in First quaternion.
   * @param q2_in Second quaternion.
   * @param tolerance Tolerance for comparison.
   * @return True if the quaternions are different, false otherwise.
   */
  bool areQuaternionsDifferent(const Quaternion &q1_in, const Quaternion &q2_in, double tolerance);

  /**
   * @brief Calculates the dot product of two quaternions.
   * @param q1 First quaternion.
   * @param q2 Second quaternion.
   * @return Dot product of the two quaternions.
   */
  double quaternionDot(const Quaternion &q1, const Quaternion &q2);

  /**
   * @brief Finds the keyframe interval [start_idx, end_idx] that contains the target_slot.
   *        Handles cases where target_slot is before the first or after the last keyframe.
   * @param target_slot The time slot index to find the interval for.
   * @param keyframes Sorted vector of keyframes for a specific unit.
   * @return std::pair<size_t, size_t> containing the start and end indices in the keyframes vector.
   *         Returns [0, 0] if target_slot <= first slot.
   *         Returns [N-2, N-1] if target_slot >= last slot (N = #keyframes).
   *         Returns [i, i+1] if keyframes[i].slot <= target_slot < keyframes[i+1].slot.
   */
  std::pair<size_t, size_t> findKeyframeInterval(int target_slot,
                                                 const std::vector<KeyframeData> &keyframes) const;

  /**
   * @brief Gets the interpolated position and orientation for a specific target slot based on keyframes.
   *        Interpolation type (Linear/Spherical) is determined by the *mode* of the *end* keyframe of the interval.
   * @param target_slot The desired time slot index.
   * @param keyframes Sorted vector of keyframes for the unit.
   * @param total_slots Total number of slots in the simulation (for clamping target_slot).
   * @return InterpolatedState struct containing the position and quaternion.
   */
  inline InterpolatedState getInterpolatedState(int target_slot,
                                                const std::vector<KeyframeData> &keyframes,
                                                size_t total_slots,
                                                size_t &keyframe_idx_hint) const;

  inline Quaternion normalizeQuaternion(const Quaternion &q) const;
};

#endif // POSITIONSIM_H
