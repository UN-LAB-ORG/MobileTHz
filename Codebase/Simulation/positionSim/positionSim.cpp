#include "positionSim.h"
#include "Software/mobileTHzEngine/mobileTHzEngine.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static bool debug = false;

positionSim::positionSim() {}

positionSim::~positionSim() {}

Quaternion positionSim::normalizeQuaternion(const Quaternion &q) const
{
    double mag_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

    // Fast path for nearly-normalized quaternions.
    if (std::abs(1.0 - mag_sq) < 1e-14)
    { // Using a small epsilon for float comparison
        return q;
    }

    // Safety check for zero-magnitude quaternions.
    if (mag_sq <= 1e-14)
    {
        return {1.0, 0.0, 0.0, 0.0};
    }

    // Calculate the inverse magnitude.
    // std::sqrt() is highly optimized by compilers and hardware.
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

// Helper to find the keyframe interval for a given slot
std::pair<size_t, size_t> positionSim::findKeyframeInterval(
    int target_slot, const std::vector<KeyframeData> &keyframes) const
{
    if (keyframes.empty())
    {
        throw std::runtime_error("Cannot find interval in empty keyframes.");
    }
    // Handle edge case: target_slot before or at the first keyframe
    if (target_slot <= keyframes.front().slot)
    {
        // If only one keyframe, interval is [0, 0].
        return {0, 0};
    }
    // Handle edge case: target_slot at or after the last keyframe
    if (target_slot >= keyframes.back().slot)
    {
        size_t last_idx = keyframes.size() - 1;
        // Interval is the last segment [last-1, last], or [0,0] if only one keyframe
        return {last_idx > 0 ? last_idx - 1 : 0, last_idx};
    }
    // Find the interval [i, i+1] such that keyframes[i].slot <= target_slot < keyframes[i+1].slot
    size_t i = 0;
    // Loop until we find the keyframe just *before* or at the target slot
    // Ensure we don't go past the second-to-last element
    while (i < keyframes.size() - 1 && keyframes[i + 1].slot <= target_slot)
    {
        i++;
    }
    // The interval is between keyframe i and keyframe i+1
    return {i, i + 1};
}

// Helper function to get interpolated state for ANY slot
inline InterpolatedState positionSim::getInterpolatedState(int target_slot,
                                                           const std::vector<KeyframeData> &keyframes,
                                                           size_t total_slots,
                                                           size_t &keyframe_idx_hint) const
{
    if (keyframes.empty())
    {
        return {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};
    }

    target_slot = std::clamp(target_slot,
                             0,
                             static_cast<int>(total_slots > 0 ? total_slots - 1 : 0));

    while (keyframe_idx_hint + 1 < keyframes.size() && target_slot >= keyframes[keyframe_idx_hint + 1].slot)
    {
        keyframe_idx_hint++;
    }

    const size_t start_idx = keyframe_idx_hint;
    const size_t end_idx = std::min(start_idx + 1, keyframes.size() - 1);

    const KeyframeData &startKeyframe = keyframes[start_idx];
    const KeyframeData &endKeyframe = keyframes[end_idx];

    double t = 0.0;
    double interval_duration = static_cast<double>(endKeyframe.slot - startKeyframe.slot);

    if (interval_duration > 1e-9)
    {
        t = static_cast<double>(target_slot - startKeyframe.slot) / interval_duration;
        t = std::clamp(t, 0.0, 1.0);
    }
    else
    {
        // This case handles single keyframes or zero-duration intervals
        return {startKeyframe.position, normalizeQuaternion(startKeyframe.orientation)};
    }

    InterpolatedState result_state;
    switch (endKeyframe.mode)
    {
    case InterpolationMode::Linear:
        result_state.position = lerpPosition(startKeyframe.position, endKeyframe.position, t);
        result_state.quaternion = nlerp(startKeyframe.orientation, endKeyframe.orientation, t);
        break;
    case InterpolationMode::Spherical:
    default:
        result_state.position = sCurvePosition(startKeyframe.position, endKeyframe.position, t);
        result_state.quaternion = slerp(startKeyframe.orientation, endKeyframe.orientation, t);
        break;
    }
    return result_state;
}

void positionSim::initialize(const nlohmann::json &positionConfig,
                             const ConfigFile &paramConfig,
                             mobileTHzEngine *engine)
{
    if (!engine)
    {
        std::cerr << "Error: positionSim::initialize called with null engine pointer." << std::endl;
        return;
    }

    if (!positionConfig.contains("positions") || !positionConfig["positions"].is_array())
    {
        std::cerr << "Error: Invalid position config: 'positions' key missing or not an array."
                  << std::endl;
        return;
    }

    double slotDuration = paramConfig.engine_slot_time_microsec / 1e6;
    if (slotDuration <= std::numeric_limits<double>::epsilon())
    {
        std::cerr << "Error: slotDuration must be positive and non-negligible." << std::endl;
        return;
    }
    size_t total_slots = engine->getNumTimeSlots();
    if (total_slots == 0)
    {
        return; // Nothing to do
    }
    size_t num_units = paramConfig.engine_units.size();
    if (num_units == 0)
    {
        return; // Nothing to do
    }

    // --- Create a map for fast label-to-index lookups ---
    std::unordered_map<std::string, size_t> label_to_index_map;
    for (size_t i = 0; i < num_units; ++i)
    {
        label_to_index_map[paramConfig.engine_units[i].label] = i;
    }

    // --- Load Keyframes ---
    std::vector<std::vector<KeyframeData>> unitKeyframes(num_units);
    try
    {
        const auto &positionsArray = positionConfig.at("positions");
        for (const auto &slotData : positionsArray)
        {
            int loadedSlot = slotData.at("slot").get<int>();
            const auto &unitsArray = slotData.at("units");

            for (const auto &unitData : unitsArray)
            {
                std::string label = unitData.at("label").get<std::string>();
                auto it = label_to_index_map.find(label);
                if (it == label_to_index_map.end())
                    continue;
                size_t unitIndex = it->second;

                Position pos = {unitData.at("x").get<double>(),
                                unitData.at("y").get<double>(),
                                unitData.at("z").get<double>()};
                Quaternion quat = {unitData.at("w").get<double>(),
                                   unitData.at("qx").get<double>(),
                                   unitData.at("qy").get<double>(),
                                   unitData.at("qz").get<double>()};
                quat = normalizeQuaternion(quat);
                std::string modeStr = unitData.value("mode", "Spherical");
                InterpolationMode mode = (modeStr == "Linear") ? InterpolationMode::Linear
                                                               : InterpolationMode::Spherical;
                unitKeyframes[unitIndex].push_back({loadedSlot, pos, quat, mode});
            }
        }
    }
    catch (const nlohmann::json::exception &e)
    {
        std::cerr << "Error processing 'positions' config: " << e.what() << std::endl;
        return;
    }

    // Sort Keyframes & Remove Duplicates
    for (auto &keyframes : unitKeyframes)
    {
        if (keyframes.empty())
            continue;
        std::sort(keyframes.begin(), keyframes.end(), [](const auto &a, const auto &b)
                  { return a.slot < b.slot; });
        auto last = std::unique(keyframes.begin(),
                                keyframes.end(),
                                [](const auto &a, const auto &b)
                                { return a.slot == b.slot; });
        keyframes.erase(last, keyframes.end());
    }

    // --- Determine First and Last ACTUAL Motion Slots ---
    size_t first_motion_start_slot = SIZE_MAX;
    size_t last_motion_end_slot = 0;
    bool motion_detected = false;
    const double POS_CHANGE_THRESHOLD_SQ = 1e-12;
    const double QUAT_DIFF_TOLERANCE = 1e-8;

    for (size_t unitIndex = 0; unitIndex < num_units; ++unitIndex)
    {
        const auto &keyframes = unitKeyframes[unitIndex];
        if (keyframes.size() < 2)
            continue;
        for (size_t i = 1; i < keyframes.size(); ++i)
        {
            const auto &prev_keyframe = keyframes[i - 1];
            const auto &current_keyframe = keyframes[i];
            if (prev_keyframe.slot < 0 || current_keyframe.slot < 0 || current_keyframe.slot <= prev_keyframe.slot)
                continue;
            bool pos_changed = positionDistanceSq(current_keyframe.position, prev_keyframe.position) > POS_CHANGE_THRESHOLD_SQ;
            bool quat_changed = areQuaternionsDifferent(current_keyframe.orientation,
                                                        prev_keyframe.orientation,
                                                        QUAT_DIFF_TOLERANCE);
            if (pos_changed || quat_changed)
            {
                motion_detected = true;
                first_motion_start_slot = std::min(first_motion_start_slot,
                                                   static_cast<size_t>(prev_keyframe.slot));
                last_motion_end_slot = std::max(last_motion_end_slot,
                                                static_cast<size_t>(current_keyframe.slot));
            }
        }
    }

    // --- Assign Final Motion Slots to Engine ---
    if (!motion_detected)
    {
        engine->firstMotionSlot = -1;
        engine->lastMotionSlot = -1;
    }
    else
    {
        engine->firstMotionSlot = (first_motion_start_slot > static_cast<size_t>(std::numeric_limits<int>::max()))
                                      ? std::numeric_limits<int>::max()
                                      : static_cast<int>(first_motion_start_slot);
        engine->lastMotionSlot = (last_motion_end_slot > static_cast<size_t>(std::numeric_limits<int>::max()))
                                     ? std::numeric_limits<int>::max()
                                     : static_cast<int>(last_motion_end_slot);
        engine->lastMotionSlot = std::max(engine->lastMotionSlot, engine->firstMotionSlot);
    }

    const double h = slotDuration;
    const double inv_h = 1.0 / h;
    const double inv_2h = 1.0 / (2.0 * h);
    const double inv_h_sq = 1.0 / (h * h);

    for (size_t unitIndex = 0; unitIndex < num_units; ++unitIndex)
    {
        const auto &keyframes = unitKeyframes[unitIndex];
        if (keyframes.empty())
        {
            static const environmentObject empty_env{};
            for (size_t slot = 0; slot < total_slots; ++slot)
            {
                // Get pointer to destination and copy the static empty object's data there
                environmentObject *env_data_ptr = reinterpret_cast<environmentObject *>(
                    engine->getDataPointer(unitIndex, slot, DataField::ENV_POS));
                *env_data_ptr = empty_env;
            }
            continue;
        }

        // --- State variables for the sliding window ---
        size_t kf_idx_hint = 0;
        // Prime the loop by calculating the first three states for our window
        InterpolatedState state_prev = getInterpolatedState(0, keyframes, total_slots, kf_idx_hint);
        InterpolatedState state_curr = state_prev;
        InterpolatedState state_next = getInterpolatedState(1, keyframes, total_slots, kf_idx_hint);

        for (size_t slot = 0; slot < total_slots; ++slot)
        {
            // Get a direct pointer to the final destination memory for this slot
            environmentObject *env_data_ptr = reinterpret_cast<environmentObject *>(
                engine->getDataPointer(unitIndex, slot, DataField::ENV_POS));

            // Shift the states for the current iteration's context
            // The state from the previous iteration is now our "previous" state.
            if (slot > 0)
            {
                state_prev = state_curr;
                state_curr = state_next;
            }

            // Calculate only ONE new state for the 'next' position in the window.
            if (slot < total_slots - 1)
            {
                // kf_idx_hint is passed by reference and updated internally
                state_next = getInterpolatedState(slot + 1, keyframes, total_slots, kf_idx_hint);
            }

            // --- Write directly into the engine's memory grid via the pointer ---
            env_data_ptr->position = state_curr.position;
            env_data_ptr->quaternion = state_curr.quaternion;

            if (slot == 0)
            {
                // Forward difference for the first point
                env_data_ptr->velocity = {(state_next.position.x - state_curr.position.x) * inv_h,
                                          (state_next.position.y - state_curr.position.y) * inv_h,
                                          (state_next.position.z - state_curr.position.z) * inv_h};
                env_data_ptr->angular_velocity = quaternionToAngularVelocity(state_curr.quaternion,
                                                                             state_next.quaternion,
                                                                             h);
                env_data_ptr->acceleration = {0, 0, 0};
                env_data_ptr->angular_acceleration = {0, 0, 0};
            }
            else if (slot == total_slots - 1)
            {
                // Backward difference for the last point
                env_data_ptr->velocity = {(state_curr.position.x - state_prev.position.x) * inv_h,
                                          (state_curr.position.y - state_prev.position.y) * inv_h,
                                          (state_curr.position.z - state_prev.position.z) * inv_h};
                env_data_ptr->angular_velocity = quaternionToAngularVelocity(state_prev.quaternion,
                                                                             state_curr.quaternion,
                                                                             h);
                env_data_ptr->acceleration = {0, 0, 0};
                env_data_ptr->angular_acceleration = {0, 0, 0};
            }
            else
            {
                // Central difference for interior points
                env_data_ptr->velocity = {(state_next.position.x - state_prev.position.x) * inv_2h,
                                          (state_next.position.y - state_prev.position.y) * inv_2h,
                                          (state_next.position.z - state_prev.position.z) * inv_2h};
                env_data_ptr->acceleration = {(state_next.position.x - 2.0 * state_curr.position.x + state_prev.position.x) * inv_h_sq,
                                              (state_next.position.y - 2.0 * state_curr.position.y + state_prev.position.y) * inv_h_sq,
                                              (state_next.position.z - 2.0 * state_curr.position.z + state_prev.position.z) * inv_h_sq};

                Position ang_vel_prev_half = quaternionToAngularVelocity(state_prev.quaternion,
                                                                         state_curr.quaternion,
                                                                         h);
                Position ang_vel_next_half = quaternionToAngularVelocity(state_curr.quaternion,
                                                                         state_next.quaternion,
                                                                         h);
                env_data_ptr->angular_velocity = ang_vel_next_half;
                env_data_ptr->angular_acceleration = calculateAngularAcceleration(ang_vel_prev_half,
                                                                                  ang_vel_next_half,
                                                                                  h);
            }
        }
    }
}

// --- Helper Function Implementations ---
Quaternion positionSim::slerp(const Quaternion &q1_in, const Quaternion &q2_in, double t) const
{
    // Make local mutable copies of quaternions. q2 might need its components flipped.
    Quaternion q1 = q1_in;
    Quaternion q2 = q2_in;

    // --- Step 1 & 2: Calculate the dot product ---
    // Direct scalar calculation.
    double dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

    // --- Step 3: Handle the "shortest path" problem (dot < 0) ---
    // If dot is negative, we need to flip one quaternion to take the shorter arc.
    if (dot < 0.0)
    {
        q2.w = -q2.w;
        q2.x = -q2.x;
        q2.y = -q2.y;
        q2.z = -q2.z;
        dot = -dot; // Also flip the dot product
    }

    // Clamp dot product to avoid domain errors with acos (should be between 0 and 1 here)
    dot = std::min(dot, 1.0);

    // --- Step 4: Check for small angles and fall back to NLERP ---
    const double SLERP_THRESHOLD = 0.99995;
    if (dot > SLERP_THRESHOLD)
    {
        // Linear interpolation (LERP): result = (1-t)*q1 + t*q2
        Quaternion result_q;
        double omt = 1.0 - t; // One minus t

        result_q.w = q1.w * omt + q2.w * t;
        result_q.x = q1.x * omt + q2.x * t;
        result_q.y = q1.y * omt + q2.y * t;
        result_q.z = q1.z * omt + q2.z * t;

        // The result of LERP is not normalized, so we must normalize it.
        // This is NLERP (Normalized Linear Interpolation).
        return normalizeQuaternion(result_q); // Calls the scalar normalizeQuaternion
    }

    // --- Step 5: Standard SLERP calculation for larger angles ---
    double theta_0 = std::acos(dot);
    double sin_theta_0 = std::sin(theta_0);

    // This check is still useful for robustness, to prevent division by very small numbers.
    if (std::abs(sin_theta_0) < 1e-12)
    {
        // If the angle is too small (or near PI, handled by dot<0 flip), quaternions are essentially
        // identical or opposite. Return one of them.
        return q1_in;
    }

    double inv_sin_theta_0 = 1.0 / sin_theta_0;
    double scale0 = std::sin((1.0 - t) * theta_0) * inv_sin_theta_0;
    double scale1 = std::sin(t * theta_0) * inv_sin_theta_0;

    // --- Step 6: Apply scales and sum ---
    Quaternion result;
    result.w = q1.w * scale0 + q2.w * scale1;
    result.x = q1.x * scale0 + q2.x * scale1;
    result.y = q1.y * scale0 + q2.y * scale1;
    result.z = q1.z * scale0 + q2.z * scale1;

    // The result of proper SLERP on unit quaternions is already normalized.
    return result;
}

// Normalized Linear Interpolation (NLERP) for Quaternions
Quaternion positionSim::nlerp(const Quaternion &q1_in, const Quaternion &q2_in, double t) const
{
    Quaternion q1 = q1_in;
    Quaternion q2 = q2_in;

    // Calculate dot product
    double dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

    // Handle dot < 0 (take shortest path by flipping one quaternion)
    if (dot < 0.0)
    {
        q2.w = -q2.w;
        q2.x = -q2.x;
        q2.y = -q2.y;
        q2.z = -q2.z;
        // No need to flip dot here for NLERP calculation itself
    }

    // Clamp t just in case
    t = std::clamp(t, 0.0, 1.0);

    // Perform linear interpolation of components
    Quaternion result;
    result.w = q1.w + t * (q2.w - q1.w);
    result.x = q1.x + t * (q2.x - q1.x);
    result.y = q1.y + t * (q2.y - q1.y);
    result.z = q1.z + t * (q2.z - q1.z);

    // Normalize the result
    return normalizeQuaternion(result);
}

double positionSim::sigmoidPosition(double t) const
{
    // Using Perlin's smoother step: 6t^5 - 15t^4 + 10t^3
    // for better C2 continuity (zero acceleration at ends)
    return t * t * t * (10.0 + t * (-15.0 + t * 6.0));
}

Position positionSim::lerpPosition(const Position &p1, const Position &p2, double t) const
{
    return {p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y), p1.z + t * (p2.z - p1.z)};
}

Position positionSim::sCurvePosition(const Position &p1, const Position &p2, double t) const
{
    // Use the sigmoid/smoothstep factor for interpolation
    double s = sigmoidPosition(t); // s is already clamped [0, 1]
    return lerpPosition(p1, p2, s);
}

Position positionSim::quaternionToAngularVelocity(const Quaternion &q1_in,
                                                  const Quaternion &q2_in,
                                                  double dt) const
{
    const Position zero_vel = {0.0, 0.0, 0.0};
    if (dt <= std::numeric_limits<double>::epsilon())
    {
        return zero_vel;
    }

    Quaternion q1 = q1_in;
    Quaternion q2 = q2_in;

    // Calculate the difference quaternion: q_diff = q2 * conjugate(q1)
    // conjugate(q1) is {q1.w, -q1.x, -q1.y, -q1.z}
    Quaternion q_diff;
    q_diff.w = q2.w * q1.w + q2.x * q1.x + q2.y * q1.y + q2.z * q1.z;
    q_diff.x = -q2.w * q1.x + q2.x * q1.w - q2.y * q1.z + q2.z * q1.y;
    q_diff.y = -q2.w * q1.y + q2.x * q1.z + q2.y * q1.w - q2.z * q1.x;
    q_diff.z = -q2.w * q1.z - q2.x * q1.y + q2.y * q1.x + q2.z * q1.w;

    // To get the shortest path, if the scalar part 'w' is negative,
    // the rotation is > 180 degrees. We use the equivalent shorter rotation
    // by negating the vector part. This correctly flips the axis for the negated angle.
    double sign = 1.0;
    if (q_diff.w < 0.0)
    {
        sign = -1.0;
    }

    // The angular velocity is 2/dt times the vector part of the
    // difference quaternion. This approximation is excellent for small rotations
    // and avoids expensive acos/sqrt.
    const double scale = 2.0 / dt;

    Position angular_velocity;
    angular_velocity.x = sign * q_diff.x * scale;
    angular_velocity.y = sign * q_diff.y * scale;
    angular_velocity.z = sign * q_diff.z * scale;

    return angular_velocity;
}

Position positionSim::calculateAngularAcceleration(const Position &ang_vel1,
                                                   const Position &ang_vel2,
                                                   double dt) const
{
    // Standard finite difference for acceleration
    if (dt <= std::numeric_limits<double>::epsilon())
    {
        // Avoid division by zero;
        return {0.0, 0.0, 0.0};
    }
    Position angular_acceleration;
    angular_acceleration.x = (ang_vel2.x - ang_vel1.x) / dt;
    angular_acceleration.y = (ang_vel2.y - ang_vel1.y) / dt;
    angular_acceleration.z = (ang_vel2.z - ang_vel1.z) / dt;
    return angular_acceleration;
}

bool positionSim::areQuaternionsDifferent(const Quaternion &q1_in,
                                          const Quaternion &q2_in,
                                          double tolerance)
{
    // Normalize inputs for reliable comparison
    Quaternion q1 = normalizeQuaternion(q1_in);
    Quaternion q2 = normalizeQuaternion(q2_in);
    double dot_abs = std::abs(quaternionDot(q1, q2));
    // Check if 1.0 - dot_abs is greater than the tolerance
    return (1.0 - dot_abs) > tolerance;
}

double positionSim::quaternionDot(const Quaternion &q1, const Quaternion &q2)
{
    return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
}

double positionSim::positionDistanceSq(const Position &p1, const Position &p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return dx * dx + dy * dy + dz * dz;
}
