#include "PerfectAlignment.h"
#include "../Software/mobileTHzEngine/mobileTHzEngine.h"
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace
{ // Use an anonymous namespace for the helper function

    /**
     * @brief Helper function to calculate and set the ideal rotary state for a specific slot.
     * @param engine The mobileTHzEngine instance.
     * @param unitIndex The index of the unit to align.
     * @param slotToAlign The time slot for which to calculate and set the alignment.
     */
    void calculateAndSetIdealRotation(mobileTHzEngine *engine, size_t unitIndex, size_t slotToAlign)
    {
        // Determine the target unit's index (assuming a 2-unit scenario).
        size_t targetUnitIndex = (unitIndex == 0) ? 1 : 0;

        try
        {
            // 1. Get position and orientation data for self and target for the specific slot.
            const environmentObject &selfEnv = engine->getEnvironmentData(unitIndex, slotToAlign);
            const environmentObject &targetEnv = engine->getEnvironmentData(targetUnitIndex,
                                                                            slotToAlign);

            // 2. Calculate the vector from self to target in the global coordinate frame.
            double globalVecX = (targetEnv.position.x - selfEnv.position.x);
            double globalVecY = (targetEnv.position.y - selfEnv.position.y);
            double globalVecZ = (targetEnv.position.z - selfEnv.position.z);

            // 3. Transform the global vector into the local coordinate system of the 'self' unit
            //    by rotating it with the inverse of the self's orientation quaternion.
            const auto &selfQuat = selfEnv.quaternion;
            double inv_q_w = selfQuat.w;
            double inv_q_x = -selfQuat.x;
            double inv_q_y = -selfQuat.y;
            double inv_q_z = -selfQuat.z;

            // Rotate the vector using the inverse quaternion (v' = q*v*q_conj)
            double t2 = inv_q_w * inv_q_x, t3 = inv_q_w * inv_q_y, t4 = inv_q_w * inv_q_z;
            double t5 = -inv_q_x * inv_q_x, t6 = inv_q_x * inv_q_y, t7 = inv_q_x * inv_q_z;
            double t8 = -inv_q_y * inv_q_y, t9 = inv_q_y * inv_q_z, t10 = -inv_q_z * inv_q_z;

            double localVecX = 2 * ((t8 + t10) * globalVecX + (t6 - t4) * globalVecY + (t7 + t3) * globalVecZ) + globalVecX;
            double localVecY = 2 * ((t6 + t4) * globalVecX + (t5 + t10) * globalVecY + (t9 - t2) * globalVecZ) + globalVecY;
            double localVecZ = 2 * ((t7 - t3) * globalVecX + (t9 + t2) * globalVecY + (t5 + t8) * globalVecZ) + globalVecZ;

            // 4. Convert the local Cartesian vector to spherical coordinates (Azimuth, Altitude).
            double azimuth_rad = std::atan2(localVecY, localVecX);
            double range = std::sqrt(localVecX * localVecX + localVecY * localVecY + localVecZ * localVecZ);
            double altitude_rad = (range > 1e-9) ? std::asin(localVecZ / range) : 0.0;

            // Convert radians to degrees.
            double azimuth_deg = azimuth_rad * 180.0 / M_PI;
            double altitude_deg = altitude_rad * 180.0 / M_PI;

            // 5. Create the ideal rotaryObject and manually set it in the time grid for the target slot.
            rotaryObject idealRotaryState;
            idealRotaryState.azimuth.angle = azimuth_deg;
            idealRotaryState.altitude.angle = -altitude_deg;
            idealRotaryState.isMoving = 0.0; // Instantaneous alignment.

            engine->setRotaryData(unitIndex, slotToAlign, idealRotaryState);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in PerfectAlignment::calculateAndSetIdealRotation for unit " << unitIndex
                      << " at slot " << slotToAlign << ": " << e.what() << std::endl;
        }
    }

} // end anonymous namespace

// Constructor
PerfectAlignment::PerfectAlignment() {}

// This algorithm has no configurable parameters.
void PerfectAlignment::configure(const ConfigFile &config, const engineUnit &unitConfig) {}

// Always report a stable, non-active state.
algorithmObject PerfectAlignment::getStatus() const
{
    return {static_cast<double>(AlgoState::MONITORING), static_cast<double>(AlgoAction::NONE)};
}

// The core logic: pre-calculate and force perfect alignment for future slots.
void PerfectAlignment::processSlot(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot)
{
    // On the very first slot, the alignment for slot 0 must be set BEFORE the
    // power calculation runs. Since algorithms run after, we must set it here.
    if (currentSlot == 0)
    {
        calculateAndSetIdealRotation(engine, unitIndex, 0);
    }

    // For every slot, calculate and set the ideal alignment for the next slot.
    // This ensures that when the engine loop begins for 'nextSlot', the rotary
    // data is already perfect.
    size_t nextSlot = currentSlot + 1;

    // Boundary check: Do not attempt to write past the end of the time grid.
    if (nextSlot < engine->getNumTimeSlots())
    {
        calculateAndSetIdealRotation(engine, unitIndex, nextSlot);
    }
}
