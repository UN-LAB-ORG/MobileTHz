// AlgorithmInterface.h
#ifndef ALGORITHM_INTERFACE_H
#define ALGORITHM_INTERFACE_H

#include "Codebase/Software/structDefinition.h"
#include <cstddef>

class mobileTHzEngine;
struct ConfigFile;
struct engineUnit;

class AlgorithmInterface
{
public:
    virtual ~AlgorithmInterface() = default;

    /**
     * @brief Called by the engine for each unit running this algorithm, every time slot.
     * @param engine Pointer to the engine instance for accessing state and issuing commands.
     * @param unitIndex The index of the unit this instance is processing for.
     * @param currentSlot The current time slot index.
     */
    virtual void processSlot(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot) = 0;

    /**
     * @brief Gets the current status of the algorithm instance.
     * @return algorithmObject struct.
     */
    virtual algorithmObject getStatus() const = 0;

    /**
     * @brief Optional: Allows external configuration/parameter setting after construction.
     * @param config Reference to the main configuration file.
     * @param unitConfig Reference to the specific configuration for the unit this algorithm runs on.
     */
    virtual void configure(const ConfigFile &config, const engineUnit &unitConfig) = 0;
};

#endif // ALGORITHM_INTERFACE_H
