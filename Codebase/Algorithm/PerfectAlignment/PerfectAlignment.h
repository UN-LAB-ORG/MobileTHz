#ifndef PERFECT_ALIGNMENT_H
#define PERFECT_ALIGNMENT_H

#include "Codebase/Algorithm/AlgorithmInterface.h"
#include "Codebase/Software/structDefinition.h"

/**
 * @class PerfectAlignment
 * @brief A benchmark algorithm that actively forces perfect alignment every slot.
 *
 * This algorithm calculates the ideal pointing vector to the target unit
 * in every time slot and directly overwrites the rotary stage state.
 * It serves as an ideal upper-bound for comparing the performance of
 * practical alignment algorithms.
 */
class PerfectAlignment : public AlgorithmInterface
{
public:
    PerfectAlignment();
    ~PerfectAlignment() override = default;

    /**
     * @brief Calculates ideal pointing angles and forces the rotary state for the current slot.
     */
    void processSlot(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot) override;

    /**
     * @brief Returns a constant 'MONITORING' status, representing the ideal state.
     */
    algorithmObject getStatus() const override;

    /**
     * @brief Does nothing, as this algorithm is parameter-free.
     */
    void configure(const ConfigFile &config, const engineUnit &unitConfig) override;
};

#endif
