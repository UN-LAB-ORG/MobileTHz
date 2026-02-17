#ifndef STATIC_H
#define STATIC_H

#include "Codebase/Algorithm/AlgorithmInterface.h"
#include "Codebase/Software/structDefinition.h"

class Static : public AlgorithmInterface
{
public:
    Static();
    ~Static() override = default;

    /**
     * @brief This function does nothing, as per the algorithm's design.
     */
    void processSlot(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot) override;

    /**
     * @brief Returns a constant 'MONITORING' and 'NONE' status.
     * @return An algorithmObject indicating a passive state.
     */
    algorithmObject getStatus() const override;

    /**
     * @brief This function does nothing, as the algorithm requires no configuration.
     */
    void configure(const ConfigFile &config, const engineUnit &unitConfig) override;
};

#endif
