#include "Static.h"

// Constructor
Static::Static()
{
    // The algorithm is immediately in a passive state.
}

// This is the core of the "do nothing" logic. The function is intentionally empty.
void Static::processSlot(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot)
{
    // Do nothing.
}

// Return a fixed, passive status. This indicates that the algorithm is running
// but not performing any actions or in an error state.
algorithmObject Static::getStatus() const
{
    return {static_cast<double>(AlgoState::MONITORING), static_cast<double>(AlgoAction::NONE)};
}

// This algorithm has no parameters, so the configure function is empty.
void Static::configure(const ConfigFile &config, const engineUnit &unitConfig)
{
    // No configuration needed.
}
