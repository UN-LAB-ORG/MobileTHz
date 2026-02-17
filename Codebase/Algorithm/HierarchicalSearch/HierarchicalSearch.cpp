#include "HierarchicalSearch.h"
#include "../Software/mobileTHzEngine/mobileTHzEngine.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <qmath.h>
#include <string>
#include <vector>

static bool debug = false;

namespace
{
    constexpr double LN10_OVER_10 = 0.23025850929940457; // std::log(10.0) / 10.0
    constexpr double TEN_OVER_LN10 = 4.342944819032518;  // 10.0 / std::log(10.0)
    constexpr double MIN_WATT_THRESHOLD = 1e-30;
    constexpr double MIN_DBM_FLOOR = -300.0;
    constexpr double LN10 = 2.302585092994046;
    constexpr double MIN_LINEAR_GAIN_THRESHOLD = 1e-30;
    constexpr double MIN_DBI_FLOOR = -300.0;

    inline double dBmtoWatts(double dBm)
    {
        if (dBm <= MIN_DBM_FLOOR)
        {
            return 0.0;
        }
        return std::exp((dBm - 30.0) * LN10_OVER_10);
    }

    inline double wattsToDbm(double watts)
    {
        if (watts <= MIN_WATT_THRESHOLD)
        {
            return MIN_DBM_FLOOR;
        }
        return TEN_OVER_LN10 * std::log(watts) + 30.0;
    }

    inline double dbiToLinear(double dBi)
    {
        // For very small dBi values, the linear gain is effectively zero.
        if (dBi <= MIN_DBI_FLOOR)
        {
            return 0.0;
        }
        return std::exp(dBi * LN10_OVER_10);
    }

    inline double linearToDbi(double linearGain)
    {
        // If the gain is at or below the threshold, return the floor value
        // to avoid std::log(0) or std::log(negative).
        if (linearGain <= MIN_LINEAR_GAIN_THRESHOLD)
        {
            return MIN_DBI_FLOOR;
        }
        return TEN_OVER_LN10 * std::log(linearGain);
    }

} // namespace

// Constructor
HierarchicalSearch::HierarchicalSearch()
{
    setState(AlgoState::REFERENCE, AlgoAction::NONE);
}

void HierarchicalSearch::configure(const ConfigFile &config, const engineUnit &unitConfig)
{
    algorithmName = unitConfig.algorithm + "_" + unitConfig.label;

    // Get slot time
    slotTimeMicrosec = config.engine_slot_time_microsec;
    if (slotTimeMicrosec <= 0)
    {
        std::cerr << "[" << algorithmName << "] Error: Invalid engine_slot_time_microsec ("
                  << slotTimeMicrosec << "). Using default 1000 us." << std::endl;
        slotTimeMicrosec = 1000.0;
    }

    // --- Reset Internal State ---
    currentState = AlgoState::REFERENCE;
    currentAction = AlgoAction::NONE;
    configured = true;
    currentReferencePower = -9999999;
    filterInitialized = false;
    shortTermEMA = -9999999;
    longTermEMA = -9999999;
    lastRawPower = -9999999;
    warmupCounter = 0;
    lastSuccessfulAlignmentSlot = 0;
    lastProcessedRxSampleSlot = std::numeric_limits<size_t>::max();
    spiralOffsets.clear();
    currentSpiralPointIndex = -1;
    spiralCenterAz = 0.0f;
    spiralCenterAlt = 0.0f;
    m_isPeakSeeking = false;
    m_peakSeekingBestPower = -9999999;
    m_peakSeekingBestIndex = -1;
    halfPowerBeamWidth = unitConfig.radiationPatterns_hpbw.at(0);
    realignSuccessMarginIncreaseRateDbPerSec = linearToDbi(
                                                   unitConfig.radiationPatterns_maxGain_linear.at(0)) *
                                               0.5;
    misalignThresholdAbsoluteDb = 3;
    realignSuccessInitialMarginDb = 0.25;

    // PRE-CALCULATE SPIRAL OFFSETS
    if (maxSpiralPoints > 0)
    {                            // Only generate if needed
        generateSpiralOffsets(); // Generate offsets relative to (0,0)

        if (debug)
            std::cout << "[" << algorithmName << "] Pre-calculated " << spiralOffsets.size()
                      << " spiral offset points." << std::endl;

        if (spiralOffsets.empty())
        {
            std::cerr
                << "[" << algorithmName
                << "] Warning: Pre-calculation generated 0 spiral offsets despite maxSpiralPoints "
                   "> 0. Check parameters (initialStepSizeDeg)."
                << std::endl;
        }
    }
    else
    {
        std::cout << "[" << algorithmName
                  << "] maxSpiralPoints set to 0. Spiral realignment is disabled." << std::endl;
    }

    setState(AlgoState::REFERENCE, AlgoAction::NONE);
}

// Set internal state and action
void HierarchicalSearch::setState(AlgoState newStatus, AlgoAction newAction)
{
    if (currentState != newStatus || currentAction != newAction)
    {
        currentState = newStatus;
        currentAction = newAction;
    }
}

// Get current algorithm status and action
algorithmObject HierarchicalSearch::getStatus() const
{
    // Cast the enums to int, then to double for the return object
    return {static_cast<double>(currentState), static_cast<double>(currentAction)};
}

// Update internal power filters (EMA, spike detection)
bool HierarchicalSearch::updatePowerFilter(double rawPower)
{
    // Ignore clearly invalid readings
    if (rawPower <= -200.0 || !std::isfinite(rawPower))
    { // Use a reasonable floor
        if (debug)
            std::cout << "[" << algorithmName << "] Invalid power reading: " << rawPower
                      << ". Ignoring." << std::endl;
        return false;
    }

    bool isSpike = false;

    if (!filterInitialized)
    {
        shortTermEMA = rawPower;
        longTermEMA = rawPower;
        lastRawPower = rawPower;
        filterInitialized = true;
        return true;
    }

    // --- Spike Detection ---
    if (std::isfinite(lastRawPower))
    {
        double rawDiff = std::abs(rawPower - lastRawPower);
        if (rawDiff > filterSpikeThresholdDb)
        {
            isSpike = true;
        }
    }
    else
    {
        // If lastRawPower wasn't finite, we can't reliably detect a spike yet.
        isSpike = false;
    }

    // --- Update EMAs ---
    if (!isSpike)
    {
        shortTermEMA = filterAlphaShort * rawPower + (1.0f - filterAlphaShort) * shortTermEMA;

        // Update Long-Term EMA only if stable or explicitly setting reference
        // Prevents reference erosion during misalignment/searching.
        if (currentState == AlgoState::MONITORING || currentState == AlgoState::REFERENCE)
        {
            longTermEMA = filterAlphaLong * rawPower + (1.0f - filterAlphaLong) * longTermEMA;
        }
        lastRawPower = rawPower; // Store this valid power reading
        return true;             // Valid reading processed
    }
    else
    {
        // if (debug)
        //     std::cout << "[" << algorithmName << "] Spike detected! Raw: " << rawPower
        //               << ", LastRaw: " << lastRawPower
        //               << ", Diff: " << std::abs(rawPower - lastRawPower) << std::endl;
        return false; // Indicate spike detected
    }
}

// --- Main Process Slot Logic ---
void HierarchicalSearch::processSlot(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot)
{
    // --- Initial Checks ---
    if (!engine)
    {
        if (currentState != AlgoState::ERROR_STATUS)
        {
            std::cerr << "[" << algorithmName << "] Error: Engine pointer null." << std::endl;
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        }
        return;
    }
    if (!configured)
    {
        if (currentState != AlgoState::ERROR_STATUS)
        {
            std::cerr << "[" << algorithmName << "] Error: Not configured." << std::endl;
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        }
        return;
    }

    // --- Get Engine Data ---
    double rawPower = -9999999;
    size_t currentRxSampleSlot = std::numeric_limits<size_t>::max();
    bool rotaryEnabled = false;
    rotaryObject currentRotaryState;
    bool isCurrentlyMoving = false;
    bool rxSourceEnabled = false;

    try
    {
        const auto &unitConf = engine->getEngineUnit(unitIndex);
        rotaryEnabled = unitConf.Rotary_enabled;

        // Prefer RxChain if enabled
        if (unitConf.RxChain_enabled)
        { // Assuming RxAntenna is implicitly required if RxChain is used
            const auto &rxChainData = engine->getRxChainData(unitIndex, currentSlot);
            rawPower = wattsToDbm(rxChainData.power_watts);
            currentRxSampleSlot = rxChainData.sample_slot;
            rxSourceEnabled = true;
        }
        else
        {
            if (currentState != AlgoState::ERROR_STATUS)
            {
                std::cerr << "[" << algorithmName << "] Error: RxChain not enabled for unit "
                          << unitIndex << std::endl;
            }
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            // Update engine state before returning
            try
            {
                engine->setAlgorithmData(unitIndex, currentSlot, getStatus());
            }
            catch (...)
            {
            }
            return;
        }

        if (rotaryEnabled)
        {
            currentRotaryState = engine->getRotaryData(unitIndex, currentSlot);
            isCurrentlyMoving = ((currentRotaryState.isMoving == 1) ? true : false);
        }
    }
    catch (const std::out_of_range &oor)
    {
        if (currentState != AlgoState::ERROR_STATUS)
        {
            std::cerr << "[" << algorithmName
                      << "] Error accessing engine unit data: " << oor.what() << std::endl;
        }
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        // Update engine state before returning
        try
        {
            engine->setAlgorithmData(unitIndex, currentSlot, getStatus());
        }
        catch (...)
        {
        }
        return;
    }
    catch (const std::exception &e)
    {
        if (currentState != AlgoState::ERROR_STATUS)
        {
            std::cerr << "[" << algorithmName << "] Error getting engine data slot " << currentSlot
                      << ": " << e.what() << std::endl;
        }
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        // Update engine state before returning
        try
        {
            engine->setAlgorithmData(unitIndex, currentSlot, getStatus());
        }
        catch (...)
        {
        }
        return;
    }

    // --- Check if the received Rx data is NEW ---
    bool isNewRxData = (currentRxSampleSlot != lastProcessedRxSampleSlot) && (currentRxSampleSlot != std::numeric_limits<size_t>::max()); // Check against invalid init

    // --- Update Internal Filter ---
    bool readingIsValid = false; // Will be set true if data is new AND filter accepts it
    if (isNewRxData && rxSourceEnabled)
    {                                                    // Ensure we have a valid source before filtering
        lastProcessedRxSampleSlot = currentRxSampleSlot; // Update tracker
        readingIsValid = updatePowerFilter(rawPower);
    }
    else
    {
        readingIsValid = false; // Stale data or no source
    }

    // --- State Machine Logic ---
    switch (currentState)
    {
    case AlgoState::REFERENCE:
    {
        // Increment counter only on valid new readings processed by the filter
        if (readingIsValid && filterInitialized)
        { // Checks if data was new AND filter accepted it
            warmupCounter++;
        }

        // Check for completion: based on counter and filter state
        if (warmupCounter >= referenceWarmupSamples && filterInitialized)
        {
            currentReferencePower = shortTermEMA; // Use the more responsive EMA after warmup
            longTermEMA = shortTermEMA;           // Initialize long-term reference to this value

            if (debug)
                std::cout << "[" << algorithmName << "] Reference Set (" << currentSlot
                          << ", from sample " << lastProcessedRxSampleSlot << "): " << shortTermEMA
                          << " dBm" << std::endl;

            lastSuccessfulAlignmentSlot = currentSlot; // Mark success time
            setState(AlgoState::MONITORING, AlgoAction::NONE);
        }
        else if (warmupCounter >= referenceWarmupSamples && !filterInitialized)
        {
            // This case implies we had enough slots pass but never got a valid reading.
            if (currentState != AlgoState::ERROR_STATUS)
            {
                std::cerr << "[" << algorithmName
                          << "] Error: Reference setting failed - no valid power readings received "
                             "during warmup period."
                          << std::endl;
            }
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        }
        // Else: continue waiting for enough valid new readings
    }
    break; // End REFERENCE_SETTING

    case AlgoState::MONITORING:
    {
        if (!filterInitialized || !std::isfinite(longTermEMA))
        {
            // Need a valid filter and reference to monitor
            // If this happens, likely due to bad signal after reference setting. Try again.
            if (currentState != AlgoState::REFERENCE)
            { // Prevent repeated logs if stuck
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Warning: Monitoring skipped - filter/reference invalid. "
                          << std::endl;
            }
            setState(AlgoState::REFERENCE, AlgoAction::NONE); // Go back to reference setting
            break;
        }

        // Misalignment Check using Absolute dB Drop from Long-Term EMA
        bool misaligned = (shortTermEMA < (currentReferencePower - misalignThresholdAbsoluteDb));

        if (misaligned)
        {
            if (debug)
                std::cout << "[" << algorithmName << "] Misalignment Detected (" << currentSlot
                          << "). ShortEMA: " << shortTermEMA
                          << " < (currentReferencePower Ref: " << currentReferencePower << " - "
                          << misalignThresholdAbsoluteDb << " dB)" << std::endl;

            if (rotaryEnabled)
            {
                // Capture current position as the center for the spiral search
                spiralCenterAz = currentRotaryState.azimuth.angle;   // Use correct field name
                spiralCenterAlt = currentRotaryState.altitude.angle; // Use correct field name
                startAlignment(engine, unitIndex, currentSlot);      // Start the spiral search
            }
            else
            {
                if (currentState != AlgoState::ERROR_STATUS)
                {
                    std::cerr << "[" << algorithmName
                              << "] Error: Misaligned but Rotary not enabled. Cannot realign."
                              << std::endl;
                }
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION); // Cannot align
            }
        }
        else
        {
            // Stay in monitoring, update last success slot as things are okay
            lastSuccessfulAlignmentSlot = currentSlot;
            setState(AlgoState::MONITORING, AlgoAction::NONE); // Ensure action is NONE
        }
    }
    break; // End MONITORING
    case AlgoState::ALIGNMENT:
    {
        // Goal: Execute spiral search until power is restored or points exhausted.

        // --- Pre-checks for Alignment State ---
        if (!filterInitialized || !std::isfinite(currentReferencePower))
        {
            if (currentState != AlgoState::ERROR_STATUS)
            {
                std::cerr << "[" << algorithmName << "] (" << currentSlot
                          << ") Error: Cannot align without valid filter/reference power."
                          << std::endl;
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            }
            break;
        }
        if (!rotaryEnabled || spiralOffsets.empty())
        {
            if (currentState != AlgoState::ERROR_STATUS)
            {
                std::cerr << "[" << algorithmName << "] (" << currentSlot
                          << ") Error: In ALIGNMENT state but Rotary disabled or no spiral offsets."
                          << std::endl;
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            }
            break;
        }
        if (currentSpiralPointIndex < 0)
        {
            // Should have been set by startAlignment, indicates an issue.
            if (currentState != AlgoState::ERROR_STATUS)
            {
                std::cerr << "[" << algorithmName << "] (" << currentSlot
                          << ") Error: In ALIGNMENT state but spiral index invalid ("
                          << currentSpiralPointIndex << ")." << std::endl;
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
            }
            break;
        }

        // --- Update Action based on Movement Status ---
        if (isCurrentlyMoving)
        {
            if (currentAction == AlgoAction::STARTING || currentAction == AlgoAction::MOVING)
            {
                setState(AlgoState::ALIGNMENT, AlgoAction::MOVING);
            } // else if currentAction is STOPPING, keep it as STOPPING
        }
        else
        { // Not moving
            if (currentAction == AlgoAction::MOVING || currentAction == AlgoAction::STARTING)
            {
                // Movement just finished (either arrived or initial start didn't move)
                setState(AlgoState::ALIGNMENT,
                         AlgoAction::NONE); // Reached point, action is now None until next decision
            } // else if currentAction is STOPPING, wait below
        }

        bool triggerHalt = false; // Flag to signal halt command needed

        // --- Continuous Power Evaluation & Peak Seeking ---
        if (readingIsValid && std::isfinite(shortTermEMA) && filterInitialized)
        {
            double currentSuccessMargin = calculateDynamicSuccessMargin(currentSlot);
            bool baseSuccessConditionMet = (shortTermEMA >= (currentReferencePower - currentSuccessMargin));

            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") ShortTermEMA: " << shortTermEMA
                          << ", Reference Power: " << currentReferencePower
                          << ", Success Margin: " << currentSuccessMargin
                          << ", Base Success Condition Met: " << baseSuccessConditionMet
                          << std::endl;

            // 1. Check if entering Peak Seeking mode
            if (baseSuccessConditionMet && !m_isPeakSeeking)
            {
                if (debug)
                    std::cout << "[" << algorithmName << "] (" << currentSlot
                              << ") Alignment threshold met. Entering Peak Seeking. Power: "
                              << std::fixed << std::setprecision(6) << shortTermEMA
                              << " >= Threshold " << (currentReferencePower - currentSuccessMargin)
                              << " at index " << currentSpiralPointIndex << "." << std::endl;
                m_isPeakSeeking = true;
                m_peakSeekingBestPower = shortTermEMA;            // Initial best power
                m_peakSeekingBestIndex = currentSpiralPointIndex; // Store index where threshold first met
            }

            // 2. If already Peak Seeking, evaluate continuously
            if (m_isPeakSeeking)
            {
                // Update best power found so far
                if (shortTermEMA > m_peakSeekingBestPower)
                {
                    if (debug && (shortTermEMA - m_peakSeekingBestPower > 0.05)) // Log only significant increases
                        std::cout << "[" << algorithmName << "] (" << currentSlot
                                  << ") Peak Seeking: Power increased to " << std::fixed
                                  << std::setprecision(6) << shortTermEMA
                                  << " (Best: " << m_peakSeekingBestPower << ")" << std::endl;
                    m_peakSeekingBestPower = shortTermEMA;
                    m_peakSeekingBestIndex = currentSpiralPointIndex; // Update index associated with peak
                }
                // Check for power decrease (HALT condition)
                else if (shortTermEMA < (m_peakSeekingBestPower - peakSeekingDecreaseThresholdDb))
                {
                    if (debug)
                        std::cout << "[" << algorithmName << "] (" << currentSlot
                                  << ") Peak Seeking: Power decreased! Current: " << std::fixed
                                  << std::setprecision(6) << shortTermEMA
                                  << ", Best: " << m_peakSeekingBestPower << ". Triggering HALT."
                                  << std::endl;
                    triggerHalt = true; // Signal halt needed
                    // Don't issue halt command yet, do it after state checks below
                }
                // else: Power is slightly below peak but within threshold, or same - continue seeking/moving
            }
            // else (!m_isPeakSeeking): Threshold not met yet, continue moving/searching
        }
        else
        {
            // No valid reading this slot, cannot make power-based decisions.
            // If moving, let it continue. If stopped, wait.
        }

        // --- State Transitions & Action Decisions ---

        // A. Handle Halt Completion
        if (!isCurrentlyMoving && currentAction == AlgoAction::STOPPING)
        {
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Halt completed. Final Best Power: " << std::fixed
                          << std::setprecision(6) << m_peakSeekingBestPower
                          << " (recorded near index " << m_peakSeekingBestIndex
                          << "). Transitioning to REFERENCE." << std::endl;

            setState(AlgoState::REFERENCE, AlgoAction::NONE);
            filterInitialized = false; // Re-establish reference at the peak
            warmupCounter = 0;
            m_isPeakSeeking = false; // Reset flag
            // Break here as we are transitioning state
            break;
        }

        // B. Handle Issuing Halt Command (if triggered and not already stopping)
        if (triggerHalt && currentAction != AlgoAction::STOPPING)
        {
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Issuing HALT command due to power decrease or end of spiral."
                          << std::endl;
            try
            {
                engine->issueRotaryCommand(unitIndex, "halt 0");
                engine->issueRotaryCommand(unitIndex, "halt 1");
                setState(currentState, AlgoAction::STOPPING); // Keep state, set action to STOPPING
                // Don't proceed to move command logic this cycle
                break; // Exit switch case for this slot after issuing halt
            }
            catch (const std::exception &e)
            {
                if (currentState != AlgoState::ERROR_STATUS)
                {
                    std::cerr << "[" << algorithmName
                              << "] Error issuing halt command: " << e.what() << std::endl;
                    setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                    m_isPeakSeeking = false;
                }
                break; // Exit on error
            }
        }

        // C. Handle Moving to Next Point (Only if stopped at target and NOT halting)
        if (!isCurrentlyMoving && currentAction == AlgoAction::NONE && !triggerHalt)
        {
            // Arrived at the intended spiral point 'currentSpiralPointIndex' (or started here)
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Arrived at spiral point " << currentSpiralPointIndex
                          << ". Power: " << std::fixed << std::setprecision(6) << shortTermEMA
                          << ". Evaluating next step." << std::endl;

            // Increment index for the *next* potential move
            int nextSpiralPointIndex = currentSpiralPointIndex + 1;

            // Check if spiral search is exhausted
            if (nextSpiralPointIndex >= spiralOffsets.size() || nextSpiralPointIndex >= maxSpiralPoints)
            {
                if (m_isPeakSeeking)
                {
                    // Reached end of spiral while peak seeking. Halt at the best point found.
                    if (debug)
                        std::cout << "[" << algorithmName << "] (" << currentSlot
                                  << ") Reached end of spiral while peak seeking. Triggering HALT "
                                     "at best point found."
                                  << std::endl;
                    // Set triggerHalt = true; It will be handled in the next cycle's "Issue Halt" block, or the one above if logic allows immediate check.
                    // Let's trigger it directly here to ensure halt is commanded now if possible.
                    triggerHalt = true;
                    // Re-check halt issuing immediately (optional but potentially faster)
                    if (triggerHalt && currentAction != AlgoAction::STOPPING)
                    {
                        goto issue_halt_now; // Use goto carefully to jump to halt logic
                    }
                }
                else
                {
                    // Reached end, never met success condition -> FAILURE
                    if (currentState != AlgoState::ERROR_STATUS)
                    {
                        std::cerr << "[" << algorithmName << "] (" << currentSlot
                                  << ") Error: Alignment FAILED - Spiral exhausted before success "
                                     "threshold met. Checked "
                                  << nextSpiralPointIndex // Show how many were attempted
                                  << " points (max allowed: " << maxSpiralPoints
                                  << ", available offsets: " << spiralOffsets.size() << ")."
                                  << std::endl;
                        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                        m_isPeakSeeking = false; // Reset flag on failure
                    }
                    break; // Exit switch case for this slot (Failure)
                }
            }
            else
            {
                // Spiral not exhausted, move to the next point
                currentSpiralPointIndex = nextSpiralPointIndex; // Update the target index
                if (debug)
                    std::cout << "[" << algorithmName << "] (" << currentSlot
                              << ") Moving to next spiral point index: " << currentSpiralPointIndex
                              << " / " << spiralOffsets.size() - 1 << std::endl;
                issueSpiralMoveCommand(engine,
                                       unitIndex,
                                       currentSlot); // Issues command, sets action to STARTING
                // Break here as we've issued a command
                break;
            }
        }
        // D. If moving and not halting, just continue (action updated at the top)
        else if (isCurrentlyMoving && currentAction == AlgoAction::MOVING && !triggerHalt)
        {
            // Actively moving towards 'currentSpiralPointIndex', power checks happened above.
            // No new command needed, just wait.
        }
        // E. Handle unexpected stops
        else if (!isCurrentlyMoving && (currentAction == AlgoAction::MOVING || currentAction == AlgoAction::STARTING) && !triggerHalt)
        {
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Warning: Stopped unexpectedly mid-move? Action="
                          << static_cast<int>(currentAction) << ". Will re-evaluate next cycle."
                          << std::endl;
            // Might have been a very short move, or an issue. Reset action to NONE so C applies next cycle.
            setState(AlgoState::ALIGNMENT, AlgoAction::NONE);
        }

        // Label for goto jump (use sparingly)
    issue_halt_now:;
        if (triggerHalt && currentAction != AlgoAction::STOPPING)
        {
            if (debug)
                std::cout << "[" << algorithmName << "] (" << currentSlot
                          << ") Issuing HALT command (goto path)." << std::endl;
            try
            {
                engine->issueRotaryCommand(unitIndex, "halt 0");
                engine->issueRotaryCommand(unitIndex, "halt 1");
                setState(currentState, AlgoAction::STOPPING);
            }
            catch (const std::exception &e)
            { /* handle error */
                setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
                m_isPeakSeeking = false;
            }
        }
    }
    break; // End ALIGNMENT

    case AlgoState::ERROR_STATUS:
        m_isPeakSeeking = false;
        // Goal: Stay in error state. Optionally try to halt movement.
        if (rotaryEnabled && isCurrentlyMoving && currentAction != AlgoAction::STOPPING && currentAction != AlgoAction::ERROR_ACTION)
        {
            engine->issueRotaryCommand(unitIndex, "halt 0");
            engine->issueRotaryCommand(unitIndex, "halt 1");
            setState(AlgoState::ERROR_STATUS, AlgoAction::STOPPING); // Keep state, set action
        }
        else if (!isCurrentlyMoving && currentAction == AlgoAction::STOPPING)
        {
            setState(AlgoState::ERROR_STATUS,
                     AlgoAction::ERROR_ACTION); // Halt finished, mark action as error action
        }
        else
        {
            setState(AlgoState::ERROR_STATUS,
                     AlgoAction::ERROR_ACTION); // Ensure state/action are correct
        }
        break; // End ERROR_STATUS

    default:
        if (currentState != AlgoState::ERROR_STATUS)
        {
            m_isPeakSeeking = false;
            setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        }
        break;
    } // End switch

    // --- Update Engine Algorithm State ---
    try
    {
        engine->setAlgorithmData(unitIndex, currentSlot, getStatus());
    }
    catch (const std::exception &e)
    {
        // Only log error if not already in error state to avoid spamming logs
        if (currentState != AlgoState::ERROR_STATUS)
        {
            std::cerr << "[" << algorithmName << "] Error setting algorithm data in engine (Slot "
                      << currentSlot << "): " << e.what() << std::endl;
        }
    }
}

// --- Helper Functions ---

// Start realignment process - Reset index and move to first point using offsets
void HierarchicalSearch::startAlignment(mobileTHzEngine *engine, size_t unitIndex, size_t currentSlot)
{
    if (!engine || spiralOffsets.empty())
    {
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        return;
    }

    // Reset spiral index to start from the beginning
    currentSpiralPointIndex = 0;
    m_isPeakSeeking = false;           // Reset peak seeking flag
    m_peakSeekingBestPower = -9999999; // Reset best power
    m_peakSeekingBestIndex = -1;       // Reset best index

    // Transition state and issue the first move command
    setState(AlgoState::ALIGNMENT, AlgoAction::NONE);       // Enter Alignment state first
    issueSpiralMoveCommand(engine, unitIndex, currentSlot); // This will set action to STARTING
}

// Generate the spiral OFFSETS relative to (0,0) - Called during configure()
// Generate the ROTATED square spiral OFFSETS relative to (0,0)
// Moves diagonally (Up-Right, Up-Left, Down-Left, Down-Right)
void HierarchicalSearch::generateSpiralOffsets() // Name kept, but stores combined steps now
{
    spiralOffsets.clear();

    if (maxSpiralPoints <= 0)
    {
        std::cout << "[" << algorithmName << "] Max spiral points <= 0, skipping step generation."
                  << std::endl;
        return;
    }

    // --- Parameters ---
    double initialDiagonalStep = halfPowerBeamWidth / 2.0f;
    // Parameter validation (ensure positive initial step, max step >= initial step)
    if (initialDiagonalStep <= 0.001f)
    {
        initialDiagonalStep = 0.1f; /* Log warning */
    }

    double currentStep = initialDiagonalStep; // Diagonal distance PER UNIT STEP for the current ring
    int segmentsPerSide = 1;                  // Number of UNIT steps per side
    int totalUnitStepsGenerated = 0;          // Track total unit steps represented
    const double invSqrt2 = static_cast<double>(M_SQRT1_2);

    if (debug)
        std::cout << "[" << algorithmName
                  << "] Generating COMBINED relative steps for 45-deg spiral. Initial unit step: "
                  << currentStep << ", Growth: " << spiralGrowthFactor << std::endl;

    // Loop until max total unit steps reached, or step size grows too large/small
    while (totalUnitStepsGenerated < maxSpiralPoints && currentStep > 0.0001f)
    {
        double stepComponent = currentStep * invSqrt2; // Az/Alt component per UNIT step

        // --- Combine steps for each side ---
        // Check remaining capacity before calculating each side's combined step

        // 1. Side: Up-Right (+Az, +Alt)
        int unitsThisSide1 = std::min(segmentsPerSide, maxSpiralPoints - totalUnitStepsGenerated);
        if (unitsThisSide1 > 0)
        {
            double totalDeltaAz = unitsThisSide1 * stepComponent;
            double totalDeltaAlt = unitsThisSide1 * stepComponent;
            spiralOffsets.push_back({totalDeltaAz, totalDeltaAlt}); // Store combined step
            totalUnitStepsGenerated += unitsThisSide1;
            if (totalUnitStepsGenerated >= maxSpiralPoints)
                break;
        }
        else if (totalUnitStepsGenerated >= maxSpiralPoints)
        {
            break;
        } // Break if exactly max points reached before this side

        // 2. Side: Up-Left (-Az, +Alt)
        int unitsThisSide2 = std::min(segmentsPerSide, maxSpiralPoints - totalUnitStepsGenerated);
        if (unitsThisSide2 > 0)
        {
            double totalDeltaAz = unitsThisSide2 * -stepComponent;
            double totalDeltaAlt = unitsThisSide2 * stepComponent;
            spiralOffsets.push_back({totalDeltaAz, totalDeltaAlt});
            totalUnitStepsGenerated += unitsThisSide2;
            if (totalUnitStepsGenerated >= maxSpiralPoints)
                break;
        }
        else if (totalUnitStepsGenerated >= maxSpiralPoints)
        {
            break;
        }

        segmentsPerSide++; // Increase length for next pair of sides

        // 3. Side: Down-Left (-Az, -Alt)
        int unitsThisSide3 = std::min(segmentsPerSide, maxSpiralPoints - totalUnitStepsGenerated);
        if (unitsThisSide3 > 0)
        {
            double totalDeltaAz = unitsThisSide3 * -stepComponent;
            double totalDeltaAlt = unitsThisSide3 * -stepComponent;
            spiralOffsets.push_back({totalDeltaAz, totalDeltaAlt});
            totalUnitStepsGenerated += unitsThisSide3;
            if (totalUnitStepsGenerated >= maxSpiralPoints)
                break;
        }
        else if (totalUnitStepsGenerated >= maxSpiralPoints)
        {
            break;
        }

        // 4. Side: Down-Right (+Az, -Alt)
        int unitsThisSide4 = std::min(segmentsPerSide, maxSpiralPoints - totalUnitStepsGenerated);
        if (unitsThisSide4 > 0)
        {
            double totalDeltaAz = unitsThisSide4 * stepComponent;
            double totalDeltaAlt = unitsThisSide4 * -stepComponent;
            spiralOffsets.push_back({totalDeltaAz, totalDeltaAlt});
            totalUnitStepsGenerated += unitsThisSide4;
            if (totalUnitStepsGenerated >= maxSpiralPoints)
                break;
        }
        else if (totalUnitStepsGenerated >= maxSpiralPoints)
        {
            break;
        }

        segmentsPerSide++; // Increase length again

        // --- Update Unit Step Size for Next Ring ---
        if (spiralGrowthFactor >= 1.0f)
        {
            currentStep *= spiralGrowthFactor;
        }
        else if (totalUnitStepsGenerated > 0)
        {
            break; // Stop if step size doesn't grow
        }
        if (currentStep < 0.0001f)
        {
            std::cout << "[" << algorithmName
                      << "] Unit step size became extremely small. Stopping." << std::endl;
            break;
        }
    } // End while loop

    if (spiralOffsets.empty() && maxSpiralPoints > 0)
    {
        std::cerr << "[" << algorithmName
                  << "] Warning: Failed to generate any combined spiral steps. Check parameters."
                  << std::endl;
    }
    else
    {
        if (debug)
            std::cout << "[" << algorithmName << "] Generated " << spiralOffsets.size()
                      << " combined relative spiral steps (representing " << totalUnitStepsGenerated
                      << " unit locations)." << std::endl;
    }
}

// Issue the 'mdd' command to move to the target point
// calculated from the current offset and the spiral center
void HierarchicalSearch::issueSpiralMoveCommand(mobileTHzEngine *engine,
                                                size_t unitIndex,
                                                size_t currentSlot)
{
    if (!engine)
        return;

    if (currentSpiralPointIndex < 0 || currentSpiralPointIndex >= spiralOffsets.size())
    {
        if (currentState != AlgoState::ERROR_STATUS)
        {
            std::cerr << "[" << algorithmName
                      << "] Error: Cannot issue spiral move - invalid step index ("
                      << currentSpiralPointIndex << " of " << spiralOffsets.size() << ")."
                      << std::endl;
        }
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
        return;
    }

    // Get the RELATIVE step for the current point index
    const auto &relativeStep = spiralOffsets[currentSpiralPointIndex];
    double relativeAz = relativeStep.deltaAz;
    double relativeAlt = relativeStep.deltaAlt;

    // Format the command string for "mdd 0 <relativeAz> 1 <relativeAlt>"
    // Ensure sufficient precision for the command parameters
    std::string moveCommand = "mdd 0 " + std::to_string(relativeAz) + " 1 " + std::to_string(relativeAlt);

    try
    {
        engine->issueRotaryCommand(unitIndex, moveCommand);

        if (debug)
            std::cout << "[" << algorithmName << "] (" << currentSlot << ", Step "
                      << currentSpiralPointIndex << "/" << spiralOffsets.size() - 1
                      << "): Issued relative command: '" << moveCommand << "'" << std::endl;

        // Set state to indicate waiting for this specific move
        setState(AlgoState::ALIGNMENT, AlgoAction::STARTING);
    }
    catch (const std::exception &e)
    {
        if (currentState != AlgoState::ERROR_STATUS)
        {
            std::cerr << "[" << algorithmName << "] Error: Failed to issue rotary command '"
                      << moveCommand << "': " << e.what() << std::endl;
        }
        setState(AlgoState::ERROR_STATUS, AlgoAction::ERROR_ACTION);
    }
}

// Calculate Dynamic Success Margin
double HierarchicalSearch::calculateDynamicSuccessMargin(size_t currentSlot) const
{
    double margin = realignSuccessMaxMarginDb; // Default to max (easiest)

    if (lastSuccessfulAlignmentSlot > 0 && currentSlot > lastSuccessfulAlignmentSlot && slotTimeMicrosec > 0)
    {
        size_t slotsElapsed = currentSlot - lastSuccessfulAlignmentSlot;
        double timeElapsedSeconds = static_cast<double>(slotsElapsed) * slotTimeMicrosec / 1'000'000.0;
        margin = realignSuccessInitialMarginDb + (static_cast<double>(timeElapsedSeconds) * realignSuccessMarginIncreaseRateDbPerSec);
        // Clamp the margin
        margin = std::max(realignSuccessInitialMarginDb,
                          std::min(realignSuccessMaxMarginDb, margin));
    }
    else if (lastSuccessfulAlignmentSlot == 0 || currentState == AlgoState::ALIGNMENT)
    {
        // If never aligned, or currently aligning, start with the tightest margin
        margin = realignSuccessInitialMarginDb;
    }

    // Ensure margin is not negative (can happen with very small configured values)
    if (margin < 0)
    {
        margin = 0.0;
    }

    return margin; // Return the calculated margin
}
