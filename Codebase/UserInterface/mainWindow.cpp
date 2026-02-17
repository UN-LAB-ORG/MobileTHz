// mainWindow.cpp
#include "mainWindow.h"

// Qt Includes
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDebug>
#include <QFuture>
#include <QMessageBox>
#include <QStackedWidget>
#include <QTimer>
#include <QtConcurrent>

// Project Includes
#include "Simulation/imuSim/imuSim.h"
#include "Simulation/positionSim/positionSim.h"
#include "Simulation/rotarySim/rotarySim.h"
#include "Software/kpiClassifier/kpiClassifier.h"
#include "Software/mobileTHzEngine/mobileTHzEngine.h"
#include "Software/structDefinition.h"
#include "Utils/darkPalette.h"
#include "algorithmSelectionPage.h"
#include "antennaConfigPage.h"
#include "configSelectionPage.h"
#include "initializationPage.h"
#include "positionConfigPage.h"
#include "realtimePage.h"
#include "simulationPage.h"

// Standard Includes
#include <algorithm>
#include <iostream>
#include <random>
#include <string>
#include <tuple>
#include <utility>

static bool debug = false;

namespace
{
    // Each thread gets its own reusable set of simulation objects.
    thread_local std::unique_ptr<SimulationObjects> t_simulation_objects;
    // We also store the config that was used to create the objects, to check compatibility.
    thread_local ConfigFile t_current_objects_config;
} // namespace

MainWindow::MainWindow(const QString &initialConfigPath, const bool consoleMode, QWidget *parent)
    : QMainWindow(parent), initialConfigPath(initialConfigPath), stackedWidget(nullptr), configSelectionPage(nullptr), antennaConfigPage(nullptr), positionConfigPage(nullptr), algorithmSelectionPage(nullptr), initializationPage(nullptr), realtimePage(nullptr), simulationPage(nullptr)
{
    setWindowTitle("Northeastern University - UNLab - MobileTHz");
    resize(1200, 1000);
    setupUI();                // Create UI elements first
    setupConnections();       // Connect signals/slots
    DarkPalette::apply(this); // Apply theme

    if (!this->initialConfigPath.isEmpty())
    {
        // A config path was provided. Load it to decide the execution path.
        qInfo() << "Initial config path provided:" << this->initialConfigPath;

        auto [loadedConfig, parsedJson, errorMsg] = ConfigSelectionPage::loadAndValidateConfigFile(
            this->initialConfigPath);

        // Check the consoleMode flag.
        if (consoleMode)
        {
            loadedConfig.display_mode = "CONSOLE";
        }
        else
        {
            loadedConfig.display_mode = "GUI";
        }

        if (!errorMsg.isEmpty())
        {
            // Loading/validation failed. Since this is a GUI launch, show an error
            // and fall back to manual selection.
            QMessageBox::critical(this,
                                  "Configuration Error",
                                  QString(
                                      "Failed to load initial configuration file:\n%1\n\nPlease "
                                      "select a valid file manually.")
                                      .arg(errorMsg));
            stackedWidget->setCurrentWidget(configSelectionPage); // Fallback
        }
        else
        {
            // Config loaded successfully. Now check the display mode.
            this->paramConfig = loadedConfig; // Store the loaded config

            if (this->paramConfig.display_mode == "CONSOLE")
            {
                qInfo()
                    << "Config specifies CONSOLE mode. Hiding GUI and starting combination runs.";

                // Hide the window so it feels like a console application.
                this->hide();

                // Schedule the console mode execution to start after the event loop begins.
                // This prevents blocking the constructor. The console run will manage application exit.
                QTimer::singleShot(0, this, [this]()
                                   {
                    int exitCode = runConsoleMode();
                    QCoreApplication::exit(exitCode); });
            }
            else
            {
                // This logic is mirrored from onConfigSelected()
                this->positionConfig = nlohmann::json(); // Reset position data
                this->isRunningCombinations = false;
                this->completedCombinations = 0;
                this->totalCombinations = 0;
                this->successfulCombinations = 0;

                // Re-calculate preset counts, which is crucial for the page skipping logic.
                if (!paramConfig.engine_units.empty())
                {
                    const auto &firstUnit = paramConfig.engine_units[0];
                    numAntennaPresets = firstUnit.preset_antenna_enabled
                                            ? firstUnit.preset_antenna_files.size()
                                            : 0;
                    size_t count = 0;
                    for (const auto &innerVec : *paramConfig.preset_position_sets)
                    {
                        count += innerVec.size();
                    }
                    numPositionPresets = paramConfig.preset_position_enabled ? count : 0;
                    numAlgorithmPresets = firstUnit.preset_algorithm_enabled
                                              ? firstUnit.preset_algorithm_names.size()
                                              : 0;
                }
                else
                {
                    numAntennaPresets = numPositionPresets = numAlgorithmPresets = 0;
                }

                // Start the GUI sequence. This will automatically skip pages if presets are configured.
                startConfigurationFlow();
            }
        }
    }
    else
    {
        // No initial path provided, show the config selection page as normal.
        stackedWidget->setCurrentWidget(configSelectionPage);
    }
}

MainWindow::~MainWindow() {}

// --- UI Setup ---

void MainWindow::setupUI()
{
    stackedWidget = new QStackedWidget(this);
    setCentralWidget(stackedWidget);

    // Instantiate pages
    configSelectionPage = new ConfigSelectionPage(this);
    configSelectionPage->setObjectName("ConfigSelectionPage");

    antennaConfigPage = new AntennaConfigPage(this);
    antennaConfigPage->setObjectName("AntennaConfigPage");

    positionConfigPage = new PositionConfigPage(this);
    positionConfigPage->setObjectName("PositionConfigPage");

    algorithmSelectionPage = new AlgorithmSelectionPage(this);
    algorithmSelectionPage->setObjectName("AlgorithmSelectionPage");

    initializationPage = new InitializationPage(this);
    initializationPage->setObjectName("InitializationPage");

    realtimePage = new RealtimePage(this);
    realtimePage->setObjectName("RealtimePage");

    simulationPage = new SimulationPage(this);
    simulationPage->setObjectName("SimulationPage");

    // Add pages to widget in logical order (though transitions are explicit)
    stackedWidget->addWidget(configSelectionPage);    // Index 0
    stackedWidget->addWidget(antennaConfigPage);      // Index 1
    stackedWidget->addWidget(positionConfigPage);     // Index 2
    stackedWidget->addWidget(algorithmSelectionPage); // Index 3
    stackedWidget->addWidget(initializationPage);     // Index 4
    stackedWidget->addWidget(realtimePage);           // Index 5
    stackedWidget->addWidget(simulationPage);         // Index 6

    // Initial widget is set in the constructor based on mode
}

void MainWindow::setupConnections()
{
    // --- Central Flow Connections ---
    connect(configSelectionPage,
            &ConfigSelectionPage::configSelected,
            this,
            &MainWindow::onConfigSelected);
    connect(antennaConfigPage, &AntennaConfigPage::nextRequested, this, &MainWindow::onAntennaNext);
    connect(positionConfigPage,
            &PositionConfigPage::nextRequested,
            this,
            &MainWindow::onPositionNext);
    connect(algorithmSelectionPage,
            &AlgorithmSelectionPage::algorithmsAssigned,
            this,
            &MainWindow::onAlgorithmNext);
    connect(initializationPage,
            &InitializationPage::initializationFinished,
            this,
            &MainWindow::onInitializationFinished);
    connect(simulationPage,
            &SimulationPage::simulationComplete,
            this,
            &MainWindow::onSimulationComplete);
    // Connect Realtime page completion signal if needed

    // --- Back Button Connections ---
    // Connect all relevant back buttons to the generic goBack slot
    connect(antennaConfigPage, &AntennaConfigPage::backRequested, this, &MainWindow::goBack);
    connect(positionConfigPage, &PositionConfigPage::backRequested, this, &MainWindow::goBack);
    connect(algorithmSelectionPage,
            &AlgorithmSelectionPage::backRequested,
            this,
            &MainWindow::goBack);
    connect(initializationPage,
            &InitializationPage::initializationCancelled,
            this,
            &MainWindow::goBack); // Treat cancel as 'back'

    // RealtimePage --> Overview Page
    connect(realtimePage,
            &RealtimePage::viewSimulationRequested,
            this,
            &MainWindow::onViewSimulationRequested);
}

void MainWindow::onViewSimulationRequested()
{
    // The sharedPointers object was created in the initialization page and holds
    // the engine instance that has been collecting data.
    if (!initializationPage || !simulationPage)
    {
        qCritical("Cannot switch to results view: page pointers are null.");
        QMessageBox::critical(this, "Error", "Internal error: A required page is not available.");
        return;
    }

    // 1. Get the shared pointers containing the engine with the collected data.
    auto sharedData = initializationPage->getSharedPointers();
    if (!sharedData || !sharedData->engine)
    {
        QMessageBox::critical(this, "Error", "No engine data available to display.");
        return;
    }

    // Stop the realtime page from updating in the background
    realtimePage->stopUpdating();

    // 2. Pass the data to the simulation page.
    simulationPage->setSharedPointers(sharedData);

    // 3. Configure the simulation page with the current experiment's config.
    // 'this->paramConfig' holds the configuration for the current run.
    simulationPage->setupPage(this->paramConfig);

    // 4. Switch the view.
    stackedWidget->setCurrentWidget(simulationPage);
}
// --- Slots Implementation ---

void MainWindow::onConfigSelected(const ConfigFile &config)
{
    // Reset state for a new GUI run
    this->paramConfig = config;
    this->positionConfig = nlohmann::json(); // Reset position data
    this->isRunningCombinations = false;
    this->completedCombinations = 0;
    this->totalCombinations = 0;
    this->successfulCombinations = 0;

    // Re-calculate preset counts for GUI mode (mainly for skipping logic)
    if (!paramConfig.engine_units.empty())
    {
        const auto &firstUnit = paramConfig.engine_units[0];
        numAntennaPresets = firstUnit.preset_antenna_enabled ? firstUnit.preset_antenna_files.size()
                                                             : 0;

        size_t count = 0;
        // Iterate through inner and outer vectors to count total strings
        for (const auto &innerVec : *paramConfig.preset_position_sets)
        {
            count += innerVec.size();
        }

        numPositionPresets = paramConfig.preset_position_enabled
                                 ? count // Total strings across all inner vectors
                                 : 0;

        numAlgorithmPresets = firstUnit.preset_algorithm_enabled
                                  ? firstUnit.preset_algorithm_names.size()
                                  : 0;
    }
    else
    {
        numAntennaPresets = numPositionPresets = numAlgorithmPresets = 0;
    }

    startConfigurationFlow(); // Start the GUI sequence
}

void MainWindow::onAntennaNext(const ConfigFile &updatedConfig)
{
    this->paramConfig = updatedConfig; // Update config from page data
    requestTransition(
        antennaConfigPage); // Request transition, indicating departure from Antenna page
}

void MainWindow::onPositionNext(const nlohmann::json &positionConfig)
{
    this->positionConfig = positionConfig; // Store position data from page
    requestTransition(
        positionConfigPage); // Request transition, indicating departure from Position page
}

void MainWindow::onAlgorithmNext(const ConfigFile &updatedConfig)
{
    this->paramConfig = updatedConfig; // Update config from page data
    requestTransition(
        algorithmSelectionPage); // Request transition, indicating departure from Algorithm page
}

void MainWindow::onInitializationFinished(bool success, const QString &errorMsg)
{
    if (success)
    {
        if (paramConfig.engine_mode == "SIMULATION")
        {
            // --- SIMULATION Mode ---

            if (paramConfig.display_mode == "CONSOLE")
            {
                // Console mode: This signifies the end of a single combination's run
                onCombinationFinished(success); // Handles logging and advancing the loop
                if (!success)
                {
                    std::cerr << "ERROR during initialization for combination ["
                              << "Ant: " << currentAntennaIndex << ", "
                              << "Pos: " << currentPositionIndex << ", "
                              << "Alg: " << currentAlgorithmIndex << "]: " << errorMsg.toStdString()
                              << std::endl;
                }
            }
            else
            {
                // Get engine via shared pointers (with safety checks)
                if (!initializationPage)
                {
                    qCritical("InitializationPage pointer is unexpectedly null!");
                    QCoreApplication::exit(1);
                    return;
                }
                mobileTHzEngine *engine = initializationPage->getSharedPointers()->engine.get();
                if (!engine)
                {
                    qCritical("Engine pointer is unexpectedly null after initialization!");
                    QCoreApplication::exit(1);
                    return;
                }

                // GUI MODE: Proceed to show the SimulationPage for results display
                QWidget *nextPage = simulationPage;
                if (nextPage)
                {
                    auto *simPage = static_cast<SimulationPage *>(nextPage);
                    simPage->setSharedPointers(
                        initializationPage->getSharedPointers()); // Pass data for display
                    simPage->setupPage(paramConfig);              // Setup the results page

                    if (nextPage != stackedWidget->currentWidget())
                    {
                        stackedWidget->setCurrentWidget(nextPage);
                    }
                    else
                    {
                        qDebug("Simulation results page is already current.");
                    }
                }
                else
                {
                    qWarning("SimulationPage is null! Cannot display results.");
                    // Go back to config selection in GUI on error
                    stackedWidget->setCurrentWidget(configSelectionPage);
                }
            }
        }
        else
        { // REALTIME Mode
            if (paramConfig.display_mode == "CONSOLE")
            {
                // This case should have been caught earlier, but handle defensively.
                std::cerr
                    << "CRITICAL: REALTIME engine_mode is not supported in CONSOLE display_mode."
                    << std::endl;
                QCoreApplication::exit(1);
                return;
            }
            // --- GUI Realtime Handling ---
            QWidget *nextPage = realtimePage;
            if (nextPage)
            {
                // Setup Realtime page with the live pointers
                auto *realtimePagePtr = static_cast<RealtimePage *>(nextPage);
                realtimePagePtr->setupPage(initializationPage->getSharedPointers(),
                                           this->paramConfig);

                if (nextPage != stackedWidget->currentWidget())
                {
                    stackedWidget->setCurrentWidget(nextPage);
                }
            }
            else
            {
                qWarning("Realtime page is null, returning to config selection.");
                stackedWidget->setCurrentWidget(configSelectionPage); // Failsafe
            }
        }
    }
    else
    { // Initialization failed
        qWarning() << "Initialization failed:" << errorMsg;
        if (paramConfig.display_mode == "CONSOLE")
        {
            std::cerr << "CRITICAL: Initialization Error: " << errorMsg.toStdString() << std::endl;
            QCoreApplication::exit(1); // Exit console on init error
        }
        else
        {
            QMessageBox::critical(this, "Initialization Error", errorMsg);
            stackedWidget->setCurrentWidget(determinePreviousPage());
        }
    }
}

void MainWindow::onSimulationComplete()
{
    if (paramConfig.display_mode == "GUI")
    {
        QMessageBox::information(this, "Simulation Complete", "Simulation finished successfully.");
        stackedWidget->setCurrentWidget(configSelectionPage); // Back to start in GUI
    }
    else
    {
        qDebug() << "onSimulationComplete called in CONSOLE mode (potentially ignorable)";
    }
}

void MainWindow::goBack()
{
    // goBack is only relevant in GUI mode
    if (paramConfig.display_mode == "CONSOLE")
    {
        qWarning("goBack() called unexpectedly in CONSOLE mode.");
        return;
    }
    QWidget *prevPage = determinePreviousPage();
    if (prevPage && prevPage != stackedWidget->currentWidget())
    {
        stackedWidget->setCurrentWidget(prevPage);
    }
    else if (!prevPage)
    {
        qWarning("Could not determine previous page, going to ConfigSelectionPage.");
        stackedWidget->setCurrentWidget(configSelectionPage); // Failsafe
    }
}

// --- Core Flow Logic Implementation ---
void MainWindow::requestTransition(QWidget *departingPage /*= nullptr*/)
{
    if (paramConfig.display_mode == "CONSOLE" || isRunningCombinations)
    {
        qWarning("requestTransition called unexpectedly in CONSOLE mode or during runs.");
        return;
    }

    // If in experimental mode and the role is Observer, skip all config pages.
    if (paramConfig.engine_mode == "EXPERIMENTAL" && paramConfig.experimental.role == "OBSERVER")
    {
        qInfo() << "[Observer Flow] Auto-processing presets before initialization...";

        // We will modify a copy of the config and only commit it if all presets succeed.
        ConfigFile observerConfig = this->paramConfig;

        // 1. Process Antenna Preset (using index 0, as it's a non-interactive flow)
        if (!observerConfig.engine_units.empty() && observerConfig.engine_units[0].preset_antenna_enabled)
        {
            if (!attemptProcessAntennaPreset(0, observerConfig))
            {
                QMessageBox::critical(
                    this,
                    "Preset Error",
                    "Failed to process the first antenna preset for the observer. Aborting.");
                stackedWidget->setCurrentWidget(configSelectionPage); // Go back to start on error
                return;
            }
        }

        // 2. Process Algorithm Preset (using index 0)
        if (!observerConfig.engine_units.empty() && observerConfig.engine_units[0].preset_algorithm_enabled)
        {
            if (!attemptProcessAlgorithmPreset(0, observerConfig))
            {
                QMessageBox::critical(
                    this,
                    "Preset Error",
                    "Failed to process the first algorithm preset for the observer. Aborting.");
                stackedWidget->setCurrentWidget(configSelectionPage); // Go back to start on error
                return;
            }
        }

        // Presets were applied successfully. Commit the changes back to the main config object.
        this->paramConfig = observerConfig;

        // Now, proceed to initialization with the updated config.
        QWidget *targetPage = initializationPage;
        if (targetPage)
        {
            static_cast<InitializationPage *>(targetPage)
                ->setParamConfig(this->paramConfig, nlohmann::json::object());
            stackedWidget->setCurrentWidget(targetPage);
        }
        else
        {
            qCritical("InitializationPage is null! Cannot proceed.");
            QMessageBox::critical(this, "Internal Error", "InitializationPage is not available.");
            stackedWidget->setCurrentWidget(configSelectionPage); // Failsafe
        }
        return; // Exit the function early, as intended.
    }

    QWidget *targetPage = nullptr;
    bool presetFailed = false;
    bool continueProcessing = true;

    bool canSkipAnt, canSkipPos, canSkipAlg;
    checkSkippablePages(canSkipAnt, canSkipPos, canSkipAlg); // Checks vector emptiness now

    enum class Step
    {
        Antenna,
        Position,
        Algorithm,
        Initialization
    };
    Step currentStep = Step::Antenna;

    // Determine the next logical step based on the page we are leaving
    if (departingPage == antennaConfigPage)
        currentStep = Step::Position;
    else if (departingPage == positionConfigPage)
        currentStep = Step::Algorithm;
    else if (departingPage == algorithmSelectionPage)
        currentStep = Step::Initialization;
    // If departingPage is null (initial call), currentStep remains Step::Antenna

    while (continueProcessing && !presetFailed && !targetPage)
    {
        // Create temporary copies for potential preset application in GUI
        ConfigFile tempGuiConfig = this->paramConfig;
        nlohmann::json tempGuiPositionConfig = this->positionConfig; // Use current state

        switch (currentStep)
        {
        case Step::Antenna:
            if (!canSkipAnt)
            { // If Antenna page cannot be skipped, go there
                targetPage = antennaConfigPage;
                if (targetPage)
                    static_cast<AntennaConfigPage *>(targetPage)->setParamConfig(paramConfig);
                continueProcessing = false; // Found target page, stop processing steps
            }
            else
            {
                // GUI: Antenna page can be skipped, attempt to process the FIRST (index 0) antenna preset automatically
                if (!paramConfig.engine_units.empty() && !paramConfig.engine_units[0].preset_antenna_files.empty())
                {
                    if (debug)
                    {
                        qInfo() << "GUI: Auto-processing first antenna preset (Index 0)";
                    }

                    if (!attemptProcessAntennaPreset(0, tempGuiConfig))
                    {
                        presetFailed = true;
                        continueProcessing = false; // Error processing preset
                        // Error message shown by attemptProcess... GUI variant
                    }
                    else
                    {
                        this->paramConfig = tempGuiConfig; // Commit changes on success
                        currentStep = Step::Position;      // Move to check the next step (Position)
                    }
                }
                else
                {
                    // This should be caught by canSkipAnt, but added as defense
                    qWarning("Antenna preset enabled but vector empty or no units in GUI flow.");
                    presetFailed = true;
                    continueProcessing = false;
                }
            }
            break;

        case Step::Position:
            if (paramConfig.engine_mode != "SIMULATION")
            {                                  // Position page only relevant in SIMULATION
                currentStep = Step::Algorithm; // Skip to next step
            }
            else if (!canSkipPos)
            { // If Position page cannot be skipped, go there
                targetPage = positionConfigPage;
                if (targetPage)
                    static_cast<PositionConfigPage *>(targetPage)->setParamConfig(paramConfig);
                continueProcessing = false; // Found target page, stop processing steps
            }
            else
            {
                // GUI: Position page can be skipped, attempt to process the FIRST (index 0) position preset automatically
                if (!(*paramConfig.preset_position_sets)[0].empty())
                {
                    // Ensure the vector is not empty before accessing index 0
                    if (debug)
                    {
                        qInfo() << "GUI: Auto-processing first position preset (Index 0)";
                    }
                    if (!attemptProcessPositionPreset(0, 0, tempGuiConfig, tempGuiPositionConfig))
                    {
                        presetFailed = true;
                        continueProcessing = false; // Error processing preset
                    }
                    else
                    {
                        // Need to commit both config (potentially modified by antenna/position) and position data
                        this->paramConfig = tempGuiConfig;
                        this->positionConfig = tempGuiPositionConfig;
                        currentStep = Step::Algorithm; // Move to check the next step (Algorithm)
                    }
                }
                else
                {
                    // This should be caught by canSkipPos, but added as defense
                    qWarning("Position preset enabled but vector empty in GUI flow.");
                    presetFailed = true;
                    continueProcessing = false;
                }
            }
            break;

        case Step::Algorithm:
            if (!canSkipAlg)
            { // If Algorithm page cannot be skipped, go there
                targetPage = algorithmSelectionPage;
                if (targetPage)
                    static_cast<AlgorithmSelectionPage *>(targetPage)
                        ->setParamConfigFile(paramConfig);
                continueProcessing = false; // Found target page, stop processing steps
            }
            else
            {
                // GUI: Algorithm page can be skipped, attempt to process the FIRST (index 0) algorithm preset automatically
                if (!paramConfig.engine_units.empty() && !paramConfig.engine_units[0].preset_algorithm_names.empty())
                {
                    if (debug)
                    {
                        qInfo() << "GUI: Auto-processing first algorithm preset (Index 0)";
                    }
                    if (!attemptProcessAlgorithmPreset(0, tempGuiConfig))
                    {
                        presetFailed = true;
                        continueProcessing = false; // Error processing preset
                    }
                    else
                    {
                        this->paramConfig = tempGuiConfig;  // Commit changes
                        currentStep = Step::Initialization; // Move to check the next step (Initialization)
                    }
                }
                else
                {
                    qWarning("Algorithm preset enabled but vector empty or no units in GUI flow.");
                    presetFailed = true;
                    continueProcessing = false;
                }
            }
            break;

        case Step::Initialization:
            // Initialization is the final setup step before running/simulating
            targetPage = initializationPage;
            // Pass the potentially modified config (from GUI auto-presets) and position data
            if (targetPage)
                static_cast<InitializationPage *>(targetPage)
                    ->setParamConfig(this->paramConfig, this->positionConfig);
            continueProcessing = false; // Found target page (or final step), stop processing steps
            break;

        default:
            qCritical() << "Unknown step in transition logic!";
            presetFailed = true;
            continueProcessing = false; // Critical error, stop
            break;
        } // End switch
    } // End while

    // --- Handle Outcome ---
    if (presetFailed)
    {
        qCritical()
            << "Preset processing failed during GUI automated flow. Returning to Config Selection.";
        targetPage = configSelectionPage; // Go back to start on failure
    }

    if (!targetPage)
    {
        // This should ideally not happen if the logic covers all cases,
        // but could occur if e.g., Initialization page is null.
        qCritical()
            << "Transition logic failed to determine a target page! Resetting to Config Selection.";
        targetPage = configSelectionPage;
    }

    // --- Perform Page Transition ---
    if (targetPage && targetPage != stackedWidget->currentWidget())
    {
        if (debug)
        {
            qDebug() << "Transitioning to page:" << targetPage->objectName();
        }
        stackedWidget->setCurrentWidget(targetPage);
    }
    else if (!targetPage)
    {
        qCritical() << "Internal Error: Target page became null unexpectedly after processing.";
        stackedWidget->setCurrentWidget(configSelectionPage); // Failsafe
    }
    else
    {
        qDebug() << "Target page" << targetPage->objectName()
                 << "is already current or no transition needed.";
    }
}

// --- Preset Handling Helpers Implementation ---
void MainWindow::checkSkippablePages(bool &canSkipAnt, bool &canSkipPos, bool &canSkipAlg) const
{
    canSkipAnt = canSkipPos = canSkipAlg = false;
    if (paramConfig.engine_units.empty())
    {
        qWarning("checkSkippablePages: No engine units defined.");
        return;
    }
    const auto &firstUnit = paramConfig.engine_units[0];

    // Check if enabled AND the corresponding vector is NOT empty
    canSkipAnt = firstUnit.preset_antenna_enabled && !firstUnit.preset_antenna_files.empty();
    canSkipPos = paramConfig.preset_position_enabled && !paramConfig.preset_position_sets->empty() && (paramConfig.engine_mode == "SIMULATION");
    canSkipAlg = firstUnit.preset_algorithm_enabled && !firstUnit.preset_algorithm_names.empty();
}

bool MainWindow::attemptProcessAntennaPreset(size_t presetIndex, ConfigFile &runConfig)
{
    // This function assumes that the antenna preset file specified at 'presetIndex'
    // for the first engine unit defines the antenna configuration for ALL units
    // in this run. The static function AntennaConfigPage::processAntennaConfiguration
    // currently reflects this by taking one file path and modifying the whole ConfigFile.
    // If each unit requires independent antenna configuration from its own file list,
    // AntennaConfigPage::processAntennaConfiguration would need refactoring.

    if (runConfig.engine_units.empty())
    {
        qWarning("Attempting to process antenna preset with no engine units.");
        return false;
    }
    if (this->paramConfig.engine_units.empty())
    {
        qCritical("Base paramConfig has no engine units during antenna preset processing.");
        return false;
    }

    const auto &firstBaseUnit = this->paramConfig.engine_units[0];

    // This function should only be called if presets are enabled (verified by startCombinationRuns)
    if (!firstBaseUnit.preset_antenna_enabled)
    {
        qWarning("Attempting to process antenna preset index when presets are not enabled "
                 "(unexpected).");
        // If presets aren't enabled, we shouldn't modify the config based on an index.
        // Return true, indicating no error, but also no action taken based on index.
        return true;
    }

    if (presetIndex >= firstBaseUnit.preset_antenna_files.size())
    {
        QString errorMsg = QString(
                               "Antenna preset index %1 is out of bounds for Unit 0 (size: %2).")
                               .arg(presetIndex)
                               .arg(firstBaseUnit.preset_antenna_files.size());
        qCritical() << errorMsg;
        std::cerr << "ERROR: " << errorMsg.toStdString() << std::endl;
        return false;
    }

    const std::string &presetFile = firstBaseUnit.preset_antenna_files[presetIndex];
    std::string fullPath = std::string(CONFIG_ANTENNA_DIR) + "/" + presetFile;

    // Pass the current runConfig to be potentially modified by the static function
    std::optional<ConfigFile> result = AntennaConfigPage::processAntennaConfiguration(fullPath,
                                                                                      runConfig);

    if (result)
    {
        runConfig = result.value();                // Update the passed-in config object
        runConfig.antennaConfig_name = presetFile; // Store the name of the antenna preset
        // Preserve original display mode (important for console loop)
        runConfig.display_mode = this->paramConfig.display_mode;
        return true;
    }
    else
    {
        QString errorMsg = QString("Failed to load/process Antenna preset file: %1 (Path: %2)")
                               .arg(QString::fromStdString(presetFile),
                                    QString::fromStdString(fullPath));
        if (this->paramConfig.display_mode == "CONSOLE")
        { // Check original mode for context
            std::cerr << "ERROR: " << errorMsg.toStdString() << std::endl;
        }
        else
        {
            qWarning() << "  -> Preset Error:" << errorMsg;
            QMessageBox::critical(this, "Preset Error", errorMsg + "\nAborting auto-flow.");
        }
        return false;
    }
}

bool MainWindow::attemptProcessPositionPreset(size_t presetSetIndex,
                                              size_t presetFileIndex,
                                              ConfigFile &runConfig,
                                              nlohmann::json &runPositionConfig)
{
    // Position presets are global, defined in paramConfig.preset_position_files.
    // This function should only be called if position presets are enabled and relevant (SIMULATION mode).
    if (!paramConfig.preset_position_enabled || paramConfig.engine_mode != "SIMULATION")
    {
        qWarning("Attempting to process position preset index when presets are not enabled or not "
                 "in SIMULATION mode (unexpected).");
        // Return true, indicating no error, but also no action taken based on index.
        // Ensure position config is default/empty.
        runPositionConfig = nlohmann::json::object();
        return true;
    }

    if (paramConfig.preset_position_sets->empty())
    {
        qCritical("Position presets enabled, but file set list is empty during preset processing.");
        std::cerr << "ERROR: Position presets enabled, but file set list is empty during preset "
                     "processing."
                  << std::endl;
        return false;
    }

    // Check if set is out of bounds
    if (presetSetIndex >= paramConfig.preset_position_sets->size())
    {
        QString errorMsg = QString("Position preset index %1 is out of bounds (size: %2).")
                               .arg(presetSetIndex)
                               .arg(paramConfig.preset_position_sets->size());
        qCritical() << errorMsg;
        std::cerr << "ERROR: " << errorMsg.toStdString() << std::endl;
        return false;
    }

    // Check if file in set is out of bounds
    if (presetFileIndex >= (*paramConfig.preset_position_sets)[presetSetIndex].size())
    {
        QString errorMsg = QString("Position preset index %1 is out of bounds (size: %2).")
                               .arg(presetFileIndex)
                               .arg((*paramConfig.preset_position_sets)[presetSetIndex].size());
        qCritical() << errorMsg;
        std::cerr << "ERROR: " << errorMsg.toStdString() << std::endl;
        return false;
    }

    const std::string &presetFile = (*paramConfig.preset_position_sets)[presetSetIndex][presetFileIndex];
    std::string fullPath = std::string(CONFIG_POSITION_DIR) + "/" + presetFile;

    // Pass the current runConfig
    // and the position config JSON object to be modified.
    std::optional<nlohmann::json> result = PositionConfigPage::processPositionConfiguration(fullPath, runConfig);

    if (result)
    {
        runPositionConfig = result.value();
        return true;
    }
    else
    {
        QString errorMsg = QString("Failed to load/process Position preset file: %1 (Path: %2)")
                               .arg(QString::fromStdString(presetFile),
                                    QString::fromStdString(fullPath));
        if (this->paramConfig.display_mode == "CONSOLE")
        {
            std::cerr << "ERROR: " << errorMsg.toStdString() << std::endl;
        }
        else
        {
            qWarning() << "  -> Preset Error:" << errorMsg;
            QMessageBox::critical(this, "Preset Error", errorMsg + "\nAborting auto-flow.");
        }
        return false;
    }
}

bool MainWindow::attemptProcessAlgorithmPreset(size_t presetIndex, ConfigFile &runConfig)
{
    if (runConfig.engine_units.empty())
    {
        qWarning("Attempting to process algorithm preset with no engine units.");
        return false;
    }
    // Ensure the base config also has units (should always be true if runConfig does)
    if (this->paramConfig.engine_units.size() != runConfig.engine_units.size())
    {
        qCritical("Mismatch between runConfig and base paramConfig engine unit counts during "
                  "algorithm preset processing.");
        return false;
    }

    // Apply the algorithm name from EACH unit's preset list at the given index
    for (size_t i = 0; i < runConfig.engine_units.size(); ++i)
    {
        auto &unit = runConfig.engine_units[i];
        const auto &baseUnit = this->paramConfig
                                   .engine_units[i]; // Get corresponding unit from original config

        // This preset function should only be called if presets are enabled for this unit
        if (!baseUnit.preset_algorithm_enabled)
        {
            qWarning() << "Skipping algorithm preset for Unit " << i
                       << " as it's not enabled in base config (unexpected).";
            continue; // Or return false if this state is considered critical
        }

        if (presetIndex >= baseUnit.preset_algorithm_names.size())
        {
            QString errorMsg = QString("Algorithm preset index %1 is out of bounds for Unit %2 (size: %3).")
                                   .arg(presetIndex)
                                   .arg(i)
                                   .arg(baseUnit.preset_algorithm_names.size());
            qCritical() << errorMsg;
            std::cerr << "ERROR: " << errorMsg.toStdString() << std::endl;
            return false; // Index out of bounds is a critical error
        }

        const std::string &algorithmName = baseUnit.preset_algorithm_names[presetIndex];
        if (debug)
        {
            qInfo() << "  -> Setting algorithm for Unit " << i
                    << " to:" << QString::fromStdString(algorithmName);
        }
        unit.algorithm = algorithmName; // Set the actual algorithm field used by the engine for this unit
    }

    return true; // Assume success if names were set
}

bool MainWindow::checkConsolePresets() const
{
    // Verify that presets needed for console mode are enabled and have non-empty vectors
    if (paramConfig.engine_units.empty())
    {
        qWarning("Console preset check failed: No engine units defined.");
        return false;
    }
    const auto &firstUnit = paramConfig.engine_units[0];

    // Check Antenna
    bool antennaOk = !firstUnit.preset_antenna_enabled || !firstUnit.preset_antenna_files.empty();

    // Check Position Sets (Required for SIMULATION == CONSOLE)
    bool positionOk = !paramConfig.preset_position_enabled || !paramConfig.preset_position_sets->empty();
    // Check each file inside each Position Set
    if (paramConfig.preset_position_enabled)
    {
        for (const auto &fileSet : *paramConfig.preset_position_sets)
        {
            if (fileSet.empty())
            {
                positionOk = false; // If any set is empty, the check fails
                break;
            }
        }
    }

    // Check Algorithm
    bool algorithmOk = !firstUnit.preset_algorithm_enabled || !firstUnit.preset_algorithm_names.empty();

    if (!antennaOk || !positionOk || !algorithmOk)
    {
        qWarning("Console preset check failed:");
        if (!antennaOk)
            qWarning(" - Antenna preset enabled but file list is empty.");
        // Position check assumes engine_mode == SIMULATION already verified
        if (!positionOk)
            qWarning(" - Position preset enabled but file list is empty.");
        if (!algorithmOk)
            qWarning(" - Algorithm preset enabled but name list is empty.");
        return false;
    }

    // Also check if at least one preset type is actually enabled and has items,
    // otherwise, there's nothing to iterate over.
    bool anyPresetExists = (firstUnit.preset_antenna_enabled && !firstUnit.preset_antenna_files.empty()) || (paramConfig.preset_position_enabled && !paramConfig.preset_position_sets->empty()) || (firstUnit.preset_algorithm_enabled && !firstUnit.preset_algorithm_names.empty());

    if (!anyPresetExists)
    {
        qWarning(
            "Console preset check failed: No presets are enabled or defined to run combinations.");
        return false;
    }

    return true; // All enabled presets have non-empty vectors
}

// --- Navigation Helpers Implementation ---
QWidget *MainWindow::determinePreviousPage()
{
    if (paramConfig.display_mode == "CONSOLE" || isRunningCombinations)
        return nullptr;

    QWidget *currentPage = stackedWidget->currentWidget();
    if (!currentPage)
        return configSelectionPage;

    bool canSkipAnt, canSkipPos, canSkipAlg;
    checkSkippablePages(canSkipAnt, canSkipPos, canSkipAlg);

    if (currentPage == antennaConfigPage)
    {
        return configSelectionPage;
    }
    if (currentPage == positionConfigPage)
    {
        return canSkipAnt ? (QWidget *)configSelectionPage : (QWidget *)antennaConfigPage;
    }
    if (currentPage == algorithmSelectionPage)
    {
        if (paramConfig.engine_mode == "SIMULATION" && !canSkipPos)
            return positionConfigPage;
        if (!canSkipAnt)
            return antennaConfigPage;
        return configSelectionPage;
    }
    if (currentPage == initializationPage)
    {
        if (!canSkipAlg)
            return algorithmSelectionPage;
        if (paramConfig.engine_mode == "SIMULATION" && !canSkipPos)
            return positionConfigPage;
        if (!canSkipAnt)
            return antennaConfigPage;
        return configSelectionPage;
    }
    if (currentPage == simulationPage || currentPage == realtimePage)
    {
        return configSelectionPage;
    }

    qWarning() << "determinePreviousPage called from unexpected page:" << currentPage->objectName()
               << ". Defaulting to ConfigSelectionPage.";
    return configSelectionPage;
}

// --- Core Flow Logic Implementation ---
int MainWindow::startConfigurationFlow()
{
    if (paramConfig.display_mode == "CONSOLE")
    {
        if (paramConfig.engine_mode != "SIMULATION")
        {
            std::cerr
                << "CRITICAL: CONSOLE display_mode only supported with SIMULATION engine_mode."
                << std::endl;
            return 99;
        }
        if (!checkConsolePresets())
        { // Checks if necessary preset vectors are non-empty
            std::cerr << "CRITICAL: One or more required presets missing, invalid, or vectors "
                         "empty for CONSOLE mode. Check preset_..._enabled flags and preset vector "
                         "definitions in the configuration."
                      << std::endl;
            return 99;
        }
        // Start the combination runs
        startCombinationRuns();
    }
    else
    { // GUI Mode
        // Initiate the GUI page flow check
        requestTransition(nullptr);
    }
    return 1;
}

// --- Combination Run Logic (Console Mode) ---
int MainWindow::runConsoleMode()
{
    if (this->paramConfig.display_mode != "CONSOLE")
    {
        std::cerr
            << "Error: runConsoleMode called but application is not in console mode configuration."
            << std::endl;
        return 1; // Error
    }
    // Directly trigger the configuration flow which leads to startCombinationRuns
    return startConfigurationFlow();
}

int MainWindow::startCombinationRuns()
{
    std::cout << "--- Starting CONSOLE Mode Combination Runs ---" << std::endl;

    if (paramConfig.engine_units.empty())
    {
        std::cerr << "CRITICAL: No engine units defined in configuration." << std::endl;
        return 99;
    }
    const auto &firstUnit = paramConfig.engine_units[0];

    // --- Step 1: Calculate Preset Counts ---
    size_t numAntennaPresets = firstUnit.preset_antenna_enabled
                                   ? firstUnit.preset_antenna_files.size()
                                   : 1;
    size_t numAlgorithmPresets = firstUnit.preset_algorithm_enabled
                                     ? firstUnit.preset_algorithm_names.size()
                                     : 1;
    size_t totalPositionFiles = 1;
    if (paramConfig.preset_position_enabled)
    {
        totalPositionFiles = 0;
        for (const auto &fileSet : *paramConfig.preset_position_sets)
        {
            totalPositionFiles += fileSet.size();
        }
        if (totalPositionFiles == 0)
            totalPositionFiles = 1; // Handle case where list is empty but enabled
    }
    totalCombinations = numAntennaPresets * totalPositionFiles * numAlgorithmPresets;

    std::cout << "Number of Antenna presets: " << numAntennaPresets << std::endl;
    std::cout << "Number of Position presets: " << totalPositionFiles << std::endl;
    std::cout << "Number of Algorithm presets: " << numAlgorithmPresets << std::endl;
    std::cout << "Total combinations to run: " << totalCombinations << std::endl;

    // --- Caching all unique preset files... ---
    std::cout << "--- Caching all unique preset files in parallel... ---" << std::endl;
    auto start_caching = std::chrono::high_resolution_clock::now();

    const auto &firstUnitBase = this->paramConfig.engine_units[0];

    // Use a thread pool for caching operations
    QThreadPool cachingPool;
    cachingPool.setMaxThreadCount(QThread::idealThreadCount());

    // 1. Cache all unique Antenna files in parallel
    if (firstUnitBase.preset_antenna_enabled)
    {
        // Define the work for a single antenna file
        auto cacheAntennaFile =
            [&](const std::string &presetFile) -> std::pair<std::string, ConfigFile>
        {
            ConfigFile tempConfig = this->paramConfig;
            // Find the index 'i' corresponding to the presetFile to call the correct preset function
            const auto &files = firstUnitBase.preset_antenna_files;
            auto it = std::find(files.begin(), files.end(), presetFile);
            if (it != files.end())
            {
                size_t index = std::distance(files.begin(), it);
                if (this->attemptProcessAntennaPreset(index, tempConfig))
                {
                    return {presetFile, tempConfig};
                }
            }
            // Use a mutex for thread-safe console output on error
            std::lock_guard<std::mutex> lock(this->consoleMutex);
            std::cerr << "CRITICAL: Failed to process antenna file '" << presetFile
                      << "' for caching." << std::endl;
            // Return an empty pair to signify failure
            return {presetFile, ConfigFile{}};
        };

        // Run the caching in parallel. Get a list of unique files first.
        std::vector<std::string> uniqueAntennaFiles = firstUnitBase.preset_antenna_files;
        std::sort(uniqueAntennaFiles.begin(), uniqueAntennaFiles.end());
        uniqueAntennaFiles.erase(std::unique(uniqueAntennaFiles.begin(), uniqueAntennaFiles.end()),
                                 uniqueAntennaFiles.end());

        QFuture<std::pair<std::string, ConfigFile>> antennaFuture = QtConcurrent::mapped(&cachingPool, uniqueAntennaFiles, cacheAntennaFile);
        antennaFuture.waitForFinished();

        // Populate the cache from the results
        for (const auto &result : antennaFuture.results())
        {
            if (!result.second.engine_mode.empty())
            { // A simple check for a valid ConfigFile
                antennaCache[result.first] = result.second;
            }
            else
            {
                return 99; // Abort on caching failure
            }
        }
    }

    // 2. Cache all unique Position files in parallel
    if (this->paramConfig.preset_position_enabled)
    {
        // Get a flat list of unique position file names
        std::vector<std::string> uniquePositionFiles;
        for (const auto &fileSet : *this->paramConfig.preset_position_sets)
        {
            uniquePositionFiles.insert(uniquePositionFiles.end(), fileSet.begin(), fileSet.end());
        }
        std::sort(uniquePositionFiles.begin(), uniquePositionFiles.end());
        uniquePositionFiles.erase(std::unique(uniquePositionFiles.begin(),
                                              uniquePositionFiles.end()),
                                  uniquePositionFiles.end());

        // Define the work for a single position file
        auto cachePositionFile =
            [&](const std::string &presetFile) -> std::pair<std::string, nlohmann::json>
        {
            ConfigFile tempConfig = this->paramConfig; // a temporary, non-shared config
            std::string fullPath = std::string(CONFIG_POSITION_DIR) + "/" + presetFile;
            if (auto result = PositionConfigPage::processPositionConfiguration(fullPath,
                                                                               tempConfig))
            {
                return {presetFile, result.value()};
            }
            // Use a mutex for thread-safe console output on error
            std::lock_guard<std::mutex> lock(this->consoleMutex);
            std::cerr << "CRITICAL: Failed to process position file '" << presetFile
                      << "' for caching." << std::endl;
            // Return an empty pair to signify failure
            return {presetFile, nlohmann::json()};
        };

        // Run the caching in parallel
        QFuture<std::pair<std::string, nlohmann::json>> positionFuture = QtConcurrent::mapped(&cachingPool, uniquePositionFiles, cachePositionFile);
        positionFuture.waitForFinished();

        // Populate the cache from the results
        for (const auto &result : positionFuture.results())
        {
            if (!result.second.is_null())
            { // Check for valid json
                positionCache[result.first] = result.second;
            }
            else
            {
                return 99; // Abort on caching failure
            }
        }
    }

    auto end_caching = std::chrono::high_resolution_clock::now();
    auto caching_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_caching - start_caching).count();
    std::cout << "--- Caching complete in " << caching_ms / 1000.0 << " seconds. ---" << std::endl;

    // --- Step 3: Generate Lightweight Index List ---
    std::vector<CombinationIndices> allCombinations;
    allCombinations.reserve(totalCombinations);
    for (size_t antIdx = 0; antIdx < numAntennaPresets; ++antIdx)
    {
        for (size_t posSetIdx = 0; posSetIdx < paramConfig.preset_position_sets->size();
             ++posSetIdx)
        {
            for (size_t posFileIdx = 0;
                 posFileIdx < (*paramConfig.preset_position_sets)[posSetIdx].size();
                 ++posFileIdx)
            {
                for (size_t algIdx = 0; algIdx < numAlgorithmPresets; ++algIdx)
                {
                    allCombinations.push_back({antIdx, posSetIdx, posFileIdx, algIdx});
                }
            }
        }
    }

    auto getEstimatedSlots = [&](const CombinationIndices &idx) -> size_t
    {
        double max_time = this->paramConfig.engine_max_time_sec;
        double slot_time = this->paramConfig.engine_slot_time_microsec;

        if (this->paramConfig.preset_position_enabled && !this->paramConfig.preset_position_sets->empty())
        {
            const std::string &f = (*this->paramConfig.preset_position_sets)[idx.posSetIdx][idx.posFileInSetIdx];
            if (this->positionCache.count(f))
            {
                const auto &posJson = this->positionCache.at(f);
                if (posJson.contains("status") && posJson["status"].contains("duration"))
                {
                    max_time = posJson["status"]["duration"].get<double>();
                }
            }
        }
        return static_cast<size_t>(std::ceil((max_time * 1e6) / slot_time));
    };

    // 2. Sort Descending first (Groups all Heavy items together)
    std::sort(allCombinations.begin(),
              allCombinations.end(),
              [&](const CombinationIndices &a, const CombinationIndices &b)
              {
                  return getEstimatedSlots(a) > getEstimatedSlots(b);
              });

    // 3. Divide into small blocks and Shuffle the BLOCKS.
    // A block size of ~4 ensures a thread stays on a size tier for a while (saving realloc),
    // but ensures we don't load 20 threads with max-size jobs simultaneously.
    size_t blockSize = 24;
    std::vector<std::vector<CombinationIndices>> blocks;

    for (size_t i = 0; i < allCombinations.size(); i += blockSize)
    {
        size_t end = std::min(i + blockSize, allCombinations.size());
        blocks.emplace_back(std::vector<CombinationIndices>(allCombinations.begin() + i,
                                                            allCombinations.begin() + end));
    }

    // 4. Shuffle the order of blocks (Mixes Heavy blocks and Light blocks)
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(blocks.begin(), blocks.end(), g);

    // 5. Flatten back into allCombinations
    allCombinations.clear();
    for (const auto &block : blocks)
    {
        allCombinations.insert(allCombinations.end(), block.begin(), block.end());
    }

    // --- Step 3: Prepare for and execute parallel runs with intelligent batching ---
    successfulCombinationsAtomic = 0;
    completedCombinationsAtomic = 0;
    int numThreads = QThread::idealThreadCount() - 1; // Reserve threads for main and KPI writer

    QThreadPool combinationPool;
    combinationPool.setMaxThreadCount(numThreads);
    std::cout << "Running combinations in parallel using up to " << numThreads << " threads."
              << std::endl;

    constexpr double BATCH_HEADROOM_FACTOR = 1.025; // Allocate 55% extra space for growth
    constexpr double BATCH_SHRINK_THRESHOLD = 0.75; // Re-allocate if new size is < 75% of current

    // The worker lambda. Captures necessary context by reference.
    auto processSingleCombinationTask = [this](const CombinationIndices &indices) -> bool
    {
        try
        {
            // --- Data Assembly from Cache ---
            PreprocessedData data;
            const auto &firstUnitBase = this->paramConfig.engine_units[0];

            // A. Get pre-cached Antenna config
            if (firstUnitBase.preset_antenna_enabled)
            {
                const std::string &presetAntennaFile = firstUnitBase.preset_antenna_files[indices.antennaIdx];
                data.runConfig = antennaCache.at(presetAntennaFile); // Creates a local copy
            }
            else
            {
                data.runConfig = this->paramConfig;
            }

            // B. Get pre-cached Position config and apply its time settings
            if (this->paramConfig.preset_position_enabled)
            {
                const std::string &presetPositionFile = (*this->paramConfig
                                                              .preset_position_sets)[indices.posSetIdx][indices.posFileInSetIdx];
                data.positionConfig = positionCache.at(presetPositionFile); // Creates a local copy
                PositionConfigPage::applyPositionTimeToConfig(data.positionConfig, data.runConfig);
                data.runConfig.positionConfig_name = presetPositionFile;
            }

            // C. Apply Algorithm preset
            if (firstUnitBase.preset_algorithm_enabled)
            {
                if (!attemptProcessAlgorithmPreset(indices.algIdx, data.runConfig))
                    return false;
            }

            // Calculate final size keys and KPI name
            data.num_units = data.runConfig.engine_units.size();
            data.num_time_slots = static_cast<size_t>(
                std::ceil((data.runConfig.engine_max_time_sec * 1e6) / data.runConfig.engine_slot_time_microsec));
            data.kpiFileName = "Set" + std::to_string(indices.posSetIdx) + "_File" + std::to_string(indices.posFileInSetIdx) + "_Ant" + std::to_string(indices.antennaIdx) + "_Alg" + std::to_string(indices.algIdx) + "_KPIs.json";

            // --- Padded Batching and Simulation (from previous steps) ---
            size_t allocated_slots = 0;
            if (t_simulation_objects)
            {
                allocated_slots = static_cast<size_t>(
                    std::ceil((t_current_objects_config.engine_max_time_sec * 1e6) / t_current_objects_config.engine_slot_time_microsec));
            }

            // Determine if we need to re-allocate the simulation objects for this thread.
            bool needsNewBatch = false;
            if (!t_simulation_objects || t_current_objects_config.engine_units.size() != data.num_units)
            {
                // Case 1: First run for this thread OR number of units changed.
                needsNewBatch = true;
            }
            else
            {
                // Case 2: The required size is larger than what's allocated. Need to grow.
                bool needsToGrow = data.num_time_slots > allocated_slots;

                // Case 3: The required size is SIGNIFICANTLY smaller. Time to shrink.
                bool needsToShrink = (data.num_time_slots < (allocated_slots * BATCH_SHRINK_THRESHOLD));

                if (needsToGrow || needsToShrink)
                {
                    needsNewBatch = true;
                }
            }

            if (needsNewBatch)
            {
                // Create a new config for the BATCH, with headroom.
                ConfigFile batchConfig = data.runConfig;
                size_t slots_to_allocate = static_cast<size_t>(data.num_time_slots * BATCH_HEADROOM_FACTOR);
                batchConfig.engine_max_time_sec = (static_cast<long double>(slots_to_allocate) * batchConfig.engine_slot_time_microsec) / 1.0e6;

                // Re-create the simulation objects with the larger batch size.
                t_simulation_objects = std::make_unique<SimulationObjects>();
                t_simulation_objects->engine = std::make_unique<mobileTHzEngine>(
                    batchConfig); // Allocate large grid
                t_simulation_objects->posSim = std::make_unique<positionSim>();
                t_simulation_objects->imuSim = std::make_unique<imuSim>();
                t_simulation_objects->powerSim = std::make_unique<powerSim>();
                t_simulation_objects->rotarySim = std::make_unique<rotarySim>();
                t_simulation_objects->kpiClassifier = std::make_unique<kpiClassifier>();
                t_simulation_objects->plotSaver = std::make_unique<plotSaver>();

                // Store the config that represents the ALLOCATED size.
                t_current_objects_config = batchConfig;
            }

            // Initialize the simulators with the data for THIS specific run.
            // The engine object is large enough, but the logic inside these initializers
            // must use the ACTUAL run duration from 'data.runConfig'.
            t_simulation_objects->engine->initialize(data.runConfig);
            t_simulation_objects->posSim->initialize(data.positionConfig,
                                                     data.runConfig,
                                                     t_simulation_objects->engine.get());
            t_simulation_objects->imuSim->initialize(data.runConfig,
                                                     t_simulation_objects->engine.get());
            t_simulation_objects->powerSim->initialize(data.runConfig,
                                                       t_simulation_objects->engine.get());
            t_simulation_objects->rotarySim->initialize(data.runConfig,
                                                        t_simulation_objects->engine.get());
            t_simulation_objects->kpiClassifier->initialize(t_simulation_objects->engine.get());

            // Run the simulation
            t_simulation_objects->engine->run(t_simulation_objects->kpiClassifier.get(),
                                              t_simulation_objects->plotSaver.get(),
                                              t_simulation_objects->powerSim.get(),
                                              t_simulation_objects->rotarySim.get(),
                                              nullptr, // powerExp
                                              nullptr, // rotaryExp
                                              nullptr  // imuExp
            );

            // Calculate and Save KPIs
            t_simulation_objects->kpiClassifier->calculateFinalKPIsAndSave(data.kpiFileName);

            completedCombinationsAtomic.fetch_add(1);
            return true;
        }
        catch (const std::exception &e)
        {
            std::lock_guard<std::mutex> lock(consoleMutex);
            std::cerr << "ERROR in Combination - Exception: " << e.what() << std::endl;
            completedCombinationsAtomic.fetch_add(1);
            return false;
        }
    };

    auto reduceResults = [&successfulCombinationsAtomic = this->successfulCombinationsAtomic](bool &,
                                                                                              const bool &singleMapResult)
    {
        if (singleMapResult)
        {
            successfulCombinationsAtomic.fetch_add(1);
        }
    };

    // --- Progress Reporting Setup ---
    completedCombinationsAtomic = 0; // Reset counter before starting
    std::atomic<bool> allWorkIsDone = false;

    std::thread progressThread([&]()
                               {
        while (!allWorkIsDone.load()) {
            size_t completed = completedCombinationsAtomic.load();
            size_t percent = (totalCombinations > 0) ? (100 * completed) / totalCombinations : 100;

            std::cout << "\rSimulation Progress: " << percent << "% (" << completed << "/"
                      << totalCombinations << ")" << std::flush;

            // Sleep for a short duration to avoid spamming the console
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        // Print the final, complete line
        size_t completed = completedCombinationsAtomic.load();
        std::cout << "\rSimulation Progress: 100% (" << completed << "/" << totalCombinations << ")"
                  << std::endl; });

    // --- Execute in Parallel and Wait ---
    QtConcurrent::blockingMappedReduced<bool>(
        &combinationPool,
        allCombinations, // Pass the vector of pre-processed data structs
        processSingleCombinationTask,
        reduceResults);

    // --- Final Summary ---
    std::cout << "\n--- All Combinations Complete ---" << std::endl;
    size_t finalSuccessCount = successfulCombinationsAtomic.load();
    std::cout << "Successfully completed runs: " << finalSuccessCount << "/" << totalCombinations
              << std::endl;
    isRunningCombinations = false; // Should already be false, but just in case

    // Join the progress thread
    allWorkIsDone.store(true);
    if (progressThread.joinable())
        progressThread.join();

    // Exit the application
    int exitCode = (finalSuccessCount == totalCombinations ? 0 : 98);
    return exitCode;
} // End of

// Executes one full sequence for a given set of preset indices
bool MainWindow::runSingleCombination(size_t antennaIdx,
                                      size_t posSetIdx,
                                      size_t posFileIdx,
                                      size_t algIdx)
{
    // Create copies/views for this specific run
    ConfigFile currentRunConfig = this->paramConfig;                    // Start with base config
    nlohmann::json currentRunPositionConfig = nlohmann::json::object(); // Start empty

    // --- Apply Presets for this Combination ---
    bool presetsOk = true;
    const auto &firstUnitBase = this->paramConfig.engine_units[0];

    // 1. Antenna Preset
    if (firstUnitBase.preset_antenna_enabled)
    { // Check if enabled
        if (firstUnitBase.preset_antenna_files.empty())
        {
            std::cerr << "ERROR: Antenna preset enabled but file list is empty (should have been "
                         "caught earlier)."
                      << std::endl;
            presetsOk = false;
        }
        else
        {
            // Pass the INDEX 'antennaIdx'
            if (!attemptProcessAntennaPreset(antennaIdx, currentRunConfig))
            {
                presetsOk = false; // Error logged in attemptProcess...
            }
        }
    }
    else
    {
        std::cout << "Skipping Antenna Preset (Not enabled)." << std::endl;
    }

    // 2. Position Preset (only if SIMULATION mode)
    if (presetsOk && paramConfig.engine_mode == "SIMULATION")
    {
        if (paramConfig.preset_position_enabled)
        { // Check if enabled
            if (paramConfig.preset_position_sets->empty())
            {
                std::cerr << "ERROR: Position preset enabled but file set list is empty "
                             "(should have been caught earlier)."
                          << std::endl;
                presetsOk = false;
            }
            else
            {
                // Pass the INDEX 'posSetIdx' and 'posFileIdx'
                if (!attemptProcessPositionPreset(posSetIdx,
                                                  posFileIdx,
                                                  currentRunConfig,
                                                  currentRunPositionConfig))
                {
                    presetsOk = false; // Error logged in attemptProcess...
                }
            }
        }
        else
        {
            std::cout << "Skipping Position Preset (Not enabled)." << std::endl;
        }
    }

    // 3. Algorithm Preset
    if (presetsOk)
    {
        if (firstUnitBase.preset_algorithm_enabled)
        { // Check if enabled
            if (firstUnitBase.preset_algorithm_names.empty())
            {
                std::cerr << "ERROR: Algorithm preset enabled but name list is empty (should have "
                             "been caught earlier)."
                          << std::endl;
                presetsOk = false;
            }
            else
            {
                // Pass the INDEX 'algIdx'
                if (!attemptProcessAlgorithmPreset(algIdx, currentRunConfig))
                {
                    presetsOk = false; // Error logged in attemptProcess...
                }
            }
        }
        else
        {
            std::cout << "Skipping Algorithm Preset (Not enabled)." << std::endl;
        }
    }

    if (!presetsOk)
    {
        std::cerr << "ERROR: Failed during preset application phase for combination." << std::endl;
        return false; // Indicate failure to set up this combination
    }

    // --- Proceed to Initialization ---
    std::cout << "Presets processed successfully. Proceeding to Initialization..." << std::endl;
    if (!initializationPage)
    {
        std::cerr << "CRITICAL: Initialization page is null. Exiting." << std::endl;
        QCoreApplication::exit(1); // Critical failure
        return false;              // Should not be reached
    }

    // Pass the configuration specific to this run
    initializationPage->setParamConfig(currentRunConfig, currentRunPositionConfig);
    std::cout << "Triggering Initialization..." << std::endl;
    initializationPage->startInitialization(); // Start the background initialization

    return true; // Setup successful, waiting for initialization result
}

void MainWindow::onCombinationFinished(bool success)
{
    if (!isRunningCombinations)
        return; // Avoid issues if called unexpectedly

    completedCombinations++;
    if (success)
    {
        successfulCombinations++;
    }

    // Use QTimer to schedule the next run, allowing event loop to process
    QTimer::singleShot(0, this, &MainWindow::runNextCombination);
}

void MainWindow::runNextCombination()
{
    if (!isRunningCombinations || completedCombinations >= totalCombinations)
    {
        // All combinations finished
        std::cout << "\n--- All Combinations Complete ---" << std::endl;
        std::cout << "Successfully completed runs: " << successfulCombinations << "/"
                  << totalCombinations << std::endl;
        isRunningCombinations = false;
        QCoreApplication::exit(successfulCombinations == totalCombinations
                                   ? 0
                                   : 1); // Exit with 0 on full success, 1 otherwise
        return;
    }

    // Calculate indices for the current combination
    // Algorithm changes fastest, then Position, then Antenna
    size_t currentComboNumber = completedCombinations;
    currentAlgorithmIndex = currentComboNumber % numAlgorithmPresets;
    size_t flatPositionIndex = (currentComboNumber / numAlgorithmPresets) % numPositionPresets; // Keep this
    currentAntennaIndex = (currentComboNumber / (numAlgorithmPresets * numPositionPresets)) % numAntennaPresets;

    auto [positionSetIndex, positionFileIndex] = getSetAndFileIndices(flatPositionIndex);

    std::cout << "\n--- Running Combination " << completedCombinations + 1 << "/"
              << totalCombinations << " ---" << std::endl;

    std::cout << "\n--- Running Combination " << completedCombinations + 1 << "/"
              << totalCombinations << " ---" << std::endl;
    std::cout << "Indices: [Set: " << positionSetIndex << ", File: " << positionFileIndex
              << ", Ant: " << currentAntennaIndex << ", Alg: " << currentAlgorithmIndex << "]"
              << std::endl;

    // Run the simulation for this specific combination
    // runSingleCombination sets up config and calls initializationPage->startInitialization()
    // The result will eventually trigger onInitializationFinished -> onCombinationFinished
    if (!runSingleCombination(currentAntennaIndex,
                              positionSetIndex,
                              positionFileIndex,
                              currentAlgorithmIndex))
    {
        // If setup for the combination fails immediately (e.g., preset file invalid)
        std::cerr << "ERROR: Failed to set up combination ["
                  << "Set: " << positionSetIndex << ", File: " << positionFileIndex
                  << ", Ant: " << currentAntennaIndex << ", Alg: " << currentAlgorithmIndex << "]"
                  << std::endl;
        onCombinationFinished(false); // Mark as failed and proceed
    }
    // Otherwise, wait for onInitializationFinished to call onCombinationFinished
}

std::pair<size_t, size_t> MainWindow::getSetAndFileIndices(size_t flatPositionIndex) const
{
    // If presets aren't enabled or are empty, return {0, 0} as there's only one "virtual" position.
    if (!paramConfig.preset_position_enabled || paramConfig.preset_position_sets->empty())
    {
        return {0, 0};
    }

    size_t remainingIndex = flatPositionIndex;
    for (size_t setIdx = 0; setIdx < paramConfig.preset_position_sets->size(); ++setIdx)
    {
        const auto &currentSet = (*paramConfig.preset_position_sets)[setIdx];
        if (remainingIndex < currentSet.size())
        {
            // Found it! The set is setIdx, and the file is the remaining index.
            return {setIdx, remainingIndex};
        }
        // Not in this set, subtract its size and check the next one.
        remainingIndex -= currentSet.size();
    }

    // This should not be reached if flatPositionIndex is valid.
    qCritical() << "Logic Error: flatPositionIndex" << flatPositionIndex << "is out of bounds.";
    return {0, 0}; // Return a safe default on error
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    // Check if we are in experimental mode, as this logic is only needed then.
    if (paramConfig.engine_mode == "EXPERIMENTAL")
    {
        std::cout << "Closing application. Cleaning up experimental resources..." << std::endl;
        // Tell the initialization page to stop the background tasks.
        if (initializationPage)
        {
            initializationPage->stopExperiment();
        }
    }

    // Accept the event, which allows the window to close and the application to terminate.
    event->accept();
}
