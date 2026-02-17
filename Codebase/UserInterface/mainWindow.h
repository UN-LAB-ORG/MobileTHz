#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <qmutex.h>
#include <stack>

#include "Software/jsonReader/jsonReader.hpp"
#include "Software/structDefinition.h"
#include "imuSim/imuSim.h"
#include "kpiClassifier/kpiClassifier.h"
#include "positionSim/positionSim.h"
#include "powerSim/powerSim.h"
#include "rotarySim/rotarySim.h"
#include "sharedPointersDefinition.h"

QT_BEGIN_NAMESPACE
class QStackedWidget;
class QCloseEvent;
QT_END_NAMESPACE

// Forward declarations
class ConfigSelectionPage;
class AntennaConfigPage;
class PositionConfigPage;
class AlgorithmSelectionPage;
class InitializationPage;
class RealtimePage;
class SimulationPage;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(const QString &initialConfigPath = QString(),
                        const bool consoleMode = false,
                        QWidget *parent = nullptr);
    ~MainWindow();

    int runConsoleMode();

protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    // Slots from pages
    void onConfigSelected(const ConfigFile &config);
    void onAntennaNext(const ConfigFile &updatedConfig);
    void onPositionNext(const nlohmann::json &positionConfig);
    void onAlgorithmNext(const ConfigFile &updatedConfig);
    void onInitializationFinished(bool success, const QString &errorMsg);
    void onViewSimulationRequested();
    void onSimulationComplete();
    void goBack();

private:
    // --- UI Setup ---
    void setupUI();
    void setupConnections();

    // --- Core Flow Logic ---
    int startConfigurationFlow();
    void requestTransition(QWidget *departingPage = nullptr);

    // --- Combination Run Logic (Console Mode) ---
    int startCombinationRuns(); // Sets up and initiates the loop
    void runNextCombination();  // Determines indices and calls runSingleCombination
    bool runSingleCombination(size_t antennaIdx,
                              size_t posSetIdx,
                              size_t posFileIdx,
                              size_t algIdx); // Executes one preset combo
    void onCombinationFinished(bool success); // Called after a single combo finishes init/sim

    // --- Preset Handling Helpers ---
    bool attemptProcessAntennaPreset(size_t presetIndex, ConfigFile &runConfig);
    bool attemptProcessPositionPreset(size_t presetSetIndex,
                                      size_t presetFileIndex,
                                      ConfigFile &runConfig,
                                      nlohmann::json &runPositionConfig);
    bool attemptProcessAlgorithmPreset(size_t presetIndex, ConfigFile &runConfig);

    bool checkConsolePresets() const;
    void checkSkippablePages(bool &canSkipAnt, bool &canSkipPos, bool &canSkipAlg) const;

    // --- Navigation Helpers ---
    QWidget *determinePreviousPage();

    // --- UI Elements ---
    QStackedWidget *stackedWidget;
    ConfigSelectionPage *configSelectionPage;
    AntennaConfigPage *antennaConfigPage;
    PositionConfigPage *positionConfigPage;
    AlgorithmSelectionPage *algorithmSelectionPage;
    InitializationPage *initializationPage;
    RealtimePage *realtimePage;
    SimulationPage *simulationPage;

    // --- State Data ---
    QString initialConfigPath;
    ConfigFile paramConfig;        // Base configuration loaded initially
    nlohmann::json positionConfig; // Base position config (if any general ones exist)

    // Caching for presets
    std::unordered_map<std::string, nlohmann::json> positionCache;
    std::unordered_map<std::string, ConfigFile> antennaCache;

    // --- Combination Run State (Console Mode) ---
    size_t currentAntennaIndex = 0;
    size_t currentPositionIndex = 0;
    size_t currentAlgorithmIndex = 0;
    size_t totalCombinations = 0;
    size_t completedCombinations = 0;
    size_t successfulCombinations = 0;  // Track successes
    bool isRunningCombinations = false; // Flag to indicate console loop is active

    // --- For Parallel Console Runs ---
    std::atomic<size_t> successfulCombinationsAtomic; // Atomic counter for successes
    std::mutex consoleMutex;                          // Mutex to protect console output (std::cout, qInfo, etc.)

    // Helper struct for combinations
    struct CombinationIndices
    {
        size_t antennaIdx;
        size_t posSetIdx;       // Index of the position set (the outer array)
        size_t posFileInSetIdx; // Index of the file within that set (the inner array)
        size_t algIdx;
    };
    std::atomic<size_t> completedCombinationsAtomic{0};

    // Converts a flat index into a combination of set and file indices
    std::pair<size_t, size_t> getSetAndFileIndices(size_t flatPositionIndex) const;

    // Store sizes for convenience
    size_t numAntennaPresets = 0;
    size_t numPositionPresets = 0;
    size_t numAlgorithmPresets = 0;

    // A thread-safe stack to hold the available object sets.
    std::stack<std::unique_ptr<SimulationObjects>> objectPool;
    QMutex poolMutex; // Mutex to protect access to the stack.
};

#endif // MAINWINDOW_H
