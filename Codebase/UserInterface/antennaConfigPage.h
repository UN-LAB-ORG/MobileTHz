#ifndef ANTENNACONFIGPAGE_H
#define ANTENNACONFIGPAGE_H

#include <QWidget>
#include <map>
#include <optional>
#include <string>
#include "Software/structDefinition.h"

// Forward declare Qt classes used as pointers in the private section
class QComboBox;
class QFrame;
class QHBoxLayout;
class QKeyEvent;
class QLabel;
class QMessageBox;
class QPushButton;
class QScrollArea;
class QVBoxLayout;
class QChart;
class QChartView;
class QLineSeries;
class QValueAxis;

class AntennaConfigPage : public QWidget
{
    Q_OBJECT
public:
    /**
     * @brief Loads antenna patterns from a file and assigns the first loaded pattern
     *        to all enabled units in the provided ConfigFile. (Non-UI)
     * @param fullAntennaConfigPath Full path to the antenna JSON configuration file.
     *                              The directory for relative CSV paths is derived from this path.
     * @param inputConfig The base configuration file.
     * @return An std::optional<ConfigFile>. Contains the updated ConfigFile if loading and
     *         assignment are successful, std::nullopt otherwise. Errors are logged to std::cerr.
     */
    static std::optional<ConfigFile> processAntennaConfiguration(
        const std::string &fullAntennaConfigPath, ConfigFile &inputConfig);

    explicit AntennaConfigPage(QWidget *parent = nullptr);

    /**
     * @brief Sets the base configuration file for the UI instance to work with.
     *        Resets any previously loaded patterns in the UI.
     * @param config The configuration file definition.
     */
    void setParamConfig(const ConfigFile &config);

signals:
    /**
     * @brief Emitted when the user clicks 'Next' and patterns are validly assigned.
     * @param updatedConfig The ConfigFile potentially modified with assigned antenna patterns.
     */
    void nextRequested(const ConfigFile &updatedConfig);

    /**
     * @brief Emitted when the user clicks the 'Back' button.
     */
    void backRequested();

protected:
    // Handle key presses (specifically Enter key for 'Next' action)
    void keyPressEvent(QKeyEvent *event) override;

private slots:
    /**
     * @brief Loads patterns from the JSON file selected in the configFileCombo.
     *        Updates the internal loadedPatternsMap and refreshes the UI list/graph.
     */
    void loadAntennaConfig();

    /**
     * @brief Updates the radiation pattern graph display based on the currently
     *        loaded patterns stored in loadedPatternsMap.
     */
    void updateGraph();

    /**
     * @brief Assigns the currently loaded patterns (first one) to the selected unit(s)
     *        in the internal paramConfigFile, based on the unitSelectionCombo.
     *        Called when unit selection changes or before emitting nextRequested.
     */
    void assignPatternsToConfig();

    /**
     * @brief Checks if patterns are validly assigned to all enabled engine units
     *        within the internal paramConfigFile. Shows a warning if issues are found.
     * @return True if patterns are validly assigned, false otherwise.
     */
    bool checkPatterns();

private:
    /**
     * @brief (Static Helper) Loads antenna patterns from a JSON configuration file.
     *        Derives the directory for relative CSV paths from the input file path.
     * @param fullAntennaConfigPath Full path to the antenna JSON file.
     * @return std::map<int, radiationPattern> - Contains loaded patterns keyed by an
     *         internal ID (1, 2, ...). Returns an EMPTY map if loading fails (error logged to cerr).
     */
    static std::map<int, radiationPattern> loadAntennaPatternsFromFile_static(
        const std::string &fullAntennaConfigPath);

    /**
     * @brief (Static Helper) Assigns the first pattern from the map to units in a ConfigFile.
     * @param inputConfig ConfigFile to modify (passed by value, returns modified copy).
     * @param loadedPatterns Map of patterns (MUST NOT BE EMPTY when called).
     * @param unitIndex Index of the engineUnit to assign to (-1 for all units).
     * @return The modified ConfigFile. Returns original if loadedPatterns is empty or index invalid.
     */
    static ConfigFile assignLoadedPatternsToConfig_static(
        ConfigFile inputConfig,
        const std::map<int, radiationPattern> &loadedPatterns,
        int unitIndex = -1);

    static double getMaxGainFromPattern(const std::map<int, radiationPattern> &patterns_map,
                                        int pattern_id);

    static double getHpbwFromPattern(const std::map<int, radiationPattern> &patterns_map,
                                     int pattern_id);

    /**
     * @brief Calculates the standard deviation for a Gaussian distribution based on HPBW.
     * @param HPBW_radians Half-Power Beamwidth in radians.
     * @return Standard deviation in radians.
     */
    static double calculateStandardDeviation(double HPBW_radians);

    /**
     * @brief Converts degrees to radians.
     * @param degrees Angle in degrees.
     * @return Angle in radians.
     */
    static double degreesToRadians(double degrees);

private:
    /**
     * @brief Initializes all UI elements and layouts for the widget.
     */
    void setupUI();

    /**
     * @brief Scans the CONFIG_ANTENNA_DIR and populates the configFileCombo dropdown.
     */
    void refreshConfigFiles();

    /**
     * @brief Populates the unitSelectionCombo dropdown based on the engine units
     *        defined in the current paramConfigFile.
     */
    void populateUnitSelectionCombo();

    /**
     * @brief Clears the list of loaded patterns in the UI (csvPatternsLayout)
     *        and triggers an updateGraph() call (which will show an empty graph
     *        if loadedPatternsMap is empty).
     */
    void clearLoadedDataAndUpdateUI();

    // --- UI Element Pointers ---
private:
    QVBoxLayout *mainLayout = nullptr;      // Overall vertical layout
    QHBoxLayout *headerLayout = nullptr;    // Top section with title and logos
    QHBoxLayout *navButtonLayout = nullptr; // Bottom section for Back/Next buttons

    // Selection Frame Elements
    QFrame *selectionFrame = nullptr;        // Frame specifically for the dropdown selectors
    QHBoxLayout *selectionLayout = nullptr;  // Layout for config file and unit dropdowns (HBox)
    QComboBox *configFileCombo = nullptr;    // Combo box for antenna JSON file selection
    QComboBox *unitSelectionCombo = nullptr; // Combo box for target engine unit selection

    // Chart Elements
    QChartView *chartView = nullptr; // View widget for displaying the chart
    QChart *chart = nullptr;         // The chart object itself

    // Buttons
    QPushButton *nextButton = nullptr;
    QPushButton *backButton = nullptr;

private:
    /**
     * @brief Stores the base configuration being modified by this UI instance.
     *        Updated by setParamConfig and assignPatternsToConfig.
     */
    ConfigFile paramConfigFile;

    /**
     * @brief Stores the patterns loaded from the currently selected JSON file
     *        for this UI instance. Populated by loadAntennaConfig.
     *        Key: Internal pattern ID (1, 2, ...), Value: pattern data.
     */
    std::map<int, radiationPattern> loadedPatternsMap;
};

#endif // ANTENNACONFIGPAGE_H
