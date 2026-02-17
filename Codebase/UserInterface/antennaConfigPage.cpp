#include "antennaConfigPage.h"

#include <algorithm>
#include <cmath>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <QComboBox>
#include <QFileInfo>
#include <QFrame>
#include <QGraphicsLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QMessageBox>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QVariant>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include "Software/jsonReader/jsonReader.hpp"

// Use namespaces to simplify code
namespace fs = std::filesystem;
using json = nlohmann::json;

namespace
{
    inline double dbiToLinear(double dbi)
    {
        return std::pow(10.0, dbi / 10.0);
    }
} // namespace

double AntennaConfigPage::calculateStandardDeviation(double HPBW_radians)
{
    // HPBW = 2 * sqrt(2 * ln(2)) * sigma
    // sigma = HPBW / (2 * sqrt(2 * ln(2)))
    if (HPBW_radians <= 1e-9)
        return 0.0;
    double factor = 2.0 * std::sqrt(2.0 * std::log(2.0)); // Approx 2.3548
    if (std::abs(factor) < 1e-9)
    {
        std::cerr << "[Warn][AntennaConfigPage StaticCalc] HPBW to Sigma factor is near zero."
                  << std::endl;
        return 0.0; // Avoid division by zero/small number
    }
    return HPBW_radians / factor;
}

double AntennaConfigPage::degreesToRadians(double degrees)
{
    return degrees * (M_PI / 180.0);
}

std::map<int, radiationPattern> AntennaConfigPage::loadAntennaPatternsFromFile_static(
    const std::string &fullAntennaConfigPath)
{
    // Initialize return map (empty signifies failure)
    std::map<int, radiationPattern> patterns;
    int currentPatternId = 0; // Local counter for assigning IDs within this load operation

    // --- Input Validation and Directory Derivation ---
    fs::path configPathObj(fullAntennaConfigPath);
    try
    { // Wrap filesystem operations in try-catch
        if (!fs::exists(configPathObj) || !fs::is_regular_file(configPathObj))
        {
            std::cerr
                << "[Error][AntennaConfigPage StaticLoad] File not found or is not a regular file: "
                << fullAntennaConfigPath << std::endl;
            return {}; // Return empty map
        }
    }
    catch (const fs::filesystem_error &e)
    {
        std::cerr << "[Error][AntennaConfigPage StaticLoad] Filesystem error checking path: "
                  << e.what() << " for path: " << fullAntennaConfigPath << std::endl;
        return {};
    }

    // Derive directory containing the config file for relative paths
    std::string configDir;
    try
    {
        configDir = configPathObj.parent_path().string();
        if (configDir.empty() || !fs::exists(configDir) || !fs::is_directory(configDir))
        {
            std::cerr << "[Warn][AntennaConfigPage StaticLoad] Could not determine valid directory "
                         "from path: "
                      << fullAntennaConfigPath
                      << ". Assuming current directory '.' for relative paths." << std::endl;
            configDir = "."; // Fallback to current directory
        }
    }
    catch (const fs::filesystem_error &e)
    {
        std::cerr << "[Error][AntennaConfigPage StaticLoad] Filesystem error getting parent path: "
                  << e.what() << " for path: " << fullAntennaConfigPath << ". Assuming '.'."
                  << std::endl;
        configDir = ".";
    }

    // --- File Reading and Parsing ---
    std::ifstream file(fullAntennaConfigPath);
    if (!file.is_open())
    {
        std::cerr << "[Error][AntennaConfigPage StaticLoad] Failed to open file: "
                  << fullAntennaConfigPath << std::endl;
        return {}; // Return empty map
    }

    json antennaConfigJSON;
    try
    {
        antennaConfigJSON = json::parse(file);
        file.close(); // Close file as soon as parsing is done
    }
    catch (const nlohmann::json::parse_error &e)
    {
        std::cerr << "[Error][AntennaConfigPage StaticLoad] JSON Parse Error: " << e.what()
                  << " in file: " << fullAntennaConfigPath << std::endl;
        file.close();
        return {}; // Return empty map
    }
    catch (const std::exception &e)
    { // Catch other potential exceptions during parsing/closing
        std::cerr << "[Error][AntennaConfigPage StaticLoad] Exception during JSON parsing/closing: "
                  << e.what() << std::endl;
        if (file.is_open())
            file.close();
        return {};
    }

    // --- Pattern Loading Logic ---
    bool patternsLoadedAtLeastOne = false;
    try
    {
        bool convertPIToDegrees = false; // Reset per file load

        // --- Load CSV Defined Patterns ---
        if (antennaConfigJSON.contains("csvDefined") && antennaConfigJSON["csvDefined"].is_array())
        {
            for (const auto &csvEntry : antennaConfigJSON["csvDefined"])
            {
                if (csvEntry.contains("name") && csvEntry["name"].is_array())
                {
                    for (const auto &name_entry : csvEntry["name"])
                    {
                        if (!name_entry.is_string())
                            continue; // Skip non-string names
                        std::string csvFileName = name_entry.get<std::string>();
                        if (csvFileName == "NONE" || csvFileName.empty())
                            continue;

                        // Construct path relative to the *derived* config directory
                        std::string csvFilePath = configDir + "/" + csvFileName;
                        try
                        { // Filesystem check inside loop
                            if (!fs::exists(csvFilePath) || !fs::is_regular_file(csvFilePath))
                            {
                                std::cerr << "[Warn][AntennaConfigPage StaticLoad] CSV file not "
                                             "found or not regular file: "
                                          << csvFilePath << " (referenced in "
                                          << fullAntennaConfigPath << ")" << std::endl;
                                continue;
                            }
                        }
                        catch (const fs::filesystem_error &e)
                        {
                            std::cerr << "[Warn][AntennaConfigPage StaticLoad] Filesystem error "
                                         "checking CSV path: "
                                      << e.what() << " for path: " << csvFilePath << std::endl;
                            continue;
                        }

                        std::ifstream csvFile(csvFilePath);
                        if (!csvFile.is_open())
                        {
                            std::cerr
                                << "[Warn][AntennaConfigPage StaticLoad] Cannot open CSV file: "
                                << csvFilePath << std::endl;
                            continue;
                        }

                        radiationPattern pattern;
                        std::string line;
                        double theta = 0.0, gain = 0.0;
                        int lineCount = 0;
                        int validLineCount = 0;     // Count lines with valid data pairs
                        convertPIToDegrees = false; // Check PI conversion per CSV

                        while (getline(csvFile, line))
                        {
                            lineCount++;
                            if (line.empty() || line[0] == '#')
                                continue; // Skip comments/empty lines

                            std::stringstream ss(line);
                            std::string thetaStr, gainStr;
                            if (!getline(ss, thetaStr, ',') || !getline(ss, gainStr, ','))
                            {
                                std::cerr << "[Warn][AntennaConfigPage StaticLoad] Bad CSV format "
                                             "on line "
                                          << lineCount << " in " << csvFileName
                                          << ". Expected at least two comma-separated values."
                                          << std::endl;
                                continue;
                            }
                            try
                            {
                                // Trim whitespace before conversion
                                thetaStr.erase(0, thetaStr.find_first_not_of(" \t\n\r\f\v"));
                                thetaStr.erase(thetaStr.find_last_not_of(" \t\n\r\f\v") + 1);
                                gainStr.erase(0, gainStr.find_first_not_of(" \t\n\r\f\v"));
                                gainStr.erase(gainStr.find_last_not_of(" \t\n\r\f\v") + 1);

                                theta = std::stod(thetaStr);
                                gain = std::stod(gainStr); // gain is in dBi from the file
                            }
                            catch (const std::invalid_argument &e)
                            {
                                std::cerr << "[Warn][AntennaConfigPage StaticLoad] Invalid number "
                                             "format on line "
                                          << lineCount << " in " << csvFileName << ": '" << thetaStr
                                          << "' or '" << gainStr << "'" << std::endl;
                                continue;
                            }
                            catch (const std::out_of_range &e)
                            {
                                std::cerr << "[Warn][AntennaConfigPage StaticLoad] Number out of "
                                             "range on line "
                                          << lineCount << " in " << csvFileName << std::endl;
                                continue;
                            }

                            // Check for PI conversion on first valid data line
                            if (validLineCount == 0 && std::abs(theta) <= (2.0 * M_PI + 1e-9))
                            { // Allow small tolerance
                                convertPIToDegrees = true;
                            }
                            if (convertPIToDegrees)
                            {
                                theta = theta * 180.0 / M_PI;
                            }

                            pattern.thetaValues.push_back(theta);
                            pattern.gainValues_linear.push_back(dbiToLinear(gain));
                            validLineCount++;
                        } // end while getline
                        csvFile.close();

                        if (validLineCount > 0)
                        {
                            patterns[currentPatternId] = pattern; // Add to the return map
                            patternsLoadedAtLeastOne = true;
                            currentPatternId++;
                        }
                        else
                        {
                            std::cerr << "[Warn][AntennaConfigPage StaticLoad] No valid data "
                                         "points found in CSV: "
                                      << csvFileName << std::endl;
                        }
                    } // end check NONE/empty
                } // end loop name_entry
            } // end check name array
        } // end loop csvEntry

        // --- Load Param Defined (Gaussian) Patterns ---
        if (antennaConfigJSON.contains("antennaType") && antennaConfigJSON["antennaType"] == "paramDefined")
        {
            if (antennaConfigJSON.contains("paramDefined") && antennaConfigJSON["paramDefined"].is_array())
            {
                for (const auto &paramEntry : antennaConfigJSON["paramDefined"])
                {
                    // Validate entry structure
                    if (!paramEntry.is_object() || !paramEntry.contains("halfpower_beamwidth_degrees") || !paramEntry.contains("maximum_gain"))
                    {
                        std::cerr << "[Warn][AntennaConfigPage StaticLoad] Incomplete Gaussian "
                                     "entry in JSON."
                                  << std::endl;
                        continue;
                    }
                    if (!paramEntry["halfpower_beamwidth_degrees"].is_number() || !paramEntry["maximum_gain"].is_number())
                    {
                        std::cerr << "[Warn][AntennaConfigPage StaticLoad] Non-numeric Gaussian "
                                     "parameters in JSON."
                                  << std::endl;
                        continue;
                    }

                    // Get parameters
                    double hpbw_deg = paramEntry["halfpower_beamwidth_degrees"];
                    double max_gain_db = paramEntry["maximum_gain"];

                    // Validate parameter values
                    if (hpbw_deg <= 0 || !std::isfinite(hpbw_deg) || !std::isfinite(max_gain_db))
                    {
                        std::cerr
                            << "[Warn][AntennaConfigPage StaticLoad] Invalid Gaussian parameter "
                               "values (HPBW="
                            << hpbw_deg << ", Gain=" << max_gain_db
                            << "). HPBW must be > 0 and both finite." << std::endl;
                        continue;
                    }

                    radiationPattern pattern;
                    double hpbw_rad = degreesToRadians(hpbw_deg);         // Use static helper
                    double stddev = calculateStandardDeviation(hpbw_rad); // Use static helper

                    if (stddev <= 1e-9)
                    {
                        std::cerr << "[Warn][AntennaConfigPage StaticLoad] Calculated Gaussian "
                                     "standard deviation is near zero for HPBW="
                                  << hpbw_deg << ", skipping pattern." << std::endl;
                        continue;
                    }

                    // Calculate adjustment factor to match peak gain
                    double max_gain_linear = dbiToLinear(max_gain_db);
                    double pdf_at_zero = (1.0 / (stddev * std::sqrt(2.0 * M_PI))); // Peak of unit Gaussian PDF
                    if (pdf_at_zero <= 1e-12)
                    { // Use smaller epsilon for divisor check
                        std::cerr << "[Warn][AntennaConfigPage StaticLoad] Gaussian PDF at zero is "
                                     "near zero, cannot calculate adjustment factor, skipping."
                                  << std::endl;
                        continue; // Avoid division by zero
                    }
                    double adjustmentFactor = max_gain_linear / pdf_at_zero;
                    if (!std::isfinite(adjustmentFactor) || adjustmentFactor <= 0)
                    {
                        std::cerr << "[Warn][AntennaConfigPage StaticLoad] Calculated Gaussian "
                                     "adjustment factor is invalid ("
                                  << adjustmentFactor << "), skipping pattern." << std::endl;
                        continue;
                    }

                    // Generate points for the Gaussian pattern
                    const int numPoints = 361; // Generate points from -180 to 180 degrees (inclusive)
                    pattern.thetaValues.reserve(numPoints);

                    for (int i = 0; i < numPoints; ++i)
                    {
                        double theta_deg = -180.0 + static_cast<double>(i);
                        double theta_rad = degreesToRadians(theta_deg);
                        pattern.thetaValues.push_back(theta_deg);

                        double exponent = -0.5 * std::pow(theta_rad / stddev, 2);
                        double normDistVal = (1.0 / (stddev * std::sqrt(2.0 * M_PI))) * std::exp(exponent);

                        // Directly calculate and store the linear gain
                        double gain_linear = adjustmentFactor * normDistVal;
                        pattern.gainValues_linear.push_back(
                            std::max(0.0, gain_linear)); // Ensure non-negative
                    }

                    if (!pattern.thetaValues.empty())
                    {
                        patterns[currentPatternId] = pattern; // Add to return map
                        patternsLoadedAtLeastOne = true;
                        currentPatternId++;
                    }
                    else
                    {
                        std::cerr << "[Warn][AntennaConfigPage StaticLoad] Failed to generate any "
                                     "points for a Gaussian pattern."
                                  << std::endl;
                    }
                } // end loop paramEntry
            } // end check paramDefined array
        } // end check antennaType
    } // end csvDefined
    catch (const std::exception &e)
    {
        std::cerr << "[Error][AntennaConfigPage StaticLoad] Exception during pattern processing: "
                  << e.what() << " in file " << fullAntennaConfigPath << std::endl;
        return {}; // Return empty map on exception
    }

    // Final success/failure check
    if (!patternsLoadedAtLeastOne)
    {
        std::cerr
            << "[Warn][AntennaConfigPage StaticLoad] No valid antenna patterns found or loaded "
               "from file: "
            << fullAntennaConfigPath << std::endl;
        return {}; // Return empty map
    }

    return patterns;
}

ConfigFile AntennaConfigPage::assignLoadedPatternsToConfig_static(
    ConfigFile inputConfig, // Take by value to modify a copy safely
    const std::map<int, radiationPattern> &loadedPatterns,
    int unitIndex)
{
    if (loadedPatterns.empty())
    {
        // This should be checked before calling, but handle defensively
        std::cerr << "[Error][AntennaConfigPage StaticAssign] Attempted assignment with empty "
                     "pattern map!"
                  << std::endl;
        return inputConfig; // Return original config
    }

    // Always use the first loaded pattern (lowest ID in the map's natural ordering)
    const radiationPattern &patternToAssign = loadedPatterns.begin()->second;
    const int targetPatternId = 0;

    if (unitIndex == -1)
    { // Assign to ALL units
        if (inputConfig.engine_units.empty())
        {
            std::cerr << "[Warn][AntennaConfigPage StaticAssign] No engine units defined in config "
                         "to assign patterns to (Assigning to ALL)."
                      << std::endl;
            // Continue, maybe config is partially formed
        }

        for (engineUnit &unit : inputConfig.engine_units)
        {
            // Assign only if the unit actually uses an antenna
            if (unit.RxAntenna_enabled || unit.TxAntenna_enabled)
            {
                unit.radiationPatterns.clear(); // Clear any previous patterns for this unit
                unit.radiationPatterns[targetPatternId] = patternToAssign;

                unit.radiationPatterns_maxGain_linear.clear(); // Clear previous max gains
                unit.radiationPatterns_hpbw.clear();           // Clear previous HPBW values
                unit.radiationPatterns_maxGain_linear[targetPatternId] = getMaxGainFromPattern(unit.radiationPatterns, targetPatternId);
                unit.radiationPatterns_hpbw[targetPatternId] = getHpbwFromPattern(unit.radiationPatterns, targetPatternId);
            }
            else
            {
                // Ensure disabled units have no patterns assigned
                unit.radiationPatterns.clear();
                unit.radiationPatterns_maxGain_linear.clear();
                unit.radiationPatterns_hpbw.clear();
            }
        }
    }
    else
    { // Assign to a specific unit index
        if (unitIndex < 0 || static_cast<size_t>(unitIndex) >= inputConfig.engine_units.size())
        {
            std::cerr << "[Error][AntennaConfigPage StaticAssign] Invalid unit index (" << unitIndex
                      << ") specified. Max index is " << inputConfig.engine_units.size() - 1 << "."
                      << std::endl;
            return inputConfig; // Return original config on invalid index
        }

        engineUnit &targetUnit = inputConfig.engine_units[unitIndex];

        // Assign only if the unit actually uses an antenna
        if (targetUnit.RxAntenna_enabled || targetUnit.TxAntenna_enabled)
        {
            targetUnit.radiationPatterns.clear(); // Clear any previous patterns
            targetUnit.radiationPatterns[targetPatternId] = patternToAssign;

            targetUnit.radiationPatterns_maxGain_linear.clear(); // Clear previous max gains
            targetUnit.radiationPatterns_hpbw.clear();           // Clear previous HPBW values
            targetUnit.radiationPatterns_maxGain_linear[targetPatternId] = getMaxGainFromPattern(targetUnit.radiationPatterns, targetPatternId);
            targetUnit.radiationPatterns_hpbw[targetPatternId] = getHpbwFromPattern(targetUnit.radiationPatterns, targetPatternId);
        }
        else
        {
            std::cout << "[Warn][AntennaConfigPage StaticAssign] Target unit '" << targetUnit.label
                      << "' has antennas disabled, clearing patterns." << std::endl;
            targetUnit.radiationPatterns.clear(); // Ensure disabled units have no patterns
            targetUnit.radiationPatterns_maxGain_linear.clear();
            targetUnit.radiationPatterns_hpbw.clear();
        }
    }

    return inputConfig; // Return the modified config
}

std::optional<ConfigFile> AntennaConfigPage::processAntennaConfiguration(
    const std::string &fullAntennaConfigPath, ConfigFile &inputConfig)
{
    // Step 1: Check if file exists
    QFileInfo fileInfo(QString::fromStdString(fullAntennaConfigPath));
    if (!fileInfo.exists() || !fileInfo.isFile())
    {
        qWarning() << "  Error: Antenna config file does not exist:"
                   << QString::fromStdString(fullAntennaConfigPath);
        return std::nullopt;
    }

    // Step 2: Load patterns using the updated static helper
    std::map<int, radiationPattern> loadedPatterns = loadAntennaPatternsFromFile_static(
        fullAntennaConfigPath);

    // Step 3: Check if loading was successful (map is not empty)
    if (loadedPatterns.empty())
    {
        // Error already logged by the loader function
        std::cerr
            << "[Error][AntennaConfigPage StaticProcess] Failed: Could not load valid patterns."
            << std::endl;
        return std::nullopt; // Return empty optional
    }

    // Step 4: Assign the loaded patterns using the static helper
    //          (Assigns the first loaded pattern (lowest ID) to all enabled units)
    ConfigFile updatedConfig = assignLoadedPatternsToConfig_static(inputConfig, loadedPatterns, -1);

    // Step 5: Check if all enabled units now have the required pattern
    bool assignmentOk = true;
    for (engineUnit &unit : updatedConfig.engine_units)
    {
        if (unit.RxAntenna_enabled || unit.TxAntenna_enabled)
        {
            // Check if the map is empty OR if the specific target ID (0) is missing
            if (unit.radiationPatterns.empty() || unit.radiationPatterns.find(0) == unit.radiationPatterns.end())
            {
                std::cerr << "[Error][AntennaConfigPage StaticProcess] Post-assignment validation "
                             "failed: Unit '"
                          << unit.label << "' requires an antenna but pattern ID 0 is missing."
                          << std::endl;
                assignmentOk = false;
                // Don't break, report all missing ones if desired
            }
            else
            {
                unit.antenna = fileInfo.fileName().toStdString(); // Store the antenna file name
            }
        }
    }

    if (!assignmentOk)
    {
        return std::nullopt; // Return empty optional if assignment validation failed
    }

    return updatedConfig; // Return the updated config wrapped in optional
}

AntennaConfigPage::AntennaConfigPage(QWidget *parent)
    : QWidget(parent) // Call base class constructor
{
    setupUI();                       // Create UI elements
    setFocusPolicy(Qt::StrongFocus); // Allow widget to receive key events
}

// Set the config file for the UI instance to work with
void AntennaConfigPage::setParamConfig(const ConfigFile &config)
{
    paramConfigFile = config;     // Store copy for UI modifications
    populateUnitSelectionCombo(); // Update dropdown based on new config's units

    loadedPatternsMap.clear();    // Clear previously loaded patterns for this UI instance
    clearLoadedDataAndUpdateUI(); // Clear the UI list and the graph

    nextButton->setEnabled(false); // Disable 'Next' until patterns are loaded for this config
    refreshConfigFiles();          // Refresh list of available antenna JSON files
}

void AntennaConfigPage::setupUI()
{
    // --- Main Vertical Layout ---
    mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);

    // --- Header Section ---
    headerLayout = new QHBoxLayout;
    QLabel *titleLabel = new QLabel("Antenna Config", this);
    titleLabel->setStyleSheet("font-size: 42px; font-weight: bold; padding-top: 25px");
    headerLayout->addWidget(titleLabel, 0, Qt::AlignLeft | Qt::AlignBottom);
    headerLayout->addStretch(1);
    QLabel *kthlogo = new QLabel(this); // Add logos...
    QPixmap kthPixmap(":/ICONS/kth.png");
    if (!kthPixmap.isNull())
        kthlogo->setPixmap(kthPixmap.scaledToHeight(titleLabel->sizeHint().height() - 25,
                                                    Qt::SmoothTransformation));
    headerLayout->addWidget(kthlogo, 0, Qt::AlignRight | Qt::AlignBottom);
    QLabel *neuLogo = new QLabel(this);
    QPixmap neuPixmap(":/ICONS/neu.png");
    if (!neuPixmap.isNull())
        neuLogo->setPixmap(neuPixmap.scaledToHeight(titleLabel->sizeHint().height() - 25,
                                                    Qt::SmoothTransformation));
    headerLayout->addWidget(neuLogo, 0, Qt::AlignRight | Qt::AlignBottom);
    QLabel *unlabLogo = new QLabel(this);
    QPixmap unlabPixmap(":/ICONS/unlab.png");
    if (!unlabPixmap.isNull())
        unlabLogo->setPixmap(unlabPixmap.scaledToHeight(titleLabel->sizeHint().height() - 25,
                                                        Qt::SmoothTransformation));
    headerLayout->addWidget(unlabLogo, 0, Qt::AlignRight | Qt::AlignBottom);

    mainLayout->addLayout(headerLayout);

    // --- Config and Unit Selection Frame ---
    selectionFrame = new QFrame(this);
    selectionFrame->setFrameShape(QFrame::StyledPanel);
    selectionFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *frameLayout = new QVBoxLayout(selectionFrame);
    frameLayout->setContentsMargins(10, 10, 10, 10);
    frameLayout->setSpacing(10);

    // -- Horizontal Layout for Dropdowns --
    selectionLayout = new QHBoxLayout();
    selectionLayout->setSpacing(15);

    // Antenna Config Selection
    QVBoxLayout *configVLayout = new QVBoxLayout();
    QLabel *selectLabel = new QLabel("Antenna Config File:", selectionFrame);
    selectLabel->setStyleSheet("font-size: 18px; font-weight: bold; border-bottom: 2px solid "
                               "#3a3a3a; border-radius: 0px; padding-bottom: 5px; color: white;");
    configVLayout->addWidget(selectLabel);
    configFileCombo = new QComboBox(selectionFrame);
    configFileCombo->setStyleSheet(
        "QComboBox { font-size: 16px; background-color: #1f1f1f; border: 1px solid #C8102E;"
        " border-radius: 3px; color: white; padding-left: 5px; min-height: 25px; }"
        "QComboBox::drop-down { border: 0px; } "
        "QComboBox::down-arrow { image: url(:/ICONS/down_arrow.png); width: 14px; height: 14px; "
        "padding-right: 5px;} "
        "QComboBox:hover { background-color: #454545; }"
        "QComboBox:on { border-bottom-left-radius: 0px; border-bottom-right-radius: 0px; }"
        "QComboBox QAbstractItemView { background-color: #1f1f1f; border: 2px solid #C8102E; "
        "selection-background-color: #C8102E; color: white; }");
    configVLayout->addWidget(configFileCombo);
    configVLayout->addStretch(1);
    selectionLayout->addLayout(configVLayout, 3);

    // Unit Selection
    QVBoxLayout *unitVLayout = new QVBoxLayout();
    QLabel *unitSelectLabel = new QLabel("Assign To Unit:", selectionFrame);
    unitSelectLabel->setStyleSheet(
        "font-size: 18px; font-weight: bold; border-bottom: 2px solid #3a3a3a; border-radius: 0px; "
        "padding-bottom: 5px; color: white;");
    unitVLayout->addWidget(unitSelectLabel);
    unitSelectionCombo = new QComboBox(selectionFrame);
    unitSelectionCombo->setStyleSheet(
        "QComboBox { font-size: 16px; background-color: #1f1f1f; border: 1px solid #3a3a3a;"
        " border-radius: 3px; color: white; padding-left: 5px; min-height: 25px; }"
        "QComboBox::drop-down { border: 0px; } "
        "QComboBox::down-arrow { image: url(:/ICONS/down_arrow.png); width: 14px; height: 14px; "
        "padding-right: 5px;} "
        "QComboBox:hover { background-color: #454545; }"
        "QComboBox:on { border-bottom-left-radius: 0px; border-bottom-right-radius: 0px; }"
        "QComboBox QAbstractItemView { background-color: #1f1f1f; border: 2px solid #3a3a3a; "
        "selection-background-color: #555555; color: white;}");
    unitVLayout->addWidget(unitSelectionCombo);
    unitVLayout->addStretch(1);
    selectionLayout->addLayout(unitVLayout, 2);

    frameLayout->addLayout(selectionLayout);
    mainLayout->addWidget(selectionFrame);

    // --- Chart ---
    chart = new QChart();
    chart->setTitle("Antenna Radiation Pattern");
    chart->setTheme(QChart::ChartThemeDark);
    chart->setBackgroundBrush(QBrush(QColor(0x1f1f1f)));
    chart->setPlotAreaBackgroundBrush(QBrush(QColor(0x2a2a2a)));
    chart->setPlotAreaBackgroundVisible(true);
    chart->setTitleBrush(QBrush(Qt::white));
    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignBottom);
    chart->legend()->setLabelColor(Qt::white);
    chart->setMargins(QMargins(5, 5, 5, 5));
    chart->layout()->setContentsMargins(0, 0, 0, 0);

    chartView = new QChartView(chart, this);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setMinimumHeight(300);
    chartView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    mainLayout->addWidget(chartView, 1);

    // --- Back/Next Buttons ---
    navButtonLayout = new QHBoxLayout;
    backButton = new QPushButton("Back", this);
    backButton->setStyleSheet("font-size: 16px; padding: 10px; background-color: #3a3a3a; color: "
                              "white; border: none; border-radius: 5px;");
    backButton->setFixedWidth(75);

    nextButton = new QPushButton("Next", this);
    nextButton->setStyleSheet("QPushButton {font-size: 16px; padding: 10px; background-color: "
                              "#C8102E; color: white; border: none; border-radius: 5px;}"
                              "QPushButton:disabled { background-color: #555; color: #aaa; }");
    nextButton->setEnabled(false);

    navButtonLayout->addWidget(backButton);
    navButtonLayout->addWidget(nextButton);
    navButtonLayout->setStretchFactor(backButton, 0);
    navButtonLayout->setStretchFactor(nextButton, 1);
    mainLayout->addLayout(navButtonLayout);

    // --- Connections ---
    connect(backButton, &QPushButton::clicked, this, &AntennaConfigPage::backRequested);

    // Next button: Assign based on current UI selection, check validity, then emit
    connect(nextButton, &QPushButton::clicked, this, [this]()
            {
                assignPatternsToConfig(); // Ensure internal config reflects UI selection
                if (checkPatterns())
                {                                              // Validate the internal config
                    emit nextRequested(this->paramConfigFile); // Emit the updated config
                }
                // If checkPatterns() fails, it shows a QMessageBox
            });

    // Combo box changes trigger loading or assignment
    connect(configFileCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &AntennaConfigPage::loadAntennaConfig); // Load new file data
    connect(unitSelectionCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &AntennaConfigPage::assignPatternsToConfig); // Re-assign current patterns to new unit selection
}

void AntennaConfigPage::refreshConfigFiles()
{
    configFileCombo->blockSignals(true); // Prevent triggering loadAntennaConfig during refresh
    configFileCombo->clear();

    std::string path_str = std::string(CONFIG_ANTENNA_DIR); // Ensure macro is defined
    fs::path antenna_dir_path(path_str);

    if (!fs::exists(antenna_dir_path) || !fs::is_directory(antenna_dir_path))
    {
        std::cerr << "[Error] Antenna config directory not found or invalid: " << path_str
                  << std::endl;
        QMessageBox::critical(this,
                              "Directory Error",
                              "Antenna configuration directory not found:\n" + QString::fromStdString(path_str));
        configFileCombo->blockSignals(false);
        return;
    }

    std::vector<std::string> files;
    try
    {
        for (const auto &entry : fs::directory_iterator(antenna_dir_path))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".json")
            {
                files.push_back(entry.path().filename().string());
            }
        }
    }
    catch (const fs::filesystem_error &e)
    {
        std::cerr << "[Error] Filesystem error reading antenna directory: " << e.what()
                  << std::endl;
        QMessageBox::warning(this, "Directory Warning", "Could not fully read antenna directory.");
        // Continue with potentially partial list
    }

    std::sort(files.begin(), files.end()); // Sort alphabetically

    if (files.empty())
    {
        std::cout << "[Error] No .json antenna config files found in " << path_str << std::endl;
    }
    else
    {
        for (const auto &filename : files)
        {
            configFileCombo->addItem(QString::fromStdString(filename));
        }
    }

    configFileCombo->blockSignals(false); // Re-enable signals

    // Automatically select and load the first file if available
    if (configFileCombo->count() > 0)
    {
        configFileCombo->setCurrentIndex(
            0);              // Select first item, automatically triggers loadAntennaConfig via connection
        loadAntennaConfig(); // Load the first file
    }
    else
    {
        // No files found, ensure UI is clear
        loadedPatternsMap.clear();
        clearLoadedDataAndUpdateUI();
        nextButton->setEnabled(false);
    }
}

void AntennaConfigPage::populateUnitSelectionCombo()
{
    unitSelectionCombo->blockSignals(true);
    unitSelectionCombo->clear();
    unitSelectionCombo->addItem("All Units", QVariant(-1)); // UserData -1 signifies all

    if (paramConfigFile.engine_units.empty())
    {
        std::cerr << "[Warn][AntennaConfigPage] populateUnitSelectionCombo: No engine units "
                     "defined in paramConfigFile."
                  << std::endl;
    }
    else
    {
        for (int i = 0; i < paramConfigFile.engine_units.size(); ++i)
        {
            const auto &unit = paramConfigFile.engine_units[i];
            // Store the vector index 'i' as UserData associated with the label
            unitSelectionCombo->addItem(QString::fromStdString(unit.label), QVariant(i));
        }
    }
    unitSelectionCombo->setCurrentIndex(0); // Default selection to "All Units"
    unitSelectionCombo->blockSignals(false);
}

void AntennaConfigPage::clearLoadedDataAndUpdateUI()
{
    // 1. Update the graph (which will now be empty if loadedPatternsMap is empty)
    updateGraph();
}

void AntennaConfigPage::loadAntennaConfig()
{
    // 1. Clear previous UI instance data first
    loadedPatternsMap.clear();
    clearLoadedDataAndUpdateUI();  // Clears UI list and graph
    nextButton->setEnabled(false); // Disable until load succeeds

    // 2. Get selected file name
    QString selectedFileName = configFileCombo->currentText();
    if (selectedFileName.isEmpty())
    {
        std::cerr << "[Error] No antenna config file selected." << std::endl;
        return; // Nothing to load
    }

    // 3. Construct the full path
    std::string fullAntennaPath = std::string(CONFIG_ANTENNA_DIR) + "/" + selectedFileName.toStdString();

    // 4. Call the static loader function
    std::map<int, radiationPattern> patterns = loadAntennaPatternsFromFile_static(fullAntennaPath);

    // 5. Process the result
    if (!patterns.empty())
    { // Check if loading succeeded (map is not empty)

        // Store results in the member variable for this UI instance
        this->loadedPatternsMap = patterns;

        // Automatically assign the loaded pattern based on current unit selection (for preview)
        assignPatternsToConfig();

        this->paramConfigFile.antennaConfig_name = selectedFileName.toStdString(); // Store the name in the config file

        updateGraph();                // Update graph display using member map
        nextButton->setEnabled(true); // Enable 'Next' as patterns are loaded
    }
    else
    {
        // Error should have been logged by the static function
        std::cerr << "[Error] Loading antenna config failed (see previous logs)." << std::endl;
        QMessageBox::critical(this,
                              "Error Loading Antenna File",
                              "Failed to load valid antenna patterns from:\n" + selectedFileName + "\nPlease check the file format and ensure referenced CSV "
                                                                                                   "files exist.\n(See console/log for details)");
        // Ensure graph is empty and button remains disabled
        updateGraph();
        nextButton->setEnabled(false);
    }
}

void AntennaConfigPage::assignPatternsToConfig()
{
    // This function modifies the UI instance's internal `paramConfigFile`
    if (this->loadedPatternsMap.empty())
    {
        // If called when no patterns are loaded (e.g., unit selection change after load failure), do nothing.
        return;
    }

    int selectedUnitIndex = unitSelectionCombo->currentData()
                                .toInt(); // -1 for ALL, 0+ for specific index

    // Call the static assigner, passing the *member* config and *member* loaded patterns map.
    // The static function returns a *modified copy*, so update the member variable.
    this->paramConfigFile = assignLoadedPatternsToConfig_static(this->paramConfigFile,   // Pass current UI config state
                                                                this->loadedPatternsMap, // Pass patterns loaded by UI
                                                                selectedUnitIndex);      // Pass UI unit selection
}

void AntennaConfigPage::updateGraph()
{
    if (!chart || !chartView)
    {
        std::cerr << "[Error] updateGraph called before chart/chartView initialized!" << std::endl;
        return;
    }

    chart->removeAllSeries(); // Clear previous data

    // --- Axis Setup (ensure axes exist) ---
    QAbstractAxis *axisX = nullptr;
    QAbstractAxis *axisY = nullptr;

    if (chart->axes(Qt::Horizontal).isEmpty())
    {
        axisX = new QValueAxis(chart); // Parent chart for auto-deletion
        chart->addAxis(axisX, Qt::AlignBottom);
    }
    else
    {
        axisX = chart->axes(Qt::Horizontal).first();
    }

    if (chart->axes(Qt::Vertical).isEmpty())
    {
        axisY = new QValueAxis(chart); // Parent chart
        chart->addAxis(axisY, Qt::AlignLeft);
    }
    else
    {
        axisY = chart->axes(Qt::Vertical).first();
    }

    // --- Configure Axes (Titles, Range, Appearance) ---
    axisX->setTitleText("Theta (degrees)");
    axisY->setTitleText("Gain (dBi)");

    // Try to cast to QValueAxis for range setting etc.
    QValueAxis *valueAxisX = qobject_cast<QValueAxis *>(axisX);
    QValueAxis *valueAxisY = qobject_cast<QValueAxis *>(axisY);

    if (valueAxisX)
        valueAxisX->setRange(-180, 180);
    // Determine Y range dynamically or use fixed range
    if (valueAxisY)
        valueAxisY->setRange(-50, 30);

    axisX->setLabelsColor(Qt::white);
    axisX->setTitleBrush(Qt::white);
    axisY->setLabelsColor(Qt::white);
    axisY->setTitleBrush(Qt::white);

    // Grid line styling
    axisX->setGridLineVisible(true);
    axisX->setMinorGridLineVisible(false);
    axisY->setGridLineVisible(true);
    axisY->setMinorGridLineVisible(true);
    QPen axisPen(QColor(80, 80, 80));
    axisPen.setWidth(1);
    QPen minorPen(QColor(50, 50, 50));
    minorPen.setStyle(Qt::DotLine);
    axisX->setLinePen(axisPen);
    axisX->setGridLinePen(axisPen);
    axisY->setLinePen(axisPen);
    axisY->setGridLinePen(axisPen);
    axisY->setMinorGridLinePen(minorPen);

    // --- Plot Data ---
    int seriesIndex = 0;
    for (const auto &pair : this->loadedPatternsMap)
    { // Use member map
        int patternId = pair.first;
        const radiationPattern &pattern = pair.second;

        if (pattern.thetaValues.size() != pattern.gainValues_linear.size())
        {
            std::cerr << "[Warn][AntennaConfigPage UI] Mismatched data points for pattern ID "
                      << patternId << std::endl;
            continue; // Skip this pattern
        }

        QLineSeries *series = new QLineSeries(chart);   // Parent chart for auto-deletion
        series->setName(QString("P%1").arg(patternId)); // Simple name for legend

        bool hasValidPoints = false;
        for (size_t i = 0; i < pattern.thetaValues.size(); ++i)
        {
            double gain_linear = pattern.gainValues_linear[i];
            // Convert linear gain to dBi for plotting
            double gain_dbi = (gain_linear > 1e-10) ? (10.0 * std::log10(gain_linear)) : -100.0;
            series->append(pattern.thetaValues[i], gain_dbi);
            hasValidPoints = true;
        }

        if (hasValidPoints)
        {
            QPen pen;
            pen.setWidth(2);
            // Cycle through some default Qt colors
            pen.setColor(QColor(Qt::GlobalColor(7 + (seriesIndex % 13))));
            series->setPen(pen);

            chart->addSeries(series);

            // Attach series to the existing axes
            series->attachAxis(axisX);
            series->attachAxis(axisY);

            seriesIndex++;
        }
        else
        {
            // If no valid points were added, delete the empty series
            std::cerr << "[Warn][AntennaConfigPage UI] No valid points to plot for pattern ID "
                      << patternId << std::endl;
            delete series;
        }
    }

    chartView->repaint(); // Request a redraw of the chart view
}

bool AntennaConfigPage::checkPatterns()
{
    // This function checks the internal `paramConfigFile` held by the UI instance.
    bool allOk = true;
    QString errorMessages;

    for (const auto &unit : this->paramConfigFile.engine_units)
    {
        // Check only units that are supposed to have an antenna enabled
        if (unit.RxAntenna_enabled || unit.TxAntenna_enabled)
        {
            // Check if the map exists AND contains the target ID (0)
            if (unit.radiationPatterns.empty() || unit.radiationPatterns.find(0) == unit.radiationPatterns.end())
            {
                QString msg = QString("Unit '%1' requires an antenna but pattern ID 0 is missing "
                                      "or unassigned.\n")
                                  .arg(QString::fromStdString(unit.label));
                std::cerr << "[Error][AntennaConfigPage Check] " << msg.toStdString();
                errorMessages += msg;
                allOk = false;
                // Don't return immediately, collect all errors
            }
        }
    }

    if (!allOk)
    {
        QMessageBox::warning(this,
                             "Pattern Assignment Incomplete",
                             "Please ensure patterns are correctly assigned:\n" + errorMessages);
    }

    return allOk;
}

double AntennaConfigPage::getMaxGainFromPattern(const std::map<int, radiationPattern> &patterns_map,
                                                int pattern_id)
{
    auto it = patterns_map.find(pattern_id);
    if (it != patterns_map.end())
    {
        // Use the renamed member variable
        const auto &gain_values_linear = it->second.gainValues_linear;
        if (!gain_values_linear.empty())
        {
            // Find the maximum linear gain
            double max_val = *std::max_element(gain_values_linear.begin(), gain_values_linear.end());
            return max_val; // Return the linear value
        }
    }

    // 3. Handle cases where pattern is not found or gain_values is empty
    const double MIN_GAIN_LINEAR_ERROR = -9999999; // Or some other suitable error value
    std::cerr << "kpiClassifier Warning: Could not find max gain for pattern ID " << pattern_id
              << " or pattern has no gain values. Returning -inf linear." << std::endl;
    return MIN_GAIN_LINEAR_ERROR;
}

double AntennaConfigPage::getHpbwFromPattern(const std::map<int, radiationPattern> &patterns_map,
                                             int pattern_id)
{
    // 1. Find the pattern and perform basic validation
    auto it = patterns_map.find(pattern_id);
    if (it == patterns_map.end())
    {
        std::cerr << "[Error][getHpbw] Pattern ID " << pattern_id << " not found." << std::endl;
        return -1.0;
    }

    const auto &pattern = it->second;
    if (pattern.thetaValues.size() < 3 || pattern.thetaValues.size() != pattern.gainValues_linear.size())
    {
        std::cerr << "[Warn][getHpbw] Pattern ID " << pattern_id
                  << " has insufficient or mismatched data points. Cannot calculate HPBW."
                  << std::endl;
        return -1.0;
    }

    // 2. To handle unsorted data robustly, create pairs and sort by theta
    std::vector<std::pair<double, double>> sorted_pattern;
    sorted_pattern.reserve(pattern.thetaValues.size());
    for (size_t i = 0; i < pattern.thetaValues.size(); ++i)
    {
        if (std::isfinite(pattern.thetaValues[i]) && std::isfinite(pattern.gainValues_linear[i]))
        {
            sorted_pattern.push_back({pattern.thetaValues[i], pattern.gainValues_linear[i]});
        }
    }
    if (sorted_pattern.size() < 3)
        return -1.0; // Not enough valid points
    std::sort(sorted_pattern.begin(), sorted_pattern.end());

    // 3. Find the maximum gain and its index in the sorted data
    auto max_it = std::max_element(sorted_pattern.begin(),
                                   sorted_pattern.end(),
                                   [](const auto &a, const auto &b)
                                   { return a.second < b.second; });

    double maxGain_linear = max_it->second;
    size_t peak_index = std::distance(sorted_pattern.begin(), max_it);
    double targetGain_linear = maxGain_linear / 2.0;

    // 4. Find the two half-power angles using interpolation
    std::optional<double> theta1, theta2;

    // Search right of the peak (increasing angle)
    for (size_t i = peak_index + 1; i < sorted_pattern.size(); ++i)
    {
        if (sorted_pattern[i].second < targetGain_linear && sorted_pattern[i - 1].second >= targetGain_linear)
        {
            const auto &p_prev = sorted_pattern[i - 1]; // Point before crossing
            const auto &p_curr = sorted_pattern[i];     // Point after crossing
            if (std::abs(p_curr.second - p_prev.second) > 1e-9)
            { // Avoid division by zero
                // Linear interpolation: y = y1 + m(x - x1) -> x = x1 + (y - y1)/m
                theta1 = p_prev.first + (targetGain_linear - p_prev.second) * (p_curr.first - p_prev.first) / (p_curr.second - p_prev.second);
            }
            break; // Found the first point, stop searching
        }
    }

    // Search left of the peak (decreasing angle)
    for (size_t i = peak_index; i > 0; --i)
    {
        if (sorted_pattern[i - 1].second < targetGain_linear && sorted_pattern[i].second >= targetGain_linear)
        {
            const auto &p_prev = sorted_pattern[i];     // Point before crossing
            const auto &p_curr = sorted_pattern[i - 1]; // Point after crossing
            if (std::abs(p_curr.second - p_prev.second) > 1e-9)
            {
                theta2 = p_prev.first + (targetGain_linear - p_prev.second) * (p_curr.first - p_prev.first) / (p_curr.second - p_prev.second);
            }
            break; // Found the second point, stop searching
        }
    }

    // 5. Calculate and return the beamwidth
    if (theta1 && theta2)
    {
        return std::abs(*theta1 - *theta2);
    }

    // If we reach here, one or both points were not found
    std::cerr << "[Warn][getHpbw] Could not determine HPBW for pattern ID " << pattern_id
              << ". The pattern may not drop by 3dB from its peak." << std::endl;
    return -1.0;
}

void AntennaConfigPage::keyPressEvent(QKeyEvent *event)
{
    // Check if Enter/Return key was pressed and the Next button is usable
    if ((event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter))
    {
        if (nextButton && nextButton->isEnabled())
        {
            // button click logic: assign, check, emit
            assignPatternsToConfig(); // Ensure current UI selection is applied internally
            if (checkPatterns())
            {                                              // Validate
                emit nextRequested(this->paramConfigFile); // Emit if valid
            }
            event->accept(); // Mark event as handled
            return;          // Stop further processing
        }
        event->accept(); // Still handle it to prevent other actions
        return;
    }

    // Call base class implementation for default handling (e.g., Tab navigation)
    QWidget::keyPressEvent(event);
}
