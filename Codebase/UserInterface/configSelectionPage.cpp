#include "configSelectionPage.h"
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QKeyEvent>
#include <QLabel>
#include <QMessageBox>
#include <QNetworkInterface>
#include <QPushButton>
#include <QScrollArea>
#include <QTimer>
#include <QTreeWidgetItemIterator>
#include <QVBoxLayout>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <tuple>

using json = nlohmann::json;

namespace
{ // Anonymous namespace to keep helper local
    inline double dBmToWatts(double dBm)
    {
        // Converts dBm to absolute power in Watts
        return 0.001 * std::pow(10.0, dBm / 10.0);
    }
} // namespace

ConfigSelectionPage::ConfigSelectionPage(QWidget *parent)
    : QWidget(parent)
{
    setupUI();
    // Refresh list initially, then load the first item if available
    QTimer::singleShot(200, this, &ConfigSelectionPage::refreshConfigFiles);
    setFocusPolicy(Qt::StrongFocus);
}

void ConfigSelectionPage::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter)
    {
        if (nextButton->isEnabled())
        {
            emit configSelected(this->configFile);
        }
    }
    else
    {
        QWidget::keyPressEvent(event);
    }
}

// --- Template Helper ---
template <typename T>
std::string getTypeName()
{
    return typeid(T).name();
}
// --- End Template Helper ---

// --- Static Validation Implementation ---
template <typename T>
bool ConfigSelectionPage::checkJsonValue(const json &j,
                                         const std::string &key,
                                         std::string &errorMessages)
{
    if (!j.contains(key))
    {
        errorMessages += "  - Missing key: '" + key + "'. Expected type similar to: " + getTypeName<T>() + ".\n";
        return false;
    }
    try
    {
        // Special handling for boolean stored as number in JSON (e.g., 0 or 1)
        if constexpr (std::is_same_v<T, bool>)
        {
            if (j[key].is_number())
            {
                j[key].get<int>(); // Check if it's a number first
            }
            else
            {
                j[key].get<bool>(); // Then try direct bool conversion
            }
        }
        else
        {
            j[key].get<T>(); // Attempt the conversion for other types.
        }
    }
    catch (const nlohmann::json::type_error &e)
    {
        errorMessages += "  - Invalid type for key '" + key + "'. Expected: " + getTypeName<T>() + ". Got: " + std::string(j[key].type_name()) + ". Error: " + e.what() + "\n";
        return false; // Type mismatch
    }
    catch (const nlohmann::json::exception &e)
    { // Catch other potential json errors
        errorMessages += "  - Error accessing/converting key '" + key + "': " + e.what() + "\n";
        return false;
    }
    return true; // Key exists and has the correct type (or convertible type for bool)
}

std::string ConfigSelectionPage::getJsonString(const json &j,
                                               const std::string &key,
                                               std::string &errorMessages,
                                               bool toUpper)
{
    if (checkJsonValue<std::string>(j, key, errorMessages))
    {
        std::string value = j[key].get<std::string>();

        // Convert to upper-case if requested
        if (toUpper)
            std::transform(value.begin(), value.end(), value.begin(), ::toupper);

        return value;
    }
    return "";
}

// Helper to safely get a boolean, allowing numbers 0/1
bool ConfigSelectionPage::getJsonBoolSafe(const json &j,
                                          const std::string &key,
                                          bool defaultValue,
                                          std::string &errorMessages)
{
    if (!j.contains(key))
    {
        errorMessages += "  - Missing key: '" + key + "'. Expected type: bool.\n";
        return defaultValue;
    }
    if (j[key].is_boolean())
    {
        return j[key].get<bool>();
    }
    else if (j[key].is_number())
    {
        try
        {
            int val = j[key].get<int>();
            if (val == 0 || val == 1)
            {
                return static_cast<bool>(val);
            }
            else
            {
                errorMessages += "  - Invalid number for bool key '" + key + "'. Expected 0 or 1. Got: " + std::to_string(val) + ".\n";
                return defaultValue;
            }
        }
        catch (const nlohmann::json::type_error &e)
        {
            errorMessages += "  - Invalid type for key '" + key + "'. Expected: bool (or 0/1). Got: " + j[key].type_name() + ".\n";
            return defaultValue;
        }
    }
    else
    {
        errorMessages += "  - Invalid type for key '" + key + "'. Expected: bool (or 0/1). Got: " + j[key].type_name() + ".\n";
        return defaultValue;
    }
}

void ConfigSelectionPage::setupUI()
{
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setSpacing(10);
    QHBoxLayout *headerLayout = new QHBoxLayout();

    QLabel *titleLabel = new QLabel("MobileTHz", this);
    titleLabel->setStyleSheet("font-size: 42px; font-weight: bold; padding-top: 25px");
    headerLayout->addWidget(titleLabel, 0, Qt::AlignLeft | Qt::AlignBottom);

    headerLayout->addStretch(1);

    //--- Logos ---
    QLabel *kthlogo = new QLabel(this);
    QPixmap kthPixmap(":/ICONS/kth.png");
    kthlogo->setPixmap(
        kthPixmap.scaledToHeight(titleLabel->sizeHint().height() - 25, Qt::SmoothTransformation));
    headerLayout->addWidget(kthlogo, 0, Qt::AlignRight | Qt::AlignBottom);

    QLabel *neuLogo = new QLabel(this);
    QPixmap neuPixmap(":/ICONS/neu.png");
    neuLogo->setPixmap(
        neuPixmap.scaledToHeight(titleLabel->sizeHint().height() - 25, Qt::SmoothTransformation));
    headerLayout->addWidget(neuLogo, 0, Qt::AlignRight | Qt::AlignBottom);

    QLabel *unlabLogo = new QLabel(this);
    QPixmap unlabPixmap(":/ICONS/unlab.png");
    unlabLogo->setPixmap(
        unlabPixmap.scaledToHeight(titleLabel->sizeHint().height() - 25, Qt::SmoothTransformation));
    headerLayout->addWidget(unlabLogo, 0, Qt::AlignRight | Qt::AlignBottom);
    //-------------

    layout->addLayout(headerLayout);

    QHBoxLayout *selectionIPLayout = new QHBoxLayout();

    //--- Config Selection Frame ---
    QFrame *configSelectionFrame = new QFrame(this);
    configSelectionFrame->setFrameShape(QFrame::StyledPanel);
    configSelectionFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *configSelectionLayout = new QVBoxLayout(configSelectionFrame);
    configSelectionLayout->setSpacing(10);

    QLabel *selectLabel = new QLabel("Select Config File:", configSelectionFrame);
    selectLabel->setStyleSheet("font-size: 18px; font-weight: bold; border-bottom: 2px solid "
                               "#3a3a3a; border-radius: 0px; padding-bottom: 5px;");
    configSelectionLayout->addWidget(selectLabel);

    configFileCombo = new QComboBox(configSelectionFrame);
    configFileCombo->setStyleSheet("QComboBox {"
                                   "   font-size: 16px;"
                                   "   background-color: #1f1f1f;"
                                   "   border: 1px solid #C8102E;"
                                   "   border-radius: 3px;"
                                   "   color: white;"
                                   "   padding-left: 5px;"
                                   "}"
                                   "QComboBox:hover {"
                                   "   background-color: #454545;"
                                   "}"
                                   "QComboBox:on {"
                                   "   border-bottom-left-radius: 0px;"
                                   "   border-bottom-right-radius: 0px;"
                                   "}"
                                   "QComboBox QAbstractItemView {"
                                   "   background-color: #1f1f1f;"
                                   "   border: 2px solid #C8102E;"
                                   "}");
    configSelectionLayout->addWidget(configFileCombo);
    selectionIPLayout->addWidget(configSelectionFrame, 3);
    //---------------------------

    //--- IP Frame ---
    QFrame *ipFrame = new QFrame(this);
    ipFrame->setFrameShape(QFrame::StyledPanel);
    ipFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *ipLayout = new QVBoxLayout(ipFrame);
    ipLayout->setSpacing(10);

    QLabel *ipLabel = new QLabel("IP Address:", ipFrame);
    ipLabel->setStyleSheet("font-size: 18px; font-weight: bold; border-bottom: 2px solid #3a3a3a; "
                           "border-radius: 0px; padding-bottom: 5px;");
    ipLayout->addWidget(ipLabel);

    ipAddressLineEdit = new QLineEdit(ipFrame);
    ipAddressLineEdit->setReadOnly(true);
    ipAddressLineEdit->setStyleSheet("QLineEdit {"
                                     "   font-size: 16px;"
                                     "   background-color: #1f1f1f;"
                                     "   border: 1px solid #1f1f1f;"
                                     "   border-radius: 5px;"
                                     "   color: white;"
                                     "   padding-left: 5px;"
                                     "}");
    ipLayout->addWidget(ipAddressLineEdit);

    QString ipAddress;
    for (const QHostAddress &address : QNetworkInterface::allAddresses())
    {
        if (address.protocol() == QAbstractSocket::IPv4Protocol && address != QHostAddress::LocalHost)
        {
            ipAddress = address.toString();
            break;
        }
    }
    ipAddressLineEdit->setText(ipAddress.isEmpty() ? "No IP found" : ipAddress);

    selectionIPLayout->addWidget(ipFrame, 1);
    //-------------

    layout->addLayout(selectionIPLayout);

    //--- Config Details Frame ---
    QFrame *configFrame = new QFrame(this);
    configFrame->setFrameShape(QFrame::StyledPanel);
    configFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *configLayout = new QVBoxLayout(configFrame);
    configLayout->setSpacing(10);
    QLabel *configLabel = new QLabel("Configuration Details", configFrame);
    configLabel->setStyleSheet("font-size: 18px; font-weight: bold; border-bottom: 2px solid "
                               "#3a3a3a; border-radius: 0px; padding-bottom: 5px;");
    configLayout->addWidget(configLabel);

    QScrollArea *scrollArea = new QScrollArea(configFrame);
    scrollArea->setWidgetResizable(true);
    scrollArea->setStyleSheet(
        "background-color: #1f1f1f; border: none;");
    configTreeWidget = new QTreeWidget(scrollArea);
    configTreeWidget->setHeaderLabels(QStringList() << "Parameter" << "Value");
    configTreeWidget->setStyleSheet("QTreeWidget {"
                                    "   font-size: 14px;"
                                    "   background-color: #1f1f1f;"
                                    "   alternate-background-color: #2a2a2a;"
                                    "   border: none;"
                                    "   color: white;"
                                    "}"
                                    "QTreeWidget::item {"
                                    "    padding: 3px;"
                                    "}"
                                    "QHeaderView::section {"
                                    "    background-color: #3a3a3a;"
                                    "    color: white;"
                                    "    padding: 4px;"
                                    "    border: 1px solid #1f1f1f;"
                                    "}");

    configTreeWidget->header()->setSectionResizeMode(QHeaderView::ResizeToContents);
    configTreeWidget->header()->setStretchLastSection(true);
    configTreeWidget->setAlternatingRowColors(true);

    QPalette treePalette = configTreeWidget->palette();
    treePalette.setColor(QPalette::Base, QColor(31, 31, 31));          // #1f1f1f
    treePalette.setColor(QPalette::AlternateBase, QColor(42, 42, 42)); // #2a2a2a
    treePalette.setColor(QPalette::Text, Qt::white);
    configTreeWidget->setPalette(treePalette);

    configTreeWidget->setEditTriggers(
        QAbstractItemView::NoEditTriggers);

    scrollArea->setWidget(configTreeWidget);
    configLayout->addWidget(scrollArea);
    layout->addWidget(configFrame);
    //-------------------------

    //--- Next Button ---
    nextButton = new QPushButton("Next", this);
    nextButton->setStyleSheet("font-size: 16px; padding: 10px; background-color: #C8102E; color: "
                              "white; border: none; border-radius: 5px;");
    nextButton->setEnabled(false); // Initially disabled until config is loaded and valid
    connect(nextButton, &QPushButton::clicked, this, [this]()
            { emit configSelected(this->configFile); });
    layout->addWidget(nextButton);
    //-----------------

    connect(configFileCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged), // Trigger on selection change
            this,
            &ConfigSelectionPage::loadSelectedConfig);
}

// Slot called when GUI combo box selection changes
void ConfigSelectionPage::refreshConfigFiles()
{
    bool signalsBlocked = configFileCombo->blockSignals(true);
    configFileCombo->clear();

    std::string configDirPath = CONFIG_PARAMETER_DIR;
    if (!std::filesystem::exists(configDirPath) || !std::filesystem::is_directory(configDirPath))
    {
        QMessageBox::warning(this,
                             "Config Directory Error",
                             QString("Configuration directory not found or invalid: %1")
                                 .arg(QString::fromStdString(configDirPath)));
        configFileCombo->blockSignals(signalsBlocked); // Unblock signals even on error
        return;
    }

    std::vector<std::string> files;
    for (const auto &entry : std::filesystem::directory_iterator(configDirPath))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".json")
        {
            files.push_back(entry.path().filename().string());
        }
    }
    std::sort(files.begin(), files.end()); // Sort alphabetically
    for (const auto &filename : files)
    {
        configFileCombo->addItem(QString::fromStdString(filename));
    }

    configFileCombo->blockSignals(signalsBlocked);

    // Trigger loading the first config if available
    if (configFileCombo->count() > 0)
    {
        configFileCombo->setCurrentIndex(0); // Ensure index is 0 before triggering load
        loadSelectedConfig();                // Load the newly selected first item
    }
    else
    {
        configTreeWidget->clear(); // Clear tree if no configs found
        nextButton->setText("No Configs Found");
        nextButton->setStyleSheet(
            "font-size: 16px; padding: 10px; background-color: #555555; color: " // Greyed out style
            "#aaaaaa; border: none; border-radius: 5px;");
        nextButton->setEnabled(false); // Ensure disabled if no configs
    }
}

// Slot to load the config currently selected in the combo box (GUI context)
void ConfigSelectionPage::loadSelectedConfig()
{
    nextButton->setEnabled(false); // Disable while loading/validating
    nextButton->setText("Loading...");
    configTreeWidget->clear(); // Clear previous config details

    QString configName = configFileCombo->currentText();
    if (configName.isEmpty())
    {
        nextButton->setText("Select Config");
        // Handle case where combo box might be empty or selection cleared
        return;
    }
    std::string configPath = std::string(CONFIG_PARAMETER_DIR) + "/" + configName.toStdString();

    // --- Call the static loading and validation function ---
    auto [loadedConfig, parsedJson, errorMsg] = loadAndValidateConfigFile(
        QString::fromStdString(configPath));

    if (!errorMsg.isEmpty())
    {
        // Handle errors reported by the static function
        nextButton->setText("Config Error");
        nextButton->setStyleSheet(
            "font-size: 16px; padding: 10px; background-color: #1b1b1b; color: "
            "white; border: none; border-radius: 5px;");
        nextButton->setEnabled(false);
        // Clear the tree and stored data on error
        configTreeWidget->clear();
        this->configJson = json{};
        this->configFile = ConfigFile{};
        emit displayInitialError(errorMsg);
        return;
    }

    // --- Success ---
    this->configFile = loadedConfig;
    this->configJson = parsedJson;                                // Store the parsed JSON for the tree view
    this->configFile.paramConfig_name = configName.toStdString(); // Store the selected name

    populateConfigTree(this->configJson, nullptr); // Populate the tree widget with the parsed JSON
    configTreeWidget->expandAll();
    configTreeWidget->resizeColumnToContents(0);

    // Enable the Next button as validation passed
    nextButton->setText("Next");
    nextButton->setStyleSheet(
        "font-size: 16px; padding: 10px; background-color: #C8102E; color: " // Normal enabled style
        "white; border: none; border-radius: 5px;");
    nextButton->setEnabled(true);
    this->setFocus(); // Allow enter key press right after selection
}

// The public static function for loading/validating
std::tuple<ConfigFile, json, QString> ConfigSelectionPage::loadAndValidateConfigFile(
    const QString &configFilePath)
{
    ConfigFile cf;             // Result config struct
    json parsedJson;           // Result parsed json object
    std::string errorMessages; // Accumulate errors here
    QString qErrorMsg;         // Final Qt error string

    // 1. Check if file exists
    QFileInfo fileInfo(configFilePath);
    if (!fileInfo.exists() || !fileInfo.isFile())
    {
        qErrorMsg = QString("Configuration file not found or is not a file:\n%1").arg(configFilePath);
        return {cf, parsedJson, qErrorMsg};
    }

    // 2. Try to open and read the file
    std::ifstream file(configFilePath.toStdString());
    if (!file.is_open())
    {
        qErrorMsg = QString("Failed to open configuration file:\n%1").arg(configFilePath);
        return {cf, parsedJson, qErrorMsg};
    }

    // 3. Try to parse the JSON
    try
    {
        parsedJson = json::parse(file);
    }
    catch (const nlohmann::json::parse_error &e)
    {
        qErrorMsg = QString("Failed to parse JSON file:\n%1\n\nError: %2\nAt byte: %3")
                        .arg(configFilePath)
                        .arg(e.what())
                        .arg(e.byte);
        file.close();
        return {cf, parsedJson, qErrorMsg}; // Return empty json on parse error too
    }
    catch (const std::exception &e)
    { // Catch other potential errors during parsing
        qErrorMsg = QString("Error processing JSON file:\n%1\n\nError: %2")
                        .arg(configFilePath)
                        .arg(e.what());
        file.close();
        return {cf, parsedJson, qErrorMsg};
    }
    file.close(); // Close file after successful parse

    // 4. Perform Validation
    const json &jsonFile = parsedJson;

    // --- Basic Engine Settings ---
    if (checkJsonValue<std::string>(jsonFile, "engine_mode", errorMessages))
    {
        cf.engine_mode = getJsonString(jsonFile, "engine_mode", errorMessages);
        if (cf.engine_mode != "SIMULATION" && cf.engine_mode != "EXPERIMENTAL")
        {
            errorMessages += "  - Invalid 'engine_mode': Must be 'SIMULATION' or 'EXPERIMENTAL'.\n";
        }
    }
    if (checkJsonValue<long double>(jsonFile, "engine_slot_time_microsec", errorMessages))
        cf.engine_slot_time_microsec = jsonFile["engine_slot_time_microsec"];
    if (checkJsonValue<long double>(jsonFile, "engine_max_time_sec", errorMessages))
        cf.engine_max_time_sec = jsonFile["engine_max_time_sec"];
    if (checkJsonValue<long long>(jsonFile, "engine_carrier_frequency", errorMessages))
    {
        cf.engine_carrier_frequency = jsonFile["engine_carrier_frequency"];
    }
    if (checkJsonValue<long long>(jsonFile, "engine_bandwidth_frequency", errorMessages) && checkJsonValue<long long>(jsonFile, "engine_temperature_kelvin", errorMessages))
    {
        cf.engine_bandwidth_frequency = jsonFile["engine_bandwidth_frequency"];
        cf.engine_temperature_kelvin = jsonFile["engine_temperature_kelvin"];
        // Calculate thermal noise in dBm first
        double thermal_noise_dbm = 10 * log10((cf.engine_bandwidth_frequency * cf.engine_temperature_kelvin * 1.380649e-23) / 1e-3);
        // CONVERT to watts and store
        cf.engine_receiver_thermal_noise_watts = 0.001 * std::pow(10.0, thermal_noise_dbm / 10.0);
    }

    // JIT Packet Settings
    if (checkJsonValue<double>(jsonFile, "JIT_packet_min_snr_db", errorMessages))
        cf.JIT_packet_min_snr_db = jsonFile["JIT_packet_min_snr_db"];
    if (checkJsonValue<double>(jsonFile, "JIT_packet_reception_delay_us", errorMessages))
        cf.JIT_packet_reception_delay_us = jsonFile["JIT_packet_reception_delay_us"];
    // KPI Link Classification
    if (checkJsonValue<double>(jsonFile, "link_min_snr_db", errorMessages))
        cf.link_min_snr_db = jsonFile["link_min_snr_db"];

    if (checkJsonValue<int>(jsonFile, "randomgen_seed", errorMessages))
        cf.randomgen_seed = jsonFile["randomgen_seed"];

    // --- Simulation Preset Position ---
    if (checkJsonValue<bool>(jsonFile, "preset_position_enabled", errorMessages))
    {
        cf.preset_position_enabled = jsonFile["preset_position_enabled"];
    }

    // Temporary vector to build the position sets data
    std::vector<std::vector<std::string>> temp_preset_position_sets;

    if (cf.preset_position_enabled)
    {
        if (jsonFile.contains("preset_position_sets") && jsonFile["preset_position_sets"].is_array())
        {
            for (const auto &set_node : jsonFile["preset_position_sets"])
            {
                if (set_node.is_array())
                {
                    std::vector<std::string> current_set;
                    for (const auto &file_node : set_node)
                    {
                        if (file_node.is_string())
                        {
                            current_set.push_back(file_node.get<std::string>());
                        }
                        else
                        {
                            errorMessages += "  - Invalid non-string type found inside a position "
                                             "set in 'preset_position_sets'.\n";
                        }
                    }
                    if (!current_set.empty())
                    {
                        temp_preset_position_sets.push_back(
                            std::move(current_set));
                    }
                    else
                    {
                        errorMessages += "  - Found an empty position set in 'preset_position_sets'.\n";
                    }
                }
                else
                {
                    errorMessages += "  - Invalid non-array type found in 'preset_position_sets'. "
                                     "Each element must be an array of filenames.\n";
                }
            }
        }
        else
        {
            errorMessages += "  - Missing or invalid 'preset_position_sets' when "
                             "'preset_position_enabled' is true. Expected an array of arrays.\n";
        }
    }

    // After parsing (or if presets are disabled/invalid),
    // create the shared_ptr from the temporary vector.
    cf.preset_position_sets = std::make_shared<const std::vector<std::vector<std::string>>>(
        std::move(temp_preset_position_sets));

    // --- Unit settings ---
    if (jsonFile.contains("engine_unit") && jsonFile["engine_unit"].is_array())
    {
        cf.engine_units.clear();
        int unit_index = 0; // For error reporting
        for (const auto &unit_data : jsonFile["engine_unit"])
        {
            std::string unit_error_prefix = "  - In engine_unit[" + std::to_string(unit_index) + "]:\n";
            std::string current_unit_errors_components; // Errors specific to components section
            std::string current_unit_errors_simulation; // Errors specific to simulation section

            if (!unit_data.is_object())
            {
                errorMessages += unit_error_prefix + "    - Entry is not a JSON object.\n";
                unit_index++;
                continue; // Skip this invalid unit entry
            }

            engineUnit unit;
            // Check label first, useful for error messages
            if (checkJsonValue<std::string>(unit_data,
                                            "label",
                                            errorMessages))
            { // Check directly in main errors
                unit.label = unit_data["label"];
                unit_error_prefix = "  - In engine_unit '" + unit.label + "':\n"; // Use label in errors if available
            }
            else
            {
                unit.label = "Unit " + std::to_string(unit_index); // Default label if missing
            }

            // Components part of engine_unit
            if (unit_data.contains("components") && unit_data["components"].is_object())
            {
                const auto &components = unit_data["components"];
                std::string comp_error_prefix = "    - In components:\n";

                unit.IMU_enabled = ConfigSelectionPage::getJsonBoolSafe(components,
                                                                        "IMU_enabled",
                                                                        false,
                                                                        current_unit_errors_components);

                unit.RxChain_enabled = ConfigSelectionPage::getJsonBoolSafe(components,
                                                                            "RxChain_enabled",
                                                                            false,
                                                                            current_unit_errors_components);

                unit.RxAntenna_enabled = ConfigSelectionPage::getJsonBoolSafe(components,
                                                                              "RxAntenna_enabled",
                                                                              false,
                                                                              current_unit_errors_components);
                unit.TxAntenna_enabled = ConfigSelectionPage::getJsonBoolSafe(components,
                                                                              "TxAntenna_enabled",
                                                                              false,
                                                                              current_unit_errors_components);

                unit.Rotary_enabled = ConfigSelectionPage::getJsonBoolSafe(components,
                                                                           "Rotary_enabled",
                                                                           false,
                                                                           current_unit_errors_components);
                if (unit.Rotary_enabled)
                {
                    if (checkJsonValue<double>(components,
                                               "Rotary_azimuth_velocity_degpersec",
                                               current_unit_errors_components))
                        unit.Rotary_azimuth_velocity_degpersec = components["Rotary_azimuth_velocity_degpersec"];
                    if (checkJsonValue<double>(components,
                                               "Rotary_azimuth_acceleration_degpersecsq",
                                               current_unit_errors_components))
                        unit.Rotary_azimuth_acceleration_degpersecsq = components["Rotary_azimuth_acceleration_degpersecsq"];
                    if (checkJsonValue<double>(components,
                                               "Rotary_altitude_velocity_degpersec",
                                               current_unit_errors_components))
                        unit.Rotary_altitude_velocity_degpersec = components["Rotary_altitude_velocity_degpersec"];
                    if (checkJsonValue<double>(components,
                                               "Rotary_altitude_acceleration_degpersecsq",
                                               current_unit_errors_components))
                        unit.Rotary_altitude_acceleration_degpersecsq = components["Rotary_altitude_acceleration_degpersecsq"];
                }

                if (unit.IMU_enabled)
                {
                    if (checkJsonValue<double>(components,
                                               "IMU_accel_noise_density",
                                               current_unit_errors_components))
                        unit.IMU_accel_noise_density = components["IMU_accel_noise_density"];
                    if (checkJsonValue<double>(components,
                                               "IMU_gyro_noise_density",
                                               current_unit_errors_components))
                        unit.IMU_gyro_noise_density = components["IMU_gyro_noise_density"];
                    if (checkJsonValue<double>(components,
                                               "IMU_accel_initial_bias_mean",
                                               current_unit_errors_components))
                        unit.IMU_accel_initial_bias_mean = components["IMU_accel_initial_bias_mean"];
                    if (checkJsonValue<double>(components,
                                               "IMU_accel_initial_bias_stddev",
                                               current_unit_errors_components))
                        unit.IMU_accel_initial_bias_stddev = components["IMU_accel_initial_bias_stddev"];
                    if (checkJsonValue<double>(components,
                                               "IMU_gyro_initial_bias_mean",
                                               current_unit_errors_components))
                        unit.IMU_gyro_initial_bias_mean = components["IMU_gyro_initial_bias_mean"];
                    if (checkJsonValue<double>(components,
                                               "IMU_gyro_initial_bias_stddev",
                                               current_unit_errors_components))
                        unit.IMU_gyro_initial_bias_stddev = components["IMU_gyro_initial_bias_stddev"];
                    if (checkJsonValue<double>(components,
                                               "IMU_samplingDelay_microsec",
                                               current_unit_errors_components))
                        unit.IMU_samplingDelay_microsec = components["IMU_samplingDelay_microsec"];
                }

                if (unit.RxChain_enabled)
                {
                    if (checkJsonValue<double>(components,
                                               "RxChain_noise_factor",
                                               current_unit_errors_components))
                    {
                        double noise_factor_db = components["RxChain_noise_factor"];
                        unit.RxChain_noise_factor_linear = std::pow(10.0, noise_factor_db / 10.0);
                    }
                    if (checkJsonValue<double>(components,
                                               "RxChain_samplingDelay_microsec",
                                               current_unit_errors_components))
                        unit.RxChain_samplingDelay_microsec = components["RxChain_samplingDelay_microsec"];
                }

                if (unit.TxAntenna_enabled)
                {
                    if (checkJsonValue<long long>(components,
                                                  "TxAntenna_transmitPower_dBm",
                                                  current_unit_errors_components))
                    {
                        long long tx_power_dbm = components["TxAntenna_transmitPower_dBm"];
                        unit.TxAntenna_transmitPower_watts = dBmToWatts(
                            static_cast<double>(tx_power_dbm));
                    }
                }

                if (checkJsonValue<bool>(components,
                                         "preset_antenna_enabled",
                                         current_unit_errors_components))
                    unit.preset_antenna_enabled = components["preset_antenna_enabled"];

                if (checkJsonValue<bool>(components,
                                         "preset_antenna_enabled",
                                         current_unit_errors_components))
                    unit.preset_antenna_enabled = components["preset_antenna_enabled"];

                if (unit.preset_antenna_enabled)
                {
                    // Check if preset_antenna_file is an array and then store into vector
                    if (components["preset_antenna_files"].is_array())
                    {
                        for (const auto &file : components["preset_antenna_files"])
                        {
                            if (file.is_string())
                            {
                                unit.preset_antenna_files.push_back(file.get<std::string>());
                            }
                            else
                            {
                                current_unit_errors_components += comp_error_prefix + "      - Invalid type in 'preset_antenna_file' array.\n";
                            }
                        }
                    }
                    else if (components["preset_antenna_files"].is_string())
                    {
                        // If not an array, check if it's a single string
                        unit.preset_antenna_files.push_back(
                            components["preset_antenna_files"].get<std::string>());
                    }
                    else
                    { // Missing file name if preset enabled
                        current_unit_errors_components += comp_error_prefix + "      - Missing 'preset_antenna_files' when "
                                                                              "'preset_antenna_enabled' is true.\n";
                    }
                }

                // for preset_algorithm_enabled
                if (checkJsonValue<bool>(components,
                                         "preset_algorithm_enabled",
                                         current_unit_errors_components))
                    unit.preset_algorithm_enabled = components["preset_algorithm_enabled"];

                if (unit.preset_algorithm_enabled)
                {
                    // Check if preset_algorithm_name is an array and then store into vector
                    if (components["preset_algorithm_names"].is_array())
                    {
                        for (const auto &file : components["preset_algorithm_names"])
                        {
                            if (file.is_string())
                            {
                                unit.preset_algorithm_names.push_back(file.get<std::string>());
                            }
                            else
                            {
                                current_unit_errors_components += comp_error_prefix + "      - Invalid type in 'preset_algorithm_file' array.\n";
                            }
                        }
                    }
                    else if (components["preset_algorithm_names"].is_string())
                    {
                        // If not an array, check if it's a single string
                        unit.preset_algorithm_names.push_back(
                            components["preset_algorithm_names"].get<std::string>());
                    }
                    else
                    { // Missing file name if preset enabled
                        current_unit_errors_components += comp_error_prefix + "      - Missing 'preset_algorithm_names' when "
                                                                              "'preset_algorithm_enabled' is true.\n";
                    }
                }
            }
            else
            {
                // Handle missing or invalid "components" object
                errorMessages += unit_error_prefix + "    - Missing or invalid 'components' object.\n";
            }

            // --- Accumulate errors from sub-sections ---
            if (!current_unit_errors_components.empty())
            {
                errorMessages += unit_error_prefix + "    - In components:\n" + current_unit_errors_components;
            }

            // Calculate derived values (after reading base values)
            if (cf.engine_slot_time_microsec > 0)
            { // Avoid division by zero
                // Only valid for SIMULATION
                if (cf.engine_mode == "SIMULATION")
                {
                    if (unit.IMU_enabled)
                    {
                        unit.IMU_samplingDelay_slots = unit.IMU_samplingDelay_microsec / cf.engine_slot_time_microsec;
                        if (unit.IMU_samplingDelay_slots <= 0 || unit.IMU_samplingDelay_slots != static_cast<long long>(unit.IMU_samplingDelay_slots))
                        {
                            errorMessages += "  - Invalid 'IMU_samplingDelay_microsec'. Calculation "
                                             "for delay slots failed.\n";
                            errorMessages += "      - Reason: The sampling delay must be a positive "
                                             "multiple of the engine slot time.\n";
                            errorMessages += "      - IMU Sampling Delay: " + std::to_string(unit.IMU_samplingDelay_microsec) + " µs\n";
                            errorMessages += "      - Engine Slot Time:   " + std::to_string(cf.engine_slot_time_microsec) + " µs\n";
                            errorMessages += "      - Calculated Slots:   " + std::to_string(unit.IMU_samplingDelay_slots) + " (must be a whole number > 0)\n";

                            unit.IMU_samplingDelay_slots = 0; // Reset to zero on error
                        }
                    }
                    else
                    {
                        unit.IMU_samplingDelay_slots = 0;
                    }

                    if (unit.RxChain_enabled)
                    {
                        unit.RxChain_samplingDelay_slots = unit.RxChain_samplingDelay_microsec / cf.engine_slot_time_microsec;
                        if (unit.RxChain_samplingDelay_slots <= 0 || unit.RxChain_samplingDelay_slots != static_cast<long long>(unit.RxChain_samplingDelay_slots))
                        {
                            errorMessages += "  - Invalid 'RxChain_samplingDelay_microsec'. "
                                             "Calculation for delay slots failed.\n";
                            errorMessages += "      - Reason: The sampling delay must be a positive "
                                             "multiple of the engine slot time.\n";
                            errorMessages += "      - RxChain Sampling Delay: " + std::to_string(unit.RxChain_samplingDelay_microsec) + " µs\n";
                            errorMessages += "      - Engine Slot Time:       " + std::to_string(cf.engine_slot_time_microsec) + " µs\n";
                            errorMessages += "      - Calculated Slots:       " + std::to_string(unit.RxChain_samplingDelay_slots) + " (must be a whole number > 0)\n";

                            unit.RxChain_samplingDelay_slots = 0; // Reset to zero on error
                        }
                    }
                    else
                    {
                        unit.RxChain_samplingDelay_slots = 0;
                    }
                }
            }
            else
            {
                unit.IMU_samplingDelay_slots = 0;
                unit.RxChain_samplingDelay_slots = 0;
                if ((unit.IMU_enabled && unit.IMU_samplingDelay_microsec > 0) || (unit.RxChain_enabled && unit.RxChain_samplingDelay_microsec > 0))
                {
                    errorMessages += "  - Invalid 'engine_slot_time_microsec' for calculating delay slots.\n";
                    errorMessages += "      - Value: " + std::to_string(cf.engine_slot_time_microsec) + " µs (must be > 0)\n";
                }
            }

            cf.engine_units.push_back(unit);
            unit_index++;
        }
    }
    else
    {
        errorMessages += "  - Missing or invalid 'engine_unit'. Expected an array of unit objects.\n";
    }

    // Experimental Settings
    if (jsonFile.contains("experimental") && jsonFile["experimental"].is_object())
    {
        const auto &experimentalSection = jsonFile["experimental"];
        cf.experimental.role = getJsonString(experimentalSection,
                                             "role",
                                             errorMessages,
                                             "ERROR"); // ERROR if missing!
        // Make sure the 'role' is either a label of an engine_unit or "OBSERVER"
        if (cf.experimental.role != "OBSERVER")
        {
            bool match_found = false;
            for (const auto &unit : cf.engine_units)
            {
                if (unit.label == cf.experimental.role)
                {
                    match_found = true;
                    break;
                }
            }
            if (!match_found)
            {
                errorMessages += "  - Invalid 'experimental.role': Must be 'OBSERVER' or match an existing 'engine_unit' label.\n";
            }
        }

        cf.experimental.observer_tcp_ip = getJsonString(experimentalSection,
                                                        "observer_tcp_ip",
                                                        errorMessages,
                                                        "ERROR"); // ERROR if missing!

        cf.experimental.observer_tcp_port = experimentalSection["observer_tcp_port"];

        // Only 'require' the below if enabled for the specific unit (no required for OBSERVER)
        if (cf.experimental.role != "OBSERVER")
        {
            // Find the matching engine_unit to check if RxAntenna and/or Rotary are enabled
            auto it = std::find_if(cf.engine_units.begin(),
                                   cf.engine_units.end(),
                                   [&cf](const engineUnit &unit)
                                   { return unit.label == cf.experimental.role; });
            if (it != cf.engine_units.end())
            {
                if (it->RxAntenna_enabled || it->RxChain_enabled)
                {
                    cf.experimental.DSO_measurement_bandwidth = experimentalSection["DSO_measurement_bandwidth"];
                    cf.experimental.DSO_IF_frequency = experimentalSection["DSO_IF_frequency"];
                }
                if (it->Rotary_enabled)
                {
                    cf.experimental.Rotary_port_azimuth = getJsonString(experimentalSection,
                                                                        "Rotary_port_azimuth",
                                                                        errorMessages,
                                                                        false); // preserve case

                    cf.experimental.Rotary_port_altitude = getJsonString(experimentalSection,
                                                                         "Rotary_port_altitude",
                                                                         errorMessages,
                                                                         false); // preserve case
                }
            }
            else
            {
                // This should not happen as we validated 'role' above, but just in case
                errorMessages += "  - Internal error: Could not find matching 'engine_unit' for 'experimental.role'.\n";
            }
        }
    }
    else
    {
        errorMessages += "  - Missing or invalid 'experimental' object.\n";
    }

    // 5. Final check of accumulated errors
    if (!errorMessages.empty())
    {
        qErrorMsg = QString("Configuration validation errors in:\n%1\n\n%2")
                        .arg(configFilePath)
                        .arg(QString::fromStdString(errorMessages));
        return {ConfigFile{},
                parsedJson,
                qErrorMsg}; // Return default config, but keep parsedJson for potential GUI display of errors
    }
    else
    {
        // Config loaded and validated successfully
        // Set the config name from the file path
        cf.paramConfig_name = fileInfo.fileName().toStdString();
        return {cf,
                parsedJson,
                QString{}}; // Return valid config, parsed json, and empty error string
    }
}

void ConfigSelectionPage::populateConfigTree(const json &j, QTreeWidgetItem *parentItem)
{
    QTreeWidget *targetWidget = configTreeWidget;

    try
    {
        if (j.is_object())
        {
            for (auto it = j.begin(); it != j.end(); ++it)
            {
                // Create a new item under the parent (or top-level if parentItem is null)
                QTreeWidgetItem *item = parentItem ? new QTreeWidgetItem(parentItem)
                                                   : new QTreeWidgetItem(targetWidget);
                item->setText(0, QString::fromStdString(it.key())); // Set the parameter name (key)

                const json &value = it.value();

                if (value.is_structured())
                { // Handle nested objects and arrays
                    item->setText(1,
                                  "");                                   // Container nodes have no direct value in the second column
                    item->setFlags(item->flags() & ~Qt::ItemIsEditable); // Make container node text non-editable
                    populateConfigTree(value, item);                     // Recurse into the nested structure
                }
                else
                {                                                           // Handle primitive types (string, number, boolean, null)
                    item->setText(1, QString::fromStdString(value.dump())); // Set the value
                }
            }
        }
        else if (j.is_array())
        {
            // If the current JSON element itself is an array (passed from recursion)
            // Create items for each element under the parentItem which represents the array key
            for (size_t i = 0; i < j.size(); ++i)
            {
                const json &element = j[i];
                QString elementLabel = QString("[%1]").arg(i); // Default label "[0]", "[1]"...

                if (parentItem && parentItem->text(0) == "engine_unit" && element.is_object() && element.contains("label") && element["label"].is_string())
                {
                    elementLabel = QString::fromStdString(element["label"].get<std::string>());
                }

                // Create a new item for the array element
                QTreeWidgetItem *elementItem = new QTreeWidgetItem(parentItem);
                elementItem->setText(0, elementLabel); // Use index or label as the "key"

                if (element.is_structured())
                {
                    elementItem->setText(1, ""); // Container node
                    elementItem->setFlags(elementItem->flags() & ~Qt::ItemIsEditable);
                    populateConfigTree(element, elementItem); // Recurse into the element
                }
                else
                {
                    elementItem->setText(1, QString::fromStdString(element.dump()));
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        // Add error message as a child item if possible, otherwise show a message box
        QString errorText = QString("Error processing JSON node: %1").arg(e.what());
        if (parentItem)
        {
            QTreeWidgetItem *errorItem = new QTreeWidgetItem(parentItem);
            errorItem->setText(0, "JSON_ERROR");
            errorItem->setText(1, errorText);
            errorItem->setForeground(0, Qt::red);
            errorItem->setForeground(1, Qt::red);
        }
        else
        {
            qWarning() << "Error populating config tree:" << errorText;
        }
    }
}

void ConfigSelectionPage::displayInitialError(const QString &errorMsg)
{
    // Clear any existing content from a previous selection
    configTreeWidget->clear();

    // Create a prominent root item for the error
    QTreeWidgetItem *errorItem = new QTreeWidgetItem(configTreeWidget);
    errorItem->setText(0, "Configuration Load Error");
    errorItem->setForeground(0, QColor("#e57373")); // A soft red color for the error
    errorItem->setExpanded(true);                   // Ensure the error is visible

    // Split the error message into lines and add them as children for readability
    QStringList lines = errorMsg.split('\n', Qt::SkipEmptyParts);
    for (const QString &line : lines)
    {
        QTreeWidgetItem *lineItem = new QTreeWidgetItem(errorItem);
        lineItem->setText(0, line);
    }

    // Update the button to reflect the error state, just like the other error paths
    nextButton->setText("Config Error");
    nextButton->setEnabled(false);
}
