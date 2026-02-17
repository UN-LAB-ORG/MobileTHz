#ifndef CONFIGSELECTIONPAGE_H
#define CONFIGSELECTIONPAGE_H

#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QString>
#include <QTreeWidget>
#include <QWidget>
#include "Software/jsonReader/jsonReader.hpp"
#include "Software/structDefinition.h"
#include <tuple>

class ConfigSelectionPage : public QWidget
{
    Q_OBJECT

public:
    ConfigSelectionPage(QWidget *parent = nullptr);

    /**
     * @brief Loads a configuration file from the specified path, parses, and validates it.
     * @param configFilePath The absolute or relative path to the JSON configuration file.
     * @return A tuple containing:
     *         1. ConfigFile: The loaded and validated configuration struct (default constructed on error).
     *         2. nlohmann::json: The parsed JSON object (empty on file read or parse error).
     *         3. QString: An error message string. Empty if loading and validation were successful.
     */
    static std::tuple<ConfigFile, nlohmann::json, QString> loadAndValidateConfigFile(
        const QString &configFilePath);

signals:
    void configSelected(const ConfigFile &configFile);

public slots:
    void displayInitialError(const QString &errorMsg);

private slots:
    void loadSelectedConfig();

private:
    QComboBox *configFileCombo;
    QLineEdit *ipAddressLineEdit;
    QTreeWidget *configTreeWidget;
    ConfigFile configFile;     // Holds the currently loaded/validated config for GUI
    nlohmann::json configJson; // Holds the raw JSON for display in GUI tree
    QPushButton *nextButton;

    void keyPressEvent(QKeyEvent *event) override;

    void refreshConfigFiles();
    void populateConfigTree(const nlohmann::json &j, QTreeWidgetItem *parentItem);
    void setupUI();

    // --- Static Validation Helpers ---
    template <typename T>
    T getJsonValue(const nlohmann::json &j,
                   const std::string &key,
                   const T &defaultValue,
                   std::string &errorMessages)
        const
    {
        try
        {
            if (j.contains(key))
            {
                try
                {
                    return j[key].get<T>();
                }
                catch (const nlohmann::json::type_error &e)
                {
                    (void)e;
                    return defaultValue;
                }
            }
        }
        catch (const nlohmann::json::exception &e)
        {
            (void)e;
            return defaultValue;
        }
    }

    static std::string getJsonString(const nlohmann::json &j,
                                     const std::string &key,
                                     std::string &errorMessages,
                                     bool toUpper = true);

    template <typename T>
    static bool checkJsonValue(const nlohmann::json &j,
                               const std::string &key,
                               std::string &errorMessages);

    static bool getJsonBoolSafe(const nlohmann::json &j,
                                const std::string &key,
                                bool defaultValue,
                                std::string &errorMessages);
};

#endif // CONFIGSELECTIONPAGE_H
