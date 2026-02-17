#include <QApplication>
#include <iostream>
#include <memory>
#include <qcommandlineparser.h>
#include <qfileinfo.h>
#include <qthread.h>
#include <vector>

#include "UserInterface/mainWindow.h"

// --- FORCED DEBUG FLAGS ---
// Testing the Simulation Framework
const bool forceConsoleForSimTesting = false;
const QString testConfigFileName = "example.json";
// Testing the Experimental Framework
const bool forceObserverTestMode = false;
const QString observerTestConfigName = "example.json";
const QStringList nodeTestConfigNames = {"example.json", "example.json"};
// --------------------------

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QApplication::setApplicationName("MobileTHz");
    QApplication::setApplicationVersion("1.0");
    QFont font = QApplication::font();
    font.setFamily("Helvetica Neue");
    font.setHintingPreference(QFont::PreferNoHinting);
    QApplication::setFont(font);

    QString configFilePath;

    if (forceObserverTestMode)
    {
        std::cout << "--- FORCED OBSERVER TEST MODE (Multi-Window GUI) ACTIVE ---" << std::endl;

        // We use a vector of unique_ptr to manage the lifetime of the windows.
        // They will be automatically deleted when main() exits.
        std::vector<std::unique_ptr<MainWindow>> windows;

        // 1. Create and configure the Observer window first.
        QString observerTestConfigFilePath = CONFIG_PARAMETER_DIR + QString("/") + observerTestConfigName;
        auto observerWindow = std::make_unique<MainWindow>(observerTestConfigFilePath);
        windows.push_back(std::move(observerWindow));
        windows.back()->show();

        // 2. Create and configure all Nodse windows.
        for (const QString &nodePath : nodeTestConfigNames)
        {
            QString nodeTestConfigPath = CONFIG_PARAMETER_DIR + QString("/") + nodePath;
            auto nodeWindow = std::make_unique<MainWindow>(nodeTestConfigPath);
            windows.push_back(std::move(nodeWindow));
            windows.back()->show();
        }

        // 3. Run the application event loop for all windows.
        return a.exec();
    }
    else if (forceConsoleForSimTesting)
    {
        // --- Original Forced Console Mode ---
        std::cout << "--- INTERNAL TEST MODE ACTIVE ---" << std::endl;
        configFilePath = CONFIG_PARAMETER_DIR + QString("/") + testConfigFileName;
        if (configFilePath.isEmpty())
        {
            std::cerr << "Error: Console mode requires a config file path." << std::endl;
            // In case the flag is true but the test path is empty
            return 1;
        }
        QFileInfo checkFile(configFilePath);
        if (!checkFile.exists() || !checkFile.isFile())
        {
            std::cerr << "Error: Configuration file not found or is not a file: "
                      << configFilePath.toStdString() << std::endl;
            return 1;
        }
        MainWindow qtWindow(configFilePath);
        return qtWindow.runConsoleMode();
    }
    else
    {
        // --- Standard GUI / Command-line  ---
        bool isConsoleMode = false;
        QCommandLineParser parser;
        parser.setApplicationDescription("MobileTHz Simulation and Real-Time Application");
        parser.addHelpOption();
        parser.addVersionOption();

        // Define the configuration file option
        QCommandLineOption configOption(QStringList() << "c" << "config",
                                        "Run in console mode using the specified <config file>.",
                                        "config file");
        parser.addOption(configOption);

        // Process the actual command line arguments given by the user
        parser.process(a);
        isConsoleMode = parser.isSet(configOption);
        if (isConsoleMode)
        {
            configFilePath = CONFIG_PARAMETER_DIR + QString("/") + parser.value(configOption);
            if (configFilePath.isEmpty())
            {
                std::cerr << "Error: Console mode requires a config file path." << std::endl;
                // In case the flag is true but the test path is empty
                return 1;
            }
            QFileInfo checkFile(configFilePath);
            if (!checkFile.exists() || !checkFile.isFile())
            {
                std::cerr << "Error: Configuration file not found or is not a file: "
                          << configFilePath.toStdString() << std::endl;
                return 1;
            }
        }

        // --- MainWindow Creation ---
        MainWindow qtWindow(configFilePath, isConsoleMode);

        // --- Run Mode Logic ---
        int returnCode = 0; // Default exit code

        if (isConsoleMode)
        {
            // Directly call the method that executes the console logic and gets the result.
            // The work is done synchronously here (due to blockingMappedReduced).
            returnCode = qtWindow.runConsoleMode();
        }
        else
        {
            // GUI mode: Show the window and run the event loop.
            qtWindow.show();
            returnCode = a.exec();
        }

        return returnCode; // Return the code obtained from either console run or a.exec()
    }
}
