#ifndef INITIALIZATIONPAGE_H
#define INITIALIZATIONPAGE_H

#include <QApplication>
#include <QBasicTimer>
#include <QCloseEvent>
#include <QComboBox>
#include <QFileDialog>
#include <QFrame>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QNetworkInterface>
#include <QPixmapCache>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QStackedWidget>
#include <QStyle>
#include <QStyleFactory>
#include <QSvgRenderer>
#include <QTabWidget>
#include <QTextEdit>
#include <QThread>
#include <QTimer>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <iostream>
#include <qfuturewatcher.h>

// Directory-specific includes
#include "Simulation/powerSim/powerSim.h"
#include "Software/jsonReader/jsonReader.hpp"
#include "Software/sharedPointersDefinition.h"
#include "UserInterface/Utils/QTextEditStreamBuf.h"

class InitializationPage : public QWidget
{
    Q_OBJECT

public:
    InitializationPage(QWidget *parent = nullptr);
    void setParamConfig(const ConfigFile &inputConfig,
                        const nlohmann::json &inputPositionConfig);
    void cleanup();
    std::shared_ptr<SharedPointers> getSharedPointers();

    /**
     * @brief Explicitly starts the component initialization process.
     *        Safe to call multiple times; initialization runs only once.
     */
    void startInitialization();
    // -------------------------

public slots:
    void stopExperiment();

signals:
    void initializationFinished(bool success, const QString &errorMsg);
    void initializationCancelled();

private slots:
    void updateLoadingIcon();

protected:
    // --- OVERRIDE ---
    /**
     * @brief Overridden to trigger initialization when the page is shown in GUI mode.
     */
    void showEvent(QShowEvent *event) override;
    // ----------------

private:
    QLabel *kpiClassifierLabel;
    QLabel *runEngineLabel;
    QLabel *rotaryLabel;
    QLabel *kpiLabel;
    QLabel *powerLabel;
    QLabel *imuLabel;
    QLabel *tcpLabel;
    QLabel *positionSimLabel;
    QLabel *mobileThzEngineLabel;
    QProgressBar *progressBar;
    QTextEdit *logDisplay;
    QLabel *selectedConfigLabel;

    QTimer *loadingTimer;
    QSvgRenderer *loadingRenderer;
    QSvgRenderer *checkmarkRenderer;
    QSvgRenderer *errorRenderer;

    ConfigFile paramConfigFile;
    nlohmann::json positionConfig;
    std::shared_ptr<SharedPointers> sharedPointers = std::make_shared<SharedPointers>();

    bool initilizationError = false;
    bool initializationStarted = false;

    // Members for stream redirection
    std::unique_ptr<QTextEditStreamBuf> customStreamBuf;
    std::streambuf *oldCoutStreamBuf = nullptr;
    void setupStdoutRedirection();
    void restoreStdoutRedirection();

    // --- Methods ---
    void setupUI();
    void initializeComponents();
    void updateProgressBar(int value, bool isError = false);
    void setLabelIcon(QLabel *label, QSvgRenderer *renderer, int size);

    std::map<int, radiationPattern> radiationPatterns;

    // Background engine run thread experimental
    QFutureWatcher<void> engineRunWatcher_; // To monitor the background engine task
};

#endif // INITIALIZATIONPAGE_H
