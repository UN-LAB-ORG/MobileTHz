#include "initializationPage.h"
#include <QEventLoop>
#include <QFrame>
#include <QFutureWatcher>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QTextEdit>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrent>
#include <iostream>
#include <qpainter.h>

#include "Experimental/imuExp/imuExp.h"
#include "Experimental/powerExp/powerExp.h"
#include "Experimental/rotaryExp/rotaryExp.h"
#include "Simulation/imuSim/imuSim.h"
#include "Simulation/positionSim/positionSim.h"
#include "Simulation/rotarySim/rotarySim.h"
#include "Software/kpiClassifier/kpiClassifier.h"
#include "Software/mobileTHzEngine/mobileTHzEngine.h"

static bool debug = false;

InitializationPage::InitializationPage(QWidget *parent)
    : QWidget(parent), loadingTimer(new QTimer(this)), initializationStarted(false) // Initialize the new flag
      ,
      initilizationError(false)
{
    // Initialize renderers here or ensure setParamConfig is always called first
    loadingRenderer = new QSvgRenderer(QString(":/ICONS/loading.svg"), this);
    checkmarkRenderer = new QSvgRenderer(QString(":/ICONS/checkmark.svg"), this);
    errorRenderer = new QSvgRenderer(QString(":/ICONS/error.svg"), this);
    loadingRenderer->setObjectName("loading");
    checkmarkRenderer->setObjectName("checkmark");
    errorRenderer->setObjectName("error");

    // Setup the basic UI structure immediately
    setupUI();
    connect(loadingTimer, &QTimer::timeout, this, &InitializationPage::updateLoadingIcon);
}

void InitializationPage::setParamConfig(const ConfigFile &inputConfig,
                                        const nlohmann::json &inputPositionConfig)
{
    paramConfigFile = inputConfig;
    positionConfig = inputPositionConfig;

    // Reset state variables for a new run
    initializationStarted = false;
    initilizationError = false;

    // Update the config file name label
    if (selectedConfigLabel)
    {
        selectedConfigLabel->setText(QString::fromStdString(paramConfigFile.paramConfig_name));
    }

    // --- DYNAMIC UI CONFIGURATION ---
    // Hide all optional labels first for a clean slate.
    positionSimLabel->hide();
    imuLabel->hide();
    powerLabel->hide();
    rotaryLabel->hide();
    kpiLabel->hide();
    runEngineLabel->hide();
    kpiClassifierLabel->hide();
    tcpLabel->hide();

    // Find the algorithm info frame to hide/show it
    QFrame *infoFrame2 = findChild<QFrame *>("infoFrame2");
    if (infoFrame2)
    {
        infoFrame2->hide();
    }

    // Now, show only the labels relevant to the current mode and role.
    if (paramConfigFile.engine_mode == "SIMULATION")
    {
        // Simulation run shows all simulation components.
        std::cout << "[UI Setup] Configuring for SIMULATION mode." << std::endl;
        positionSimLabel->show();
        imuLabel->setText("IMU Sim");
        imuLabel->show();
        powerLabel->setText("Power Sim");
        powerLabel->show();
        rotaryLabel->setText("Rotary Sim");
        rotaryLabel->show();
        kpiLabel->show();
        runEngineLabel->show();
        kpiClassifierLabel->show();
        if (infoFrame2)
            infoFrame2->show(); // Show algorithm list in sim mode
    }
    else if (paramConfigFile.engine_mode == "EXPERIMENTAL")
    {
        if (paramConfigFile.experimental.role == "OBSERVER")
        {
            // Observer only cares about the TCP connection and the engine.
            std::cout << "[UI Setup] Configuring for EXPERIMENTAL mode (Observer)." << std::endl;
            tcpLabel->show();
            // The mobileThzEngineLabel is always visible, which is correct.
        }
        else // This is an Experimental Unit (AP or UE)
        {
            std::cout << "[UI Setup] Configuring for EXPERIMENTAL mode (Unit: "
                      << paramConfigFile.experimental.role << ")." << std::endl;
            // An active unit connects to hardware and TCP.
            imuLabel->setText("IMU Sensor");
            imuLabel->show();
            powerLabel->setText("Power HW");
            powerLabel->show();
            rotaryLabel->setText("Rotary HW");
            rotaryLabel->show();
            tcpLabel->show();
            kpiLabel->show();
            if (infoFrame2)
                infoFrame2->show(); // Show algorithm for active units
        }
    }

    // Clear log and reset progress bar for the new run.
    if (logDisplay)
        logDisplay->clear();
    if (progressBar)
        updateProgressBar(0);
}

void InitializationPage::setupUI()
{
    QWidget *page = this;
    QVBoxLayout *mainLayout = new QVBoxLayout(page);
    mainLayout->setSpacing(10);

    // Header section (Title and logos)
    QHBoxLayout *headerLayout = new QHBoxLayout();
    QLabel *titleLabel = new QLabel("Initializing...", page);
    titleLabel->setStyleSheet("font-size: 42px; font-weight: bold; padding-top: 25px");
    headerLayout->addWidget(titleLabel, 0, Qt::AlignLeft | Qt::AlignBottom);
    headerLayout->addStretch(1);

    QLabel *kthlogo = new QLabel(page);
    QPixmap kthPixmap(":/ICONS/kth.png");
    kthlogo->setPixmap(
        kthPixmap.scaledToHeight(titleLabel->sizeHint().height() - 25, Qt::SmoothTransformation));
    headerLayout->addWidget(kthlogo, 0, Qt::AlignRight | Qt::AlignBottom);

    QLabel *neuLogo = new QLabel(page);
    QPixmap neuPixmap(":/ICONS/neu.png");
    neuLogo->setPixmap(
        neuPixmap.scaledToHeight(titleLabel->sizeHint().height() - 25, Qt::SmoothTransformation));
    headerLayout->addWidget(neuLogo, 0, Qt::AlignRight | Qt::AlignBottom);

    QLabel *unlabLogo = new QLabel(page);
    QPixmap unlabPixmap(":/ICONS/unlab.png");
    unlabLogo->setPixmap(
        unlabPixmap.scaledToHeight(titleLabel->sizeHint().height() - 25, Qt::SmoothTransformation));
    headerLayout->addWidget(unlabLogo, 0, Qt::AlignRight | Qt::AlignBottom);

    mainLayout->addLayout(headerLayout);

    progressBar = new QProgressBar(page);
    progressBar->setStyleSheet(
        "QProgressBar { border: 2px solid #3a3a3a; border-radius: 5px; text-align: center; } "
        "QProgressBar::chunk { background-color: #C8102E; width: 20px; }");
    progressBar->setTextVisible(true);
    mainLayout->addWidget(progressBar);

    // Content section
    QWidget *contentWidget = new QWidget(page);
    QHBoxLayout *contentLayout = new QHBoxLayout(contentWidget);
    contentLayout->setSpacing(10);
    contentLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->addWidget(contentWidget, 1);

    // Left column (Config and Algorithm info)
    QVBoxLayout *leftColumnLayout = new QVBoxLayout();

    QFrame *initFrame = new QFrame(page);
    initFrame->setFrameShape(QFrame::StyledPanel);
    initFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *initLayout = new QVBoxLayout(initFrame);

    QLabel *statusTitle = new QLabel("Status", initFrame);
    statusTitle->setStyleSheet("font-size: 18px; font-weight: bold; padding-bottom: 5px; "
                               "border-bottom: 2px solid #3a3a3a; border-radius: 0px;");
    initLayout->addWidget(statusTitle);

    QString labelStyle = "font-size: 16px; color: #1f1f1f;";

    // We always have the underlying mobileTHzEngine
    mobileThzEngineLabel = new QLabel("MobileTHz Engine", initFrame);
    mobileThzEngineLabel->setStyleSheet(labelStyle);
    initLayout->addWidget(mobileThzEngineLabel);

    // Create all possible labels, they will be shown/hidden in setParamConfig
    positionSimLabel = new QLabel("Position Sim", initFrame);
    positionSimLabel->setStyleSheet(labelStyle);
    initLayout->addWidget(positionSimLabel);
    positionSimLabel->hide(); // Hide initially

    imuLabel = new QLabel("IMU Sim/Sensor", initFrame);
    imuLabel->setStyleSheet(labelStyle);
    initLayout->addWidget(imuLabel);
    imuLabel->hide(); // Hide initially

    powerLabel = new QLabel("Power Sim/HW", initFrame);
    powerLabel->setStyleSheet(labelStyle);
    initLayout->addWidget(powerLabel);

    rotaryLabel = new QLabel("Rotary Sim/HW", initFrame);
    rotaryLabel->setStyleSheet(labelStyle);
    initLayout->addWidget(rotaryLabel);

    kpiLabel = new QLabel("KPI Classifier", initFrame);
    kpiLabel->setStyleSheet(labelStyle);
    initLayout->addWidget(kpiLabel);

    runEngineLabel = new QLabel("Running Engine (Sim)", initFrame);
    runEngineLabel->setStyleSheet(labelStyle);
    initLayout->addWidget(runEngineLabel);
    runEngineLabel->hide(); // Hide initially

    kpiClassifierLabel = new QLabel("KPI Classifier", initFrame);
    kpiClassifierLabel->setStyleSheet(labelStyle);
    initLayout->addWidget(kpiClassifierLabel);
    kpiClassifierLabel->hide(); // Hide initially

    tcpLabel = new QLabel("TCP Connection", initFrame);
    tcpLabel->setStyleSheet(labelStyle);
    initLayout->addWidget(tcpLabel);
    tcpLabel->hide(); // Hide initially
    leftColumnLayout->addWidget(initFrame);

    QFrame *infoFrame1 = new QFrame(page);
    infoFrame1->setFrameShape(QFrame::StyledPanel);
    infoFrame1->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *infoLayout1 = new QVBoxLayout(infoFrame1);

    QLabel *configLabel = new QLabel("Config File:", infoFrame1);
    configLabel->setStyleSheet("font-size: 18px; font-weight: bold; border-bottom: 2px solid "
                               "#3a3a3a; border-radius: 0px; padding-bottom: 5px;");
    infoLayout1->addWidget(configLabel);

    selectedConfigLabel = new QLabel(paramConfigFile.paramConfig_name.c_str(), infoFrame1);
    selectedConfigLabel->setObjectName("selectedConfigLabel");
    selectedConfigLabel->setStyleSheet("font-size: 16px; color: white;");
    infoLayout1->addWidget(selectedConfigLabel);

    QFrame *infoFrame2 = new QFrame(page);
    infoFrame2->setObjectName("infoFrame2");
    infoFrame2->setFrameShape(QFrame::StyledPanel);
    infoFrame2->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *infoLayout2 = new QVBoxLayout(infoFrame2);

    QLabel *algorithmLabel = new QLabel("Running Algorithms:", infoFrame2);
    algorithmLabel->setStyleSheet("font-size: 18px; font-weight: bold; border-bottom: 2px solid "
                                  "#3a3a3a; border-radius: 0px; padding-bottom: 5px;");
    infoLayout2->addWidget(algorithmLabel);

    infoLayout2->addStretch(1);

    leftColumnLayout->addWidget(infoFrame1);
    leftColumnLayout->addWidget(infoFrame2);
    contentLayout->addLayout(leftColumnLayout, 1);

    // Right column (Log)
    QVBoxLayout *rightColumnLayout = new QVBoxLayout();
    logDisplay = new QTextEdit(page);
    logDisplay->setReadOnly(true);
    logDisplay->setStyleSheet(
        "font-size: 14px; background-color: #1f1f1f; border-radius: 5px; padding: 10px;");
    rightColumnLayout->addWidget(logDisplay);

    contentLayout->addLayout(rightColumnLayout, 5); // Adjust the stretch factor as needed
}

void InitializationPage::initializeComponents()
{
    // Setup stdout redirection before any output.
    setupStdoutRedirection();
    logDisplay->clear();
    updateProgressBar(0);
    loadingTimer->start(100); // Update loading icon every 100 ms

    if (sharedPointers)
    {
        try
        {
            // Delete simulator objects (check for null first)
            if (sharedPointers->posSim)
            {
                sharedPointers->posSim = nullptr;
            }
            if (sharedPointers->imuSim)
            {
                sharedPointers->imuSim = nullptr;
            }
            if (sharedPointers->powerSim)
            {
                sharedPointers->powerSim = nullptr;
            }
            if (sharedPointers->rotarySim)
            {
                sharedPointers->rotarySim = nullptr;
            }
            if (sharedPointers->rotaryExp)
            {
                sharedPointers->rotaryExp = nullptr;
            }
            if (sharedPointers->powerExp)
            {
                sharedPointers->powerExp = nullptr;
            }
            if (sharedPointers->imuExp)
            {
                sharedPointers->imuExp->disconnect(); // Ensure BLE is disconnected
                sharedPointers->imuExp = nullptr;
            }
            if (sharedPointers->engine)
            {
                sharedPointers->engine = nullptr;
            }
            if (sharedPointers->kpiClassifier)
            {
                sharedPointers->kpiClassifier = nullptr;
            }
        }
        catch (...)
        {
            std::cerr << "[Config] Error during cleanup of previous components.\n";
        }
    }

    if (debug)
    {
        std::cout << "[Config] Initializing components...\n";
    }

    QCoreApplication::processEvents();

    int progress = 0;
    //  Lambda function for executing steps
    auto executeStep =
        [this, &progress](int stepProgress, const std::function<bool()> &step, QLabel *label)
    {
        if (!label)
            return false; //  If label is null, skip

        updateProgressBar(progress);
        QCoreApplication::processEvents();

        label->setProperty("loading", true); // Set loading state
        label->setProperty("success", false);
        setLabelIcon(label, loadingRenderer, 20);
        QCoreApplication::processEvents();

        QEventLoop loop;
        bool success = false;

        QFuture<bool> future = QtConcurrent::run(
            [&]()
            { return step(); }); // Run step in separate thread

        QFutureWatcher<bool> watcher;
        connect(&watcher, &QFutureWatcher<bool>::finished, &loop, &QEventLoop::quit);
        watcher.setFuture(future);

        loop.exec();               // Wait for step to finish
        success = future.result(); // Get result

        if (success)
        {
            label->setProperty("loading", false);
            label->setProperty("success", true);
            setLabelIcon(label, checkmarkRenderer, 20); // Success icon
        }
        else
        {
            label->setProperty("loading", false);
            label->setProperty("success", false);
            setLabelIcon(label, errorRenderer, 20); // Error icon
            std::cout << "[Config] Step failed. Aborting initialization.\n";
            updateProgressBar(100, true); // Error progress
            initilizationError = true;
        }
        progress += stepProgress;          // Increment progress
        updateProgressBar(progress);       // Update
        QCoreApplication::processEvents(); // Process events

        return success;
    };

    // We always have the underlying mobileTHzEngine
    try
    {
        if (!executeStep(
                25,
                [this]()
                {
                    sharedPointers->engine = std::make_unique<mobileTHzEngine>(paramConfigFile);
                    sharedPointers->engine->initialize(paramConfigFile);
                    return true;
                },
                mobileThzEngineLabel))
            return;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Config] Error 1 during initialization: " << e.what() << std::endl;
        updateProgressBar(0);
        emit initializationFinished(false, QString("Error 1 Initialization failed: ") + e.what());
        return;
    }

    // Two different trees, depending on if we are in simulation or realtime mode
    if (paramConfigFile.engine_mode == "SIMULATION")
    {
        try
        {
            // --- Position Simulation ---
            if (!executeStep(
                    15,
                    [this]()
                    {
                        sharedPointers->posSim = std::make_unique<positionSim>();
                        sharedPointers->posSim->initialize(positionConfig,
                                                           paramConfigFile,
                                                           sharedPointers->engine.get());

                        return true;
                    },
                    positionSimLabel))
                return;

            // --- IMU Simulation ---
            if (!executeStep(
                    15,
                    [this]()
                    {
                        sharedPointers->imuSim = std::make_unique<imuSim>();
                        sharedPointers->imuSim->initialize(paramConfigFile,
                                                           sharedPointers->engine.get());
                        return true;
                    },
                    imuLabel))
                return;

            // --- Power Simulation ---
            if (!executeStep(
                    5,
                    [this]()
                    {
                        sharedPointers->powerSim = std::make_unique<powerSim>();
                        sharedPointers->powerSim->initialize(paramConfigFile,
                                                             sharedPointers->engine.get());
                        return true;
                    },
                    powerLabel))
                return;

            // --- Rotary Simulation ---
            if (!executeStep(
                    5,
                    [this]()
                    {
                        sharedPointers->rotarySim = std::make_unique<rotarySim>();
                        sharedPointers->rotarySim->initialize(paramConfigFile,
                                                              sharedPointers->engine.get());
                        return true;
                    },
                    rotaryLabel))
                return;

            // --- KPI Simulation ---
            if (!executeStep(
                    5,
                    [this]()
                    {
                        sharedPointers->kpiClassifier = std::make_unique<kpiClassifier>();
                        sharedPointers->kpiClassifier->initialize(sharedPointers->engine.get());
                        sharedPointers->plotSaver = std::make_unique<plotSaver>();
                        sharedPointers->plotSaver->initialize(sharedPointers->engine.get());
                        return true;
                    },
                    kpiLabel))
                return;

            // --- Run Engine ---
            if (!executeStep(
                    25,
                    [this]()
                    {
                        sharedPointers->engine->run(sharedPointers->kpiClassifier.get(),
                                                    sharedPointers->plotSaver.get(),
                                                    sharedPointers->powerSim.get(),
                                                    sharedPointers->rotarySim.get(),
                                                    nullptr,
                                                    nullptr,
                                                    nullptr,
                                                    nullptr);
                        return true;
                    },
                    runEngineLabel))
                return;

            // --- Gather KPIs ---
            if (!executeStep(
                    5,
                    [this]()
                    {
                        // --- Calculate KPIs with a unique filename ---
                        QString timeStamp = QDateTime::currentDateTime().toString(
                            "yyyy-MM-dd_HH-mm-ss-zzz");
                        QString uniqueKPIName = QString("%1_KPIs").arg(timeStamp);
                        sharedPointers->kpiClassifier->calculateFinalKPIsAndSave(
                            uniqueKPIName.toStdString());
                        QString uniqueTikzFileName = QString("%1_RxPowerVsTime").arg(timeStamp);
                        sharedPointers->plotSaver->saveRxPowerVsTime(uniqueTikzFileName.toStdString());
                        return true;
                    },
                    kpiClassifierLabel))
                return;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[Config] Error 2 during initialization: " << e.what() << std::endl;
            updateProgressBar(0);
            emit initializationFinished(false,
                                        QString("Error 2 Initialization failed: ") + e.what());
            return;
        }
    }
    // --- EXPERIMENTAL MODE INITIALIZATION ---
    else if (paramConfigFile.engine_mode == "EXPERIMENTAL")
    {
        if (!executeStep(
                20,
                [this]()
                {
                    sharedPointers->engine = std::make_unique<mobileTHzEngine>(paramConfigFile);
                    sharedPointers->engine->initialize(paramConfigFile);
                    return true;
                },
                mobileThzEngineLabel))
            return;

        // --- Rotary Table ---
        if (!executeStep(
                20,
                [this]()
                {
                    sharedPointers->rotaryExp = std::make_unique<rotaryExp>();
                    sharedPointers->rotaryExp->initialize(paramConfigFile);
                    return true;
                },
                rotaryLabel))
            return;

        // --- Power Measurement Thread ---
        if (!executeStep(
                20,
                [this]()
                {
                    sharedPointers->powerExp = std::make_unique<powerExp>();
                    sharedPointers->powerExp->initialize(paramConfigFile,
                                                         sharedPointers->engine.get());
                    return true;
                },
                powerLabel))
            return;

        // --- IMU Sensor ---
        if (!executeStep(
                20,
                [this]()
                {
                    sharedPointers->imuExp = std::make_unique<imuExp>();
                    sharedPointers->imuExp->initialize(paramConfigFile);
                    return true;
                },
                imuLabel))
            return;

        // --- Gather KPIs ---
        if (!executeStep(
                5,
                [this]()
                {
                    sharedPointers->kpiClassifier = std::make_unique<kpiClassifier>();
                    sharedPointers->kpiClassifier->initialize(sharedPointers->engine.get());
                    sharedPointers->plotSaver = std::make_unique<plotSaver>();
                    sharedPointers->plotSaver->initialize(sharedPointers->engine.get());
                    return true;
                },
                kpiLabel))
            return;

        // Role-specific initialization
        if (paramConfigFile.experimental.role == "OBSERVER")
        {
            // An observer ONLY needs to start the TCP server.
            if (!executeStep(
                    80,
                    [this]() { // Give this single step most of the progress bar
                        sharedPointers->tcpExp = std::make_unique<tcpExp>();
                        return sharedPointers->tcpExp->startServer(
                            paramConfigFile.experimental.observer_tcp_port);
                    },
                    tcpLabel))
                return;
        }
        else // This is an Experimental Unit (AP or UE)
        {
            // --- TCP Connection (as client) ---
            if (!executeStep(
                    20,
                    [this]()
                    {
                        sharedPointers->tcpExp = std::make_unique<tcpExp>();
                        const std::string &my_role = paramConfigFile.experimental.role;
                        sharedPointers->tcpExp
                            ->connectToServer(paramConfigFile.experimental.observer_tcp_ip,
                                              paramConfigFile.experimental.observer_tcp_port,
                                              my_role);
                        return true;
                    },
                    tcpLabel))
                return;
        }

        // After all executeStep calls have successfully completed:
        if (initilizationError)
        {
            // If any step failed, clean up and signal failure
            restoreStdoutRedirection();
            emit initializationFinished(false, "A component failed to initialize.");
            return;
        }

        // --- Launch engine->run() in a background thread ---
        auto engine_ptr = sharedPointers->engine.get();
        auto kpi_classifier_ptr = sharedPointers->kpiClassifier.get();
        auto plotSaver_saver_ptr = sharedPointers->plotSaver.get();
        auto power_exp_ptr = sharedPointers->powerExp.get();
        auto rotary_exp_ptr = sharedPointers->rotaryExp.get();
        auto imu_exp_ptr = sharedPointers->imuExp.get();
        auto tcp_exp_ptr = sharedPointers->tcpExp.get();

        // Use QtConcurrent to run the engine's main loop in a background thread.
        QFuture<void> engineFuture = QtConcurrent::run([=]()
                                                       {
            try {
                // This is the code that will execute in the new thread.
                engine_ptr->run(kpi_classifier_ptr,
                                plotSaver_saver_ptr,
                                nullptr, // no powerSim
                                nullptr, // no rotarySim
                                power_exp_ptr,
                                rotary_exp_ptr,
                                imu_exp_ptr,
                                tcp_exp_ptr);
            } catch (const std::exception &e) {
                std::cerr << "[Engine Thread] CRITICAL ERROR: " << e.what() << std::endl;
            } });

        engineRunWatcher_.setFuture(engineFuture);

        connect(&engineRunWatcher_, &QFutureWatcher<void>::finished, this, [this]()
                {
            // This lambda will be executed on the main thread after engine->run() has finished.

            if (paramConfigFile.experimental.role == "OBSERVER") {
                std::cout << "[Observer] Engine run finished. Calculating final KPIs..."
                          << std::endl;

                QString timeStamp = QDateTime::currentDateTime().toString(
                    "yyyy-MM-dd_HH-mm-ss-zzz");
                QString uniqueKPIName = QString("%1_KPIs").arg(timeStamp);

                // Safety check: ensure the pointers are still valid
                if (sharedPointers && sharedPointers->kpiClassifier && sharedPointers->plotSaver) {
                    sharedPointers->kpiClassifier->calculateFinalKPIsAndSave(
                        uniqueKPIName.toStdString());

                    QString uniqueFileName1 = QString("%1_RxPowerVsTime").arg(timeStamp);
                    QString uniqueFileName2 = QString("%1_IMUvsTime").arg(timeStamp);
                    sharedPointers->plotSaver->saveRxPowerVsTime(uniqueFileName1.toStdString());
                    sharedPointers->plotSaver->saveIMUDataVsTime(uniqueFileName2.toStdString());

                } else {
                    std::cerr << "[Observer] Could not save KPIs, shared pointers are invalid."
                              << std::endl;
                }
            } });
    }

    loadingTimer->stop();
    disconnect(loadingTimer, &QTimer::timeout, this, &InitializationPage::updateLoadingIcon);
    restoreStdoutRedirection();

    emit initializationFinished(true, "");
}

void InitializationPage::updateProgressBar(int value, bool isError)
{
    QMetaObject::invokeMethod(
        this,
        [this, value, isError]()
        {
            progressBar->setValue(value);
            if (isError)
            {
                progressBar->setStyleSheet(
                    "QProgressBar { border: 2px solid grey; border-radius: 5px; text-align: "
                    "center; } "
                    "QProgressBar::chunk { background-color: #edb95e; width: 20px; }");
            }
            else
            {
                progressBar->setStyleSheet(
                    "QProgressBar { border: 2px solid grey; border-radius: 5px; text-align: "
                    "center; } "
                    "QProgressBar::chunk { background-color: #C8102E; width: 20px; }");
            }
        },
        Qt::QueuedConnection);
}

std::shared_ptr<SharedPointers> InitializationPage::getSharedPointers()
{
    return sharedPointers;
}

void InitializationPage::setLabelIcon(QLabel *label, QSvgRenderer *renderer, int size)
{
    if (!label || !renderer)
        return;

    QString currentText = label->text();
    QString cacheKey = QString("%1_%2").arg(renderer->objectName()).arg(size);

    QPixmap iconPixmap;
    if (!QPixmapCache::find(cacheKey, &iconPixmap))
    {
        iconPixmap = QPixmap(size, size);
        iconPixmap.fill(Qt::transparent);
        QPainter iconPainter(&iconPixmap);
        renderer->render(&iconPainter);
        QPixmapCache::insert(cacheKey, iconPixmap);
    }

    QHBoxLayout *layout = label->findChild<QHBoxLayout *>();
    if (!layout)
    {
        layout = new QHBoxLayout(label);
        layout->setContentsMargins(0, 0, 0, 0);
    }

    while (QLayoutItem *item = layout->takeAt(0))
    {
        delete item->widget();
        delete item;
    }

    QLabel *iconLabel = new QLabel(label);
    iconLabel->setPixmap(iconPixmap);
    iconLabel->setFixedSize(size, size);
    layout->addWidget(iconLabel);

    QLabel *textLabel = new QLabel(currentText, label);
    textLabel->setStyleSheet("color: white;");
    layout->addWidget(textLabel);

    label->setText(currentText);
}

void InitializationPage::updateLoadingIcon()
{
    QTimer::singleShot(0, this, [this]()
                       {
        //Iterate through labels shown
        for (QLabel *label : {rotaryLabel, imuLabel, powerLabel, tcpLabel, positionSimLabel}) {
            if (label) {
                QString currentText = label->text();
                if (label->property("loading").toBool()) {
                    setLabelIcon(label, loadingRenderer, 20);
                } else {
                    QSvgRenderer *renderer = label->property("success").toBool() ? checkmarkRenderer
                                                                                 : errorRenderer;
                    setLabelIcon(label, renderer, 20);
                }
                label->setText(currentText);
            }
        } });
}

void InitializationPage::cleanup()
{
    std::cout << "[Cleanup - Init] Starting cleanup process\n";
    QCoreApplication::processEvents();

    try
    {
        loadingTimer->stop();
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Init] Error stopping timer: " << e.what() << "\n";
    }

    QCoreApplication::processEvents();

    try
    {
        if (sharedPointers->rotaryExp)
        {
            sharedPointers->rotaryExp = nullptr;
            std::cout << "[Cleanup - Init] RotaryTable Deleted\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Init] Error cleaning up RotaryTable: " << e.what() << "\n";
    }
    QCoreApplication::processEvents();

    try
    {
        if (sharedPointers->imuExp)
        {
            sharedPointers->imuExp->disconnect();
            sharedPointers->imuExp = nullptr;
            std::cout << "[Cleanup - Init] IMU Sensor Deleted\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Init] Error cleaning up IMU Sensor: " << e.what() << "\n";
    }

    QCoreApplication::processEvents();

    try
    {
        if (sharedPointers->powerExp)
        {
            sharedPointers->powerExp = nullptr;
            std::cout << "[Cleanup - Init] PowerThread Deleted\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Init] Error cleaning up PowerThread: " << e.what() << "\n";
    }

    QCoreApplication::processEvents();

    try
    {
        if (sharedPointers->powerSim)
        {
            sharedPointers->powerSim = nullptr;
            std::cout << "[Cleanup - Init] PowerSim Deleted\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Init] Error cleaning up PowerSim: " << e.what() << "\n";
    }
    QCoreApplication::processEvents();

    try
    {
        if (sharedPointers->posSim)
        {
            sharedPointers->posSim = nullptr;
            std::cout << "[Cleanup - Init] PositionSim Deleted\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Init] Error cleaning up PositionSim: " << e.what() << "\n";
    }
    QCoreApplication::processEvents();

    try
    {
        if (sharedPointers->imuSim)
        {
            sharedPointers->imuSim = nullptr;
            std::cout << "[Cleanup - Init] IMUSim Deleted\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Init] Error cleaning up IMUSim: " << e.what() << "\n";
    }
    QCoreApplication::processEvents();

    try
    {
        if (customStreamBuf)
        {
            std::cout.rdbuf(oldCoutStreamBuf);
            customStreamBuf.reset();
            std::cout << "[Cleanup - Init] Custom Stream Buffer Reset\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Init] Error resetting custom stream buffer: " << e.what() << "\n";
    }

    QCoreApplication::processEvents();

#if ENABLE_DSO
    try
    {
        if (sharedPointers->dsoViSession != VI_NULL)
        {
            ViUInt32 bytes_written;
            const char *display_update_command = ":SYSTem:GUI ON";
            ViStatus status = viWrite(sharedPointers->dsoViSession,
                                      (ViConstBuf)display_update_command,
                                      (ViUInt32)strlen(display_update_command),
                                      &bytes_written);
            if (status < VI_SUCCESS)
            {
                std::cerr << "[Cleanup] DSO Error: Could not turn on display updates\n";
            }
            else
            {
                std::cout << "[Cleanup] Display updates turned on successfully\n";
            }

            status = viClose(sharedPointers->dsoViSession);
            if (status < VI_SUCCESS)
            {
                std::cerr << "[Cleanup] Error closing DSO VISA session\n";
            }
            else
            {
                std::cout << "[Cleanup - Init] DSO VISA Session Closed\n";
            }
            sharedPointers->dsoViSession = VI_NULL;
        }

        if (sharedPointers->defaultRMViSession != VI_NULL)
        {
            viClose(sharedPointers->defaultRMViSession);
            std::cout << "[Cleanup - Init] VISA Cleanup Completed\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Init] Error during DSO cleanup: " << e.what() << "\n";
    }
#endif
    emit initializationCancelled();
    std::cout << "[Cleanup - Init] Process Completed Successfully\n";
}

void InitializationPage::setupStdoutRedirection()
{
    if (!logDisplay)
        return; // Safety check
}

// --- Implement restoreStdoutRedirection ---
void InitializationPage::restoreStdoutRedirection()
{
    // Restore std::cout
    if (oldCoutStreamBuf)
    {
        std::cout.rdbuf(oldCoutStreamBuf);
        oldCoutStreamBuf = nullptr; // Avoid restoring multiple times
    }
    customStreamBuf.reset(); // Deletes the custom buffer
}

void InitializationPage::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event); // Call base class implementation
    // Automatically start initialization when the page becomes visible in GUI mode
    if (debug)
    {
        qDebug() << "InitializationPage::showEvent: Triggering startInitialization...";
    }

    // make sure UI is fully loaded before starting initialization
    QCoreApplication::processEvents();

    // wait 10ms before starting initialization
    QTimer::singleShot(10, this, &InitializationPage::startInitialization);
}

void InitializationPage::startInitialization()
{
    if (initializationStarted)
    {
        qDebug() << "InitializationPage::startInitialization called but initialization already "
                    "started or finished.";
        return;
    }
    if (paramConfigFile.paramConfig_name.empty())
    {
        qWarning() << "InitializationPage::startInitialization called before configuration is set.";
        return;
    }

    if (debug)
    {
        qDebug() << "InitializationPage::startInitialization: Starting component initialization...";
    }

    initializationStarted = true; // Set flag
    initilizationError = false;   // Reset error flag
    initializeComponents();       // Call the actual initialization logic
}

void InitializationPage::stopExperiment()
{
    std::cout << "[GUI] Stop requested by user. Shutting down experimental components..."
              << std::endl;

    // 1. Tell the engine's logic loops to terminate.
    if (sharedPointers && sharedPointers->engine)
    {
        sharedPointers->engine->requestStop();
    }

    // 2. Tell the TCP component's networking threads to unblock and terminate.
    if (sharedPointers && sharedPointers->tcpExp)
    {
        sharedPointers->tcpExp->stop();
    }
}
