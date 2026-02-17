#include "realtimePage.h"
#include <QBrush>
#include <QFontDatabase>
#include <QFormLayout>
#include <QGraphicsEllipseItem>
#include <QGroupBox>
#include <QLabel>
#include <QPen>
#include <QScrollArea>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtGui/qquaternion.h>
#include <qcoreapplication.h>
#include <qscrollbar.h>
#include "Software/mobileTHzEngine/mobileTHzEngine.h"
#include "Software/sharedPointersDefinition.h"
#include "Experimental/tcpExp/tcpExp.h"

RealtimePage::RealtimePage(QWidget *parent)
    : QWidget(parent), updateTimer(new QTimer(this))
{
    setupUI();
    connect(updateTimer, &QTimer::timeout, this, &RealtimePage::updateDisplay);
    connect(viewSimulationButton,
            &QPushButton::clicked,
            this,
            &RealtimePage::viewSimulationRequested);
}

RealtimePage::~RealtimePage()
{
}

void RealtimePage::setupPage(std::shared_ptr<SharedPointers> sp, const ConfigFile &config)
{
    this->sharedPointers = sp;
    this->paramConfig = config;

    // --- Set Initial State ---
    // Clear any data from a previous run
    messageLog->clear();
    connectedNodesList->clear();

    // Set the role label
    roleLabel->setText(QString::fromStdString(paramConfig.experimental.role));

    // --- Control Visibility of Button and Nodes List ---
    bool isObserver = (paramConfig.experimental.role == "OBSERVER");
    viewSimulationButton->setVisible(isObserver); // Show button ONLY for Observer

    QWidget *nodesFrame = connectedNodesList->parentWidget();
    if (nodesFrame)
    {
        nodesFrame->setVisible(isObserver);
    }

    // The update timer will be started by the showEvent
}

void RealtimePage::setupUI()
{
    // --- Main Layout ---
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);

    // --- Header ---
    QHBoxLayout *headerLayout = new QHBoxLayout();
    QLabel *titleLabel = new QLabel("Real-Time Monitor", this);
    titleLabel->setStyleSheet("font-size: 42px; font-weight: bold; padding-top: 25px");
    headerLayout->addWidget(titleLabel, 0, Qt::AlignLeft | Qt::AlignBottom);
    headerLayout->addStretch(1);
    mainLayout->addLayout(headerLayout);

    // --- Content Area (split view) ---
    QHBoxLayout *contentLayout = new QHBoxLayout();
    mainLayout->addLayout(contentLayout, 1); // Give content area stretching priority

    // --- Left Column (Status) ---
    QVBoxLayout *leftColumnLayout = new QVBoxLayout();
    contentLayout->addLayout(leftColumnLayout, 1); // Stretch factor 1

    // Status Frame
    QFrame *statusFrame = new QFrame(this);
    statusFrame->setFrameShape(QFrame::StyledPanel);
    statusFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QFormLayout *statusLayout = new QFormLayout(statusFrame);
    statusLayout->setLabelAlignment(Qt::AlignRight);
    statusLayout->setRowWrapPolicy(QFormLayout::WrapAllRows);

    QLabel *roleTitle = new QLabel("Role:", statusFrame);
    roleTitle->setStyleSheet("font-size: 16px; font-weight: bold;");
    roleLabel = new QLabel("N/A", statusFrame);
    roleLabel->setStyleSheet("font-size: 16px;");
    statusLayout->addRow(roleTitle, roleLabel);

    QLabel *statusTitle = new QLabel("Status:", statusFrame);
    statusTitle->setStyleSheet("font-size: 16px; font-weight: bold;");
    statusLabel = new QLabel("Disconnected", statusFrame);
    statusLabel->setStyleSheet("font-size: 16px; color: #C8102E; font-weight: bold;");
    statusLayout->addRow(statusTitle, statusLabel);

    leftColumnLayout->addWidget(statusFrame);

    // Connected Nodes Frame (for Observer)
    QFrame *nodesFrame = new QFrame(this);
    nodesFrame->setObjectName("nodesFrame");
    nodesFrame->setFrameShape(QFrame::StyledPanel);
    nodesFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *nodesLayout = new QVBoxLayout(nodesFrame);

    QLabel *nodesTitle = new QLabel("Connected Nodes", nodesFrame);
    nodesTitle->setStyleSheet("font-size: 18px; font-weight: bold; border-bottom: 2px solid "
                              "#3a3a3a; border-radius: 0px; padding-bottom: 5px;");
    nodesLayout->addWidget(nodesTitle);

    connectedNodesList = new QListWidget(nodesFrame);
    connectedNodesList->setStyleSheet(
        "QListWidget { font-size: 14px; background-color: #1f1f1f; border: none; }");
    nodesLayout->addWidget(connectedNodesList);

    leftColumnLayout->addWidget(nodesFrame);
    leftColumnLayout->addStretch(1); // Pushes frames to the top

    viewSimulationButton = new QPushButton("View Results", this);
    viewSimulationButton->setStyleSheet("QPushButton {"
                                        "    font-size: 16px;"
                                        "    color: white;"
                                        "    font-weight: bold;"
                                        "    background-color: #3a3a3a;"
                                        "    border: 1px solid #C8102E;"
                                        "    border-radius: 5px;"
                                        "    padding: 10px;"
                                        "}"
                                        "QPushButton:hover {"
                                        "    background-color: #C8102E;"
                                        "    border-color: #E01133;"
                                        "}"
                                        "QPushButton:pressed {"
                                        "    background-color: #A00D24;"
                                        "    border-color: #A00D24;"
                                        "}"
                                        "QPushButton:disabled {"
                                        "    background-color: #2a2a2a;"
                                        "    color: #777777;"
                                        "    border-color: #444444;"
                                        "}");
    viewSimulationButton->setCursor(Qt::PointingHandCursor);
    viewSimulationButton->hide(); // Initially hidden
    leftColumnLayout->addWidget(viewSimulationButton);

    leftColumnLayout->addStretch(1); // Pushes frames and button to the top

    // --- Right Column (Log) ---
    QVBoxLayout *rightColumnLayout = new QVBoxLayout();
    contentLayout->addLayout(rightColumnLayout, 4);

    QFrame *logFrame = new QFrame(this);
    logFrame->setFrameShape(QFrame::StyledPanel);
    logFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *logLayout = new QVBoxLayout(logFrame);

    QLabel *logTitle = new QLabel("Message Log", logFrame);
    logTitle->setStyleSheet("font-size: 18px; font-weight: bold; border-bottom: 2px solid #3a3a3a; "
                            "border-radius: 0px; padding-bottom: 5px;");
    logLayout->addWidget(logTitle);

    messageLog = new QTextEdit(logFrame);
    messageLog->setReadOnly(true);
    messageLog->setStyleSheet("QTextEdit { font-family: Consolas, 'Courier New', monospace; "
                              "font-size: 14px; background-color: #1f1f1f; border: none; }");
    logLayout->addWidget(messageLog);

    rightColumnLayout->addWidget(logFrame);
}

void RealtimePage::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
    startUpdating();
}

void RealtimePage::hideEvent(QHideEvent *event)
{
    QWidget::hideEvent(event);
    stopUpdating();
}

void RealtimePage::startUpdating()
{
    if (!updateTimer->isActive())
    {
        updateTimer->start(250); // Update 4 times per second
    }
}

void RealtimePage::stopUpdating()
{
    updateTimer->stop();
}

void RealtimePage::updateDisplay()
{
    if (!sharedPointers || !sharedPointers->tcpExp)
    {
        return; // Safety check, do nothing if tcp component isn't available
    }

    auto tcp = sharedPointers->tcpExp.get();

    // --- 1. Update Connection Status ---
    tcpExp::ConnectionStatus status = tcp->getConnectionStatus();
    switch (status)
    {
    case tcpExp::ConnectionStatus::Connected:
        statusLabel->setText("Connected");
        statusLabel->setStyleSheet("font-size: 16px; color: #50C878; font-weight: bold;");
        break;
    case tcpExp::ConnectionStatus::Connecting:
        statusLabel->setText("Connecting...");
        statusLabel->setStyleSheet("font-size: 16px; color: #FFBF00; font-weight: bold;");
        break;
    case tcpExp::ConnectionStatus::Disconnected:
    default:
        statusLabel->setText("Disconnected");
        statusLabel->setStyleSheet("font-size: 16px; color: #C8102E; font-weight: bold;");
        break;
    }

    // --- 2. Update Connected Nodes List (Observer Only) ---
    if (paramConfig.experimental.role == "OBSERVER")
    {
        QStringList currentClients = tcp->getConnectedClients();
        // Simple but effective: clear and re-populate
        if (connectedNodesList->count() != currentClients.count())
        {
            connectedNodesList->clear();
            connectedNodesList->addItems(currentClients);
        }
    }

    // --- 3. Process and Display New Messages ---
    std::vector<tcpExp::LogEntry> newMessages = tcp->getAndClearLogQueue();
    if (!newMessages.empty())
    {
        for (const auto &msg : newMessages)
        {
            QString htmlMessage;
            QString directionColor = (msg.direction == tcpExp::Direction::Sent)
                                         ? "#ADD8E6"
                                         : "#98FB98";
            QString typeColor;

            switch (msg.type)
            {
            case tcpExp::MessageType::Update:
                typeColor = "#87CEEB";
                break;
            case tcpExp::MessageType::SensorData:
                typeColor = "#90EE90";
                break;
            case tcpExp::MessageType::Command:
                typeColor = "#FFD700";
                break;
            case tcpExp::MessageType::Connection:
                typeColor = "#D3D3D3";
                break;
            default:
                typeColor = "white";
                break;
            }

            // Component 1: Timestamp
            QString timestamp_html = QString("<font color='#AAAAAA'>[%1]</font>").arg(msg.timestamp);

            // Component 2: Message Type
            QString type_html = QString("<font color='%1'><b>%2</b></font>")
                                    .arg(typeColor, msg.type_str);

            // Component 3: Direction and Role
            QString direction_html;
            const QString my_role = QString::fromStdString(paramConfig.experimental.role);
            const QString other_role = msg.role;

            if (msg.direction == tcpExp::Direction::Sent)
            {
                // Format: [Me] --> [Other]
                direction_html = QString("<font color='%1'>[%2] %3 [%4]</font>")
                                     .arg(directionColor, my_role, msg.direction_str, other_role);
            }
            else
            { // Received
                // Format: [Other] <-- [Me]
                direction_html = QString("<font color='%1'>[%2] %3 [%4]</font>")
                                     .arg(directionColor, other_role, msg.direction_str, my_role);
            }

            // Handle cases where the other role might be empty (like a connection message)
            if (other_role.isEmpty())
            {
                if (msg.direction == tcpExp::Direction::Sent)
                {
                    direction_html = QString("<font color='%1'>[%2] %3</font>")
                                         .arg(directionColor, my_role, msg.direction_str);
                }
                else
                {
                    direction_html = QString("<font color='%1'>%2 [%3]</font>")
                                         .arg(directionColor, msg.direction_str, my_role);
                }
            }

            // Assemble the final line from the safe components
            QString formatted_log_line = QString("%1 %2 %3: %4")
                                             .arg(timestamp_html)
                                             .arg(type_html)
                                             .arg(direction_html)
                                             .arg(msg.content);

            messageLog->append(formatted_log_line);
        }
        messageLog->verticalScrollBar()->setValue(messageLog->verticalScrollBar()->maximum());
    }
}
