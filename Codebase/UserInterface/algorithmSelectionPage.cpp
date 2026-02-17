#include "algorithmSelectionPage.h"

#include <algorithm>
#include <map>
#include <qevent.h>
#include <qtimer.h>
#include <QApplication>
#include <QComboBox>
#include <QDrag>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMessageBox>
#include <QMimeData>
#include <QPushButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QVariant>

static bool debug = false;

// Define the algorithm names here
const std::map<QString, std::string> AlgorithmSelectionPage::algorithmMap = {
    {"Hierarchical Search", "HierarchicalSearch"},
    {"IMU-Assist", "IMUAssist"},
    {"Just-In-Time (AP)", "JustInTime_AP"},
    {"Just-In-Time (UE)", "JustInTime_UE"},
    {"Static", "Static"},
    {"Perfect Alignment", "PerfectAlignment"},
    {"None", "NONE"} // Explicit "None" option
};

// Helper function to get valid internal algorithm names
std::vector<std::string> AlgorithmSelectionPage::getValidInternalAlgorithmNames()
{
    std::vector<std::string> validNames;
    validNames.reserve(algorithmMap.size());
    for (const auto &pair : algorithmMap)
    {
        validNames.push_back(pair.second);
    }
    return validNames;
}

DraggableListWidget::DraggableListWidget(QWidget *parent)
    : QListWidget(parent)
{
}

void DraggableListWidget::startDrag(Qt::DropActions supportedActions)
{
    QListWidgetItem *item = currentItem();
    if (!item)
        return;

    QMimeData *mimeData = new QMimeData;
    mimeData->setText(item->text());

    QDrag *drag = new QDrag(this);
    drag->setMimeData(mimeData);

    // Start the drag operation
    Qt::DropAction dropAction = drag->exec(supportedActions, Qt::CopyAction);
    Q_UNUSED(dropAction); // Suppress unused variable warning
}

UnitDropTargetFrame::UnitDropTargetFrame(int unitIdx, const QString &unitName, QWidget *parent)
    : QFrame(parent), unitIndex_(unitIdx)
{
    setAcceptDrops(true);
    setFrameShape(QFrame::StyledPanel);

    // Base styling for the unit box
    setStyleSheet(R"(
        QFrame {
            background-color: #1f1f1f;
            border: 2px solid #3a3a3a;
            border-radius: 5px;
            padding: 5px;
        }
        QFrame:hover {
            border: 2px solid #C8102E; /* Highlight on hover */
        }
        QFrame[dragActive="true"] { /* Custom property for active drag */
            border: 2px dashed #C8102E;
            background-color: #2f1f1f; /* Slightly lighter background when drag is active */
        }
    )");
    // Set initial custom property for drag highlighting
    setProperty("dragActive", false);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(5);

    unitNameLabel_ = new QLabel(unitName, this);
    unitNameLabel_->setStyleSheet("font-size: 16px; font-weight: bold; color: white;");
    layout->addWidget(unitNameLabel_, 0, Qt::AlignCenter);

    assignedAlgoLabel_ = new QLabel("None", this); // Initial assignment
    assignedAlgoLabel_->setStyleSheet(
        "font-size: 14px; color: #BBBBBB;"); // Dimmer for assigned algo name
    layout->addWidget(assignedAlgoLabel_, 0, Qt::AlignCenter);

    layout->addStretch(1); // Push content to top

    QLabel *dropHintLabel = new QLabel("Drag & Drop Algorithm Here", this);
    dropHintLabel->setStyleSheet("font-size: 10px; color: #555555;");
    layout->addWidget(dropHintLabel, 0, Qt::AlignCenter);
}

void UnitDropTargetFrame::setAssignedAlgorithm(const QString &algoName)
{
    assignedAlgoLabel_->setText(algoName);
}

void UnitDropTargetFrame::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasText())
    {
        event->acceptProposedAction();
        // Set a property to indicate an active drag over this widget
        setProperty("dragActive", true);
        style()->polish(this); // Force stylesheet re-evaluation
    }
    else
    {
        event->ignore();
    }
}

void UnitDropTargetFrame::dragMoveEvent(QDragMoveEvent *event)
{
    if (event->mimeData()->hasText())
    {
        event->acceptProposedAction();
        // Keep dragActive true, ensures persistent visual feedback during drag over
        if (!property("dragActive").toBool())
        {
            setProperty("dragActive", true);
            style()->polish(this);
        }
    }
    else
    {
        event->ignore();
        // If the data is not accepted, ensure dragActive is false
        if (property("dragActive").toBool())
        {
            setProperty("dragActive", false);
            style()->polish(this);
        }
    }
}

void UnitDropTargetFrame::dropEvent(QDropEvent *event)
{
    const QMimeData *mimeData = event->mimeData();
    if (mimeData->hasText())
    {
        QString algorithmDisplayName = mimeData->text();
        auto it = AlgorithmSelectionPage::algorithmMap.find(algorithmDisplayName);
        if (it != AlgorithmSelectionPage::algorithmMap.end())
        {
            emit algorithmDropped(unitIndex_, QString::fromStdString(it->second));
            event->acceptProposedAction();
        }
        else
        {
            event->ignore();
            QMessageBox::warning(this,
                                 "Invalid Algorithm",
                                 "Dragged item is not a recognized algorithm.");
        }
    }
    else
    {
        event->ignore();
    }
    // Always ensure dragActive is reset after a drop, regardless of accept/ignore
    setProperty("dragActive", false);
    // Use singleShot(0) to ensure polish happens AFTER the entire drag operation concludes
    QTimer::singleShot(0, this, [this]()
                       { style()->polish(this); });
}

void UnitDropTargetFrame::dragLeaveEvent(QDragLeaveEvent *event)
{
    Q_UNUSED(event);
    if (property("dragActive").toBool())
    {
        setProperty("dragActive", false);
        QTimer::singleShot(0, this, [this]()
                           { style()->polish(this); });
    }
}

AlgorithmSelectionPage::AlgorithmSelectionPage(QWidget *parent)
    : QWidget(parent)
{
    setupUI();
}

// Set the base parameter config file and update UI
void AlgorithmSelectionPage::setParamConfigFile(const ConfigFile &configFile)
{
    paramConfigFile = configFile; // Store a mutable copy
    configName = QString::fromStdString(paramConfigFile.paramConfig_name);

    // Create the visual drop targets for units
    createUnitDropTargets();
    // Update all unit display boxes with their current assignments
    updateAssignmentDisplay();

    nextButton->setEnabled(
        true); // Enable next by default, user can proceed if current config is fine
}

// Setup the UI elements and layout
void AlgorithmSelectionPage::setupUI()
{
    mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);
    mainLayout->setContentsMargins(10, 10, 10, 10);

    // --- Header Section ---
    headerLayout = new QHBoxLayout;
    QLabel *titleLabel = new QLabel("Algorithm Assignment", this);
    titleLabel->setStyleSheet("font-size: 42px; font-weight: bold; padding-top: 25px");
    headerLayout->addWidget(titleLabel, 0, Qt::AlignLeft | Qt::AlignBottom);
    headerLayout->addStretch(1);
    // --- Logos ---
    QLabel *kthlogo = new QLabel(this);
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

    // --- Main Content Area (Split into Left: Algorithms, Right: Units Grid) ---
    QHBoxLayout *contentLayout = new QHBoxLayout();
    mainLayout->addLayout(contentLayout, 1); // Content area takes all vertical stretch

    // --- Left Column: Available Algorithms Frame (Drag Source) ---
    availableAlgorithmsFrame = new QFrame(this);
    availableAlgorithmsFrame->setFrameShape(QFrame::StyledPanel);
    availableAlgorithmsFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *algoFrameVLayout = new QVBoxLayout(availableAlgorithmsFrame);
    algoFrameVLayout->setContentsMargins(10, 10, 10, 10);
    algoFrameVLayout->setSpacing(8);

    QLabel *availableAlgoLabel = new QLabel("Available Algorithms:", availableAlgorithmsFrame);
    availableAlgoLabel->setStyleSheet(
        "font-size: 18px; font-weight: bold; border-bottom: 2px solid #3a3a3a; border-radius: 0px; "
        "padding-bottom: 5px; color: white;");
    algoFrameVLayout->addWidget(availableAlgoLabel);

    availableAlgorithms = new DraggableListWidget(
        availableAlgorithmsFrame);
    availableAlgorithms->setStyleSheet(R"(
        QListWidget {
            font-size: 16px;
            background-color: #1f1f1f;
            border: 1px solid #3a3a3a;
            border-radius: 3px;
            color: white;
            padding: 3px;
        }
        QListWidget::item { padding: 4px; }
        QListWidget::item:selected {
            background-color: #C8102E; /* Red highlight for selected algorithm */
            color: white;
            border-radius: 2px;
        }
    )");
    availableAlgorithms->setDragEnabled(true);
    availableAlgorithms->setDragDropMode(QAbstractItemView::DragOnly);
    availableAlgorithms->setDefaultDropAction(
        Qt::MoveAction);

    // Populate list with display names and store internal names in UserRole
    for (const auto &pair : algorithmMap)
    {
        QListWidgetItem *item = new QListWidgetItem(pair.first); // Display name
        item->setData(Qt::UserRole,
                      QString::fromStdString(pair.second)); // Store internal name as QString
        availableAlgorithms->addItem(item);
    }
    algoFrameVLayout->addWidget(availableAlgorithms, 1); // List takes vertical stretch

    // "Apply to ALL" button
    QPushButton *applyToAllButton = new QPushButton("Apply Selected to ALL Units",
                                                    availableAlgorithmsFrame);
    applyToAllButton->setStyleSheet(
        "font-size: 16px; padding: 8px; background-color: #3a3a3a; color: "
        "white; border: 1px solid #C8102E; border-radius: 5px;");
    connect(applyToAllButton, &QPushButton::clicked, this, [this]()
            {
        QListWidgetItem *selectedAlgoItem = availableAlgorithms->currentItem();
        if (selectedAlgoItem) {
            QString internalAlgoName = selectedAlgoItem->data(Qt::UserRole)
                                           .toString(); // Get internal name
            handleAlgorithmDrop(-1, internalAlgoName);  // -1 signifies "ALL" units
        } else {
            QMessageBox::information(
                this,
                "No Algorithm Selected",
                "Please select an algorithm from the list to apply to ALL units.");
        } });
    algoFrameVLayout->addWidget(applyToAllButton);

    contentLayout->addWidget(availableAlgorithmsFrame,
                             1); // Left column takes 1 unit of horizontal stretch

    // --- Right Column: Unit Drop Targets Frame ---
    unitsDisplayFrame = new QFrame(this);
    unitsDisplayFrame->setFrameShape(QFrame::StyledPanel);
    unitsDisplayFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *unitsFrameVLayout = new QVBoxLayout(unitsDisplayFrame);
    unitsFrameVLayout->setContentsMargins(10, 10, 10, 10);
    unitsFrameVLayout->setSpacing(8);

    QLabel *unitsTitleLabel = new QLabel("Assign to Individual Unit:", unitsDisplayFrame);
    unitsTitleLabel->setStyleSheet(
        "font-size: 18px; font-weight: bold; border-bottom: 2px solid #3a3a3a; border-radius: 0px; "
        "padding-bottom: 5px; color: white;");
    unitsFrameVLayout->addWidget(unitsTitleLabel);

    unitsGridLayout = new QGridLayout();              // Grid layout for unit boxes
    unitsGridLayout->setSpacing(10);                  // Spacing between unit boxes
    unitsFrameVLayout->addLayout(unitsGridLayout, 1); // Grid takes vertical stretch

    unitsFrameVLayout->addStretch(1); // Push units to top if fewer units than space

    contentLayout->addWidget(unitsDisplayFrame,
                             2); // Right column takes 2 units of horizontal stretch

    // --- Navigation Buttons ---
    QHBoxLayout *navButtonLayout = new QHBoxLayout();
    navButtonLayout->setSpacing(10);
    backButton = new QPushButton("Back", this);
    backButton->setStyleSheet("font-size: 16px; padding: 10px; background-color: #3a3a3a; color: "
                              "white; border: none; border-radius: 5px;");
    backButton->setFixedWidth(75);
    nextButton = new QPushButton("Run Algorithms", this);
    nextButton->setStyleSheet("font-size: 16px; padding: 10px; background-color: #C8102E; color: "
                              "white; border: none; border-radius: 5px;");

    navButtonLayout->addWidget(backButton);
    navButtonLayout->addWidget(nextButton);
    navButtonLayout->setStretchFactor(backButton, 0);
    navButtonLayout->setStretchFactor(nextButton, 1);
    mainLayout->addLayout(navButtonLayout);

    // Set main layout for the page
    setLayout(mainLayout);

    // Connections
    connect(backButton, &QPushButton::clicked, this, &AlgorithmSelectionPage::goBack);
    connect(nextButton, &QPushButton::clicked, this, &AlgorithmSelectionPage::nextClicked);
}

// Creates the visual QFrame drop targets for each unit
void AlgorithmSelectionPage::createUnitDropTargets()
{
    // Clear any existing unit boxes before recreating (if setParamConfigFile is called multiple times)
    QLayoutItem *item;
    while ((item = unitsGridLayout->takeAt(0)) != nullptr)
    {
        if (item->widget())
        {
            delete item->widget();
        }
        delete item;
    }
    unitWidgets.clear(); // Clear the map of stored widgets

    int row = 0;
    int col = 0;
    const int MAX_COLS = 3; // Adjust number of columns in the grid as needed

    for (int i = 0; i < paramConfigFile.engine_units.size(); ++i)
    {
        const engineUnit &unit = paramConfigFile.engine_units[i];
        UnitDropTargetFrame *unitBox = new UnitDropTargetFrame(i,
                                                               QString::fromStdString(unit.label),
                                                               unitsDisplayFrame);
        unitsGridLayout->addWidget(unitBox, row, col);

        // Connect the custom signal from the UnitDropTargetFrame to a slot in AlgorithmSelectionPage
        connect(unitBox,
                &UnitDropTargetFrame::algorithmDropped,
                this,
                &AlgorithmSelectionPage::handleAlgorithmDrop);

        // Store pointer to the unitBox itself for later updates via its methods
        unitWidgets[i].unitBox = unitBox; // Store the box itself

        col++;
        if (col >= MAX_COLS)
        {
            col = 0;
            row++;
        }
    }
    unitsGridLayout->setRowStretch(row, 1);             // Stretch the last row to fill vertical space if needed
    unitsGridLayout->setColumnStretch(MAX_COLS - 1, 1); // Ensure last column stretches if not full
}

// Slot to handle an algorithm being dropped onto a unit or applied to all
void AlgorithmSelectionPage::handleAlgorithmDrop(int unitIndex, const QString &algorithmInternalName)
{
    bool configWasModified = false;

    if (unitIndex == -1)
    { // Special case for "Apply to ALL Units" button
        // Apply the preset algorithm to all units in the internal config
        std::optional<ConfigFile> result = processAlgorithmPreset(algorithmInternalName.toStdString(), this->paramConfigFile);
        if (result)
        {
            // Update the page's config with the new, all-units assignment
            this->paramConfigFile = result.value();
            configWasModified = true;
        }
        else
        {
            QMessageBox::warning(
                this,
                "Assignment Error",
                "Could not apply the selected algorithm to ALL units. Check logs.");
        }
    }
    else
    { // Specific unit (drag-and-drop onto a UnitDropTargetFrame)
        if (unitIndex >= 0 && unitIndex < paramConfigFile.engine_units.size())
        {
            // Check if the algorithm for this specific unit is actually changing
            if (paramConfigFile.engine_units[unitIndex].algorithm != algorithmInternalName.toStdString())
            {
                paramConfigFile.engine_units[unitIndex].algorithm = algorithmInternalName
                                                                        .toStdString();
                configWasModified = true;
            }
        }
        else
        {
            qWarning() << "Invalid unit index" << unitIndex << "received from drop event.";
        }
    }

    // If any changes were made, update the UI and enable the "Next" button
    if (configWasModified)
    {
        nextButton->setEnabled(true); // A valid change has occurred
        updateAssignmentDisplay();    // Refresh all unit display boxes
    }
}

// Update the assignment display for all units
void AlgorithmSelectionPage::updateAssignmentDisplay()
{
    // Iterate through all units in the internal config and update their corresponding UnitDropTargetFrame
    for (int i = 0; i < paramConfigFile.engine_units.size(); ++i)
    {
        const engineUnit &unit = paramConfigFile.engine_units[i];
        QString displayAlgoName = "None"; // Default if not found or is "none"/" "

        // Find the display name for the unit's assigned internal algorithm name
        std::string internalName = unit.algorithm;
        if (internalName.empty())
        {
            internalName = "NONE"; // Treat empty as "NONE" for display lookup
        }

        bool found = false;
        for (const auto &pair : algorithmMap)
        {
            if (pair.second == internalName)
            {                                 // Match internal name (value)
                displayAlgoName = pair.first; // Get display name (key)
                found = true;
                break;
            }
        }
        // If internal name wasn't found in map values (and wasn't "none"), show it directly with a warning
        if (!found && internalName != "NONE")
        { // Note: "NONE" is now handled by the loop and found=true
            displayAlgoName = QString::fromStdString(internalName) + " (?)";
            qWarning() << "Internal algorithm name '" << QString::fromStdString(internalName)
                       << "' for unit '" << QString::fromStdString(unit.label)
                       << "' not found in algorithmMap values!";
        }

        // Update the corresponding UnitDropTargetFrame widget
        if (unitWidgets.contains(i) && unitWidgets[i].unitBox)
        {
            // Cast to UnitDropTargetFrame* to access its specific methods
            UnitDropTargetFrame *unitBox = static_cast<UnitDropTargetFrame *>(
                unitWidgets[i].unitBox);
            unitBox->setAssignedAlgorithm(displayAlgoName);
        }
        else
        {
            qWarning() << "UnitDropTargetFrame not found for unit index" << i;
        }
    }
}

// Slot for the "Next" button click
void AlgorithmSelectionPage::nextClicked()
{
    bool allUnitsAssignedValidly = true;
    for (const auto &unit : paramConfigFile.engine_units)
    {
        std::string internalName = unit.algorithm;

        // Treat empty algorithm string as "NONE" for validation
        if (internalName.empty())
        {
            internalName = "NONE";
        }

        // Check if the assigned internal name is one of our known valid internal algorithm names
        bool isValid = (std::find(AlgorithmSelectionPage::getValidInternalAlgorithmNames().begin(),
                                  AlgorithmSelectionPage::getValidInternalAlgorithmNames().end(),
                                  internalName) != AlgorithmSelectionPage::getValidInternalAlgorithmNames().end());

        if (!isValid)
        {
            QMessageBox::warning(this,
                                 "Invalid Assignment",
                                 QString("Unknown or invalid algorithm '%1' assigned for unit: %2")
                                     .arg(QString::fromStdString(unit.algorithm))
                                     .arg(QString::fromStdString(unit.label)));
            allUnitsAssignedValidly = false;
        }
    }

    if (allUnitsAssignedValidly)
    {
        emit algorithmsAssigned(paramConfigFile); // Emit the signal with the final config
    }
    else
    {
        // If validation failed, keep the "Next" button enabled so the user can fix it
        nextButton->setEnabled(true);
    }
}

// Slot for the "Back" button click
void AlgorithmSelectionPage::goBack()
{
    emit backRequested();
}

// Key press event handler (Enter key)
void AlgorithmSelectionPage::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter)
    {
        nextClicked(); // Trigger the next button's logic
    }
    else
    {
        QWidget::keyPressEvent(event); // Call base class implementation for other keys
    }
}

// Static function to process algorithm preset (applies to all units)
std::optional<ConfigFile> AlgorithmSelectionPage::processAlgorithmPreset(
    const std::string &presetAlgorithmName, const ConfigFile &currentConfig)
{
    // 1. Validate the preset algorithm name
    std::vector<std::string> validNames = getValidInternalAlgorithmNames();
    std::string nameToValidate = presetAlgorithmName;

    if (nameToValidate.empty())
    {
        qInfo() << "Preset algorithm name is empty, treating as 'NONE' for validation.";
        nameToValidate = "NONE"; // Treat empty as "NONE" for validation check
    }

    bool isValid = (std::find(validNames.begin(), validNames.end(), nameToValidate) != validNames.end());

    if (!isValid)
    {
        qWarning() << "Invalid preset algorithm name specified:"
                   << QString::fromStdString(presetAlgorithmName);
        qWarning() << "Valid internal names are:";
        for (const auto &name : validNames)
        {
            qWarning() << "  -" << QString::fromStdString(name);
        }
        return std::nullopt;
    }

    // 2. Check if there are units to apply to
    if (currentConfig.engine_units.empty())
    {
        qWarning()
            << "Cannot apply algorithm preset: No engine units defined in the configuration.";
        return std::nullopt;
    }

    // 3. Create a modifiable copy and apply the algorithm
    ConfigFile updatedConfig = currentConfig;
    if (debug)
    {
        qInfo() << "Applying algorithm '" << QString::fromStdString(nameToValidate)
                << "' to all units.";
    }
    for (engineUnit &unit : updatedConfig.engine_units)
    {
        unit.algorithm = nameToValidate; // Update all units
    }

    // 4. Return the updated configuration
    return updatedConfig;
}
