
#include "positionConfigPage.h"
#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <QGridLayout>
#include <QMouseEvent>
#include <QPen>
#include <QQuaternion>
#include <QWheelEvent>
#include "Software/jsonReader/jsonReader.hpp"
#include "UserInterface/Utils/directionArrowItem.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <qscrollbar.h>
#include <qtimer.h>

using json = nlohmann::json;
namespace fs = std::filesystem;

PositionConfigPage::PositionConfigPage(QWidget *parent)
    : QWidget(parent), panning(false), zoomFactor(1.0), viewRange(200.0), unitSizeWorld(10.0), unitSizePixels(50.0)
{
    setupUI();
    setFocusPolicy(Qt::StrongFocus);
}

void PositionConfigPage::setParamConfig(const ConfigFile &config)
{
    paramConfigFile = config;

    // We wait so UI can load before loading config files
    QTimer::singleShot(10, this, &PositionConfigPage::refreshConfigFiles);
}

void PositionConfigPage::setupUI()
{
    mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);
    // Note: No contentsMargins set here, relying on default or parent

    // --- Header Section ---
    headerLayout = new QHBoxLayout;
    QLabel *titleLabel = new QLabel("Position Config", this);
    titleLabel->setStyleSheet("font-size: 42px; font-weight: bold; padding-top: 25px");
    headerLayout->addWidget(titleLabel, 0, Qt::AlignLeft | Qt::AlignBottom);
    headerLayout->addStretch(1);

    // --- Logos ---
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
    mainLayout->addLayout(headerLayout);

    // --- Top: Config File Selection ---
    QFrame *configSelectionFrame = new QFrame(this);
    configSelectionFrame->setFrameShape(QFrame::StyledPanel);
    configSelectionFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    configSelectionLayout = new QVBoxLayout(configSelectionFrame);
    configSelectionLayout->setContentsMargins(10, 10, 10, 10);

    QLabel *selectLabel = new QLabel("Position Config:", configSelectionFrame);
    selectLabel->setStyleSheet("font-size: 18px; font-weight: bold; border-bottom: 2px solid "
                               "#3a3a3a; border-radius: 0px; padding-bottom: 5px;");
    configSelectionLayout->addWidget(selectLabel);

    QHBoxLayout *loadSaveLayout = new QHBoxLayout();
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
    loadSaveLayout->addWidget(configFileCombo, 1); // Combo box stretches horizontally

    saveConfigButton = new QPushButton("Save Config", configSelectionFrame);
    saveConfigButton->setFixedWidth(100);
    saveConfigButton->setStyleSheet(
        "font-size: 16px; padding: 2px; background-color: #3a3a3a; color: "
        "white; border: 1px solid #C8102E; border-radius: 5px;");
    connect(saveConfigButton, &QPushButton::clicked, this, &PositionConfigPage::saveConfig);
    loadSaveLayout->addWidget(saveConfigButton, 0); // Save button fixed width

    configSelectionLayout->addLayout(loadSaveLayout);
    mainLayout->addWidget(configSelectionFrame,
                          0); // Config selection frame takes minimal vertical space

    // --- Content Layout (Scene + Properties) ---
    QHBoxLayout *contentLayout = new QHBoxLayout();
    mainLayout->addLayout(contentLayout, 1); // This layout gets all available vertical stretch

    // --- XY Scene (Left Column) ---
    scene = new QGraphicsScene(this);
    scene->setSceneRect(-200, -200, 400, 400); // Initial scene rect
    scene->setBackgroundBrush(Qt::transparent);

    // --- Grid ---
    QPen gridPen(QColor(50, 50, 50));
    gridPen.setWidth(1);
    gridPen.setStyle(Qt::DashLine);
    // Grid drawing moved to updateScene() for dynamic scaling
    QPen axisPen(Qt::white);
    axisPen.setWidth(2);
    // Axes drawing moved to updateScene() for dynamic scaling

    view = new QGraphicsView(scene, this);
    view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding); // Allow view to expand
    view->setMouseTracking(true);
    view->setRenderHint(QPainter::Antialiasing);
    connect(scene,
            &QGraphicsScene::selectionChanged,
            this,
            &PositionConfigPage::updatePropertiesDisplay);
    view->viewport()->installEventFilter(this);
    contentLayout->addWidget(view, 5); // Map view gets 5 units of horizontal stretch

    // --- Properties (Right Column) ---
    rightColumnLayout = new QVBoxLayout(); // Container for propertiesFrame and timeControlFrame
    contentLayout->addLayout(rightColumnLayout,
                             3); // Right column gets 3 units of horizontal stretch

    propertiesFrame = new QFrame(this);
    propertiesFrame->setFrameShape(QFrame::StyledPanel);
    propertiesFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *propertiesFrameLayout = new QVBoxLayout(propertiesFrame);
    propertiesFrameLayout->setContentsMargins(10, 10, 10, 10); // Padding inside the frame

    // Unit Properties
    unitGroupBox = new QGroupBox("Unit", propertiesFrame);
    unitGroupBox->setStyleSheet(
        "QGroupBox { font-size: 16px; font-weight: bold; color: white; border: 1px solid #3a3a3a; "
        "border-radius: 5px; margin-top: 14px; } QGroupBox::title { subcontrol-origin: margin; "
        "subcontrol-position: top left; padding: 0 3px; }");
    QFormLayout *unitFormLayout = new QFormLayout(unitGroupBox);
    unitFormLayout->setLabelAlignment(Qt::AlignLeft); // Align labels to left
    unitNameLabel = new QLabel(unitGroupBox);
    unitNameLabel->setStyleSheet("font-size: 14px; color: white;");
    unitFormLayout->addRow("Name:", unitNameLabel);
    unitIDLabel = new QLabel(unitGroupBox);
    unitIDLabel->setStyleSheet("font-size: 14px; color: white;");
    unitFormLayout->addRow("ID:", unitIDLabel);
    propertiesFrameLayout->addWidget(unitGroupBox);

    // Position Properties
    positionGroupBox = new QGroupBox("Position (in Meters)", propertiesFrame);
    positionGroupBox->setStyleSheet(
        "QGroupBox { font-size: 16px; font-weight: bold; color: white; border: 1px solid #3a3a3a; "
        "border-radius: 5px; margin-top: 14px; } QGroupBox::title { subcontrol-origin: margin; "
        "subcontrol-position: top left; padding: 0 3px; }");
    QFormLayout *positionFormLayout = new QFormLayout(positionGroupBox);
    positionFormLayout->setLabelAlignment(Qt::AlignLeft);
    posXLabel = new QLabel(positionGroupBox);
    posXLabel->setStyleSheet("font-size: 14px; color: white;");
    posYLabel = new QLabel(positionGroupBox);
    posYLabel->setStyleSheet("font-size: 14px; color: white;");
    posZLabel = new QLabel(positionGroupBox);
    posZLabel->setStyleSheet("font-size: 14px; color: white;");
    positionFormLayout->addRow("X:", posXLabel);
    positionFormLayout->addRow("Y:", posYLabel);
    positionFormLayout->addRow("Z:", posZLabel);
    propertiesFrameLayout->addWidget(positionGroupBox);

    // Orientation Properties
    orientationGroupBox = new QGroupBox("Orientation (Quaternion)", propertiesFrame);
    orientationGroupBox->setStyleSheet(
        "QGroupBox { font-size: 16px; font-weight: bold; color: white; border: 1px solid #3a3a3a; "
        "border-radius: 5px; margin-top: 14px;} QGroupBox::title {subcontrol-origin: "
        "margin;subcontrol-position: top left; padding: 0 3px;}");
    QFormLayout *orientationFormLayout = new QFormLayout(orientationGroupBox);
    orientationFormLayout->setLabelAlignment(Qt::AlignLeft);
    quatWLabel = new QLabel(orientationGroupBox);
    quatWLabel->setStyleSheet("font-size: 14px; color: white;");
    quatXLabel = new QLabel(orientationGroupBox);
    quatXLabel->setStyleSheet("font-size: 14px; color: white;");
    quatYLabel = new QLabel(orientationGroupBox);
    quatYLabel->setStyleSheet("font-size: 14px; color: white;");
    quatZLabel = new QLabel(orientationGroupBox);
    quatZLabel->setStyleSheet("font-size: 14px; color: white;");
    orientationFormLayout->addRow("W:", quatWLabel);
    orientationFormLayout->addRow("X:", quatXLabel);
    orientationFormLayout->addRow("Y:", quatYLabel);
    orientationFormLayout->addRow("Z:", quatZLabel);
    propertiesFrameLayout->addWidget(orientationGroupBox);

    // Interpolation Properties
    interpolationGroupBox = new QGroupBox("Interpolation Mode", propertiesFrame);
    interpolationGroupBox->setStyleSheet(
        "QGroupBox { font-size: 16px; font-weight: bold; color: white; border: 1px solid #3a3a3a; "
        "border-radius: 5px; margin-top: 14px; } QGroupBox::title { subcontrol-origin: margin; "
        "subcontrol-position: top left; padding: 0 3px; }");
    QFormLayout *interpolationFormLayout = new QFormLayout(interpolationGroupBox);
    interpolationFormLayout->setLabelAlignment(Qt::AlignLeft);
    interpolationModeLabel = new QLabel("Mode:", interpolationGroupBox);
    interpolationModeLabel->setStyleSheet("font-size: 14px; color: white;");
    interpolationModeComboBox = new QComboBox(interpolationGroupBox);
    interpolationModeComboBox->setStyleSheet("QComboBox {"
                                             "   font-size: 14px;"
                                             "   background-color: #2f2f2f;"
                                             "   border: 1px solid #3a3a3a;"
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
                                             "   background-color: #2f2f2f;"
                                             "   border: 1px solid #3a3a3a;"
                                             "}");
    interpolationModeComboBox->addItem("Spherical",
                                       QVariant::fromValue(InterpolationMode::Spherical));
    interpolationModeComboBox->addItem("Linear", QVariant::fromValue(InterpolationMode::Linear));
    interpolationFormLayout->addRow(interpolationModeLabel, interpolationModeComboBox);
    connect(interpolationModeComboBox,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &PositionConfigPage::updateInterpolationMode);
    propertiesFrameLayout->addWidget(interpolationGroupBox);

    // --- "Save Points" Button ---
    savePointsButton = new QPushButton("Save Points", propertiesFrame);
    savePointsButton->setStyleSheet(
        "font-size: 16px; padding: 10px; background-color: #3a3a3a; color: "
        "white; border: 1px solid #C8102E; border-radius: 5px;");
    connect(savePointsButton, &QPushButton::clicked, this, &PositionConfigPage::savePoints);
    propertiesFrameLayout->addWidget(savePointsButton);

    propertiesFrameLayout->addStretch(1);             // Properties content stretches to fill space
    rightColumnLayout->addWidget(propertiesFrame, 1); // Properties frame gets vertical stretch

    // --- Time Slider and Control Section ---
    QFrame *timeControlFrame = new QFrame(this);
    timeControlFrame->setFrameShape(QFrame::StyledPanel);
    timeControlFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");

    QVBoxLayout *timeControlLayout = new QVBoxLayout(timeControlFrame);
    timeControlLayout->setContentsMargins(10, 10, 10, 10);

    // Slider Layout
    QHBoxLayout *sliderControlLayout = new QHBoxLayout();

    // Fast Rewind Button
    QPushButton *fastRewindButton = new QPushButton("<<", this);
    fastRewindButton->setStyleSheet(
        "font-size: 16px; padding: 5px; background-color: #3a3a3a; color: "
        "white; border: 1px solid #C8102E; border-radius: 5px;");
    connect(fastRewindButton,
            &QPushButton::clicked,
            this,
            &PositionConfigPage::goToPreviousActionSlot);
    sliderControlLayout->addWidget(fastRewindButton, 0); // Fixed width

    timeSlider = new QSlider(Qt::Horizontal, this);
    timeSlider->setMinimum(0);
    timeSlider->setMaximum(100);
    timeSlider->setTickInterval(1);
    connect(timeSlider, &QSlider::valueChanged, this, &PositionConfigPage::updatePositions);
    sliderControlLayout->addWidget(timeSlider, 1); // Slider stretches horizontally

    // Fast Forward Button
    QPushButton *fastForwardButton = new QPushButton(">>", this);
    fastForwardButton->setStyleSheet(
        "font-size: 16px; padding: 5px; background-color: #3a3a3a; color: "
        "white; border: 1px solid #C8102E; border-radius: 5px;");
    connect(fastForwardButton, &QPushButton::clicked, this, &PositionConfigPage::goToNextActionSlot);
    sliderControlLayout->addWidget(fastForwardButton, 0); // Fixed width

    timeControlLayout->addLayout(sliderControlLayout);

    timeLabel = new QLabel("Time Slot: 0", this);
    timeLabel->setStyleSheet("font-size: 16px; color: white; text-align: center;");
    timeLabel->setAlignment(Qt::AlignCenter);
    timeControlLayout->addWidget(timeLabel);

    rightColumnLayout->addWidget(timeControlFrame,
                                 0); // Time control frame takes minimal vertical space

    // --- Buttons (Back/Next) ---
    buttonLayout = new QHBoxLayout;
    backButton = new QPushButton("Back", this);
    QString buttonStyle = "font-size: 16px; padding: 10px; background-color: #C8102E; color: "
                          "white; border: none; border-radius: 5px;";
    nextButton = new QPushButton("Next", this);
    nextButton->setStyleSheet(buttonStyle);
    backButton->setStyleSheet("font-size: 16px; padding: 10px; background-color: #3a3a3a; color: "
                              "white; border: none; border-radius: 5px;");
    backButton->setFixedWidth(75);
    connect(backButton, &QPushButton::clicked, this, &PositionConfigPage::backRequested);
    connect(nextButton, &QPushButton::clicked, this, [this]()
            {
        if (positions.empty()) {
            QMessageBox::warning(
                this,
                "No Position Data",
                "Please create or load a position configuration before proceeding.");
            return;
        }
        nlohmann::json positionConfig = getPositionConfigFromScene();
        emit nextRequested(positionConfig); });

    buttonLayout->addWidget(backButton, 0); // Back button fixed width
    buttonLayout->addWidget(nextButton, 1); // Next button stretches horizontally

    mainLayout->addLayout(buttonLayout, 0); // Buttons layout takes minimal vertical space

    setLayout(mainLayout); // Set the main layout for the page

    updateScene(); // Initial scene update, re-adds grid and axes
    configFileCombo->addItem("New Scenario");
    connect(configFileCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &PositionConfigPage::loadConfig);
}

void PositionConfigPage::refreshConfigFiles()
{
    int currentIdx = configFileCombo->currentIndex(); // Store
    configFileCombo->clear();
    configFileCombo->addItem("New Scenario"); // Always have "New Config" as the first option.

    std::string path = std::string(CONFIG_POSITION_DIR);
    if (!fs::exists(path))
    {
        std::cerr << "Error: Position config directory does not exist: " << path << std::endl;
        return;
    }

    if (fs::is_directory(path))
    {
        for (const auto &entry : fs::directory_iterator(path))
        {
            if (entry.path().extension() == ".json")
            {
                configFileCombo->addItem(QString::fromStdString(entry.path().filename().string()));
            }
        }
    }
    else
    {
        QMessageBox::critical(this,
                              "Position Config Error",
                              "Position config directory is not a directory: " + QString::fromStdString(path));
    }
    configFileCombo->setCurrentIndex(currentIdx); // Restore

    // Now that we have the parameter config and have refreshed the file list, create default.
    createDefaultScenario();
}

void PositionConfigPage::createUnitItems(const std::vector<engineUnit> &units)
{
    clearScene(); // Clear

    qreal spacing = 1.0; // World units
    qreal totalWidth = (units.size() - 1) * spacing;
    qreal startX = -totalWidth / 2.0; // Center
    qreal yPos = 0;

    QList<QColor> unitColors = {Qt::red,
                                Qt::green,
                                Qt::blue,
                                Qt::cyan,
                                Qt::magenta,
                                Qt::yellow,
                                Qt::darkRed,
                                Qt::darkGreen,
                                Qt::darkBlue,
                                Qt::darkCyan,
                                Qt::darkMagenta,
                                Qt::darkYellow};
    int colorIndex = 0;

    for (size_t i = 0; i < units.size(); ++i)
    {
        // Create the MovableUnitItem
        MovableUnitItem *item = new MovableUnitItem(-unitSizePixels / 2,
                                                    -unitSizePixels / 2,
                                                    unitSizePixels,
                                                    unitSizePixels); // Create item
        QPen border(unitColors[colorIndex % unitColors.size()], 2);
        item->setPen(border);
        item->setBrush(QBrush(QColor(0x3a3a3a)));
        colorIndex++;
        item->setUnitIndex(i);
        item->setUnitName(units[i].label);

        // --- Configure Arrows ---
        // Base Arrow (Cyan Dashed): Make INTERACTIVE
        DirectionArrowItem *baseArrow = item->getBaseArrow();
        if (baseArrow)
        {
            baseArrow->setInteractive(true); // Enable mouse interaction
            baseArrow->setVisible(true);
        }

        // Effective Arrow (Yellow Solid): Make INVISIBLE and NON-INTERACTIVE
        DirectionArrowItem *effectiveArrow = item->getEffectiveArrow();
        if (effectiveArrow)
        {
            effectiveArrow->setInteractive(false); // Ensure non-interactive
            effectiveArrow->setVisible(false);     // Hide it on this page
        }
        // -------------------------

        // Set initial position (e.g., default spacing)
        qreal xPos = startX + i * spacing;
        item->setPos(
            toSceneCoordinates(Position{static_cast<float>(xPos), static_cast<float>(yPos), 0}));
        // Initial orientation is identity (set in MovableUnitItem constructor)

        scene->addItem(item);
        unitItems.push_back(item);

        // Create and position the label.
        QGraphicsTextItem *label = new QGraphicsTextItem(QString::fromStdString(units[i].label));
        label->setDefaultTextColor(Qt::white);
        label->setPos(0, 0);
        scene->addItem(label);
        unitLabels.push_back(label);
    }
    updateLabelPositions();
}

void PositionConfigPage::clearScene()
{
    scene->clear();

    QList<QGraphicsItem *> itemsCopy = scene->items();
    for (QGraphicsItem *item : itemsCopy)
    {
        // Check if the item is a MovableUnitItem or a QGraphicsTextItem.
        if (item->type() == QGraphicsItem::UserType + 1 || item->type() == QGraphicsTextItem::Type)
        {
            // Remove the item from the scene.
            scene->removeItem(item);
            item->setParentItem(nullptr);
            delete item;
        }
    }
    // Clear the item lists.
    unitItems.clear();
    unitLabels.clear();

    // Reset the view's transformation.
    view->resetTransform();
    zoomFactor = 1.0; // Reset zoom factor

    // --- Re-add the grid and axes ---
    QPen gridPen(QColor(50, 50, 50));
    gridPen.setWidth(1);
    gridPen.setStyle(Qt::DashLine);
    qreal gridSize = 50; // pixels per grid square

    for (qreal x = qRound(scene->sceneRect().left() / gridSize) * gridSize;
         x <= scene->sceneRect().right();
         x += gridSize)
    {
        scene->addLine(x, scene->sceneRect().top(), x, scene->sceneRect().bottom(), gridPen);
    }
    for (qreal y = qRound(scene->sceneRect().top() / gridSize) * gridSize;
         y <= scene->sceneRect().bottom();
         y += gridSize)
    {
        scene->addLine(scene->sceneRect().left(), y, scene->sceneRect().right(), y, gridPen);
    }
    QPen axisPen(Qt::white); // White axes
    axisPen.setWidth(2);
    scene->addLine(scene->sceneRect().left(), 0, scene->sceneRect().right(), 0, axisPen); // X-axis
    scene->addLine(0, scene->sceneRect().top(), 0, scene->sceneRect().bottom(), axisPen); // Y-axis
}

void PositionConfigPage::savePoints()
{
    int currentTimeSlot = timeSlider->value();

    // 1. Ensure vectors are large enough.
    if (currentTimeSlot >= positions.size())
    {
        positions.resize(currentTimeSlot + 1);
        interpolationModes.resize(currentTimeSlot + 1);
        endSimulationFlags.resize(currentTimeSlot + 1, false);
    }
    positions[currentTimeSlot].resize(unitItems.size(),
                                      std::make_pair(Position{0.0, 0.0, 0.0},
                                                     Quaternion{1.0, 0.0, 0.0, 0.0}));
    interpolationModes[currentTimeSlot].resize(unitItems.size(), InterpolationMode::Spherical);

    // 2. Iterate through all unit items
    for (size_t i = 0; i < unitItems.size(); ++i)
    {
        MovableUnitItem *item = unitItems[i];

        // 3. Get position and orientation from the item.
        QPointF scenePos = item->scenePos();
        Position worldPos = fromSceneCoordinates(scenePos);     // Convert to world
        Quaternion baseOrientation = item->getBaseQuaternion(); // Base orientation

        // 4. Store the data in the vectors.
        positions[currentTimeSlot][i].first = worldPos; // Store Position

        positions[currentTimeSlot][i].second = baseOrientation; // Store Quaternion

        // 5. Store interpolation mode{
        interpolationModes[currentTimeSlot][i] = qvariant_cast<InterpolationMode>(
            interpolationModeComboBox->currentData());
    }

    updateLabelPositions();    // Update
    updatePropertiesDisplay(); // Update
}

void PositionConfigPage::updatePositions(int slot)
{
    timeLabel->setText(QString("Time: %1 s")
                           .arg(slot * paramConfigFile.engine_slot_time_microsec / 1e6, 0, 'f', 3));

    // Clear future positions
    for (MovableUnitItem *item : unitItems)
    {
        item->clearFuturePosition();
    }

    if (slot < positions.size() && !positions[slot].empty())
    {
        // We have stored positions AND orientations for this slot.
        for (size_t i = 0; i < unitItems.size(); ++i)
        {
            // Ensure index is valid for both unitItems and the loaded data for this slot
            if (i < positions[slot].size())
            {
                unitItems[i]->setFlag(QGraphicsItem::ItemSendsGeometryChanges,
                                      false); // Disable signals during update

                // Set Position
                QPointF scenePos = toSceneCoordinates(positions[slot][i].first);
                unitItems[i]->setPos(scenePos);

                // Set Quaternion (using your struct)
                Quaternion storedQuat = positions[slot][i].second;
                unitItems[i]->setBaseQuaternion(positions[slot][i].second);

                unitItems[i]->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
                unitItems[i]->setVisible(true);
            }
            else
            {
                // Handle cases where loaded data might have fewer units than expected
                if (i < unitItems.size())
                {
                    unitItems[i]->setVisible(false);
                }
            }
        }
        // Handle cases where there are more unitItems than loaded data points
        for (size_t i = positions[slot].size(); i < unitItems.size(); ++i)
        {
            unitItems[i]->setVisible(false); // Hide extra items
        }
    }

    // Make all items visible again if they were hidden previously (optional, depends on desired behavior)
    if (!(slot < positions.size() && !positions[slot].empty()))
    {
        // If a slot has no explicit data, items stay hidden or at last known position.
    }
    else
    {
        // Ensure items that *do* have data are visible
        for (size_t i = 0; i < positions[slot].size() && i < unitItems.size(); ++i)
        {
            unitItems[i]->setVisible(true);
        }
    }

    // Set future position (only for the *next* slot)
    if (slot + 1 < positions.size() && !positions[slot + 1].empty())
    { // Check next slot has data
        for (size_t i = 0; i < unitItems.size(); ++i)
        {
            // Ensure the next slot has data for this specific unit index
            if (i < positions[slot + 1].size())
            {
                unitItems[i]->setFuturePosition(toSceneCoordinates(positions[slot + 1][i].first));
            }
            else
            {
                unitItems[i]
                    ->clearFuturePosition(); // No future pos if next slot lacks data for this unit
            }
        }
    }
    else
    {
        // No data in the next slot, clear all future positions
        for (MovableUnitItem *item : unitItems)
        {
            item->clearFuturePosition();
        }
    }

    updateLabelPositions();
    updatePropertiesDisplay();
}

void PositionConfigPage::updatePropertiesDisplay()
{
    QList<QGraphicsItem *> selected = scene->selectedItems();
    MovableUnitItem *selectedUnit = nullptr;

    // --- Find the selected MovableUnitItem ---
    if (!selected.isEmpty())
    {
        selectedUnit = qgraphicsitem_cast<MovableUnitItem *>(selected.first());
        if (!selectedUnit)
        {
            DirectionArrowItem *arrow = qgraphicsitem_cast<DirectionArrowItem *>(selected.first());
            if (arrow)
            {
                selectedUnit = qgraphicsitem_cast<MovableUnitItem *>(arrow->parentItem());
            }
        }
    }

    // --- Update Display IF a unit is selected ---
    if (selectedUnit)
    {
        int unitIndex = selectedUnit->getUnitIndex();
        std::string unitName = selectedUnit->getUnitName();
        int currentTimeSlot = timeSlider->value();

        // --- 1. Get LIVE X, Y, and Orientation from the Scene Item ---
        QPointF liveScenePos = selectedUnit->scenePos();
        // Use fromSceneCoordinates JUST to get the correctly scaled world X and Y
        // (We will ignore its z=0 output for display)
        Position liveWorldXY_temp = fromSceneCoordinates(liveScenePos);
        float liveWorldX = liveWorldXY_temp.x;                       // Live world X (scaled units)
        float liveWorldY = liveWorldXY_temp.y;                       // Live world Y (scaled units)
        Quaternion liveBaseQuat = selectedUnit->getBaseQuaternion(); // Live orientation
        // -------------------------------------------------------------

        // --- 2. Get STORED Z and Interpolation Mode from Internal Data ---
        float storedWorldZ = 0.0f;                                   // Default Z (scaled units)
        InterpolationMode storedMode = InterpolationMode::Spherical; // Default mode

        // Check if the slot and unit index are valid within the stored data
        if (currentTimeSlot >= 0 && currentTimeSlot < positions.size() && !positions[currentTimeSlot].empty() && // Check inner vector not empty
            unitIndex >= 0 && unitIndex < positions[currentTimeSlot].size())
        {
            // Access the stored Z value (already in scaled world units)
            storedWorldZ = positions[currentTimeSlot][unitIndex].first.z;
        }
        // Get interpolation mode (needs similar bounds check)
        if (currentTimeSlot >= 0 && currentTimeSlot < interpolationModes.size() && !interpolationModes[currentTimeSlot].empty() && // Check inner vector not empty
            unitIndex >= 0 && unitIndex < interpolationModes[currentTimeSlot].size())
        {
            storedMode = interpolationModes[currentTimeSlot][unitIndex];
        }
        // ---------------------------------------------------------------

        // --- 3. Calculate Display Values (Convert internal scaled units back to Meters) ---
        double displayScale = paramConfigFile.pixelsPerMeter > 1e-9 ? paramConfigFile.pixelsPerMeter
                                                                    : 1.0; // Avoid division by zero
        double displayX = liveWorldX / displayScale;                       // Use live X
        double displayY = liveWorldY / displayScale;                       // Use live Y
        double displayZ = storedWorldZ / displayScale;                     // Use stored Z!
        // ---------------------------------------------------------------------------

        // --- 4. Update Labels ---
        unitNameLabel->setText(QString::fromStdString(unitName));
        unitIDLabel->setText(QString::number(unitIndex));
        posXLabel->setText(QString::number(displayX, 'f', 2));        // Live X in meters
        posYLabel->setText(QString::number(displayY, 'f', 2));        // Live Y in meters
        posZLabel->setText(QString::number(displayZ, 'f', 2));        // Stored Z in meters
        quatWLabel->setText(QString::number(liveBaseQuat.w, 'f', 4)); // Live W
        quatXLabel->setText(QString::number(liveBaseQuat.x, 'f', 4)); // Live qx
        quatYLabel->setText(QString::number(liveBaseQuat.y, 'f', 4)); // Live qy
        quatZLabel->setText(QString::number(liveBaseQuat.z, 'f', 4)); // Live qz
        // --------------------

        // --- 5. Update Interpolation Mode ComboBox ---
        int index = interpolationModeComboBox->findData(QVariant::fromValue(storedMode));
        if (index == -1)
        {
            index = 0;
        }

        interpolationModeComboBox->setCurrentIndex(index);
    }
    else
    {
        // --- Clear Display IF NO unit is selected ---
        unitNameLabel->setText("---");
        unitIDLabel->setText("---");
        posXLabel->setText("---");
        posYLabel->setText("---");
        posZLabel->setText("---");
        quatWLabel->setText("---");
        quatXLabel->setText("---");
        quatYLabel->setText("---");
        quatZLabel->setText("---");
        interpolationModeComboBox->setCurrentIndex(0);
    }
}

void PositionConfigPage::loadConfig()
{
    if (configFileCombo->currentIndex() == 0)
    { // "New Scenario" is selected
        createDefaultScenario();
        loadedConfigPath.clear();
        positionConfigName = "New Scenario";
        return;
    }

    positionConfigName = configFileCombo->currentText();
    if (positionConfigName.isEmpty())
    {
        createDefaultScenario();
        loadedConfigPath.clear();
        return;
    }

    // Construct the full path
    loadedConfigPath = std::string(CONFIG_POSITION_DIR) + "/" + positionConfigName.toStdString();

    // --- Use the static function for loading, validation, AND CONVERSION ---
    std::optional<nlohmann::json> configResult = PositionConfigPage::processPositionConfiguration(loadedConfigPath, this->paramConfigFile);

    // --- Check the result ---
    if (!configResult)
    {
        // Error already logged/shown by the static function
        return;
    }

    // --- Success: Process the validated JSON ---
    json configJson = configResult.value();

    std::vector<std::string> loadedUnitLabels;
    try
    {
        // Extract unit labels (already validated)
        loadedUnitLabels = configJson["unit_labels"].get<std::vector<std::string>>();

        // Set the time slider maximum based on param config (remains the same)
        if (paramConfigFile.engine_slot_time_microsec > 0)
        {
            timeSlider->setMaximum(
                static_cast<int>(std::ceil(paramConfigFile.engine_max_time_sec * 1e6 / paramConfigFile.engine_slot_time_microsec)));
            timeSlider->setTickInterval(
                std::max(1, timeSlider->maximum() / 10)); // Ensure tick interval is at least 1
        }
        else
        {
            qWarning() << "Cannot set time slider max: engine_slot_time_microsec is zero or "
                          "negative in paramConfig.";
            timeSlider->setMaximum(0);
            timeSlider->setTickInterval(1);
        }

        // Clear existing internal data *before* loading new data.
        positions.clear();
        interpolationModes.clear();

        clearScene(); // Resets view transform, clears items, re-adds grid/axes

        // --- Parse position data directly using slot indices from configJson ---
        const auto &positionSlotsJson = configJson["positions"];
        for (const auto &slotData : positionSlotsJson)
        {
            // Static function already validated presence and type
            int engineSlot = slotData["slot"]; // Use the slot index directly from the JSON

            // Resize internal vectors if needed for this engineSlot
            if (engineSlot >= positions.size())
            {
                positions.resize(engineSlot + 1);
                interpolationModes.resize(engineSlot + 1);
                endSimulationFlags.resize(engineSlot + 1, false);
            }

            // Read and store the endSimulation flag for this specific slot
            endSimulationFlags[engineSlot] = slotData.value("endSimulation", false);

            // Initialize the vectors for this specific slot if empty
            if (positions[engineSlot].empty())
            {
                positions[engineSlot].resize(loadedUnitLabels.size(),
                                             std::make_pair(Position{0.0, 0.0, 0.0},
                                                            Quaternion{1.0, 0.0, 0.0, 0.0}));
                interpolationModes[engineSlot].resize(loadedUnitLabels.size(),
                                                      InterpolationMode::Spherical);
            }

            // Process units within the slot
            const auto &unitsJson = slotData["units"]; // Already validated as array
            for (const auto &unitData : unitsJson)
            {
                // Static function already validated structure
                std::string label = unitData["label"];
                auto it = std::find(loadedUnitLabels.begin(), loadedUnitLabels.end(), label);
                if (it == loadedUnitLabels.end())
                {
                    qWarning() << "Unit label mismatch detected after validation:"
                               << QString::fromStdString(label) << "in slot" << engineSlot;
                    continue; // Should ideally not happen if validation is robust
                }
                size_t unitIndex = std::distance(loadedUnitLabels.begin(), it);

                // Extract position (apply UI scaling)
                Position pos;
                pos.x = unitData.value("x", 0.0f);
                pos.y = unitData.value("y", 0.0f);
                pos.z = unitData.value("z", 0.0f);

                // Apply UI scaling (converting meters from file to internal world units for display)
                // Use paramConfigFile.pixelsPerMeter (which should be set based on UI settings)
                if (paramConfigFile.pixelsPerMeter > 1e-9)
                {
                    pos.x *= paramConfigFile.pixelsPerMeter;
                    pos.y *= paramConfigFile.pixelsPerMeter;
                    pos.z *= paramConfigFile.pixelsPerMeter;
                }
                else
                {
                    qWarning() << "pixelsPerMeter is near zero in paramConfig, position scaling "
                                  "might be incorrect.";
                }

                // Extract quaternion
                Quaternion quat;
                quat.w = unitData.value("w", 1.0f);
                quat.x = unitData.value("qx", 0.0f);
                quat.y = unitData.value("qy", 0.0f);
                quat.z = unitData.value("qz", 0.0f);

                // Store position and quaternion in internal vectors
                if (unitIndex < positions[engineSlot].size())
                { // Bounds check
                    positions[engineSlot][unitIndex] = {pos, quat};
                }
                else
                {
                    qWarning() << "Unit index" << unitIndex << "out of bounds for slot"
                               << engineSlot << "label" << QString::fromStdString(label);
                }

                // Extract interpolation mode
                std::string modeStr = unitData.value("mode", "Spherical"); // Default
                InterpolationMode mode = (modeStr == "Linear") ? InterpolationMode::Linear
                                                               : InterpolationMode::Spherical;

                if (unitIndex < interpolationModes[engineSlot].size())
                { // Bounds check
                    interpolationModes[engineSlot][unitIndex] = mode;
                } // Else: warning already issued above

            } // End loop over units in slot
        } // End loop over slots

        // --- Post-processing: Update UI ---
        // Create engine units structure needed for createUnitItems based on loaded labels
        std::vector<engineUnit> loadedEngineUnits;
        loadedEngineUnits.reserve(loadedUnitLabels.size());
        for (const auto &label : loadedUnitLabels)
        {
            // Find corresponding unit in paramConfigFile to get other flags if needed, or just use label
            auto paramUnitIt = std::find_if(paramConfigFile.engine_units.begin(),
                                            paramConfigFile.engine_units.end(),
                                            [&label](const engineUnit &u)
                                            {
                                                return u.label == label;
                                            });
            if (paramUnitIt != paramConfigFile.engine_units.end())
            {
                loadedEngineUnits.push_back(*paramUnitIt); // Copy existing unit definition
            }
            else
            {
                qWarning() << "Loaded unit label '" << QString::fromStdString(label)
                           << "' not found in main paramConfig. Using default flags.";
                loadedEngineUnits.push_back(
                    {label, "", "", false, false, false, false}); // Create with default flags
            }
        }

        createUnitItems(loadedEngineUnits); // Create visual items based on loaded labels/units
        updatePositions(0);                 // Update display to show the first time slot
        updatePropertiesDisplay();          // Update the properties panel
    }
    catch (const nlohmann::json::exception &e)
    {
        // Catch errors during JSON *access* (should be less likely after static validation)
        QMessageBox::critical(this,
                              "Position Config Error",
                              "Error processing position data content: " + QString(e.what()));
        qWarning() << "Error processing JSON content:" << e.what();
        // Consider reverting to default state here as well
        return;
    }
    catch (const std::exception &e)
    {
        // Catch standard exceptions during processing
        QMessageBox::critical(this,
                              "Position Config Error",
                              "An unexpected error occurred during processing: " + QString(e.what()));
        qWarning() << "Standard exception during position processing:" << e.what();
        return;
    }
}

void PositionConfigPage::saveConfig()
{
    QString filename;

    bool ok;
    filename = QInputDialog::getText(this,
                                     tr("Save Position Config"),
                                     tr("Config Name:"),
                                     QLineEdit::Normal,
                                     "config",
                                     &ok);

    if (!ok || filename.isEmpty())
    {
        return; // Cancelled
    }
    if (!filename.endsWith(".json", Qt::CaseInsensitive))
    {
        filename += ".json";
    }
    loadedConfigPath = (QString::fromStdString(std::string(CONFIG_POSITION_DIR)) + "/" + filename)
                           .toStdString();

    json outputJson = getPositionConfigFromScene();

    std::ofstream file(loadedConfigPath);
    if (!file.is_open())
    {
        QMessageBox::critical(this,
                              "Position Config Error",
                              "Failed to open file for saving: " + QString::fromStdString(loadedConfigPath));
        return;
    }

    file << outputJson.dump(4);
    file.close();

    QMessageBox::information(this,
                             "Saved",
                             "Position configuration saved to: " + QString::fromStdString(loadedConfigPath));
}

nlohmann::json PositionConfigPage::getPositionConfigFromScene()
{
    json outputJson;
    outputJson["engine_slot_time_microsec"] = paramConfigFile.engine_slot_time_microsec;

    std::vector<std::string> unit_labels;
    for (const auto &unit : paramConfigFile.engine_units)
    {
        unit_labels.push_back(unit.label);
    }

    outputJson["unit_labels"] = unit_labels;

    json positionsJson = json::array();
    for (size_t slot = 0; slot < positions.size(); ++slot)
    {
        if (!positions[slot].empty())
        { //  Only save slots with data.
            json slotData;
            slotData["slot"] = slot;

            if (slot < endSimulationFlags.size() && endSimulationFlags[slot])
            {
                slotData["endSimulation"] = true;
            }

            json unitsData = json::array();
            for (size_t unitIndex = 0; unitIndex < positions[slot].size(); ++unitIndex)
            {
                if (unitIndex < unit_labels.size())
                {
                    std::string label = unit_labels[unitIndex];
                    json unitPosition;
                    unitPosition["label"] = label;
                    unitPosition["x"] = positions[slot][unitIndex].first.x / paramConfigFile.pixelsPerMeter;
                    unitPosition["y"] = positions[slot][unitIndex].first.y / paramConfigFile.pixelsPerMeter;
                    unitPosition["z"] = positions[slot][unitIndex].first.z / paramConfigFile.pixelsPerMeter;
                    unitPosition["w"] = positions[slot][unitIndex].second.w;
                    unitPosition["qx"] = positions[slot][unitIndex].second.x;
                    unitPosition["qy"] = positions[slot][unitIndex].second.y;
                    unitPosition["qz"] = positions[slot][unitIndex].second.z;

                    // --- Save Interpolation Mode ---
                    InterpolationMode mode = interpolationModes[slot][unitIndex];
                    std::string modeStr = "Spherical"; // Default
                    if (mode == InterpolationMode::Linear)
                    {
                        modeStr = "Linear";
                    }
                    unitPosition["mode"] = modeStr; // Save

                    unitsData.push_back(unitPosition);
                }
            }
            slotData["units"] = unitsData;
            positionsJson.push_back(slotData);
        }
    }

    outputJson["positions"] = positionsJson;

    return outputJson;
}

void PositionConfigPage::goToNextActionSlot()
{
    int currentSlot = timeSlider->value();
    for (int slot = currentSlot + 1; slot <= timeSlider->maximum(); ++slot)
    {
        if (slot < positions.size() && !positions[slot].empty())
        {
            timeSlider->setValue(slot);
            return;
        }
    }
}

void PositionConfigPage::goToPreviousActionSlot()
{
    int currentSlot = timeSlider->value();
    for (int slot = currentSlot - 1; slot >= 0; --slot)
    {
        if (slot < positions.size() && !positions[slot].empty())
        {
            timeSlider->setValue(slot);
            return;
        }
    }
}

void PositionConfigPage::updateLabelPositions()
{
    for (size_t i = 0; i < unitItems.size(); ++i)
    {
        if (i < unitLabels.size())
        { // bounds
            unitLabels[i]->setPos(unitItems[i]->pos() + QPointF(-0.2 * unitSizePixels, -1 * unitSizePixels));
        }
    }
}

// Convert world coordinates to scene coordinates (for display)
QPointF PositionConfigPage::toSceneCoordinates(const Position &pos) const
{
    qreal scaleFactor = unitSizePixels / unitSizeWorld;        //  Pixels per world unit
    return QPointF(pos.x * scaleFactor, -pos.y * scaleFactor); // Invert y for Qt coords
}

// Convert scene coordinates (pixels) to world coordinates
Position PositionConfigPage::fromSceneCoordinates(const QPointF &scenePos) const
{
    qreal scaleFactor = unitSizePixels / unitSizeWorld; // Pixels per world unit
    Position pos;
    pos.x = scenePos.x() / scaleFactor;
    pos.y = -scenePos.y() / scaleFactor; // Invert y
    pos.z = 0;                           // Assuming z=0
    return pos;
}

// Updates the scene based on current viewRange and unitSizeWorld.
void PositionConfigPage::updateScene()
{
    // 1.  Set the scene rectangle based on viewRange.  The sceneRect is in *pixel*
    //     coordinates, so we need to convert viewRange (world units) to pixels.

    qreal scaleFactor = unitSizePixels / unitSizeWorld; // Pixels per world unit
    qreal sceneWidthPixels = viewRange * scaleFactor;
    qreal sceneHeightPixels = viewRange * scaleFactor;
    scene->setSceneRect(-sceneWidthPixels / 2,
                        -sceneHeightPixels / 2,
                        sceneWidthPixels,
                        sceneHeightPixels);

    // 2.  Recreate the grid based on the new sceneRect.
    QPen gridPen(QColor(50, 50, 50));
    gridPen.setWidth(1);
    gridPen.setStyle(Qt::DashLine);
    qreal gridSizePixels = 50; //  Fixed grid size in *pixels*

    // Remove old grid lines
    QList<QGraphicsItem *> items = scene->items();
    for (QGraphicsItem *item : items)
    {
        if (item->type() == QGraphicsLineItem::Type)
        { // Check
            scene->removeItem(item);
            delete item; // Clean
        }
    }

    for (qreal x = qRound(scene->sceneRect().left() / gridSizePixels) * gridSizePixels;
         x <= scene->sceneRect().right();
         x += gridSizePixels)
    {
        scene->addLine(x, scene->sceneRect().top(), x, scene->sceneRect().bottom(), gridPen);
    }
    for (qreal y = qRound(scene->sceneRect().top() / gridSizePixels) * gridSizePixels;
         y <= scene->sceneRect().bottom();
         y += gridSizePixels)
    {
        scene->addLine(scene->sceneRect().left(), y, scene->sceneRect().right(), y, gridPen);
    }
    QPen axisPen(Qt::white); // White
    axisPen.setWidth(2);
    scene->addLine(scene->sceneRect().left(), 0, scene->sceneRect().right(), 0, axisPen);
    scene->addLine(0, scene->sceneRect().top(), 0, scene->sceneRect().bottom(), axisPen);

    // 3.  Reposition existing items based on their world coordinates.
    for (size_t i = 0; i < unitItems.size(); ++i)
    {
        int slot = timeSlider->value();
        //  Use the currently selected time slot's position data, if available.
        if (slot < positions.size() && positions[slot].size() > i)
        {
            QPointF scenePos = toSceneCoordinates(positions[slot][i].first);
            unitItems[i]->setFlag(QGraphicsItem::ItemSendsGeometryChanges, false); // Disable
            unitItems[i]->setPos(scenePos);
            unitItems[i]->setBaseQuaternion(positions[slot][i].second); // Keep rotation
            unitItems[i]->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
            unitItems[i]->setRect(-unitSizePixels / 2,
                                  -unitSizePixels / 2,
                                  unitSizePixels,
                                  unitSizePixels); // Apply size
        }
    }

    // 4.  Update label positions.
    updateLabelPositions();

    // 5.  Update the view's transformation (to maintain zoom).
    QTransform transform;
    transform.scale(zoomFactor, zoomFactor);
    view->setTransform(transform);

    // 6.  Ensure the view's scene rectangle is updated.
    view->setSceneRect(scene->sceneRect());
}

bool PositionConfigPage::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == view->viewport())
    {
        if (event->type() == QEvent::MouseMove)
        {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);

            // Rotation (Left Mouse Button) - Now handled by DirectionArrowItem
            if (mouseEvent->buttons() & Qt::LeftButton)
            {
                if (!scene->selectedItems().isEmpty())
                {
                    updateLabelPositions();    //  Keep labels updated
                    updatePropertiesDisplay(); //  Update properties live
                }
            }
            // --- Panning ---
            else if (mouseEvent->buttons() & Qt::MiddleButton)
            {
                if (!panning)
                {
                    panning = true;
                    lastPanPoint = mouseEvent->pos();
                }
                else
                {
                    QPoint delta = mouseEvent->pos() - lastPanPoint;
                    view->horizontalScrollBar()->setValue(view->horizontalScrollBar()->value() - delta.x());
                    view->verticalScrollBar()->setValue(view->verticalScrollBar()->value() - delta.y());
                    lastPanPoint = mouseEvent->pos();
                }
            }
        }
        // --- Mouse Release ---
        else if (event->type() == QEvent::MouseButtonRelease)
        {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);
            if (mouseEvent->button() == Qt::MiddleButton)
            {
                panning = false;
            }
        }
        // --- Zoom ---
        else if (event->type() == QEvent::Wheel)
        {
            QWheelEvent *wheelEvent = static_cast<QWheelEvent *>(event);
            qreal delta = wheelEvent->angleDelta().y();
            qreal zoomStep = 0.05;

            if (delta > 0)
            {
                zoomFactor += zoomStep;
            }
            else
            {
                zoomFactor -= zoomStep;
            }
            zoomFactor = std::clamp(zoomFactor, 0.1, 10.0);
            QTransform transform;
            transform.scale(zoomFactor, zoomFactor);
            view->setTransform(transform);

            return true;
        }
    }
    return QWidget::eventFilter(watched, event);
}
void PositionConfigPage::createDefaultScenario()
{
    positions.clear();
    interpolationModes.clear();
    endSimulationFlags.clear();
    clearScene();
    positions.resize(1);
    interpolationModes.resize(1);
    endSimulationFlags.resize(1, false);

    const std::vector<engineUnit> &units = paramConfigFile.engine_units;
    if (units.empty())
    {
        QMessageBox::warning(this,
                             "No Unit Labels",
                             "No unit labels are defined in the parameter configuration.");
        return;
    }

    createUnitItems(units); // Pass the vector of engineUnit structs
    positions[0].resize(units.size(),
                        std::make_pair(Position{0.0, 0.0, 0.0}, Quaternion{1.0, 0.0, 0.0, 0.0}));
    interpolationModes[0].resize(units.size(), InterpolationMode::Spherical);

    qreal spacing = 10.0;
    qreal startX = 0;

    for (size_t i = 0; i < units.size(); ++i)
    {
        //  Calculate initial positions in *world* coordinates.
        qreal xPos = startX + i * spacing;
        qreal yPos = startX + i * spacing;
        positions[0][i].first.x = xPos;
        positions[0][i].first.y = yPos;
        positions[0][i].first.z = 0;
        positions[0][i].second = {1.0, 0.0, 0.0, 0.0}; // Identity
        if (i < unitItems.size())
        {
            //  Convert world coordinates to scene coordinates for display.
            unitItems[i]->setFlag(QGraphicsItem::ItemSendsGeometryChanges, false);
            unitItems[i]->setPos(toSceneCoordinates(positions[0][i].first));
            unitItems[i]->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
        }
    }

    updatePositions(0);        // Update the display to show the initial positions
    updatePropertiesDisplay(); // Update property display
}

void PositionConfigPage::updateInterpolationMode(int index)
{
    // Get the selected unit.
    QList<QGraphicsItem *> selected = scene->selectedItems();
    if (selected.isEmpty())
    {
        return;
    }

    MovableUnitItem *selectedUnit = qgraphicsitem_cast<MovableUnitItem *>(selected.first());
    if (!selectedUnit)
    {
        return;
    }

    // Get selected interpolation mode
    InterpolationMode newMode = qvariant_cast<InterpolationMode>(
        interpolationModeComboBox->currentData());

    // Get slot and unit index
    int currentTimeSlot = timeSlider->value();
    int unitIndex = selectedUnit->getUnitIndex();

    // Update vector
    if (currentTimeSlot >= interpolationModes.size())
    {
        interpolationModes.resize(currentTimeSlot + 1);
    }
    if (interpolationModes[currentTimeSlot].size() <= unitIndex)
    {
        interpolationModes[currentTimeSlot].resize(unitItems.size());
    }
    interpolationModes[currentTimeSlot][unitIndex] = newMode;
}

void PositionConfigPage::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter)
    {
        if (nextButton->isEnabled())
        {
            // Logic from nextButton's clicked slot:
            if (positions.empty())
            {
                QMessageBox::warning(
                    this,
                    "No Position Data",
                    "Please create or load a position configuration before proceeding.");
                return;
            }
            nlohmann::json positionConfig = getPositionConfigFromScene();
            emit nextRequested(positionConfig);
        }
    }
    else
    {
        QWidget::keyPressEvent(event); // Call base class for other keys
    }
}

std::optional<nlohmann::json> PositionConfigPage::processPositionConfiguration(
    const std::string &filePath, ConfigFile &paramConfig)
{
    // 1. Check if file exists
    QFileInfo fileInfo(QString::fromStdString(filePath));
    if (!fileInfo.exists() || !fileInfo.isFile())
    {
        qWarning() << "  Error: Position config file does not exist:"
                   << QString::fromStdString(filePath);
        return std::nullopt;
    }

    // 2. Open and read the file
    std::ifstream file(filePath);
    if (!file.is_open())
    {
        qWarning() << "  Error: Failed to open position config file:"
                   << QString::fromStdString(filePath);
        return std::nullopt;
    }

    json originalJson;
    try
    {
        originalJson = json::parse(file);
        file.close(); // Close file promptly after parsing
    }
    catch (const nlohmann::json::parse_error &e)
    {
        qWarning() << "  Error: Failed to parse position JSON file:"
                   << QString::fromStdString(filePath) << "Error:" << e.what();
        file.close(); // Ensure closed even on error
        return std::nullopt;
    }

    // 3. Apply any necessary conversions based on paramConfig
    applyPositionTimeToConfig(originalJson, paramConfig);

    // 4 Save the file name to our paramConfig
    paramConfig.positionConfig_name = fileInfo.fileName().toStdString();

    // 5. Basic Validation (Structure, Types, Time Compatibility)
    try
    {
        // Check for essential top-level keys and types
        if (!originalJson.contains("unit_labels") || !originalJson["unit_labels"].is_array())
        {
            qWarning() << "  Error: Position config missing or invalid 'unit_labels' (array "
                          "expected) in file:"
                       << QString::fromStdString(filePath);
            return std::nullopt;
        }
        if (!originalJson.contains("engine_slot_time_microsec") || !originalJson["engine_slot_time_microsec"].is_number())
        {
            qWarning() << "  Error: Position config missing or invalid 'engine_slot_time_microsec' "
                          "(number expected) in file:"
                       << QString::fromStdString(filePath);
            return std::nullopt;
        }
        if (!originalJson.contains("positions") || !originalJson["positions"].is_array())
        {
            qWarning() << "  Error: Position config missing or invalid 'positions' (array "
                          "expected) in file:"
                       << QString::fromStdString(filePath);
            return std::nullopt;
        }

        // Validate structure within the 'positions' array
        for (const auto &slotData : originalJson["positions"])
        {
            if (!slotData.is_object())
            {
                qWarning() << "  Error: Invalid item in 'positions' array (object expected).";
                return std::nullopt;
            }
            if (!slotData.contains("slot") || !slotData["slot"].is_number_integer())
            {
                qWarning()
                    << "  Error: Missing/invalid 'slot' (integer expected) in position data.";
                return std::nullopt;
            }
            if (!slotData.contains("units") || !slotData["units"].is_array())
            {
                qWarning() << "  Error: Missing/invalid 'units' (array expected) in slot data.";
                return std::nullopt;
            }

            for (const auto &unitData : slotData["units"])
            {
                if (!unitData.is_object())
                {
                    qWarning() << "  Error: Invalid item in 'units' array (object expected).";
                    return std::nullopt;
                }
                if (!unitData.contains("label") || !unitData["label"].is_string())
                {
                    qWarning()
                        << "  Error: Missing/invalid 'label' (string expected) in unit data.";
                    return std::nullopt;
                }
                if (!unitData.contains("x") || !unitData["x"].is_number())
                {
                    qWarning() << "  Error: Missing/invalid 'x' (number expected) in unit data.";
                    return std::nullopt;
                }
                if (!unitData.contains("y") || !unitData["y"].is_number())
                {
                    qWarning() << "  Error: Missing/invalid 'y' (number expected) in unit data.";
                    return std::nullopt;
                }
                if (!unitData.contains("z") || !unitData["z"].is_number())
                {
                    qWarning() << "  Error: Missing/invalid 'z' (number expected) in unit data.";
                    return std::nullopt;
                }
                // Quaternion components
                if (!unitData.contains("w") || !unitData["w"].is_number())
                {
                    qWarning() << "  Error: Missing/invalid 'w' (number expected) in unit data.";
                    return std::nullopt;
                }
                if (!unitData.contains("qx") || !unitData["qx"].is_number())
                {
                    qWarning() << "  Error: Missing/invalid 'qx' (number expected) in unit data.";
                    return std::nullopt;
                }
                if (!unitData.contains("qy") || !unitData["qy"].is_number())
                {
                    qWarning() << "  Error: Missing/invalid 'qy' (number expected) in unit data.";
                    return std::nullopt;
                }
                if (!unitData.contains("qz") || !unitData["qz"].is_number())
                {
                    qWarning() << "  Error: Missing/invalid 'qz' (number expected) in unit data.";
                    return std::nullopt;
                }
                if (unitData.contains("mode") && !unitData["mode"].is_string())
                {
                    qWarning() << "  Error: Invalid 'mode' (string expected) in unit data.";
                    return std::nullopt;
                }
            }
        }

        // --- Time Slot Compatibility Check ---
        long double loadedSlotTimeMicroSec = originalJson["engine_slot_time_microsec"];
        if (paramConfig.engine_slot_time_microsec <= 0)
        {
            qWarning()
                << "  Error: Parameter config has invalid engine_slot_time_microsec (<= 0). Cannot "
                   "process position config.";
            return std::nullopt;
        }

        constexpr long double time_tolerance = 1e-9; // Tolerance for float comparison
        bool requiresSlotConversion = std::abs(paramConfig.engine_slot_time_microsec - loadedSlotTimeMicroSec) > time_tolerance;
        bool compatible = true; // Assume compatible unless proven otherwise

        if (requiresSlotConversion)
        {
            // If times are different, check if one is an integer multiple of the other
            long double ratio1 = loadedSlotTimeMicroSec / paramConfig.engine_slot_time_microsec;
            long double ratio2 = paramConfig.engine_slot_time_microsec / loadedSlotTimeMicroSec;

            // Check if ratios are close to an integer value
            bool isMultiple1 = std::abs(ratio1 - std::round(ratio1)) < time_tolerance;
            bool isMultiple2 = std::abs(ratio2 - std::round(ratio2)) < time_tolerance;

            if (!isMultiple1 && !isMultiple2)
            {
                std::cerr << "  Error: Incompatible time slot durations between parameter config ("
                          << ". Durations must be equal or integer multiples.";
                compatible = false; // Mark as incompatible
            }
        }

        if (!compatible)
        {
            return std::nullopt; // Return if time steps are incompatible
        }

        // --- Perform Slot Conversion ---
        if (requiresSlotConversion)
        {
            double timeScaleFactor = loadedSlotTimeMicroSec / static_cast<double>(paramConfig.engine_slot_time_microsec);

            json convertedJson;
            // Copy metadata, importantly using the TARGET time step
            convertedJson["unit_labels"] = originalJson["unit_labels"];
            convertedJson["engine_slot_time_microsec"] = paramConfig.engine_slot_time_microsec; // Use target time step
            convertedJson["positions"] = json::array();                                         // Initialize empty positions array

            for (const auto &originalSlotData : originalJson["positions"])
            {
                int loadedSlot = originalSlotData["slot"];
                int engineSlot = static_cast<int>(
                    std::round(loadedSlot * timeScaleFactor)); // Convert slot index

                json convertedSlotData = originalSlotData; // Copy structure (includes "units" array etc.)
                convertedSlotData["slot"] = engineSlot;    // Update the slot number
                convertedJson["positions"].push_back(convertedSlotData);
            }
            return convertedJson; // <<< Return the CONVERTED JSON
        }
        else
        {
            // No conversion needed, return the original (validated) JSON
            return originalJson; // <<< Return the ORIGINAL JSON
        }
    }
    catch (const nlohmann::json::exception
               &e)
    { // Catch potential type/access errors during validation/conversion
        qWarning() << "  Error: JSON processing error in position config file:"
                   << QString::fromStdString(filePath) << "Error:" << e.what();
        return std::nullopt;
    }
    catch (const std::exception &e)
    { // Catch other potential errors
        qWarning() << "  Error: Unexpected error processing position config file:"
                   << QString::fromStdString(filePath) << "Error:" << e.what();
        return std::nullopt;
    }

    // Should not be reached if logic is correct, but acts as a failsafe
    qWarning() << "  Error: Reached end of processPositionConfiguration unexpectedly.";
    return std::nullopt;
}

void PositionConfigPage::applyPositionTimeToConfig(const nlohmann::json &positionJson,
                                                   ConfigFile &paramConfig)
{
    // Find the earliest slot that requests to end the simulation.
    std::optional<int> endSimulationSlot;
    if (positionJson.contains("positions") && positionJson["positions"].is_array())
    {
        for (const auto &slotData : positionJson["positions"])
        {
            if (slotData.value("endSimulation", false))
            { // Check if flag is present and true
                int currentSlot = slotData["slot"].get<int>();
                if (!endSimulationSlot.has_value() || currentSlot < endSimulationSlot.value())
                {
                    endSimulationSlot = currentSlot;
                }
            }
        }
    }

    // If a slot was found that requests the simulation to end
    if (endSimulationSlot.has_value())
    {
        if (positionJson.contains("engine_slot_time_microsec") && positionJson["engine_slot_time_microsec"].is_number())
        {
            double loadedSlotTime = positionJson["engine_slot_time_microsec"].get<double>();
            double new_max_time_sec = static_cast<double>(endSimulationSlot.value() + 1) * loadedSlotTime / 1e6;

            // Only override if the new max time is less than the current one
            if (new_max_time_sec < paramConfig.engine_max_time_sec)
            {
                if (paramConfig.display_mode != "CONSOLE") // Avoid spamming console logs
                {
                    std::cout << "Position config requests simulation end at slot "
                              << endSimulationSlot.value() << ". Overriding engine_max_time_sec to "
                              << new_max_time_sec << "s.\n";
                }
                paramConfig.engine_max_time_sec = new_max_time_sec;
            }
        }
    }
}
