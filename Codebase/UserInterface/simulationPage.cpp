#include "simulationPage.h"
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
#include <iostream>
#include <qcoreapplication.h>

#include "Experimental/imuExp/imuExp.h"
#include "Experimental/powerExp/powerExp.h"
#include "Simulation/imuSim/imuSim.h"
#include "Simulation/positionSim/positionSim.h"
#include "Simulation/rotarySim/rotarySim.h"
#include "Software/kpiClassifier/kpiClassifier.h"
#include "Software/mobileTHzEngine/mobileTHzEngine.h"
#include "Software/sharedPointersDefinition.h"

namespace
{
    inline double degreesToRadiansSimPage(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    inline double wattsToDbmSimPage(double watts)
    {
        if (watts <= 1e-18)
        {                  // Use a small threshold to avoid log(0)
            return -300.0; // Return a sensible floor value for display
        }
        return 10.0 * std::log10(watts * 1000.0);
    }

    inline double linearToDbi(double linear_gain)
    {
        if (linear_gain <= 1e-10)
        { // Gain should be > 0
            return -99.0;
        }
        return 10.0 * std::log10(linear_gain);
    }

    // Helper to Convert Az/Alt (Degrees) to Quaternion
    Quaternion quaternionFromAzAltDegreesSimPage(double azimuthDegrees, double altitudeDegrees)
    {
        // Qt uses scalar-first constructor
        QQuaternion azQuat = QQuaternion::fromAxisAndAngle(0.0f, 0.0f, 1.0f, azimuthDegrees);   // Z-axis
        QQuaternion altQuat = QQuaternion::fromAxisAndAngle(0.0f, 1.0f, 0.0f, altitudeDegrees); // Y-axis

        // Combine: Azimuth first, then Altitude
        QQuaternion combinedQ = azQuat * altQuat;

        return {combinedQ.scalar(), combinedQ.x(), combinedQ.y(), combinedQ.z()}; // Return struct
    }
    // Helper to multiply struct Quaternions
    Quaternion multiplyQuaternionsSimPage(const Quaternion &q1, const Quaternion &q2)
    {
        return toQuaternionStruct(toQQuaternion(q1) * toQQuaternion(q2)); // Leverage QQuaternion multiplication
    }

    inline const char *algoStateToString(AlgoState status)
    {
        switch (status)
        {
        case AlgoState::REFERENCE:
            return "REFERENCE";
        case AlgoState::MONITORING:
            return "MONITORING";
        case AlgoState::ALIGNMENT:
            return "ALIGNMENT";
        case AlgoState::PANIC_ALIGNMENT:
            return "PANIC_ALIGNMENT";
        case AlgoState::ERROR_STATUS:
            return "ERROR_STATE";
        case AlgoState::ZUPT:
            return "ZUPT";
        default:
            return "UNKNOWN_STATUS"; // Handles invalid integer values after cast
        }
    }

    inline const char *algoActionToString(AlgoAction action)
    {
        switch (action)
        {
        case AlgoAction::NONE:
            return "NONE";
        case AlgoAction::STARTING:
            return "STARTING";
        case AlgoAction::STOPPING:
            return "STOPPING";
        case AlgoAction::MOVING:
            return "MOVING";
        case AlgoAction::ERROR_ACTION:
            return "ERROR_ACTION";
        default:
            return "UNKNOWN_ACTION"; // Handles invalid integer values after cast
        }
    }

    inline const char *classifierClassToString(classifierClass class_)
    {
        switch (class_)
        {
        case classifierClass::INITIALIZING:
            return "INITIALIZING";
        case classifierClass::ALIGNED:
            return "ALIGNED";
        case classifierClass::MISALIGNED:
            return "MISALIGNED";
        default:
            return "UNKNOWN_ACTION"; // Handles invalid integer values after cast
        }
    }

    class ZoomableView : public QGraphicsView
    {
    public:
        explicit ZoomableView(QGraphicsScene *scene, QWidget *parent = nullptr)
            : QGraphicsView(scene, parent)
        {
            setRenderHint(QPainter::Antialiasing);
            setDragMode(QGraphicsView::ScrollHandDrag);
            setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
            setResizeAnchor(QGraphicsView::AnchorViewCenter);
        }

    protected:
        // Override drawBackground to create an "infinite" grid that responds to pan and zoom
        void drawBackground(QPainter *painter, const QRectF &rect) override
        {
            QGraphicsView::drawBackground(painter, rect);

            // --- Calculate dynamic alpha based on zoom level ---
            // transform().m11() gives the current horizontal scale (zoom)
            qreal zoom = transform().m11();

            // Define the maximum alpha (opacity) when zoom is at 1.0 or higher
            const int maxGridAlpha = 200;
            const int maxAxisAlpha = 250;

            // Calculate a dynamic alpha. As zoom < 1.0, alpha decreases.
            // Use qBound to clamp the value between 0 and the max desired alpha.
            int gridAlpha = qBound(0, static_cast<int>(maxGridAlpha * zoom), maxGridAlpha);
            int axisAlpha = qBound(0, static_cast<int>(maxAxisAlpha * zoom), maxAxisAlpha);

            // Create colors with the new dynamic alpha values
            QColor gridColor = QColor(50, 50, 50, gridAlpha);
            QColor axisColor = QColor(255, 255, 255, axisAlpha);

            // Define pens for grid and axes
            QPen gridPen(gridColor, 0);
            gridPen.setStyle(Qt::DashLine);
            QPen axisPen(axisColor, 0);

            // Fill the background
            painter->fillRect(rect, QColor(0x1f, 0x1f, 0x1f));

            // Get the visible scene rectangle
            qreal left = rect.left();
            qreal right = rect.right();
            qreal top = rect.top();
            qreal bottom = rect.bottom();
            qreal gridSize = 50.0;

            // Align grid to prevent "swimming" on pan
            qreal firstX = floor(left / gridSize) * gridSize;
            qreal firstY = floor(top / gridSize) * gridSize;

            // Draw vertical grid lines
            painter->setPen(gridPen);
            for (qreal x = firstX; x <= right; x += gridSize)
            {
                painter->drawLine(QPointF(x, top), QPointF(x, bottom));
            }

            // Draw horizontal grid lines
            for (qreal y = firstY; y <= bottom; y += gridSize)
            {
                painter->drawLine(QPointF(left, y), QPointF(right, y));
            }

            // --- Draw Main Axes (on top of grid) ---
            painter->setPen(axisPen);
            painter->drawLine(QPointF(0, top), QPointF(0, bottom)); // Y-axis
            painter->drawLine(QPointF(left, 0), QPointF(right, 0)); // X-axis
        }

        // Override the wheel event to handle zooming with the mouse wheel.
        void wheelEvent(QWheelEvent *event) override
        {
            const qreal scaleFactor = 1.15;
            if (event->angleDelta().y() > 0)
            {
                scale(scaleFactor, scaleFactor);
            }
            else
            {
                scale(1.0 / scaleFactor, 1.0 / scaleFactor);
            }
        }
    };
} // namespace

// Helper to get property by ID
PlotProperty SimulationPage::getPropertyById(const QString &propertyId) const
{
    for (const auto &prop : availableProperties)
    {
        if (prop.internalId == propertyId)
        {
            return prop;
        }
    }
    // Return a default/invalid property if not found
    return {"Invalid", "invalid", ""};
}

SimulationPage::SimulationPage(QWidget *parent)
    : QWidget(parent), viewRange(1000.0), unitSizeWorld(10.0), unitSizePixels(50.0), unitSelectorCombo(nullptr),
      yAxisSelectorCombo(nullptr), xAxisSelectorCombo(nullptr), graphTitleLabel(nullptr), dataSeries(new QLineSeries()),
      currentTimeIndicatorSeries(new QLineSeries()), dataChart(nullptr),
      axisX(nullptr), axisY(nullptr), currentYAxisMin(0.0),
      currentYAxisMax(0.0),
      engine(nullptr)
{
    setupUI();
    setupGraphSelectors();
}

SimulationPage::~SimulationPage() {}

void SimulationPage::setupPage(const ConfigFile &config)
{
    paramConfigFile = config;

    if (!mainLayout)
    {
        setupUI(); // Should already be called by constructor, but safe check
    }
    if (!unitSelectorCombo)
    { // Make sure selectors are set up
        setupGraphSelectors();
    }

    if (sharedPointers->engine)
    {
        engine = sharedPointers->engine.get();

        // 1. Populate the selectors
        populateSelectors();

        // 2. Setup slider range based on actual data size
        int numSlotsTotal = static_cast<int>(engine->getNumTimeSlots());
        timeSlider->setMinimum(0);
        timeSlider->setMaximum(numSlotsTotal > 0 ? numSlotsTotal - 1 : 0);
        timeSlider->setValue(0);

        // Connect slider to *updateDisplay* which calls everything else needed
        disconnect(timeSlider, &QSlider::valueChanged, this, nullptr); // Disconnect previous
        connect(timeSlider, &QSlider::valueChanged, this, &SimulationPage::updateDisplay);

        // 3. Create scene items
        createUnitItems(paramConfigFile.engine_units); // Also creates TimeValueWidgets now

        // When the tab changes, reload data for the new unit at the currently selected time slot.
        // A lambda function is used to capture the current state of the timeSlider.
        connect(unitTabWidget, &QTabWidget::currentChanged, this, [this](int newIndex)
                {
            // The `newIndex` from the signal is not needed here.
            Q_UNUSED(newIndex);

            // Preserve the slot by getting the current value from the time slider.
            int preservedSlot = timeSlider->value();

            // Call the main update function with the preserved slot. updateDisplay() will
            // automatically use the new tab's index to load the correct unit data.
            updateDisplay(preservedSlot); });

        // 4. Perform initial display update for everything else (XY view, labels)
        updateDisplay(0); // Update all visuals for the initial state (slot 0)
    }
    else
    {
        std::cerr << "Error in SimulationPage::setupPage: Engine is null!" << std::endl;
        dataSeries->clear();
        currentTimeIndicatorSeries->clear();
        if (axisX)
            axisX->setRange(0, 0);
        if (axisY)
            axisY->setRange(-100, 0);
        timeSlider->setEnabled(false);
        unitSelectorCombo->setEnabled(false);
        xAxisSelectorCombo->setEnabled(false);
        yAxisSelectorCombo->setEnabled(false);
    }
}

void SimulationPage::setSharedPointers(std::shared_ptr<SharedPointers> pointers)
{
    this->sharedPointers = pointers;
}

QString SimulationPage::formatVec3(const Position &p, int width, int prec, char f) const
{
    return QString("%1 %2 %3")
        .arg(p.x, width, f, prec, QLatin1Char(' '))
        .arg(p.y, width, f, prec, QLatin1Char(' '))
        .arg(p.z, width, f, prec, QLatin1Char(' '));
}

QString SimulationPage::formatQuat(const Quaternion &q, int width, int prec, char f) const
{
    return QString("%1 %2 %3 %4")
        .arg(q.w, width, f, prec, QLatin1Char(' '))
        .arg(q.x, width, f, prec, QLatin1Char(' '))
        .arg(q.y, width, f, prec, QLatin1Char(' '))
        .arg(q.z, width, f, prec, QLatin1Char(' '));
}

QString SimulationPage::formatRotaryAxis(const rotaryAxis &axis, int width, int prec, char f) const
{
    return QString("Ang: %1, Vel: %2, Acc: %3")
        .arg(axis.angle, width, f, prec, QLatin1Char(' '))
        .arg(axis.velocity, width, f, prec, QLatin1Char(' '))
        .arg(axis.acceleration, width, f, prec, QLatin1Char(' '));
}

QString SimulationPage::formatPacketData(const packetObject &packet) const
{
    const int fieldWidth = 6;
    const int precision = 2;
    // Calculate time in seconds based on slot duration
    double slotDurationMicrosec = paramConfigFile.engine_slot_time_microsec;
    double txTimeSec = static_cast<double>(packet.transmit_start_slot) * slotDurationMicrosec / 1e6;
    double rxEndTimeSec = static_cast<double>(packet.receive_end_slot) * slotDurationMicrosec / 1e6;

    // Use a multi-line raw string literal for cleaner formatting
    QString packetInfo = QString(QLatin1String(R"(
Tx Start: Slot %1 (%2 s)
Rx End: Slot %3 (%4 s)<br>
IMU Samples: %5
Delta Pos (X,Y,Z): %8
Delta Rot (W,X,Y,Z): %12)"))
                             .arg(static_cast<qint64>(packet.transmit_start_slot))
                             .arg(txTimeSec, fieldWidth, 'f', precision, QLatin1Char(' '))
                             .arg(static_cast<qint64>(packet.receive_end_slot))
                             .arg(rxEndTimeSec, fieldWidth, 'f', precision, QLatin1Char(' '))
                             .arg(static_cast<qint64>(packet.imu_data.size()))
                             .arg(formatVec3(packet.delta_position, fieldWidth, precision))
                             .arg(formatQuat(packet.delta_rotation, fieldWidth, precision));

    return packetInfo;
}

void SimulationPage::setupUI()
{
    // Rest of setup UI, not changed
    mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);
    mainLayout->setContentsMargins(10, 10, 10, 10);

    // --- Header Section ---
    QHBoxLayout *headerLayout = new QHBoxLayout();
    QLabel *titleLabel = new QLabel("Simulation", this);
    titleLabel->setStyleSheet("font-size: 42px; font-weight: bold; padding-top: 25px");
    headerLayout->addWidget(titleLabel, 0, Qt::AlignLeft | Qt::AlignBottom);
    headerLayout->addStretch(1);

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

    // --- Main Content Layout ---
    QHBoxLayout *contentLayout = new QHBoxLayout();
    mainLayout->addLayout(contentLayout);

    // --- Left Side (Graph Plot and XY View) ---
    QVBoxLayout *leftColumnLayout = new QVBoxLayout();
    contentLayout->addLayout(leftColumnLayout, 1);

    // --- Graph Plot Frame (Top Left) ---
    QFrame *graphPlotFrame = new QFrame(this);
    graphPlotFrame->setFrameShape(QFrame::StyledPanel);
    graphPlotFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *graphPlotLayout = new QVBoxLayout(graphPlotFrame);

    // --- Graph Title ---
    graphTitleLabel = new QLabel("Graph View", this);
    graphTitleLabel->setStyleSheet("font-size: 18px; font-weight: bold; padding-bottom: 5px;");
    graphPlotLayout->addWidget(graphTitleLabel);

    // --- Layout for Selectors (Combo Boxes) ---
    QHBoxLayout *selectorsLayout = new QHBoxLayout();

    // Unit Selector
    unitSelectorCombo = new QComboBox(this);
    QLabel *unitLabel = new QLabel("Unit:", this);
    unitLabel->setMaximumWidth(30);
    selectorsLayout->addWidget(unitLabel);
    selectorsLayout->addWidget(unitSelectorCombo);

    // Y-Axis Selector
    yAxisSelectorCombo = new QComboBox(this);
    QLabel *yAxisLabel = new QLabel("Unit:", this);
    yAxisLabel->setMaximumWidth(30);
    selectorsLayout->addWidget(yAxisLabel);
    selectorsLayout->addWidget(yAxisSelectorCombo);

    // X-Axis Selector
    xAxisSelectorCombo = new QComboBox(this);
    QLabel *xAxisLabel = new QLabel("Unit:", this);
    xAxisLabel->setMaximumWidth(30);
    selectorsLayout->addWidget(xAxisLabel);
    selectorsLayout->addWidget(xAxisSelectorCombo);

    graphPlotLayout->addLayout(selectorsLayout);

    // Separator Line
    QFrame *hLine = new QFrame();
    hLine->setFrameShape(QFrame::HLine);
    hLine->setFrameShadow(QFrame::Sunken);
    hLine->setStyleSheet("background-color: #3a3a3a;");
    graphPlotLayout->addWidget(hLine);

    // --- Chart ---
    dataChart = new QChart();
    dataChart->addSeries(dataSeries);
    dataChart->addSeries(currentTimeIndicatorSeries);
    dataChart->legend()->hide();
    dataChart->setBackgroundBrush(QBrush(QColor("#1f1f1f")));
    dataChart->setPlotAreaBackgroundBrush(QBrush(QColor("#1f1f1f")));
    dataChart->setPlotAreaBackgroundVisible(true);
    dataChart->setMargins(QMargins(0, 0, 0, 0));

    // --- Style the main data series ---
    QPen seriesPen(QColor(0, 180, 180));
    seriesPen.setWidth(2);
    dataSeries->setPen(seriesPen);

    // --- Style the current time indicator series ---
    QPen indicatorPen(Qt::red);
    indicatorPen.setWidth(2);
    indicatorPen.setStyle(Qt::DotLine);
    currentTimeIndicatorSeries->setPen(indicatorPen);

    // Common Axis Styling
    QPen gridPen(QColor(50, 50, 50));
    QPen axisPen(Qt::white);
    QBrush axisBrush(Qt::white);

    // X Axis Styling (Titles will be set dynamically)
    axisX = new QValueAxis;
    axisX->setLabelFormat("%.1f");
    axisX->setTickCount(11);
    axisX->setMinorTickCount(1);
    axisX->setLinePen(axisPen);
    axisX->setGridLinePen(gridPen);
    axisX->setLabelsBrush(axisBrush);
    axisX->setTitleBrush(axisBrush);
    axisX->setMinorGridLineVisible(false);
    dataChart->addAxis(axisX, Qt::AlignBottom);
    dataSeries->attachAxis(axisX);
    currentTimeIndicatorSeries->attachAxis(axisX);

    // Y Axis Styling
    axisY = new QValueAxis;
    axisY->setLabelFormat("%.1f");
    axisY->setLinePen(axisPen);
    axisY->setGridLinePen(gridPen);
    axisY->setLabelsBrush(axisBrush);
    axisY->setTitleBrush(axisBrush);
    axisY->setMinorGridLineVisible(false);
    dataChart->addAxis(axisY, Qt::AlignLeft);
    dataSeries->attachAxis(axisY);
    currentTimeIndicatorSeries->attachAxis(axisY);

    QChartView *chartView = new QChartView(dataChart, this);
    chartView->setRenderHint(QPainter::Antialiasing);
    graphPlotLayout->addWidget(chartView);
    leftColumnLayout->addWidget(graphPlotFrame, 1);

    // --- Connect Selector Signals ---
    connect(unitSelectorCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &SimulationPage::updateGraphSelections);
    connect(yAxisSelectorCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &SimulationPage::updateGraphSelections);
    connect(xAxisSelectorCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &SimulationPage::updateGraphSelections);

    // --- XY View (Bottom Left) ---
    QFrame *xyViewFrame = new QFrame(this);
    xyViewFrame->setFrameShape(QFrame::StyledPanel);
    xyViewFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *xyViewLayout = new QVBoxLayout(xyViewFrame);

    // --- Create a layout for the title and the new view selector ---
    QHBoxLayout *titleAndSelectorLayout = new QHBoxLayout();

    QLabel *xyViewTitle = new QLabel("View Plane:", this);
    xyViewTitle->setStyleSheet("font-size: 18px; font-weight: bold; border-bottom: 2px solid "
                               "#3a3a3a; border-radius: 0px; padding-bottom: 5px;");

    // --- Create and configure the view selector combo box ---
    viewSelectorCombo = new QComboBox(this);
    viewSelectorCombo->addItem("XY", static_cast<int>(ViewPlane::XY));
    viewSelectorCombo->addItem("XZ", static_cast<int>(ViewPlane::XZ));
    viewSelectorCombo->addItem("YZ", static_cast<int>(ViewPlane::YZ));

    // --- Add widgets to the title layout ---
    titleAndSelectorLayout->addWidget(xyViewTitle);
    titleAndSelectorLayout->addStretch();
    viewSelectorCombo->setFixedWidth(100);
    titleAndSelectorLayout->addWidget(viewSelectorCombo);

    // --- Add the title layout to the main view layout ---
    xyViewLayout->addLayout(titleAndSelectorLayout);

    scene = new QGraphicsScene(this);
    scene->setSceneRect(-100000, -100000, 200000, 200000);
    view = new ZoomableView(scene, this);
    view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    xyViewLayout->addWidget(view);

    leftColumnLayout->addWidget(xyViewFrame, 1);

    // --- Right Side (Time Values and Slider) ---
    QVBoxLayout *rightColumnLayout = new QVBoxLayout();
    contentLayout->addLayout(rightColumnLayout, 2);

    // --- Time Values (Top Right) ---
    QFrame *timeValuesFrame = new QFrame(this);
    timeValuesFrame->setFrameShape(QFrame::StyledPanel);
    timeValuesFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *timeValuesFrameLayout = new QVBoxLayout(timeValuesFrame);
    QLabel *timeValuesTitle = new QLabel("Current Simulation Values", this);
    timeValuesTitle->setStyleSheet("font-size: 18px; font-weight: bold; border-bottom: 2px solid "
                                   "#3a3a3a; border-radius: 0px; padding-bottom: 5px;");
    timeValuesFrameLayout->addWidget(timeValuesTitle);

    // Create the QTabWidget
    unitTabWidget = new QTabWidget(this);
    unitTabWidget->setStyleSheet(
        "QTabWidget::pane { border: 0; }"
        "QTabBar::tab { background: #3a3a3a; color: white; padding: 5px; border-top-left-radius: "
        "4px; border-top-right-radius: 4px; }"
        "QTabBar::tab:selected { background: #C8102E; }"
        "QTabWidget::tab-bar { left: 5px; }");

    timeValuesFrameLayout->addWidget(unitTabWidget);

    rightColumnLayout->addWidget(timeValuesFrame);

    // --- Time Control (Bottom Right) - Slider ---
    QFrame *timeControlFrame = new QFrame(this);
    timeControlFrame->setFrameShape(QFrame::StyledPanel);
    timeControlFrame->setStyleSheet("background-color: #1f1f1f; border-radius: 5px;");
    QVBoxLayout *timeControlLayout = new QVBoxLayout(timeControlFrame);

    timeSlider = new QSlider(Qt::Horizontal, this);
    timeSlider->setMinimum(0);
    timeSlider->setMaximum(100);
    timeSlider->setTickInterval(1);
    connect(timeSlider, &QSlider::valueChanged, this, &SimulationPage::updateDisplay);
    timeControlLayout->addWidget(timeSlider);

    timeLabel = new QLabel("Time Slot: 0", this);
    timeLabel->setStyleSheet("font-size: 16px; color: white; text-align: center;");
    timeLabel->setAlignment(Qt::AlignCenter);
    timeControlLayout->addWidget(timeLabel);

    rightColumnLayout->addWidget(timeControlFrame);
    connect(viewSelectorCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &SimulationPage::onViewPlaneChanged);

    this->setLayout(mainLayout);
}

void SimulationPage::onViewPlaneChanged(int index)
{
    // Get the enum value from the combo box's data
    currentViewPlane = static_cast<ViewPlane>(viewSelectorCombo->itemData(index).toInt());
    // Trigger a full redraw of the scene at the current time
    updateDisplay(timeSlider->value());
}

void SimulationPage::updateDisplay(int slot)
{
    if (!engine)
        return;

    int numSlotsTotal = static_cast<int>(engine->getNumTimeSlots());
    slot = std::max(0, std::min(slot, numSlotsTotal > 0 ? numSlotsTotal - 1 : 0));

    double timeInSeconds = static_cast<double>(slot) * paramConfigFile.engine_slot_time_microsec / 1e6;
    timeLabel->setText(QString("Time Slot: %1 / %2 (%3 s)")
                           .arg(slot)
                           .arg(numSlotsTotal > 0 ? numSlotsTotal - 1 : 0)
                           .arg(timeInSeconds, 0, 'f', 3));

    updateXYView(slot);
    updateTimeValuesDisplay(slot);
    updateScrubberLine(slot); // Update the vertical line
}

void SimulationPage::updateScrubberLine(int slot)
{
    if (!engine || !dataChart || !axisX || !axisY || !currentTimeIndicatorSeries || !xAxisSelectorCombo || !unitSelectorCombo)
    {
        return;
    }

    int numSlotsTotal = static_cast<int>(engine->getNumTimeSlots());
    if (numSlotsTotal <= 0)
    {
        currentTimeIndicatorSeries->clear();
        return;
    }

    slot = std::max(0, std::min(slot, numSlotsTotal - 1));

    // --- Calculate X Coordinate based on Selected X-Axis ---
    QString xPropertyId = xAxisSelectorCombo->currentData().toString();
    int unitIndex = unitSelectorCombo->currentData().toInt(); // Get selected unit index
    if (unitIndex < 0 || unitIndex >= engine->getNumUnits() || xPropertyId.isEmpty())
    {
        currentTimeIndicatorSeries->clear();
        return;
    }

    qreal currentXValue = getPropertyValue(unitIndex, slot, xPropertyId);

    // --- Update Current Time Indicator Line ---
    QVector<QPointF> indicatorPoints;
    // Use the Y-axis range calculated during the last updateGraphPlot call
    indicatorPoints.append(QPointF(currentXValue, currentYAxisMin));
    indicatorPoints.append(QPointF(currentXValue, currentYAxisMax));

    currentTimeIndicatorSeries->replace(indicatorPoints);
}

void SimulationPage::updateTimeValuesDisplay(int slot)
{
    if (!engine || slot < 0 || static_cast<size_t>(slot) >= engine->getNumTimeSlots())
    {
        return;
    }

    // --- Only update the currently visible tab's content ---
    int currentTabIndex = unitTabWidget->currentIndex();
    if (currentTabIndex < 0 || static_cast<size_t>(currentTabIndex) >= engine->getNumUnits())
    {
        return; // No valid tab selected or no units
    }

    size_t unitIndex = static_cast<size_t>(currentTabIndex);
    const engineUnit &unitConfig = engine->getEngineUnit(unitIndex); // Get config for enabled checks
    const auto &currentUnitLabels = unitDisplayLabels[unitIndex];    // Get the map of labels for this specific unit

    // Define consistent formatting parameters for numerical values
    const int fieldWidth = 6;
    const int precision = 2;     // Keep 2 decimal places for most values
    const int quatPrecision = 4; // Quaternions usually need more precision
    const int slotPrecision = 0; // Slots are integers

    // --- Fetch and Update Environment Data ---
    if (currentUnitLabels.contains("env_pos_x"))
    {
        environmentObject envData = engine->getEnvironmentData(unitIndex, slot);
        currentUnitLabels["env_pos_x"]->setText(QString::number(envData.position.x, 'f', precision));
        currentUnitLabels["env_pos_y"]->setText(QString::number(envData.position.y, 'f', precision));
        currentUnitLabels["env_pos_z"]->setText(QString::number(envData.position.z, 'f', precision));

        currentUnitLabels["env_vel_x"]->setText(QString::number(envData.velocity.x, 'f', precision));
        currentUnitLabels["env_vel_y"]->setText(QString::number(envData.velocity.y, 'f', precision));
        currentUnitLabels["env_vel_z"]->setText(QString::number(envData.velocity.z, 'f', precision));

        currentUnitLabels["env_acc_x"]->setText(
            QString::number(envData.acceleration.x, 'f', precision));
        currentUnitLabels["env_acc_y"]->setText(
            QString::number(envData.acceleration.y, 'f', precision));
        currentUnitLabels["env_acc_z"]->setText(
            QString::number(envData.acceleration.z, 'f', precision));

        currentUnitLabels["env_quat_w"]->setText(
            QString::number(envData.quaternion.w, 'f', quatPrecision));
        currentUnitLabels["env_quat_x"]->setText(
            QString::number(envData.quaternion.x, 'f', quatPrecision));
        currentUnitLabels["env_quat_y"]->setText(
            QString::number(envData.quaternion.y, 'f', quatPrecision));
        currentUnitLabels["env_quat_z"]->setText(
            QString::number(envData.quaternion.z, 'f', quatPrecision));

        currentUnitLabels["env_ang_vel_x"]->setText(
            QString::number(envData.angular_velocity.x, 'f', precision));
        currentUnitLabels["env_ang_vel_y"]->setText(
            QString::number(envData.angular_velocity.y, 'f', precision));
        currentUnitLabels["env_ang_vel_z"]->setText(
            QString::number(envData.angular_velocity.z, 'f', precision));

        currentUnitLabels["env_ang_acc_x"]->setText(
            QString::number(envData.angular_acceleration.x, 'f', precision));
        currentUnitLabels["env_ang_acc_y"]->setText(
            QString::number(envData.angular_acceleration.y, 'f', precision));
        currentUnitLabels["env_ang_acc_z"]->setText(
            QString::number(envData.angular_acceleration.z, 'f', precision));
    }

    // --- Fetch and Update IMU Data ---
    if (currentUnitLabels.contains("imu_acc_x") && unitConfig.IMU_enabled)
    {
        imuObject imuData = engine->getIMUData(unitIndex, slot);
        currentUnitLabels["imu_acc_x"]->setText(
            QString::number(imuData.acceleration.x, 'f', precision));
        currentUnitLabels["imu_acc_y"]->setText(
            QString::number(imuData.acceleration.y, 'f', precision));
        currentUnitLabels["imu_acc_z"]->setText(
            QString::number(imuData.acceleration.z, 'f', precision));

        currentUnitLabels["imu_ang_vel_x"]->setText(
            QString::number(imuData.angular_velocity.x, 'f', precision));
        currentUnitLabels["imu_ang_vel_y"]->setText(
            QString::number(imuData.angular_velocity.y, 'f', precision));
        currentUnitLabels["imu_ang_vel_z"]->setText(
            QString::number(imuData.angular_velocity.z, 'f', precision));

        currentUnitLabels["imu_quat_w"]->setText(
            QString::number(imuData.quaternion.w, 'f', quatPrecision));
        currentUnitLabels["imu_quat_x"]->setText(
            QString::number(imuData.quaternion.x, 'f', quatPrecision));
        currentUnitLabels["imu_quat_y"]->setText(
            QString::number(imuData.quaternion.y, 'f', quatPrecision));
        currentUnitLabels["imu_quat_z"]->setText(
            QString::number(imuData.quaternion.z, 'f', quatPrecision));

        currentUnitLabels["imu_sample_slot"]->setText(
            QString::number(imuData.sample_slot, 'f', slotPrecision));
    }
    else
    {
        if (currentUnitLabels.contains("imu_acc_x"))
        {
            currentUnitLabels["imu_acc_x"]->setText("N/A");
            currentUnitLabels["imu_acc_y"]->setText("N/A");
            currentUnitLabels["imu_acc_z"]->setText("N/A");
            currentUnitLabels["imu_ang_vel_x"]->setText("N/A");
            currentUnitLabels["imu_ang_vel_y"]->setText("N/A");
            currentUnitLabels["imu_ang_vel_z"]->setText("N/A");
            currentUnitLabels["imu_quat_w"]->setText("N/A");
            currentUnitLabels["imu_quat_x"]->setText("N/A");
            currentUnitLabels["imu_quat_y"]->setText("N/A");
            currentUnitLabels["imu_quat_z"]->setText("N/A");
            currentUnitLabels["imu_sample_slot"]->setText("N/A");
        }
    }

    // --- Fetch and Update RxChain Data ---
    if (currentUnitLabels.contains("rxChain_power") && unitConfig.RxChain_enabled)
    {
        rxChainObject rxChainData = engine->getRxChainData(unitIndex, slot);
        currentUnitLabels["rxChain_power"]->setText(
            QString::number(wattsToDbmSimPage(rxChainData.power_watts), 'f', precision));
        currentUnitLabels["rxChain_sample_slot"]->setText(
            QString::number(rxChainData.sample_slot, 'f', slotPrecision));
    }
    else
    {
        if (currentUnitLabels.contains("rxChain_power"))
        {
            currentUnitLabels["rxChain_power"]->setText("N/A");
            currentUnitLabels["rxChain_sample_slot"]->setText("N/A");
        }
    }

    // --- Fetch and Update Tx Antenna Data ---
    if (currentUnitLabels.contains("txAntenna_gain") && unitConfig.TxAntenna_enabled)
    {
        antennaObject txAntennaData = engine->getTxAntennaData(unitIndex, slot);
        currentUnitLabels["txAntenna_gain"]->setText(
            QString::number(linearToDbi(txAntennaData.gain_linear), 'f', precision));
        currentUnitLabels["txAntenna_power"]->setText(
            QString::number(wattsToDbmSimPage(txAntennaData.power_watts), 'f', precision));
        currentUnitLabels["txAntenna_pattern"]->setText(
            QString::number(txAntennaData.radiationPatternID, 'f', 0));
    }
    else
    {
        if (currentUnitLabels.contains("txAntenna_gain"))
        {
            currentUnitLabels["txAntenna_gain"]->setText("N/A");
            currentUnitLabels["txAntenna_power"]->setText("N/A");
            currentUnitLabels["txAntenna_pattern"]->setText("N/A");
        }
    }

    // --- Fetch and Update Rx Antenna Data ---
    if (currentUnitLabels.contains("rxAntenna_gain") && unitConfig.RxAntenna_enabled)
    {
        antennaObject rxAntennaData = engine->getRxAntennaData(unitIndex, slot);
        currentUnitLabels["rxAntenna_gain"]->setText(
            QString::number(linearToDbi(rxAntennaData.gain_linear), 'f', precision));
        currentUnitLabels["rxAntenna_power"]->setText(
            QString::number(wattsToDbmSimPage(rxAntennaData.power_watts), 'f', precision));
        currentUnitLabels["rxAntenna_pattern"]->setText(
            QString::number(rxAntennaData.radiationPatternID, 'f', 0));
    }
    else
    {
        if (currentUnitLabels.contains("rxAntenna_gain"))
        {
            currentUnitLabels["rxAntenna_gain"]->setText("N/A");
            currentUnitLabels["rxAntenna_power"]->setText("N/A");
            currentUnitLabels["rxAntenna_pattern"]->setText("N/A");
        }
    }

    // --- Fetch and Update Rotary Data ---
    if (currentUnitLabels.contains("rotary_az_angle") && unitConfig.Rotary_enabled)
    {
        rotaryObject rotaryData = engine->getRotaryData(unitIndex, slot);
        // Individual components for Azimuth
        currentUnitLabels["rotary_az_angle"]->setText(
            QString::number(rotaryData.azimuth.angle, 'f', precision));
        currentUnitLabels["rotary_az_vel"]->setText(
            QString::number(rotaryData.azimuth.velocity, 'f', precision));
        currentUnitLabels["rotary_az_acc"]->setText(
            QString::number(rotaryData.azimuth.acceleration, 'f', precision));
        // Individual components for Altitude
        currentUnitLabels["rotary_alt_angle"]->setText(
            QString::number(rotaryData.altitude.angle, 'f', precision));
        currentUnitLabels["rotary_alt_vel"]->setText(
            QString::number(rotaryData.altitude.velocity, 'f', precision));
        currentUnitLabels["rotary_alt_acc"]->setText(
            QString::number(rotaryData.altitude.acceleration, 'f', precision));
        // Is Moving
        currentUnitLabels["rotary_is_moving"]->setText((rotaryData.isMoving == 1.0) ? "Yes" : "No");
    }
    else
    {
        if (currentUnitLabels.contains("rotary_az_angle"))
        {
            currentUnitLabels["rotary_az_angle"]->setText("N/A");
            currentUnitLabels["rotary_az_vel"]->setText("N/A");
            currentUnitLabels["rotary_az_acc"]->setText("N/A");
            currentUnitLabels["rotary_alt_angle"]->setText("N/A");
            currentUnitLabels["rotary_alt_vel"]->setText("N/A");
            currentUnitLabels["rotary_alt_acc"]->setText("N/A");
            currentUnitLabels["rotary_is_moving"]->setText("N/A");
        }
    }

    // --- Fetch and Update Algorithm Data ---
    if (currentUnitLabels.contains("algo_status"))
    {
        algorithmObject algoData = engine->getAlgorithmData(unitIndex, slot);
        currentUnitLabels["algo_status"]->setText(
            algoStateToString(static_cast<AlgoState>(algoData.status)));
        currentUnitLabels["algo_action"]->setText(
            algoActionToString(static_cast<AlgoAction>(algoData.action)));
    }
    else
    {
        if (currentUnitLabels.contains("algo_status"))
        {
            currentUnitLabels["algo_status"]->setText("N/A");
            currentUnitLabels["algo_action"]->setText("N/A");
        }
    }

    // --- Packet Data Display (Sent) ---
    const auto &allPacketsInLayer = engine->getPacketLayer();
    if (packetDataDisplays.contains(unitIndex))
    {
        QString packetDisplayText;
        int packetsDisplayedCount = 0;
        const int MAX_PACKETS_TO_DISPLAY_PER_UNIT = 3;

        for (auto it = allPacketsInLayer.rbegin(); it != allPacketsInLayer.rend(); ++it)
        {
            const packetObject &packet = *it;
            // Filter: Only show packets SENT by *this* unit and transmitted by the current slot
            if (packet.unit_index == unitIndex && packet.transmit_start_slot <= slot)
            {
                if (packetsDisplayedCount >= MAX_PACKETS_TO_DISPLAY_PER_UNIT)
                {
                    packetDisplayText.append(QString("<br><i>... +%1 more sent packets...</i>")
                                                 .arg(std::distance(it, allPacketsInLayer.rend())));
                    break;
                }
                packetDisplayText.append(
                    QString("<hr><b>Packet ID: %1</b><br>").arg(static_cast<qint64>(packet.id)));
                packetDisplayText.append(formatPacketData(packet));
                packetsDisplayedCount++;
            }
        }
        if (packetDisplayText.isEmpty())
        {
            packetDisplayText = "<i>No packets sent by this unit yet.</i>";
        }
        packetDataDisplays[unitIndex]->setHtml(packetDisplayText);
    }

    // --- Received Packet Data Display ---
    if (receivedPacketDataDisplays.contains(unitIndex))
    {
        QString receivedPacketDisplayText;
        int receivedPacketsDisplayedCount = 0;
        const int MAX_RECEIVED_PACKETS_TO_DISPLAY_PER_UNIT = 3; // Set a different limit if desired...

        // Iterate through all packets in reverse order to show the most recent ones first
        for (auto it = allPacketsInLayer.rbegin(); it != allPacketsInLayer.rend(); ++it)
        {
            const packetObject &packet = *it;
            // Filter: Show packets RECEIVED by *this* unit (sent by others) AND fully received by the current slot
            if (packet.unit_index != unitIndex && packet.receive_end_slot <= slot)
            { // Packet was sent by another unit AND received by this unit
                if (receivedPacketsDisplayedCount >= MAX_RECEIVED_PACKETS_TO_DISPLAY_PER_UNIT)
                {
                    receivedPacketDisplayText.append(
                        QString("<br><i>... +%1 more received packets...</i>")
                            .arg(std::distance(it, allPacketsInLayer.rend())));
                    break;
                }
                // Append formatted packet data, with a clear separator and bold header
                receivedPacketDisplayText.append(
                    QString("<hr><b>Packet ID: %1 (from Unit %2)</b><br>")
                        .arg(static_cast<qint64>(packet.id))
                        .arg(static_cast<qint64>(packet.unit_index))); // Show sender unit
                receivedPacketDisplayText.append(
                    formatPacketData(packet)); // Reuse existing formatter
                receivedPacketsDisplayedCount++;
            }
        }
        if (receivedPacketDisplayText.isEmpty())
        {
            receivedPacketDisplayText = "<i>No packets received by this unit yet.</i>";
        }
        receivedPacketDataDisplays[unitIndex]->setHtml(receivedPacketDisplayText);
    }

    // --- Fetch and Update Classifier Data ---
    if (currentUnitLabels.contains("classifier_class"))
    {
        classifierClass classifierData = engine->getClassifierData(unitIndex, slot);
        currentUnitLabels["classifier_class"]->setText(classifierClassToString(classifierData));
    }
    else
    {
        if (currentUnitLabels.contains("classifier_class"))
        {
            currentUnitLabels["classifier_class"]->setText("N/A");
        }
    }
}

void SimulationPage::createTimeValueWidgets()
{
    // Clear previous tabs and cached labels/widgets if setupPage is called multiple times
    unitTabWidget->clear();
    unitDisplayLabels.clear();  // Clear the main map of labels
    packetDataDisplays.clear(); // Clear packet text edits map

    auto engineUnits = engine->getAllEngineUnits();

    // Define consistent styles to be reused (matching your UI examples)
    const QString valueLabelStyle = "font-size: 14px; color: white;";
    const QString componentLabelStyle = "font-size: 14px; color: #BBBBBB;";

    const QString groupBoxStyle = R"(
        QGroupBox {
            font-size: 14px; /* Matches content labels, distinct from Simulation page title */
            font-weight: bold;
            color: white;
            border: 1px solid #3a3a3a;
            border-radius: 5px;
            margin-top: 10px; /* Space from previous groupbox */
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            padding: 0 3px; /* Small padding around title text */
        }
    )";
    const QString textEditStyle = "font-size: 14px; color: white; background-color: #2b2b2b; border: 1px solid #3a3a3a;";

    for (size_t unitIndex = 0; unitIndex < engineUnits.size(); ++unitIndex)
    {
        // --- 1. Create a QWidget for the tab content ---
        QWidget *tabContentWidget = new QWidget(unitTabWidget);
        QVBoxLayout *tabLayout = new QVBoxLayout(tabContentWidget);
        tabLayout->setContentsMargins(5, 5, 5, 5);
        tabLayout->setSpacing(5);

        // --- 2. Create a QScrollArea for the tab content ---
        QScrollArea *tabScrollArea = new QScrollArea(unitTabWidget);
        tabScrollArea->setWidgetResizable(true);
        tabScrollArea->setStyleSheet(
            "QScrollArea { border: none; background-color: transparent; }");
        tabScrollArea->setWidget(tabContentWidget);

        // --- 3. Add Tab to QTabWidget ---
        unitTabWidget->addTab(tabScrollArea,
                              QString("Unit %1 (%2)")
                                  .arg(unitIndex)
                                  .arg(QString::fromStdString(engineUnits[unitIndex].label)));

        // Initialize the inner QMap for this unit's labels
        unitDisplayLabels[unitIndex] = QMap<QString, QLabel *>();

        // --- Helper Lambda for creating value-label pairs ---
        auto createAndStoreValueLabel = [&](const QString &labelText,
                                            const QString &internalIdSuffix,
                                            QLayout *parentLayout,
                                            QWidget *parentWidget,
                                            int valueStretch = 0)
        {
            QLabel *compLabel = nullptr;
            if (!labelText.isEmpty())
            { // Only create component label if text is provided
                compLabel = new QLabel(labelText, parentWidget);
                compLabel->setStyleSheet(componentLabelStyle);
            }

            QLabel *compVal = new QLabel("---", parentWidget);
            compVal->setStyleSheet(valueLabelStyle);
            compVal->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));

            if (QHBoxLayout *hLayout = qobject_cast<QHBoxLayout *>(parentLayout))
            {
                if (compLabel)
                    hLayout->addWidget(compLabel);
                hLayout->addWidget(compVal, valueStretch);
            }
            else if (QFormLayout *fLayout = qobject_cast<QFormLayout *>(parentLayout))
            {
                // For QFormLayout, add the component label to the label column, and the value label to the field column
                // If no component label (labelText is empty), use the compVal directly in the field role.
                if (compLabel)
                {
                    fLayout->addRow(compLabel, compVal);
                }
                else
                {
                    fLayout->addRow(
                        compVal); // Add just the value label, it will span both columns or be in the field role
                }
            }
            else
            {
                if (compLabel)
                    parentLayout->addWidget(compLabel);
                parentLayout->addWidget(compVal);
            }

            unitDisplayLabels[unitIndex][internalIdSuffix] = compVal; // Store pointer
            return compVal;
        };

        // --- Helper Lambda for adding a row where the "field" is a QHBoxLayout ---
        auto addHorizontalValueRow = [&](QFormLayout *formLayout,
                                         const QString &rowLabelText,
                                         QHBoxLayout *hBoxLayout)
        {
            QLabel *rowLabel = new QLabel(rowLabelText, formLayout->parentWidget());
            rowLabel->setStyleSheet(componentLabelStyle); // Label for the row (e.g., "Position:")
            formLayout->addRow(rowLabel, hBoxLayout);
        };

        // --- Environment Data Group Box ---
        QGroupBox *envGroupBox = new QGroupBox("Environment Data", tabContentWidget);
        envGroupBox->setCheckable(true);
        envGroupBox->setChecked(true);
        envGroupBox->setStyleSheet(groupBoxStyle);
        QFormLayout *envFormLayout = new QFormLayout(envGroupBox);
        envFormLayout->setLabelAlignment(Qt::AlignLeft);
        envFormLayout->setContentsMargins(5, 5, 5, 5);
        envFormLayout->setHorizontalSpacing(10);

        // Environment: Position (X,Y,Z)
        QHBoxLayout *envPosLayout = new QHBoxLayout();
        envPosLayout->setContentsMargins(0, 0, 0, 0);
        envPosLayout->setSpacing(5);
        addHorizontalValueRow(envFormLayout, "Position:", envPosLayout);
        createAndStoreValueLabel("X:", "env_pos_x", envPosLayout, envGroupBox, 1);
        createAndStoreValueLabel("Y:", "env_pos_y", envPosLayout, envGroupBox, 1);
        createAndStoreValueLabel("Z:", "env_pos_z", envPosLayout, envGroupBox, 1);

        // Environment: Velocity (X,Y,Z)
        QHBoxLayout *envVelLayout = new QHBoxLayout();
        envVelLayout->setContentsMargins(0, 0, 0, 0);
        envVelLayout->setSpacing(5);
        addHorizontalValueRow(envFormLayout, "Velocity:", envVelLayout);
        createAndStoreValueLabel("X:", "env_vel_x", envVelLayout, envGroupBox, 1);
        createAndStoreValueLabel("Y:", "env_vel_y", envVelLayout, envGroupBox, 1);
        createAndStoreValueLabel("Z:", "env_vel_z", envVelLayout, envGroupBox, 1);

        // Environment: Acceleration (X,Y,Z)
        QHBoxLayout *envAccLayout = new QHBoxLayout();
        envAccLayout->setContentsMargins(0, 0, 0, 0);
        envAccLayout->setSpacing(5);
        addHorizontalValueRow(envFormLayout, "Accel.:", envAccLayout);
        createAndStoreValueLabel("X:", "env_acc_x", envAccLayout, envGroupBox, 1);
        createAndStoreValueLabel("Y:", "env_acc_y", envAccLayout, envGroupBox, 1);
        createAndStoreValueLabel("Z:", "env_acc_z", envAccLayout, envGroupBox, 1);

        // Environment: Angular Velocity (X,Y,Z)
        QHBoxLayout *envAngVelLayout = new QHBoxLayout();
        envAngVelLayout->setContentsMargins(0, 0, 0, 0);
        envAngVelLayout->setSpacing(5);
        addHorizontalValueRow(envFormLayout, "Ang. Vel.:", envAngVelLayout);
        createAndStoreValueLabel("X:", "env_ang_vel_x", envAngVelLayout, envGroupBox, 1);
        createAndStoreValueLabel("Y:", "env_ang_vel_y", envAngVelLayout, envGroupBox, 1);
        createAndStoreValueLabel("Z:", "env_ang_vel_z", envAngVelLayout, envGroupBox, 1);

        // Environment: Angular Acceleration (X,Y,Z)
        QHBoxLayout *envAngAccLayout = new QHBoxLayout();
        envAngAccLayout->setContentsMargins(0, 0, 0, 0);
        envAngAccLayout->setSpacing(5);
        addHorizontalValueRow(envFormLayout, "Ang. Accel.:", envAngAccLayout);
        createAndStoreValueLabel("X:", "env_ang_acc_x", envAngAccLayout, envGroupBox, 1);
        createAndStoreValueLabel("Y:", "env_ang_acc_y", envAngAccLayout, envGroupBox, 1);
        createAndStoreValueLabel("Z:", "env_ang_acc_z", envAngAccLayout, envGroupBox, 1);

        // Environment: Quaternion (W,X,Y,Z)
        QHBoxLayout *envQuatLayout = new QHBoxLayout();
        envQuatLayout->setContentsMargins(0, 0, 0, 0);
        envQuatLayout->setSpacing(5);
        addHorizontalValueRow(envFormLayout, "Quaternion:", envQuatLayout);
        createAndStoreValueLabel("W:", "env_quat_w", envQuatLayout, envGroupBox, 1);
        createAndStoreValueLabel("X:", "env_quat_x", envQuatLayout, envGroupBox, 1);
        createAndStoreValueLabel("Y:", "env_quat_y", envQuatLayout, envGroupBox, 1);
        createAndStoreValueLabel("Z:", "env_quat_z", envQuatLayout, envGroupBox, 1);

        tabLayout->addWidget(envGroupBox);
        connect(envGroupBox, &QGroupBox::toggled, envGroupBox, &QGroupBox::setVisible);

        // --- IMU Data Group Box ---
        if (engineUnits[unitIndex].IMU_enabled)
        {
            QGroupBox *imuGroupBox = new QGroupBox("IMU Data", tabContentWidget);
            imuGroupBox->setCheckable(true);
            imuGroupBox->setChecked(true);
            imuGroupBox->setStyleSheet(groupBoxStyle);
            QFormLayout *imuFormLayout = new QFormLayout(imuGroupBox);
            imuFormLayout->setLabelAlignment(Qt::AlignLeft);
            imuFormLayout->setContentsMargins(5, 5, 5, 5);
            imuFormLayout->setHorizontalSpacing(10);

            // IMU: Acceleration (X,Y,Z)
            QHBoxLayout *imuAccLayout = new QHBoxLayout();
            imuAccLayout->setContentsMargins(0, 0, 0, 0);
            imuAccLayout->setSpacing(5);
            addHorizontalValueRow(imuFormLayout, "Accel.:", imuAccLayout);
            createAndStoreValueLabel("X:", "imu_acc_x", imuAccLayout, imuGroupBox, 1);
            createAndStoreValueLabel("Y:", "imu_acc_y", imuAccLayout, imuGroupBox, 1);
            createAndStoreValueLabel("Z:", "imu_acc_z", imuAccLayout, imuGroupBox, 1);

            // IMU: Angular Velocity (X,Y,Z)
            QHBoxLayout *imuAngVelLayout = new QHBoxLayout();
            imuAngVelLayout->setContentsMargins(0, 0, 0, 0);
            imuAngVelLayout->setSpacing(5);
            addHorizontalValueRow(imuFormLayout, "Ang. Vel.:", imuAngVelLayout);
            createAndStoreValueLabel("X:", "imu_ang_vel_x", imuAngVelLayout, imuGroupBox, 1);
            createAndStoreValueLabel("Y:", "imu_ang_vel_y", imuAngVelLayout, imuGroupBox, 1);
            createAndStoreValueLabel("Z:", "imu_ang_vel_z", imuAngVelLayout, imuGroupBox, 1);

            // IMU: Quaternion (W,X,Y,Z)
            QHBoxLayout *imuQuatLayout = new QHBoxLayout();
            imuQuatLayout->setContentsMargins(0, 0, 0, 0);
            imuQuatLayout->setSpacing(5);
            addHorizontalValueRow(imuFormLayout, "Quaternion:", imuQuatLayout);
            createAndStoreValueLabel("W:", "imu_quat_w", imuQuatLayout, imuGroupBox, 1);
            createAndStoreValueLabel("X:", "imu_quat_x", imuQuatLayout, imuGroupBox, 1);
            createAndStoreValueLabel("Y:", "imu_quat_y", imuQuatLayout, imuGroupBox, 1);
            createAndStoreValueLabel("Z:", "imu_quat_z", imuQuatLayout, imuGroupBox, 1);

            // IMU: Sample Slot
            createAndStoreValueLabel("Sample Slot:", "imu_sample_slot", imuFormLayout, imuGroupBox);

            tabLayout->addWidget(imuGroupBox);
            connect(imuGroupBox, &QGroupBox::toggled, imuGroupBox, &QGroupBox::setVisible);
        }

        // --- RxChain Data Group Box ---
        if (engineUnits[unitIndex].RxChain_enabled)
        {
            QGroupBox *rxChainGroupBox = new QGroupBox("RxChain Data", tabContentWidget);
            rxChainGroupBox->setCheckable(true);
            rxChainGroupBox->setChecked(true);
            rxChainGroupBox->setStyleSheet(groupBoxStyle);
            QFormLayout *rxChainFormLayout = new QFormLayout(rxChainGroupBox);
            rxChainFormLayout->setLabelAlignment(Qt::AlignLeft);
            rxChainFormLayout->setContentsMargins(5, 5, 5, 5);
            rxChainFormLayout->setHorizontalSpacing(10);
            QHBoxLayout *rxChainPowerSampleLayout = new QHBoxLayout();
            rxChainPowerSampleLayout->setContentsMargins(0, 0, 0, 0);
            rxChainPowerSampleLayout->setSpacing(10);

            // Power Value (takes first half of available stretch space)
            QLabel *rxChainPowerVal = new QLabel("---", rxChainGroupBox);
            rxChainPowerVal->setStyleSheet(valueLabelStyle);
            rxChainPowerVal->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
            rxChainPowerSampleLayout->addWidget(rxChainPowerVal, 1);
            unitDisplayLabels[unitIndex]["rxChain_power"] = rxChainPowerVal;

            // Sample Slot Label (takes minimum width)
            QLabel *rxChainSampleSlotLabel = new QLabel("Slot:",
                                                        rxChainGroupBox);
            rxChainSampleSlotLabel->setStyleSheet(componentLabelStyle);
            rxChainPowerSampleLayout->addWidget(rxChainSampleSlotLabel);

            // Sample Slot Value (takes second half of available stretch space)
            QLabel *rxChainSampleSlotVal = new QLabel("---", rxChainGroupBox);
            rxChainSampleSlotVal->setStyleSheet(valueLabelStyle);
            rxChainSampleSlotVal->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
            rxChainPowerSampleLayout->addWidget(rxChainSampleSlotVal, 1);
            unitDisplayLabels[unitIndex]["rxChain_sample_slot"] = rxChainSampleSlotVal;

            addHorizontalValueRow(rxChainFormLayout,
                                  "Power (dBm):",
                                  rxChainPowerSampleLayout);

            tabLayout->addWidget(rxChainGroupBox);
            connect(rxChainGroupBox, &QGroupBox::toggled, rxChainGroupBox, &QGroupBox::setVisible);
        }

        // --- Tx Antenna Data Group Box ---
        if (engineUnits[unitIndex].TxAntenna_enabled)
        {
            QGroupBox *txAntennaGroupBox = new QGroupBox("Tx Antenna Data", tabContentWidget);
            txAntennaGroupBox->setCheckable(true);
            txAntennaGroupBox->setChecked(true);
            txAntennaGroupBox->setStyleSheet(groupBoxStyle);
            QFormLayout *txAntennaFormLayout = new QFormLayout(txAntennaGroupBox);
            txAntennaFormLayout->setLabelAlignment(Qt::AlignLeft);
            txAntennaFormLayout->setContentsMargins(5, 5, 5, 5);
            txAntennaFormLayout->setHorizontalSpacing(10);

            // Gain and Power on the same horizontal line, with equal spacing
            QHBoxLayout *txAntennaGainPowerLayout = new QHBoxLayout();
            txAntennaGainPowerLayout->setContentsMargins(0, 0, 0, 0);
            txAntennaGainPowerLayout->setSpacing(10);

            // Gain Value (takes first half of available stretch space)
            QLabel *txAntennaGainVal = new QLabel("---", txAntennaGroupBox);
            txAntennaGainVal->setStyleSheet(valueLabelStyle);
            txAntennaGainVal->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
            txAntennaGainPowerLayout->addWidget(txAntennaGainVal, 1);
            unitDisplayLabels[unitIndex]["txAntenna_gain"] = txAntennaGainVal;

            // Power Label
            QLabel *txAntennaPowerLabel = new QLabel("Power (dBm):", txAntennaGroupBox);
            txAntennaPowerLabel->setStyleSheet(componentLabelStyle);
            txAntennaGainPowerLayout->addWidget(txAntennaPowerLabel);

            // Power Value (takes second half of available stretch space)
            QLabel *txAntennaPowerVal = new QLabel("---", txAntennaGroupBox);
            txAntennaPowerVal->setStyleSheet(valueLabelStyle);
            txAntennaPowerVal->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
            txAntennaGainPowerLayout->addWidget(txAntennaPowerVal, 1);
            unitDisplayLabels[unitIndex]["txAntenna_power"] = txAntennaPowerVal;

            addHorizontalValueRow(txAntennaFormLayout, "Gain (dBi):", txAntennaGainPowerLayout);

            // Radiation Pattern on its own line
            createAndStoreValueLabel("Rad Pattern:",
                                     "txAntenna_pattern",
                                     txAntennaFormLayout,
                                     txAntennaGroupBox);

            tabLayout->addWidget(txAntennaGroupBox);
            connect(txAntennaGroupBox,
                    &QGroupBox::toggled,
                    txAntennaGroupBox,
                    &QGroupBox::setVisible);
        }

        // --- Rx Antenna Data Group Box ---
        if (engineUnits[unitIndex].RxAntenna_enabled)
        {
            QGroupBox *rxAntennaGroupBox = new QGroupBox("Rx Antenna Data", tabContentWidget);
            rxAntennaGroupBox->setCheckable(true);
            rxAntennaGroupBox->setChecked(true);
            rxAntennaGroupBox->setStyleSheet(groupBoxStyle);
            QFormLayout *rxAntennaFormLayout = new QFormLayout(rxAntennaGroupBox);
            rxAntennaFormLayout->setLabelAlignment(Qt::AlignLeft);
            rxAntennaFormLayout->setContentsMargins(5, 5, 5, 5);
            rxAntennaFormLayout->setHorizontalSpacing(10);

            // Gain and Power on the same horizontal line, with equal spacing
            QHBoxLayout *rxAntennaGainPowerLayout = new QHBoxLayout();
            rxAntennaGainPowerLayout->setContentsMargins(0, 0, 0, 0);
            rxAntennaGainPowerLayout->setSpacing(10);

            QLabel *rxAntennaGainVal = new QLabel("---", rxAntennaGroupBox);
            rxAntennaGainVal->setStyleSheet(valueLabelStyle);
            rxAntennaGainVal->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
            rxAntennaGainPowerLayout->addWidget(rxAntennaGainVal, 1);
            unitDisplayLabels[unitIndex]["rxAntenna_gain"] = rxAntennaGainVal;

            QLabel *rxAntennaPowerLabel = new QLabel("Power (dBm):", rxAntennaGroupBox);
            rxAntennaPowerLabel->setStyleSheet(componentLabelStyle);
            rxAntennaGainPowerLayout->addWidget(rxAntennaPowerLabel);

            QLabel *rxAntennaPowerVal = new QLabel("---", rxAntennaGroupBox);
            rxAntennaPowerVal->setStyleSheet(valueLabelStyle);
            rxAntennaPowerVal->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
            rxAntennaGainPowerLayout->addWidget(rxAntennaPowerVal, 1);
            unitDisplayLabels[unitIndex]["rxAntenna_power"] = rxAntennaPowerVal;

            addHorizontalValueRow(rxAntennaFormLayout, "Gain (dBi):", rxAntennaGainPowerLayout);

            // Radiation Pattern on its own line
            createAndStoreValueLabel("Rad Pattern:",
                                     "rxAntenna_pattern",
                                     rxAntennaFormLayout,
                                     rxAntennaGroupBox);

            tabLayout->addWidget(rxAntennaGroupBox);
            connect(rxAntennaGroupBox,
                    &QGroupBox::toggled,
                    rxAntennaGroupBox,
                    &QGroupBox::setVisible);
        }

        // --- Rotary Data Group Box ---
        if (engineUnits[unitIndex].Rotary_enabled)
        {
            QGroupBox *rotaryGroupBox = new QGroupBox("Rotary Data", tabContentWidget);
            rotaryGroupBox->setCheckable(true);
            rotaryGroupBox->setChecked(true);
            rotaryGroupBox->setStyleSheet(groupBoxStyle);
            QFormLayout *rotaryFormLayout = new QFormLayout(rotaryGroupBox);
            rotaryFormLayout->setLabelAlignment(Qt::AlignLeft);
            rotaryFormLayout->setContentsMargins(5, 5, 5, 5);
            rotaryFormLayout->setHorizontalSpacing(10);

            // Azimuth components (Angle, Velocity, Acceleration) on one line
            QHBoxLayout *rotaryAzLayout = new QHBoxLayout();
            rotaryAzLayout->setContentsMargins(0, 0, 0, 0);
            rotaryAzLayout->setSpacing(5);
            addHorizontalValueRow(rotaryFormLayout, "Azimuth (deg):", rotaryAzLayout);
            createAndStoreValueLabel("Ang:", "rotary_az_angle", rotaryAzLayout, rotaryGroupBox, 1);
            createAndStoreValueLabel("Vel:", "rotary_az_vel", rotaryAzLayout, rotaryGroupBox, 1);
            createAndStoreValueLabel("Acc:", "rotary_az_acc", rotaryAzLayout, rotaryGroupBox, 1);

            // Altitude components (Angle, Velocity, Acceleration) on another line
            QHBoxLayout *rotaryAltLayout = new QHBoxLayout();
            rotaryAltLayout->setContentsMargins(0, 0, 0, 0);
            rotaryAltLayout->setSpacing(5);
            addHorizontalValueRow(rotaryFormLayout, "Altitude (deg):", rotaryAltLayout);
            createAndStoreValueLabel("Ang:", "rotary_alt_angle", rotaryAltLayout, rotaryGroupBox, 1);
            createAndStoreValueLabel("Vel:", "rotary_alt_vel", rotaryAltLayout, rotaryGroupBox, 1);
            createAndStoreValueLabel("Acc:", "rotary_alt_acc", rotaryAltLayout, rotaryGroupBox, 1);

            // "Is Moving" on its own line
            createAndStoreValueLabel("Is Moving:",
                                     "rotary_is_moving",
                                     rotaryFormLayout,
                                     rotaryGroupBox);

            tabLayout->addWidget(rotaryGroupBox);
            connect(rotaryGroupBox, &QGroupBox::toggled, rotaryGroupBox, &QGroupBox::setVisible);
        }

        // --- Algorithm Data Group Box ---
        QString algorithmLabelText = QString::fromStdString(engineUnits[unitIndex].algorithm);
        if (algorithmLabelText.isEmpty() || algorithmLabelText == "none")
        {
            algorithmLabelText = "No Algorithm"; // Default for units without an algorithm
        }
        algorithmLabelText += " Data"; // Append " Data" for consistency

        QGroupBox *algorithmGroupBox = new QGroupBox(algorithmLabelText, tabContentWidget);
        algorithmGroupBox->setCheckable(true);
        algorithmGroupBox->setChecked(true);
        algorithmGroupBox->setStyleSheet(groupBoxStyle);
        QFormLayout *algorithmFormLayout = new QFormLayout(algorithmGroupBox);
        algorithmFormLayout->setLabelAlignment(Qt::AlignLeft);
        algorithmFormLayout->setContentsMargins(5, 5, 5, 5);
        algorithmFormLayout->setHorizontalSpacing(10);

        // Status and Action on the same horizontal line, with equal spacing for their value parts
        QHBoxLayout *algoStatusActionLayout = new QHBoxLayout();
        algoStatusActionLayout->setContentsMargins(0, 0, 0, 0);
        algoStatusActionLayout->setSpacing(10); // Space between status and action parts

        // Status Value
        QLabel *algoStatusVal = new QLabel("---", algorithmGroupBox);
        algoStatusVal->setStyleSheet(valueLabelStyle);
        algoStatusVal->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
        algoStatusActionLayout->addWidget(algoStatusVal, 1);         // Give stretch to value
        unitDisplayLabels[unitIndex]["algo_status"] = algoStatusVal; // Store the value label

        // Action Label
        QLabel *algoActionLabel = new QLabel("Action:", algorithmGroupBox);
        algoActionLabel->setStyleSheet(componentLabelStyle);
        algoStatusActionLayout->addWidget(algoActionLabel); // Add the label for "Action:"

        // Action Value
        QLabel *algoActionVal = new QLabel("---", algorithmGroupBox);
        algoActionVal->setStyleSheet(valueLabelStyle);
        algoActionVal->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
        algoStatusActionLayout->addWidget(algoActionVal, 1);         // Give stretch to value
        unitDisplayLabels[unitIndex]["algo_action"] = algoActionVal; // Store the value label

        // Add this QHBoxLayout as the field for the "Status:" row in the QFormLayout
        addHorizontalValueRow(algorithmFormLayout, "Status:", algoStatusActionLayout);

        tabLayout->addWidget(algorithmGroupBox);
        connect(algorithmGroupBox, &QGroupBox::toggled, algorithmGroupBox, &QGroupBox::setVisible);

        // --- Packet Data Group Box (Sent) ---
        QGroupBox *packetGroupBox = new QGroupBox("Packet Data (Sent)", tabContentWidget);
        packetGroupBox->setCheckable(true);
        packetGroupBox->setChecked(true);
        packetGroupBox->setStyleSheet(groupBoxStyle);
        QVBoxLayout *packetLayout = new QVBoxLayout(packetGroupBox);
        packetLayout->setContentsMargins(5, 5, 5, 5);

        QTextEdit *packetDataTextEdit = new QTextEdit(packetGroupBox);
        packetDataTextEdit->setReadOnly(true);
        packetDataTextEdit->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
        packetDataTextEdit->setStyleSheet(textEditStyle);
        packetDataTextEdit->setMinimumHeight(150);
        packetDataTextEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        packetLayout->addWidget(packetDataTextEdit);

        tabLayout->addWidget(packetGroupBox);
        connect(packetGroupBox, &QGroupBox::toggled, packetGroupBox, &QGroupBox::setVisible);
        packetDataDisplays[unitIndex] = packetDataTextEdit; // Store for sent packets

        // --- Packet Data Group Box (Received) ---
        QGroupBox *receivedPacketGroupBox = new QGroupBox("Packet Data (Received)",
                                                          tabContentWidget);
        receivedPacketGroupBox->setCheckable(true);
        receivedPacketGroupBox->setChecked(true);
        receivedPacketGroupBox->setStyleSheet(groupBoxStyle);
        QVBoxLayout *receivedPacketLayout = new QVBoxLayout(receivedPacketGroupBox);
        receivedPacketLayout->setContentsMargins(5, 5, 5, 5);

        QTextEdit *receivedPacketDataTextEdit = new QTextEdit(receivedPacketGroupBox);
        receivedPacketDataTextEdit->setReadOnly(true);
        receivedPacketDataTextEdit->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
        receivedPacketDataTextEdit->setStyleSheet(textEditStyle);
        receivedPacketDataTextEdit->setMinimumHeight(150);
        receivedPacketDataTextEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        receivedPacketLayout->addWidget(receivedPacketDataTextEdit);

        tabLayout->addWidget(receivedPacketGroupBox);
        connect(receivedPacketGroupBox,
                &QGroupBox::toggled,
                receivedPacketGroupBox,
                &QGroupBox::setVisible);
        receivedPacketDataDisplays[unitIndex] = receivedPacketDataTextEdit;

        tabLayout->addStretch(1); // Pushes all content to the top within the scroll area

        // --- Classifier Data Group Box ---
        QGroupBox *classifierGroupBox = new QGroupBox("Classifier Data", tabContentWidget);
        classifierGroupBox->setCheckable(true);
        classifierGroupBox->setChecked(true);
        classifierGroupBox->setStyleSheet(groupBoxStyle);
        QFormLayout *classifierFormLayout = new QFormLayout(classifierGroupBox);
        classifierFormLayout->setLabelAlignment(Qt::AlignLeft);
        classifierFormLayout->setContentsMargins(5, 5, 5, 5);
        classifierFormLayout->setHorizontalSpacing(10);

        // Class Label and Value on the same line
        createAndStoreValueLabel("Class:",
                                 "classifier_class",
                                 classifierFormLayout,
                                 classifierGroupBox);

        tabLayout->addWidget(classifierGroupBox);
        connect(classifierGroupBox, &QGroupBox::toggled, classifierGroupBox, &QGroupBox::setVisible);
    }
}

void SimulationPage::updateXYView(int slot)
{
    if (!engine || slot < 0 || static_cast<size_t>(slot) >= engine->getNumTimeSlots())
    {
        qWarning() << "[SimulationPage::updateXYView] Invalid slot or engine:" << slot;
        return;
    }

    for (size_t i = 0; i < unitItems.size(); ++i)
    {
        if (i < engine->getNumUnits())
        {
            environmentObject envData;
            rotaryObject rotData;
            try
            {
                envData = engine->getEnvironmentData(i, slot);
                if (engine->getEngineUnit(i).Rotary_enabled)
                {
                    rotData = engine->getRotaryData(i, slot);
                }
            }
            catch (const std::exception &e)
            {
                qWarning() << "[SimulationPage::updateXYView] Error fetching data for unit" << i
                           << " slot" << slot << ":" << e.what();
                if (i < unitItems.size())
                    unitItems[i]->setVisible(false);
                continue;
            }

            // --- Position Handling ---
            Position worldPos = envData.position;

            worldPos.x *= paramConfigFile.pixelsPerMeter;
            worldPos.y *= paramConfigFile.pixelsPerMeter;
            worldPos.z *= paramConfigFile.pixelsPerMeter;

            QPointF scenePos;
            qreal scale = paramConfigFile.pixelsPerMeter;

            // Project the 3D world position to the 2D scene based on the current view
            switch (currentViewPlane)
            {
            case ViewPlane::XY:
                scenePos.setX(worldPos.x * scale);
                scenePos.setY(-worldPos.y * scale); // Invert Y
                break;
            case ViewPlane::XZ:
                scenePos.setX(worldPos.x * scale);
                scenePos.setY(-worldPos.z * scale); // Map Z to Y-axis and invert
                break;
            case ViewPlane::YZ:
                scenePos.setX(worldPos.y * scale);  // Map Y to X-axis
                scenePos.setY(-worldPos.z * scale); // Map Z to Y-axis and invert
                break;
            }

            // --- Orientation Handling ---
            Quaternion baseQ_struct = envData.quaternion;
            Quaternion rotaryQ_struct = quaternionFromAzAltDegreesSimPage(rotData.azimuth.angle,
                                                                          rotData.altitude.angle);
            Quaternion effectiveQ_struct = multiplyQuaternionsSimPage(baseQ_struct, rotaryQ_struct);

            unitItems[i]->setFlag(QGraphicsItem::ItemSendsGeometryChanges, false);

            unitItems[i]->setPos(scenePos);

            // --- Tell the item which plane to use for its arrow projection ---
            unitItems[i]->setViewPlane(currentViewPlane);

            unitItems[i]->setBaseQuaternion(baseQ_struct);
            unitItems[i]->setEffectiveQuaternion(effectiveQ_struct);

            // Ensure arrows are visible
            if (unitItems[i]->getBaseArrow())
                unitItems[i]->getBaseArrow()->setVisible(true);
            if (unitItems[i]->getEffectiveArrow())
                unitItems[i]->getEffectiveArrow()->setVisible(true);

            unitItems[i]->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
            unitItems[i]->setVisible(true); // Ensure unit itself is visible
        }
        else
        {
            // Handle cases where unitItems might exist but no corresponding engine unit or envData
            if (i < unitItems.size())
            {
                unitItems[i]->setVisible(false); // Hide items that don't have data for this step
            }
        }
    }
    updateLabelPositions(); // Update text label positions relative to items
}

void SimulationPage::createUnitItems(const std::vector<engineUnit> &units)
{
    clearScene(); // Clear existing items

    qreal spacing = 1.0;
    qreal totalWidth = (units.size() - 1) * spacing;
    qreal startX = -totalWidth / 2.0;
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
                                Qt::darkYellow}; // Example colors
    int colorIndex = 0;

    for (size_t i = 0; i < units.size(); ++i)
    {
        MovableUnitItem *item = new MovableUnitItem(-unitSizePixels / 2,
                                                    -unitSizePixels / 2,
                                                    unitSizePixels,
                                                    unitSizePixels);
        QPen border(unitColors[colorIndex % unitColors.size()], 2);
        item->setPen(border);
        item->setBrush(QBrush(QColor(0x3a3a3a)));
        colorIndex++;
        item->setUnitIndex(i);

        // --- Configure Arrows for Simulation Display ---
        // Base Arrow (Cyan Dashed): NON-INTERACTIVE, VISIBLE
        DirectionArrowItem *baseArrow = item->getBaseArrow();
        if (baseArrow)
        {
            baseArrow->setInteractive(false); // No user interaction
            baseArrow->setVisible(true);      // Show base orientation
        }

        // Effective Arrow (Yellow Solid): NON-INTERACTIVE, VISIBLE
        DirectionArrowItem *effectiveArrow = item->getEffectiveArrow();
        if (effectiveArrow)
        {
            effectiveArrow->setInteractive(false); // No user interaction
            effectiveArrow->setVisible(true);      // Show effective orientation
        }
        // ---------------------------------------------

        // Set initial position (default) - will be updated by updateXYView
        qreal xPos = startX + i * spacing;
        item->setPos(
            toSceneCoordinates(Position{static_cast<float>(xPos), static_cast<float>(yPos), 0}));

        item->setFlag(QGraphicsItem::ItemIsMovable, false);
        item->setFlag(QGraphicsItem::ItemIsSelectable, false);

        scene->addItem(item);
        unitItems.push_back(item);

        QGraphicsTextItem *label = new QGraphicsTextItem(QString::fromStdString(units[i].label));
        label->setDefaultTextColor(Qt::white);
        label->setPos(0, 0);
        scene->addItem(label);
        unitLabels.push_back(label);
    }
    updateLabelPositions();   // Set initial label positions
    createTimeValueWidgets(); // Create the time value widgets (labels, etc.)
}

void SimulationPage::clearScene()
{
    QList<QGraphicsItem *> itemsCopy = scene->items();
    for (QGraphicsItem *item : itemsCopy)
    {
        if (item->type() == QGraphicsItem::UserType + 1 || item->type() == QGraphicsTextItem::Type)
        {
            scene->removeItem(item);
            item->setParentItem(nullptr);
            delete item;
        }
    }

    unitItems.clear();
    unitLabels.clear();
}

void SimulationPage::updateLabelPositions()
{
    for (size_t i = 0; i < unitItems.size(); ++i)
    {
        if (i < unitLabels.size())
        {
            unitLabels[i]->setPos(unitItems[i]->pos() + QPointF(-0.2 * unitSizePixels, -1 * unitSizePixels));
        }
    }
}

QPointF SimulationPage::toSceneCoordinates(const Position &pos) const
{
    qreal scaleFactor = unitSizePixels / unitSizeWorld;        //  Pixels per world unit
    return QPointF(pos.x * scaleFactor, -pos.y * scaleFactor); // Invert y for Qt coords
}

// Convert scene coordinates (pixels) to world coordinates
Position SimulationPage::fromSceneCoordinates(const QPointF &scenePos) const
{
    qreal scaleFactor = unitSizePixels / unitSizeWorld; // Pixels per world unit
    Position pos;
    pos.x = scenePos.x() / scaleFactor;
    pos.y = -scenePos.y() / scaleFactor;
    pos.z = 0;
    return pos;
}

void SimulationPage::cleanup()
{
    std::cout << "[Cleanup - Sim] Starting cleanup process\n";
    QCoreApplication::processEvents();

    this->setEnabled(false);

    QCoreApplication::processEvents();

    try
    {
        if (sharedPointers->rotaryExp)
        {
            sharedPointers->rotaryExp = nullptr;
            std::cout << "[Cleanup - Sim] RotaryTable Deleted\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Sim] Error cleaning up RotaryTable: " << e.what() << "\n";
    }
    QCoreApplication::processEvents();

    try
    {
        if (sharedPointers->imuExp)
        {
            sharedPointers->imuExp->disconnect();
            sharedPointers->imuExp = nullptr;
            std::cout << "[Cleanup - Sim] IMU Sensor Deleted\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Sim] Error cleaning up IMU Sensor: " << e.what() << "\n";
    }

    QCoreApplication::processEvents();

    try
    {
        if (sharedPointers->powerExp)
        {
            sharedPointers->powerExp = nullptr;
            std::cout << "[Cleanup - Sim] PowerThread Deleted\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Sim] Error cleaning up PowerThread: " << e.what() << "\n";
    }

    QCoreApplication::processEvents();

    try
    {
        if (sharedPointers->powerSim)
        {
            sharedPointers->powerSim = nullptr;
            std::cout << "[Cleanup - Sim] PowerSim Deleted\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Sim] Error cleaning up PowerSim: " << e.what() << "\n";
    }
    try
    {
        if (sharedPointers->posSim)
        {
            sharedPointers->posSim = nullptr;
            std::cout << "[Cleanup - Sim] PositionSim Deleted\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Sim] Error cleaning up positionSim: " << e.what() << "\n";
    }
    QCoreApplication::processEvents();

    try
    {
        if (sharedPointers->imuSim)
        {
            sharedPointers->imuSim = nullptr;
            std::cout << "[Cleanup - Sim] IMUSim Deleted\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Cleanup - Sim] Error cleaning up IMUSim: " << e.what() << "\n";
    }

    QCoreApplication::processEvents();

    if (engine)
    {
        delete engine;
        engine = nullptr;
        std::cout << "[Cleanup - Sim] Engine Deleted\n";
    }

    std::cout << "[Cleanup - Sim] Process Completed Successfully\n";
}

// Define the available properties for plotting
void SimulationPage::setupGraphSelectors()
{
    availableProperties.clear();

    // Time & Slot
    availableProperties << PlotProperty{"Time", "time", "s"};
    availableProperties << PlotProperty{"Slot Index", "slot", ""};

    // Environment
    availableProperties << PlotProperty{"Env Pos X", "env_pos_x", "m"};
    availableProperties << PlotProperty{"Env Pos Y", "env_pos_y", "m"};
    availableProperties << PlotProperty{"Env Pos Z", "env_pos_z", "m"};
    availableProperties << PlotProperty{"Env Vel X", "env_vel_x", "m/s"};
    availableProperties << PlotProperty{"Env Vel Y", "env_vel_y", "m/s"};
    availableProperties << PlotProperty{"Env Vel Z", "env_vel_z", "m/s"};
    availableProperties << PlotProperty{"Env Acc X", "env_acc_x", "m/s²"};
    availableProperties << PlotProperty{"Env Acc Y", "env_acc_y", "m/s²"};
    availableProperties << PlotProperty{"Env Acc Z", "env_acc_z", "m/s²"};
    availableProperties << PlotProperty{"Env Ang Vel X",
                                        "env_ang_vel_x",
                                        "deg/s"};
    availableProperties << PlotProperty{"Env Ang Vel Y", "env_ang_vel_y", "deg/s"};
    availableProperties << PlotProperty{"Env Ang Vel Z", "env_ang_vel_z", "deg/s"};
    availableProperties << PlotProperty{"Env Ang Acc X", "env_ang_acc_x", "deg/s²"};
    availableProperties << PlotProperty{"Env Ang Acc Y", "env_ang_acc_y", "deg/s²"};
    availableProperties << PlotProperty{"Env Ang Acc Z", "env_ang_acc_z", "deg/s²"};

    // IMU
    availableProperties << PlotProperty{"IMU Acc X", "imu_acc_x", "m/s²"};
    availableProperties << PlotProperty{"IMU Acc Y", "imu_acc_y", "m/s²"};
    availableProperties << PlotProperty{"IMU Acc Z", "imu_acc_z", "m/s²"};
    availableProperties << PlotProperty{"IMU Ang Vel X", "imu_ang_vel_x", "deg/s"};
    availableProperties << PlotProperty{"IMU Ang Vel Y", "imu_ang_vel_y", "deg/s"};
    availableProperties << PlotProperty{"IMU Ang Vel Z", "imu_ang_vel_z", "deg/s"};

    // RxChain
    availableProperties << PlotProperty{"RxChain Power", "rxChain_power", "dBm"};

    // RxAntenna
    availableProperties << PlotProperty{"Rx Gain", "rx_gain", "dBi"};
    availableProperties << PlotProperty{"Rx Power", "rx_power", "dBm"};

    // TxAntenna
    availableProperties << PlotProperty{"Tx Gain", "tx_gain", "dBi"};
    availableProperties << PlotProperty{"Tx Power", "tx_power", "dBm"};

    // Algorithm
    availableProperties << PlotProperty{"Algo Status ID", "algo_status", ""};
    availableProperties << PlotProperty{"Algo Action ID", "algo_action", ""};
}

// Populate the comboboxes based on available units and properties
void SimulationPage::populateSelectors()
{
    if (!engine || !unitSelectorCombo || !xAxisSelectorCombo || !yAxisSelectorCombo)
    {
        qWarning() << "Cannot populate selectors: engine or comboboxes are null.";
        return;
    }

    // Block signals to prevent unnecessary updates
    unitSelectorCombo->blockSignals(true);
    xAxisSelectorCombo->blockSignals(true);
    yAxisSelectorCombo->blockSignals(true);

    unitSelectorCombo->clear();
    xAxisSelectorCombo->clear();
    yAxisSelectorCombo->clear();

    // Populate Unit Selector
    const auto &units = engine->getAllEngineUnits();
    for (int i = 0; i < units.size(); ++i)
    {
        unitSelectorCombo
            ->addItem(QString("Unit %1 (%2)").arg(i).arg(QString::fromStdString(units[i].label)), i);
    }

    // Populate Axis Selectors
    for (const auto &prop : availableProperties)
    {
        // Check if property is valid for *any* unit first (simplification)
        // More advanced: check if valid for the *currently selected* unit
        bool propertyRelevant = true; // Assume relevant for now

        // Basic check: if it's an IMU property, check if *any* unit has IMU
        if (prop.internalId.startsWith("imu_"))
        {
            bool anyImu = false;
            for (const auto &unit : units)
                if (unit.IMU_enabled)
                {
                    anyImu = true;
                    break;
                }
            if (!anyImu)
                propertyRelevant = false;
        }
        // Similar checks for RxChain, Rx, Tx...
        if (prop.internalId.startsWith("rxChain_"))
        {
            bool anyRxChain = false;
            for (const auto &unit : units)
                if (unit.RxChain_enabled)
                {
                    anyRxChain = true;
                    break;
                }
            if (!anyRxChain)
                propertyRelevant = false;
        }
        if (prop.internalId.startsWith("rx_"))
        {
            bool anyRx = false;
            for (const auto &unit : units)
                if (unit.RxAntenna_enabled)
                {
                    anyRx = true;
                    break;
                }
            if (!anyRx)
                propertyRelevant = false;
        }
        if (prop.internalId.startsWith("tx_"))
        {
            bool anyTx = false;
            for (const auto &unit : units)
                if (unit.TxAntenna_enabled)
                {
                    anyTx = true;
                    break;
                }
            if (!anyTx)
                propertyRelevant = false;
        }

        if (propertyRelevant)
        {
            xAxisSelectorCombo->addItem(prop.displayName, prop.internalId);
            yAxisSelectorCombo->addItem(prop.displayName, prop.internalId);
        }
    }

    // Set default selections (e.g., First Rx Unit, Rx Power vs Slot)
    int defaultUnitIdx = -1;
    QString defaultYProp = "slot"; // Fallback
    QString defaultXProp = "slot"; // Fallback

    for (int i = 0; i < units.size(); ++i)
    {
        if (units[i].RxAntenna_enabled)
        {
            defaultUnitIdx = i;
            defaultYProp = "rx_power";
            defaultXProp = "slot";
            break;
        }
    }
    if (defaultUnitIdx == -1)
    { // If no Rx, try RxChain
        for (int i = 0; i < units.size(); ++i)
        {
            if (units[i].RxChain_enabled)
            {
                defaultUnitIdx = i;
                defaultYProp = "rxChain_power";
                defaultXProp = "slot";
                break;
            }
        }
    }
    if (defaultUnitIdx == -1 && !units.empty())
    { // Fallback to first unit, Env Pos X vs Slot
        defaultUnitIdx = 0;
        defaultYProp = "env_pos_x";
        defaultXProp = "slot";
    }

    if (defaultUnitIdx != -1)
        unitSelectorCombo->setCurrentIndex(defaultUnitIdx);

    int xIdx = xAxisSelectorCombo->findData(defaultXProp);
    if (xIdx != -1)
        xAxisSelectorCombo->setCurrentIndex(xIdx);
    else
        xAxisSelectorCombo->setCurrentIndex(0);

    int yIdx = yAxisSelectorCombo->findData(defaultYProp);
    if (yIdx != -1)
        yAxisSelectorCombo->setCurrentIndex(yIdx);
    else
        yAxisSelectorCombo->setCurrentIndex(0);

    // Unblock signals to allow for further updates
    unitSelectorCombo->blockSignals(false);
    xAxisSelectorCombo->blockSignals(false);
    yAxisSelectorCombo->blockSignals(false);

    updateGraphSelections();

    unitSelectorCombo->setEnabled(true);
    xAxisSelectorCombo->setEnabled(true);
    yAxisSelectorCombo->setEnabled(true);
}

// Slot triggered when any selector changes
void SimulationPage::updateGraphSelections()
{
    // Just call the main update function
    updateGraphPlot();
}

// Retrieve the value for a specific property, unit, and slot
qreal SimulationPage::getPropertyValue(int unitIndex, int slotIndex, const QString &propertyId) const
{
    // Use size_t for engine comparisons for consistency and safety
    size_t uIdx = static_cast<size_t>(unitIndex);
    size_t sIdx = static_cast<size_t>(slotIndex);

    // Check engine pointer and indices
    if (!engine || unitIndex < 0 || uIdx >= engine->getNumUnits() || slotIndex < 0 || sIdx >= engine->getNumTimeSlots())
    {
        return 0.0; // Return default/invalid value
    }

    // --- Time/Slot ---
    if (propertyId == "time")
    {
        return static_cast<qreal>(sIdx) * paramConfigFile.engine_slot_time_microsec / 1e6;
    }
    if (propertyId == "slot")
    {
        return static_cast<qreal>(sIdx);
    }

    // Use try-catch blocks for engine calls as they might throw std::out_of_range
    try
    {
        // --- Environment ---
        if (propertyId.startsWith("env_"))
        {
            environmentObject data = engine->getEnvironmentData(uIdx, sIdx); // Fetch data on demand
            if (propertyId == "env_pos_x")
                return data.position.x;
            if (propertyId == "env_pos_y")
                return data.position.y;
            if (propertyId == "env_pos_z")
                return data.position.z;
            if (propertyId == "env_vel_x")
                return data.velocity.x;
            if (propertyId == "env_vel_y")
                return data.velocity.y;
            if (propertyId == "env_vel_z")
                return data.velocity.z;
            if (propertyId == "env_acc_x")
                return data.acceleration.x;
            if (propertyId == "env_acc_y")
                return data.acceleration.y;
            if (propertyId == "env_acc_z")
                return data.acceleration.z;
            if (propertyId == "env_ang_vel_x")
                return data.angular_velocity.x;
            if (propertyId == "env_ang_vel_y")
                return data.angular_velocity.y;
            if (propertyId == "env_ang_vel_z")
                return data.angular_velocity.z;
            if (propertyId == "env_ang_acc_x")
                return data.angular_acceleration.x;
            if (propertyId == "env_ang_acc_y")
                return data.angular_acceleration.y;
            if (propertyId == "env_ang_acc_z")
                return data.angular_acceleration.z;
        }

        // --- IMU ---
        if (propertyId.startsWith("imu_"))
        {
            // Check if IMU is actually enabled for this unit in the config
            if (!engine->getEngineUnit(uIdx).IMU_enabled)
                return 0.0;
            imuObject data = engine->getIMUData(uIdx, sIdx);
            if (propertyId == "imu_acc_x")
                return data.acceleration.x;
            if (propertyId == "imu_acc_y")
                return data.acceleration.y;
            if (propertyId == "imu_acc_z")
                return data.acceleration.z;
            if (propertyId == "imu_ang_vel_x")
                return data.angular_velocity.x;
            if (propertyId == "imu_ang_vel_y")
                return data.angular_velocity.y;
            if (propertyId == "imu_ang_vel_z")
                return data.angular_velocity.z;
        }

        // --- RxChain ---
        if (propertyId == "rxChain_power")
        {
            if (!engine->getEngineUnit(uIdx).RxChain_enabled)
                return -120.0;
            rxChainObject data = engine->getRxChainData(uIdx, sIdx);
            return wattsToDbmSimPage(data.power_watts);
        }

        // --- RxAntenna ---
        if (propertyId.startsWith("rx_"))
        {
            if (!engine->getEngineUnit(uIdx).RxAntenna_enabled)
                return (propertyId == "rx_power" ? -120.0 : 0.0);
            antennaObject data = engine->getRxAntennaData(uIdx, sIdx);
            if (propertyId == "rx_gain")
                return linearToDbi(data.gain_linear);
            if (propertyId == "rx_power")
                return wattsToDbmSimPage(data.power_watts);
        }

        // --- TxAntenna ---
        if (propertyId.startsWith("tx_"))
        {
            if (!engine->getEngineUnit(uIdx).TxAntenna_enabled)
                return (propertyId == "tx_power" ? -120.0 : 0.0);
            antennaObject data = engine->getTxAntennaData(uIdx, sIdx);
            if (propertyId == "tx_gain")
                return linearToDbi(data.gain_linear);
            if (propertyId == "tx_power")
                return wattsToDbmSimPage(data.power_watts);
        }

        // --- Algorithm ---
        if (propertyId.startsWith("algo_"))
        {
            // Algorithms always exist conceptually
            algorithmObject data = engine->getAlgorithmData(uIdx, sIdx);
            if (propertyId == "algo_status")
                return static_cast<qreal>(data.status);
            if (propertyId == "algo_action")
                return static_cast<qreal>(data.action);
        }
    }
    catch (const std::out_of_range &oor)
    {
        // Engine throws out_of_range for invalid indices (should be caught earlier, but safety net)
        qWarning() << "Out of range error in getPropertyValue for unit" << unitIndex << "slot"
                   << slotIndex << ":" << oor.what();
        return 0.0; // Or NaN
    }
    catch (const std::exception &e)
    {
        // Catch other potential errors during data fetching
        qWarning() << "Error in getPropertyValue for unit" << unitIndex << "slot" << slotIndex
                   << ":" << e.what();
        return 0.0; // Or NaN
    }

    return 0.0;
}

// Update the plot based on current selections
void SimulationPage::updateGraphPlot()
{
    if (!engine || !dataChart || !axisX || !axisY || !dataSeries || !unitSelectorCombo || !xAxisSelectorCombo || !yAxisSelectorCombo)
    {
        qWarning() << "Cannot update graph plot: Missing engine or UI elements.";
        return;
    }

    int unitIndex = unitSelectorCombo->currentData().toInt();
    QString yPropertyId = yAxisSelectorCombo->currentData().toString();
    QString xPropertyId = xAxisSelectorCombo->currentData().toString();

    // Add check for valid unit index
    if (unitIndex < 0 || static_cast<size_t>(unitIndex) >= engine->getNumUnits() || yPropertyId.isEmpty() || xPropertyId.isEmpty())
    {
        qWarning() << "Cannot update graph plot: Invalid unit index or property selection.";

        dataSeries->clear(); // Clear plot if selection invalid
        axisX->setRange(0, 0);
        axisY->setRange(0, 1);
        graphTitleLabel->setText("Graph View - Invalid Selection");
        currentTimeIndicatorSeries->clear(); // Clear scrubber line too
        currentYAxisMin = 0;                 // Reset Y range for scrubber
        currentYAxisMax = 1;
        return;
    }

    PlotProperty xProp = getPropertyById(xPropertyId);
    PlotProperty yProp = getPropertyById(yPropertyId);

    // --- Generate Data Points (with Aggregation) ---
    QVector<QPointF> newDataPoints;
    int numSlotsTotal = static_cast<int>(engine->getNumTimeSlots());

    if (numSlotsTotal <= 0)
    {
        dataSeries->clear(); // Clear if no data
        axisX->setRange(0, 0);
        axisY->setRange(0, 1);                            // Default range
        graphTitleLabel->setText("Graph View - No Data"); // Update title
        currentTimeIndicatorSeries->clear();
        currentYAxisMin = 0;
        currentYAxisMax = 1;
        return;
    }

    newDataPoints.reserve(std::min(numSlotsTotal, GRAPH_TARGET_POINTS * 2 + 2));

    qreal globalMinY = std::numeric_limits<qreal>::max();
    qreal globalMaxY = std::numeric_limits<qreal>::lowest();
    qreal globalMinX = std::numeric_limits<qreal>::max();
    qreal globalMaxX = std::numeric_limits<qreal>::lowest();

    // Decide whether to aggregate based on the number of slots vs target points
    if (numSlotsTotal <= GRAPH_TARGET_POINTS)
    {
        // --- No Aggregation Needed (Few Points) ---
        for (int slot = 0; slot < numSlotsTotal; ++slot)
        {
            qreal xVal = getPropertyValue(unitIndex, slot, xPropertyId);
            qreal yVal = getPropertyValue(unitIndex, slot, yPropertyId);
            newDataPoints.append(QPointF(xVal, yVal));

            // Track overall min/max for axis scaling
            if (xVal < globalMinX)
                globalMinX = xVal;
            if (xVal > globalMaxX)
                globalMaxX = xVal;
            if (yVal < globalMinY)
                globalMinY = yVal;
            if (yVal > globalMaxY)
                globalMaxY = yVal;
        }
    }
    else
    {
        // --- Apply Min/Max Aggregation ---
        int pointsPerBin = static_cast<int>(
            std::ceil(static_cast<double>(numSlotsTotal) / GRAPH_TARGET_POINTS));
        if (pointsPerBin < 1)
            pointsPerBin = 1; // Should not happen with ceil, but safeguard

        // Make sure to include the very first point
        qreal firstX = getPropertyValue(unitIndex, 0, xPropertyId);
        qreal firstY = getPropertyValue(unitIndex, 0, yPropertyId);
        newDataPoints.append(QPointF(firstX, firstY));
        globalMinX = firstX;
        globalMaxX = firstX;
        globalMinY = firstY;
        globalMaxY = firstY;

        for (int binStartSlot = 0; binStartSlot < numSlotsTotal; binStartSlot += pointsPerBin)
        {
            int binEndSlot = std::min(binStartSlot + pointsPerBin - 1, numSlotsTotal - 1);

            if (binStartSlot > binEndSlot)
                break; // Should not happen

            // Find min/max within the current bin
            qreal minYInBin = std::numeric_limits<qreal>::max();
            qreal maxYInBin = std::numeric_limits<qreal>::lowest();
            qreal xAtMinY = getPropertyValue(unitIndex,
                                             binStartSlot,
                                             xPropertyId); // Initialize with first point's X
            qreal xAtMaxY = xAtMinY;
            bool firstPointInBin = true;

            for (int slot = binStartSlot; slot <= binEndSlot; ++slot)
            {
                qreal xVal = getPropertyValue(unitIndex, slot, xPropertyId);
                qreal yVal = getPropertyValue(unitIndex, slot, yPropertyId);

                // Update global min/max
                if (xVal < globalMinX)
                    globalMinX = xVal;
                if (xVal > globalMaxX)
                    globalMaxX = xVal;
                if (yVal < globalMinY)
                    globalMinY = yVal;
                if (yVal > globalMaxY)
                    globalMaxY = yVal;

                // Update bin min/max
                if (firstPointInBin)
                {
                    minYInBin = yVal;
                    maxYInBin = yVal;
                    xAtMinY = xVal;
                    xAtMaxY = xVal;
                    firstPointInBin = false;
                }
                else
                {
                    if (yVal < minYInBin)
                    {
                        minYInBin = yVal;
                        xAtMinY = xVal;
                    }
                    if (yVal > maxYInBin)
                    {
                        maxYInBin = yVal;
                        xAtMaxY = xVal;
                    }
                }
            }

            // Add the min and max points for the bin to the plot data
            // Ensure points are added in correct X order to avoid line crossing back
            QPointF minP(xAtMinY, minYInBin);
            QPointF maxP(xAtMaxY, maxYInBin);

            if (minP == maxP)
            { // If min and max are the same point in the bin
                // Add only if different from the last point added
                if (newDataPoints.isEmpty() || newDataPoints.last() != minP)
                {
                    newDataPoints.append(minP);
                }
            }
            else
            {
                // Add points in X order
                if (minP.x() <= maxP.x())
                {
                    if (newDataPoints.isEmpty() || newDataPoints.last() != minP)
                        newDataPoints.append(minP);
                    // Check again in case minP was the same as last point
                    if (newDataPoints.isEmpty() || newDataPoints.last() != maxP)
                        newDataPoints.append(maxP);
                }
                else
                {
                    if (newDataPoints.isEmpty() || newDataPoints.last() != maxP)
                        newDataPoints.append(maxP);
                    // Check again in case maxP was the same as last point
                    if (newDataPoints.isEmpty() || newDataPoints.last() != minP)
                        newDataPoints.append(minP);
                }
            }
        } // End of loop through bins

        // Optional: Add the very last point to ensure the graph extends fully
        qreal lastX = getPropertyValue(unitIndex, numSlotsTotal - 1, xPropertyId);
        qreal lastY = getPropertyValue(unitIndex, numSlotsTotal - 1, yPropertyId);
        QPointF lastP(lastX, lastY);
        if (newDataPoints.isEmpty() || newDataPoints.last() != lastP)
        {
            newDataPoints.append(lastP);
            // Ensure last point is included in global bounds calculation
            if (lastX < globalMinX)
                globalMinX = lastX;
            if (lastX > globalMaxX)
                globalMaxX = lastX;
            if (lastY < globalMinY)
                globalMinY = lastY;
            if (lastY > globalMaxY)
                globalMaxY = lastY;
        }

    } // End of aggregation logic

    // --- Update Data Series ---
    dataSeries->replace(newDataPoints); // Replace with potentially aggregated data

    // --- Calculate and Set Axis Ranges (using global min/max found during iteration) ---
    qreal xPadding = (globalMaxX - globalMinX) * 0.05;
    qreal yPadding = (globalMaxY - globalMinY) * 0.05;

    // Handle cases with zero range or single point
    if (qAbs(globalMaxX - globalMinX) < 1e-9)
    {                                                 // Effectively zero range
        xPadding = qMax(1.0, qAbs(globalMinX * 0.1)); // Use 1.0 or 10% padding
        globalMinX -= xPadding / 2.0;                 // Center the single point
        globalMaxX += xPadding / 2.0;
    }
    else
    {
        // No padding on the left for time/slot usually
        if (xPropertyId != "time" && xPropertyId != "slot")
        {
            globalMinX -= xPadding;
        }
        globalMaxX += xPadding;
    }

    if (qAbs(globalMaxY - globalMinY) < 1e-9)
    {
        // Use common defaults for certain units if range is zero
        if (yProp.unitLabel == "dBm")
        {
            globalMinY = -100;
            globalMaxY = 0;
        }
        else if (yProp.unitLabel == "dBi")
        {
            globalMinY = -10;
            globalMaxY = 20;
        }
        else
        {
            yPadding = qMax(1.0, qAbs(globalMinY * 0.1)); // Use 1.0 or 10%
            globalMinY -= yPadding / 2.0;                 // Center
            globalMaxY += yPadding / 2.0;
        }
    }
    else
    {
        globalMinY -= yPadding;
        globalMaxY += yPadding;
    }

    if (yProp.unitLabel == "dBm")
    {
        qreal thermalNoiseWatts = engine->receiverThermalNoise_watts;
        qreal thermalNoiseDbm = wattsToDbmSimPage(thermalNoiseWatts);

        // Set minimum to thermal noise - 3dB
        globalMinY = thermalNoiseDbm - 3.0;
    }

    axisX->setRange(globalMinX, globalMaxX);
    axisY->setRange(globalMinY, globalMaxY);

    // --- Store Y Range for Scrubber ---
    currentYAxisMin = globalMinY; // Use the calculated padded range
    currentYAxisMax = globalMaxY;

    // --- Update Axis Titles ---
    axisX->setTitleText(QString("%1 (%2)").arg(xProp.displayName).arg(xProp.unitLabel));
    axisY->setTitleText(QString("%1 (%2)").arg(yProp.displayName).arg(yProp.unitLabel));

    // --- Update Chart Title ---
    QString unitText = unitSelectorCombo->currentText();
    graphTitleLabel->setText(
        QString("%1 vs %2 for %3").arg(yProp.displayName).arg(xProp.displayName).arg(unitText));

    // --- Update Scrubber Position ---
    // Call updateScrubberLine with the current slider value to redraw it correctly using the new axis ranges
    updateScrubberLine(timeSlider->value());
}
