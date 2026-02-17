// SimulationPage.h
#ifndef SIMULATIONPAGE_H
#define SIMULATIONPAGE_H

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGroupBox> //For group box
#include <QHBoxLayout>
#include <QLabel>
#include <QMap> // Add this for efficient widget storage
#include <QSlider>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QVector>
#include <QWidget>
#include <QtCharts/QLineSeries>
#include <qchart.h>
#include <qcombobox.h>
#include <qvalueaxis.h>

#include "Software/mobileTHzEngine/mobileTHzEngine.h"
#include "Software/structDefinition.h"
#include "UserInterface/Utils/movableUnitItem.h"

// Forward declare
class CsvExportThread;
class rotarylib;
class powerExp;
class TcpClass;
class imuExp;
class powerSim;
class positionSim;
class imuSim;
struct SharedPointers;
struct PlotProperty
{
    QString displayName; // e.g., "Environment Position X"
    QString internalId;  // e.g., "env_pos_x" (used for retrieval)
    QString unitLabel;   // e.g., "m", "dBm", "deg/s"
};

class SimulationPage : public QWidget
{
    Q_OBJECT
public:
    SimulationPage(QWidget *parent = nullptr);
    ~SimulationPage();
    void cleanup();

    void setupPage(const ConfigFile &config);
    void setSharedPointers(std::shared_ptr<SharedPointers> pointers);

signals:
    void simulationComplete();

private slots:
    void updateDisplay(int slot);
    void updateGraphSelections();
    void updateScrubberLine(int slot);
    void onViewPlaneChanged(int index);

private:
    const int GRAPH_TARGET_POINTS = 100000;

    // UI elements
    QTabWidget *unitTabWidget;
    QSlider *timeSlider;
    QVBoxLayout *mainLayout;
    QGraphicsScene *scene;
    QGraphicsView *view;
    std::vector<MovableUnitItem *> unitItems;
    std::vector<QGraphicsTextItem *> unitLabels;
    QLabel *timeLabel;

    // --- Graph Selectors ---
    QComboBox *unitSelectorCombo;
    QComboBox *yAxisSelectorCombo;
    QComboBox *xAxisSelectorCombo;
    QComboBox *viewSelectorCombo;
    QLabel *graphTitleLabel;

    // --- Member to store current view plane ---
    ViewPlane currentViewPlane = ViewPlane::XY;

    // --- Store Pointers to INDIVIDUAL field Labels ---
    QMap<size_t, QMap<QString, QLabel *>> unitDisplayLabels;
    QMap<size_t, QTextEdit *> packetDataDisplays;
    QMap<size_t, QTextEdit *> receivedPacketDataDisplays;

    // --- Store pointers to group boxes ---
    QMap<int, QGroupBox *> unitGroupBoxes;

    ConfigFile paramConfigFile;
    QStringList algorithmList;

    std::shared_ptr<SharedPointers> sharedPointers;
    mobileTHzEngine *engine;

    // --- Charting ---
    QLineSeries *dataSeries;
    QLineSeries *currentTimeIndicatorSeries;
    QChart *dataChart;
    QValueAxis *axisX;
    QValueAxis *axisY;
    qreal currentYAxisMin;
    qreal currentYAxisMax;

    // --- Helper Data ---
    QList<PlotProperty> availableProperties;

    // --- Private Methods ---
    void setupUI();
    void createUnitItems(const std::vector<engineUnit> &unit_labels);
    void clearScene();
    void updateLabelPositions();
    QPointF toSceneCoordinates(const Position &pos) const;
    Position fromSceneCoordinates(const QPointF &scenePos) const;
    void updateXYView(int slot);
    void updateTimeValuesDisplay(int slot);

    // --- New Graphing Methods ---
    void setupGraphSelectors();
    void populateSelectors();
    void updateGraphPlot();
    qreal getPropertyValue(int unitIndex, int slotIndex, const QString &propertyId) const;
    PlotProperty getPropertyById(const QString &propertyId) const;

    // --- Formatting Methods ---
    QString formatVec3(const Position &p, int width = 6, int prec = 2, char f = 'f') const;
    QString formatQuat(const Quaternion &q, int width = 6, int prec = 2, char f = 'f') const;
    QString formatRotaryAxis(const rotaryAxis &axis,
                             int width = 6,
                             int prec = 2,
                             char f = 'f') const;
    QString formatPacketData(const packetObject &packet) const;

    void createTimeValueWidgets();

    qreal viewRange;
    qreal unitSizeWorld;
    qreal unitSizePixels;
};

#endif // SIMULATIONPAGE_H
