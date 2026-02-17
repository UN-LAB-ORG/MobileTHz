
#ifndef POSITIONCONFIGPAGE_H
#define POSITIONCONFIGPAGE_H

#include <QComboBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QFrame>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGroupBox>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollBar>
#include <QSlider>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWidget>
#include "Software/jsonReader/jsonReader.hpp"
#include "Software/structDefinition.h"
#include "UserInterface/Utils/movableUnitItem.h"
#include <optional>
#include <string>

class PositionConfigPage : public QWidget
{
    Q_OBJECT

public:
    PositionConfigPage(QWidget *parent = nullptr);
    void setParamConfig(const ConfigFile &config);

    /**
     * @brief Loads and validates a position configuration JSON file.
     * @param filePath Absolute path to the position config JSON file.
     * @param paramConfig The main application configuration (needed for validation, e.g., time steps).
     * @return std::optional<nlohmann::json> containing the parsed and validated position config
     *         if successful, std::nullopt otherwise. Errors are logged via qWarning.
     */
    static std::optional<nlohmann::json> processPositionConfiguration(const std::string &filePath,
                                                                      ConfigFile &paramConfig);

    static void applyPositionTimeToConfig(const nlohmann::json &positionJson,
                                          ConfigFile &paramConfig);

signals:
    void nextRequested(nlohmann::json positionConfig);
    void backRequested();

private slots:
    void loadConfig();
    void savePoints();
    void updatePositions(int slot);
    void saveConfig();
    void updatePropertiesDisplay();
    void updateInterpolationMode(int index);
    void goToNextActionSlot();
    void goToPreviousActionSlot();

private:
    // --- UI ---
    QVBoxLayout *mainLayout;
    QHBoxLayout *topLayout;
    QHBoxLayout *headerLayout;
    QVBoxLayout *configSelectionLayout;
    QHBoxLayout *creationLayout;
    QHBoxLayout *buttonLayout;
    QVBoxLayout *rightColumnLayout;
    QFrame *propertiesFrame;

    // Widgets
    QComboBox *configFileCombo;
    QPushButton *loadButton;
    QPushButton *nextButton;
    QPushButton *backButton;
    QPushButton *savePointsButton;
    QPushButton *saveConfigButton;
    QPushButton *loadConfigButton;

    QGraphicsScene *scene;
    QGraphicsView *view;
    QSlider *timeSlider;
    QLabel *timeLabel;

    // Properties
    QGroupBox *unitGroupBox;
    QLabel *unitNameLabel;
    QLabel *unitIDLabel;

    QGroupBox *positionGroupBox;
    QLabel *posXLabel;
    QLabel *posYLabel;
    QLabel *posZLabel;

    QGroupBox *orientationGroupBox;
    QLabel *quatWLabel;
    QLabel *quatXLabel;
    QLabel *quatYLabel;
    QLabel *quatZLabel;

    QGroupBox *interpolationGroupBox;
    QLabel *interpolationModeLabel;
    QComboBox *interpolationModeComboBox;

    // --- Data ---
    std::vector<MovableUnitItem *> unitItems;
    std::vector<QGraphicsTextItem *> unitLabels;
    ConfigFile paramConfigFile;
    QString paramConfigName;
    QString positionConfigName;
    std::vector<std::vector<std::pair<Position, Quaternion>>> positions;
    std::vector<std::vector<InterpolationMode>> interpolationModes;
    std::vector<bool> endSimulationFlags;
    std::string loadedConfigPath;

    // --- UI and Helpers ---
    void setupUI();
    void refreshConfigFiles();
    void createUnitItems(const std::vector<engineUnit> &unit_labels);
    void clearScene();
    void createDefaultScenario();
    void updateLabelPositions();
    void updateScene();
    QPointF toSceneCoordinates(const Position &pos) const;
    Position fromSceneCoordinates(const QPointF &scenePos) const;

    // -- Scene to Position Config --
    nlohmann::json getPositionConfigFromScene();

    // View and Unit Parameters
    qreal viewRange;      // How much of the world is visible
    qreal unitSizeWorld;  // How big is a unit in the world
    qreal unitSizePixels; // How big is a unit in pixels

    void keyPressEvent(QKeyEvent *event) override;

    // Mouse Interaction
    QPoint lastPanPoint;
    bool panning;
    qreal zoomFactor;

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;
};

#endif // POSITIONCONFIGPAGE_H
