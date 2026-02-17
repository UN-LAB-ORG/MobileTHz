#ifndef ALGORITHMSELECTIONPAGE_H
#define ALGORITHMSELECTIONPAGE_H

#include <QComboBox>
#include <QDrag>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMimeData>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <QMap>
#include <map>
#include <string>
#include "Software/structDefinition.h"

// This class handles the custom drag behavior for the QListWidget
class DraggableListWidget : public QListWidget
{
    Q_OBJECT

public:
    explicit DraggableListWidget(QWidget *parent = nullptr);

protected:
    // Override this method to customize the drag operation
    void startDrag(Qt::DropActions supportedActions) override;
};

// This custom QFrame acts as a drop target for algorithms
class UnitDropTargetFrame : public QFrame
{
Q_OBJECT // Q_OBJECT must be at the top level

    public : explicit UnitDropTargetFrame(int unitIdx, const QString &unitName, QWidget *parent = nullptr);
    void setAssignedAlgorithm(const QString &algoName);
    int getUnitIndex() const { return unitIndex_; }

signals:
    void algorithmDropped(int unitIndex, const QString &algorithmInternalName);

protected:
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dragMoveEvent(QDragMoveEvent *event) override;
    void dropEvent(QDropEvent *event) override;
    void dragLeaveEvent(QDragLeaveEvent *event) override;

private:
    int unitIndex_;
    QLabel *unitNameLabel_;
    QLabel *assignedAlgoLabel_;
};

class AlgorithmSelectionPage : public QWidget
{
    Q_OBJECT

protected:
    void keyPressEvent(QKeyEvent *event) override;

public:
    explicit AlgorithmSelectionPage(QWidget *parent = nullptr);
    void setParamConfigFile(const ConfigFile &configFile);

    /**
     * @brief Validates a preset algorithm name and applies it to all units in a ConfigFile.
     * @param presetAlgorithmName The internal algorithm name (e.g., "HierarchicalSearch", "NONE"). Case-sensitive matching against map values.
     * @param currentConfig The configuration file to modify.
     * @return std::optional<ConfigFile> containing the updated config if the name is valid
     *         and applied, std::nullopt otherwise. Errors are logged via qWarning.
     */
    static std::optional<ConfigFile> processAlgorithmPreset(const std::string &presetAlgorithmName,
                                                            const ConfigFile &currentConfig);

signals:
    void algorithmsAssigned(const ConfigFile &updatedConfig);
    void backRequested();

private slots:
    void nextClicked();
    void goBack();
    void updateAssignmentDisplay();
    // Slot to handle an algorithm being dropped onto a unit
    void handleAlgorithmDrop(int unitIndex, const QString &algorithmInternalName);

private:
    void setupUI();
    void createUnitDropTargets();          // Function to create the visual unit boxes
    void updateUnitDisplay(int unitIndex); // Update a single unit's box

    // --- UI Elements ---
    QVBoxLayout *mainLayout;
    QHBoxLayout *headerLayout;

    // Left Column for Algorithm List (Source for Drag)
    QFrame *availableAlgorithmsFrame;
    DraggableListWidget *availableAlgorithms;

    // Right Column for Unit Drop Targets
    QFrame *unitsDisplayFrame;
    QGridLayout *unitsGridLayout;

    // QMap to store pointers to the dynamic UI elements for each unit
    struct UnitDisplayWidgets
    {
        UnitDropTargetFrame *unitBox = nullptr;
    };
    QMap<int, UnitDisplayWidgets> unitWidgets; // Maps unit index to its display widgets

    QPushButton *nextButton;
    QPushButton *backButton;

    // --- Data ---
    ConfigFile paramConfigFile; // Stores the config being modified
    QString configName;         // Base config name for display

public:
    static const std::map<QString, std::string> algorithmMap; // Declaration
private:
    // Helper function to get valid internal algorithm names (private static)
    static std::vector<std::string> getValidInternalAlgorithmNames();
};

#endif // ALGORITHMSELECTIONPAGE_H
