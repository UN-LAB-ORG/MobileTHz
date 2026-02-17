#ifndef REALTIMEPAGE_H
#define REALTIMEPAGE_H

#include <QFrame>
#include <QLabel>
#include <QListWidget>
#include <QTextEdit>
#include <QTimer>
#include <QWidget>
#include <memory>
#include <qpushbutton.h>

#include "Software/mobileTHzEngine/mobileTHzEngine.h"
#include "Software/structDefinition.h"

class mobileTHzEngine;
class tcpExp;

class RealtimePage : public QWidget
{
    Q_OBJECT

public:
    RealtimePage(QWidget *parent = nullptr);
    ~RealtimePage();

    /**
     * @brief Configures the page with live data pointers from initialization.
     * This must be called before the page is shown.
     * @param sp The shared pointers to live engine/experimental components.
     * @param config The configuration file for this run.
     */
    void setupPage(std::shared_ptr<SharedPointers> sp, const ConfigFile &config);

    void stopUpdating();
    void startUpdating();

protected:
    // Override show/hide events to control the update timer automatically
    void showEvent(QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;

private slots:
    /**
     * @brief The main update loop, triggered by the QTimer.
     * Polls the TCP component for new data and updates the UI.
     */
    void updateDisplay();

signals:
    void viewSimulationRequested();

private:
    void setupUI();

    // --- Data Members ---
    std::shared_ptr<SharedPointers> sharedPointers;
    ConfigFile paramConfig;
    QTimer *updateTimer;

    // --- UI Widgets ---
    // Left Column
    QLabel *roleLabel;
    QLabel *statusLabel;
    QListWidget *connectedNodesList;

    // Right Column
    QTextEdit *messageLog;
    QPushButton *viewSimulationButton;
};

#endif // REALTIMEPAGE_H
