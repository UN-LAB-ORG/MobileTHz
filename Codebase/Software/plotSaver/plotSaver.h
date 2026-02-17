#ifndef PLOTSAVER_H
#define PLOTSAVER_H

#include <string>

class QChartView;
class QChart;

class mobileTHzEngine;

class plotSaver
{
public:
    plotSaver();
    ~plotSaver();

    void initialize(mobileTHzEngine *engine);

    void saveRxPowerVsTime(const std::string &kpi_base_name) const;

    void saveIMUDataVsTime(const std::string &kpi_base_name) const;

private:
    mobileTHzEngine *engine_ = nullptr;
    bool initialized_ = false;
    void renderAndSaveChart(QChart *chart, const std::string &filePath, const std::string &title) const;
};

#endif // PLOTSAVER_H