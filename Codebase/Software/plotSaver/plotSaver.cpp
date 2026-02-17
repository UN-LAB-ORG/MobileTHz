#include "plotSaver.h"
#include "Codebase/Software/mobileTHzEngine/mobileTHzEngine.h"
#include "Codebase/Software/structDefinition.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtGui/QPainter>
#include <QtGui/QImage>
#include <QtCore/QFile>
#include <QtSvg/QSvgGenerator>
#include <QtGui/QPdfWriter>

// Define the results directory, which should be consistent across the project.
#ifndef RESULTS_DIR
#define RESULTS_DIR "Results"
#endif

namespace
{
    // Helper function to convert watts to dBm.
    inline double wattsToDbm(double watts)
    {
        if (watts <= 1e-30)
        {
            return -300.0;
        }
        return 10.0 * std::log10(watts) + 30.0;
    }

    // Helper to sanitize strings for use in LaTeX labels/captions
    std::string sanitizeForLatex(std::string str)
    {
        size_t pos = 0;
        while ((pos = str.find('_', pos)) != std::string::npos)
        {
            str.replace(pos, 1, "\\_");
            pos += 2;
        }
        return str;
    }
} // namespace

plotSaver::plotSaver() {}
plotSaver::~plotSaver() {}

void plotSaver::initialize(mobileTHzEngine *engine)
{
    if (!engine)
    {
        throw std::invalid_argument("plotSaver: engine pointer cannot be null.");
    }
    engine_ = engine;
    initialized_ = true;
}

void plotSaver::saveRxPowerVsTime(const std::string &kpi_base_name) const
{
    if (!initialized_ || !engine_)
    {
        std::cerr << "plotSaver Error: Not initialized." << std::endl;
        return;
    }

    const ConfigFile &config = engine_->getConfig();
    if (config.display_mode != "GUI")
    {
        return;
    }

    int primaryUeIndex = -1;
    const auto &engine_units = engine_->getAllEngineUnits();
    for (size_t i = 0; i < engine_units.size(); ++i)
    {
        if (engine_units[i].label.find("UE") != std::string::npos)
        {
            primaryUeIndex = static_cast<int>(i);
            break;
        }
    }

    if (primaryUeIndex == -1)
    {
        std::cerr << "plotSaver Warning: No UE found to log power data." << std::endl;
        return;
    }

    // --- Save LaTeX Plot to File ---
    try
    {
        std::filesystem::path resultsDirPath(RESULTS_DIR);

        // Construct filename from the base name, replacing the extension.
        std::filesystem::path base_path(kpi_base_name);
        base_path.replace_extension(".tex");
        std::filesystem::path filePath = resultsDirPath / base_path;

        // Ensure the parent directory exists
        if (filePath.has_parent_path())
        {
            std::filesystem::create_directories(filePath.parent_path());
        }

        std::ofstream outFile(filePath);
        if (!outFile)
        {
            throw std::runtime_error("Could not open file for writing: " + filePath.string());
        }

        const double time_step_s = config.engine_slot_time_microsec / 1.0e6;
        constexpr double SAMPLING_INTERVAL_S = 0.010;
        size_t slot_step = static_cast<size_t>(std::round(SAMPLING_INTERVAL_S / time_step_s));
        if (slot_step == 0)
            slot_step = 1;

        std::string safe_title = sanitizeForLatex(base_path.stem().string());

        outFile << "\\documentclass[border=5mm]{standalone}\n"
                << "\\usepackage{pgfplots}\n"
                << "\\pgfplotsset{compat=1.18}\n\n"
                << "\\begin{document}\n"
                << "\\begin{tikzpicture}\n"
                << "    \\begin{axis}[\n"
                << "        width=15cm, height=7cm,\n"
                << "        title={" << safe_title << "},\n"
                << "        xlabel={Time (s)},\n"
                << "        ylabel={Received Power (dBm)},\n"
                << "        grid=major,\n"
                << "    ]\n\n"
                << "    \\addplot[blue, mark=none] coordinates {\n";

        outFile << std::fixed << std::setprecision(6);
        const size_t num_time_slots = engine_->getNumTimeSlots();
        for (size_t slot = 0; slot < num_time_slots; slot += slot_step)
        {
            const double current_time_s = static_cast<double>(slot) * time_step_s;
            const double power_watts = engine_->getRxChainData(primaryUeIndex, slot).power_watts;
            outFile << "        (" << current_time_s << ", " << wattsToDbm(power_watts) << ")\n";
        }

        outFile << "    };\n"
                << "    \\end{axis}\n"
                << "\\end{tikzpicture}\n"
                << "\\end{document}\n";

        outFile.close();

        // Mimic the debug output style from the example
        std::cout << "plotSaver: Plot saved to: "
                  << std::filesystem::absolute(filePath).string() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "plotSaver: Standard exception while preparing plot for writing: "
                  << e.what() << std::endl;
    }
}

void plotSaver::saveIMUDataVsTime(const std::string &kpi_base_name) const
{
    if (!initialized_ || !engine_)
    {
        std::cerr << "plotSaver Error: Not initialized." << std::endl;
        return;
    }
    const ConfigFile &config = engine_->getConfig();
    if (config.display_mode != "GUI" || config.engine_mode != "EXPERIMENTAL")
    {
        return;
    }

    int ue_idx = -1; // Find primary UE index...
    const auto &engine_units = engine_->getAllEngineUnits();
    for (size_t i = 0; i < engine_units.size(); ++i)
    {
        if (engine_units[i].label.find("UE") != std::string::npos)
        {
            ue_idx = static_cast<int>(i);
            break;
        }
    }
    if (ue_idx == -1)
    {
        std::cerr << "plotSaver Warning: No UE found to log IMU data." << std::endl;
        return;
    }

    // --- Data Preparation ---
    const double time_step_s = config.engine_slot_time_microsec / 1.0e6;
    const size_t num_slots = engine_->getNumTimeSlots();

    // Create series objects to hold the data
    auto *accel_x = new QLineSeries();
    accel_x->setName("X");
    auto *accel_y = new QLineSeries();
    accel_y->setName("Y");
    auto *accel_z = new QLineSeries();
    accel_z->setName("Z");

    auto *ang_vel_x = new QLineSeries();
    ang_vel_x->setName("X");
    auto *ang_vel_y = new QLineSeries();
    ang_vel_y->setName("Y");
    auto *ang_vel_z = new QLineSeries();
    ang_vel_z->setName("Z");

    auto *ang_accel_x = new QLineSeries();
    ang_accel_x->setName("X");
    auto *ang_accel_y = new QLineSeries();
    ang_accel_y->setName("Y");
    auto *ang_accel_z = new QLineSeries();
    ang_accel_z->setName("Z");

    imuObject prev_imu = engine_->getIMUData(ue_idx, 0);
    for (size_t slot = 1; slot < num_slots; ++slot)
    {
        imuObject imu = engine_->getIMUData(ue_idx, slot);
        double time = static_cast<double>(slot) * time_step_s;

        accel_x->append(time, imu.acceleration.x);
        accel_y->append(time, imu.acceleration.y);
        accel_z->append(time, imu.acceleration.z);

        ang_vel_x->append(time, imu.angular_velocity.x);
        ang_vel_y->append(time, imu.angular_velocity.y);
        ang_vel_z->append(time, imu.angular_velocity.z);

        if (time_step_s > 1e-20)
        {
            ang_accel_x->append(time, (imu.angular_velocity.x - prev_imu.angular_velocity.x) / time_step_s);
            ang_accel_y->append(time, (imu.angular_velocity.y - prev_imu.angular_velocity.y) / time_step_s);
            ang_accel_z->append(time, (imu.angular_velocity.z - prev_imu.angular_velocity.z) / time_step_s);
        }
        prev_imu = imu;
    }

    // --- Chart Creation and Configuration ---
    // Chart 1: Acceleration
    auto *accel_chart = new QChart();
    accel_chart->addSeries(accel_x);
    accel_chart->addSeries(accel_y);
    accel_chart->addSeries(accel_z);
    accel_chart->createDefaultAxes();
    if (!accel_chart->axes(Qt::Vertical).isEmpty())
    {
        qobject_cast<QValueAxis *>(accel_chart->axes(Qt::Vertical).first())->setTitleText("Acceleration (m/s²)");
    }
    accel_chart->legend()->setVisible(true);
    accel_chart->legend()->setAlignment(Qt::AlignRight);
    accel_chart->setTitle(QString::fromStdString(kpi_base_name + " IMU Data"));

    // Chart 2: Angular Velocity
    auto *ang_vel_chart = new QChart();
    ang_vel_chart->addSeries(ang_vel_x);
    ang_vel_chart->addSeries(ang_vel_y);
    ang_vel_chart->addSeries(ang_vel_z);
    ang_vel_chart->createDefaultAxes();
    if (!ang_vel_chart->axes(Qt::Vertical).isEmpty())
    {
        qobject_cast<QValueAxis *>(ang_vel_chart->axes(Qt::Vertical).first())->setTitleText("Angular Vel (rad/s)");
    }
    ang_vel_chart->legend()->setVisible(false);

    // Chart 3: Angular Acceleration
    auto *ang_accel_chart = new QChart();
    ang_accel_chart->addSeries(ang_accel_x);
    ang_accel_chart->addSeries(ang_accel_y);
    ang_accel_chart->addSeries(ang_accel_z);
    ang_accel_chart->createDefaultAxes();
    if (!ang_accel_chart->axes(Qt::Horizontal).isEmpty())
    {
        qobject_cast<QValueAxis *>(ang_accel_chart->axes(Qt::Horizontal).first())->setTitleText("Time (s)");
    }
    if (!ang_accel_chart->axes(Qt::Vertical).isEmpty())
    {
        qobject_cast<QValueAxis *>(ang_accel_chart->axes(Qt::Vertical).first())->setTitleText("Angular Accel (rad/s²)");
    }
    ang_accel_chart->legend()->setVisible(false);

    // --- File Path Setup ---
    std::filesystem::path resultsDirPath(RESULTS_DIR);
    std::string filename_base = kpi_base_name + "_IMU_Plots";
    std::filesystem::path png_filePath = resultsDirPath / (filename_base + ".png");
    std::filesystem::path pdf_filePath = resultsDirPath / (filename_base + ".pdf");

    // MODIFICATION: Define constants for much higher resolution image.
    const int IMAGE_WIDTH = 3000;
    const int PLOT_HEIGHT = 1000;
    const int TOTAL_HEIGHT = PLOT_HEIGHT * 3;

    // --- Rendering to PNG Image ---
    auto render_chart = [&](QPainter *painter, QChart *chart, const QRectF &targetRect)
    {
        QGraphicsScene scene;
        // Set the chart size directly. It's a QGraphicsObject.
        chart->resize(targetRect.size());
        scene.addItem(chart);
        // Render the specific part of the scene (the chart) to the target rectangle
        scene.render(painter, targetRect, QRectF(QPointF(0, 0), targetRect.size()));
    };

    // --- Rendering to PNG Image ---
    QImage image(IMAGE_WIDTH, TOTAL_HEIGHT, QImage::Format_ARGB32);
    image.fill(Qt::white);
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);

    render_chart(&painter, accel_chart, QRectF(0, 0, IMAGE_WIDTH, PLOT_HEIGHT));
    render_chart(&painter, ang_vel_chart, QRectF(0, PLOT_HEIGHT, IMAGE_WIDTH, PLOT_HEIGHT));
    render_chart(&painter, ang_accel_chart, QRectF(0, 2 * PLOT_HEIGHT, IMAGE_WIDTH, PLOT_HEIGHT));
    painter.end();

    if (image.save(QString::fromStdString(png_filePath.string())))
    {
        std::cout << "plotSaver: IMU plot saved to: " << std::filesystem::absolute(png_filePath).string() << std::endl;
    }
    else
    {
        std::cerr << "plotSaver Error: Failed to save PNG file to " << png_filePath.string() << std::endl;
    }

    // --- Save to PDF ---
    QPdfWriter writer(QString::fromStdString(pdf_filePath.string()));
    writer.setPageSize(QPageSize(QSize(IMAGE_WIDTH, TOTAL_HEIGHT)));
    QPainter pdf_painter(&writer);
    pdf_painter.setRenderHint(QPainter::Antialiasing);

    render_chart(&pdf_painter, accel_chart, QRectF(0, 0, IMAGE_WIDTH, PLOT_HEIGHT));
    render_chart(&pdf_painter, ang_vel_chart, QRectF(0, PLOT_HEIGHT, IMAGE_WIDTH, PLOT_HEIGHT));
    render_chart(&pdf_painter, ang_accel_chart, QRectF(0, 2 * PLOT_HEIGHT, IMAGE_WIDTH, PLOT_HEIGHT));
    pdf_painter.end();

    std::cout << "plotSaver: IMU plot saved to: " << std::filesystem::absolute(pdf_filePath).string() << std::endl;

    // --- Cleanup ---
    delete accel_chart;
    delete ang_vel_chart;
    delete ang_accel_chart;
}
