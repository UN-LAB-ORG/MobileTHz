#include "darkPalette.h"
#include <QApplication>
#include <QStyleFactory>

void DarkPalette::apply(QWidget *widget)
{
    QApplication::setStyle(QStyleFactory::create("Fusion"));

    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(42, 42, 42));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(200, 16, 46));
    darkPalette.setColor(QPalette::Highlight, QColor(200, 16, 46));
    darkPalette.setColor(QPalette::HighlightedText, Qt::white);

    if (widget)
    {
        widget->setPalette(darkPalette);
    }
    else
    {
        QApplication::setPalette(darkPalette);
    }
}
