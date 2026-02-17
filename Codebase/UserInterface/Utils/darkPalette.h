#ifndef DARKPALETTE_H
#define DARKPALETTE_H

#include <QApplication>
#include <QPalette>
#include <QWidget>

class DarkPalette
{
public:
    static void apply(QWidget *widget = nullptr);
};

#endif // DARKPALETTE_H
