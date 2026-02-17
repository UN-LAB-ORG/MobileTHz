// DirectionArrowItem.h
#ifndef DIRECTIONARROWITEM_H
#define DIRECTIONARROWITEM_H

#include <QGraphicsLineItem>
#include <QMouseEvent>
#include "UserInterface/Utils/movableUnitItem.h"

class MovableUnitItem;
class DirectionArrowItem : public QGraphicsLineItem
{
public:
    DirectionArrowItem(MovableUnitItem *parentUnit);

    // Set whether this arrow allows user interaction to change orientation
    void setInteractive(bool interactive);

protected:
    // Only handle events if interactive
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    void paint(QPainter *painter,
               const QStyleOptionGraphicsItem *option,
               QWidget *widget) override; // Keep paint override

private:
    // Calculate the new BASE quaternion based on mouse drag
    void updateBaseOrientationFromMouse(const QPointF &scenePos);

    MovableUnitItem *parentUnit;
    bool isDragging;
    bool isInteractiveFlag;
};

#endif // DIRECTIONARROWITEM_H
