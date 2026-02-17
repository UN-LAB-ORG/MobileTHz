#include "directionArrowItem.h"
#include <QGraphicsScene>
#include <QPainter>
#include <QQuaternion>
#include <QStyleOptionGraphicsItem>
#include <QtMath>
#include "movableUnitItem.h"
#include <qgraphicssceneevent.h>

DirectionArrowItem::DirectionArrowItem(MovableUnitItem *parentUnit)
    : QGraphicsLineItem(parentUnit), parentUnit(parentUnit), isDragging(false), isInteractiveFlag(false)
{
    QPen arrowPen(Qt::yellow); // Default pen
    arrowPen.setWidth(4);
    setPen(arrowPen);

    // Initial flags - hover/selection depend on isInteractiveFlag
    setFlag(QGraphicsItem::ItemIsSelectable, false);
    setFlag(QGraphicsItem::ItemIsMovable, false);
    setAcceptHoverEvents(false);

    setZValue(parentUnit->zValue() + 1);
}

void DirectionArrowItem::setInteractive(bool interactive)
{
    isInteractiveFlag = interactive;
    // Update flags based on interactivity
    setFlag(QGraphicsItem::ItemIsSelectable, interactive);
    setAcceptHoverEvents(interactive);
}

// Calculate the new BASE orientation quaternion and update the parent
void DirectionArrowItem::updateBaseOrientationFromMouse(const QPointF &scenePos)
{
    if (!parentUnit)
        return;

    QPointF itemCenterScene = parentUnit->scenePos() + parentUnit->boundingRect().center(); // Center in scene coords
    qreal dx = scenePos.x() - itemCenterScene.x();
    qreal dy = scenePos.y() - itemCenterScene.y();

    if (qFuzzyCompare(dx, 0.0) && qFuzzyCompare(dy, 0.0))
    {
        return; // Avoid atan2(0,0)
    }

    // Calculate angle from positive X-axis in scene coordinates
    qreal angleRad = std::atan2(-dy, dx); // atan2(y, x), Y is inverted in scene coords

    // Create quaternion representing rotation around Z-axis (in world frame)
    QQuaternion q = QQuaternion::fromAxisAndAngle(0.0f, 0.0f, 1.0f, qRadiansToDegrees(angleRad));

    // Update the parent unit's BASE quaternion
    parentUnit->setBaseQuaternion(toQuaternionStruct(q));
}

// --- Mouse Events ---
void DirectionArrowItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (isInteractiveFlag && event->button() == Qt::LeftButton)
    {
        isDragging = true;
        // Ensure parent is selected when arrow interaction starts
        if (parentUnit && scene())
        {
            // Select only the parent unit
            scene()->clearSelection();
            parentUnit->setSelected(true);
        }
        event->accept();
    }
    else
    {
        // Pass event up if not interactive or not left button
        QGraphicsLineItem::mousePressEvent(event);
    }
}

void DirectionArrowItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if (isInteractiveFlag && isDragging)
    {
        updateBaseOrientationFromMouse(event->scenePos());
        event->accept();
    }
    else
    {
        QGraphicsLineItem::mouseMoveEvent(event); // Allow base class handling if needed
    }
}

void DirectionArrowItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    if (isInteractiveFlag && event->button() == Qt::LeftButton)
    {
        isDragging = false;
        event->accept();
    }
    else
    {
        QGraphicsLineItem::mouseReleaseEvent(event);
    }
}

// Paint override (mainly to prevent selection box on the line itself)
void DirectionArrowItem::paint(QPainter *painter,
                               const QStyleOptionGraphicsItem *option,
                               QWidget *widget)
{
    QStyleOptionGraphicsItem myOption = *option;
    myOption.state &= ~QStyle::State_Selected; // Don't draw selection indicator for the line
    QGraphicsLineItem::paint(painter, &myOption, widget);
}
