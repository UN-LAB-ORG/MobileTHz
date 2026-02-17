// movableUnitItem.cpp
#include "movableUnitItem.h"
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QQuaternion>
#include <QStyleOptionGraphicsItem>
#include <QWidget>
#include <QtMath>
#include "directionArrowItem.h"

MovableUnitItem::MovableUnitItem(qreal x, qreal y, qreal w, qreal height, QGraphicsItem *parent)
    : QGraphicsEllipseItem(x, y, w, height, parent), unitIndex(-1), baseQuaternion({1.0, 0.0, 0.0, 0.0}),
      effectiveQuaternion({1.0, 0.0, 0.0, 0.0}),
      hasFuturePosition(false), width(w)
{
    arrowLengthEffective = width / 1.25;
    arrowLengthBase = width / 2;

    setFlag(QGraphicsItem::ItemIsMovable);
    setFlag(QGraphicsItem::ItemIsSelectable);
    setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    setAcceptHoverEvents(true);

    // --- Effective Orientation Arrow (e.g., Yellow Solid) ---
    // This arrow is display-only everywhere
    effectiveDirectionArrow = new DirectionArrowItem(this);
    effectiveDirectionArrow->setParentItem(this);
    QPen effectivePen(Qt::yellow);
    effectivePen.setWidth(4);
    effectiveDirectionArrow->setPen(effectivePen);
    // Disable interactivity explicitly for this arrow always
    effectiveDirectionArrow->setInteractive(false);
    updateEffectiveArrowDirection();
    // set z value higher than the ellipse
    effectiveDirectionArrow->setZValue(zValue() + 1);

    // --- Base Orientation Arrow (e.g., Cyan) ---
    // This arrow will be interactive on PositionConfigPage
    baseDirectionArrow = new DirectionArrowItem(this);
    baseDirectionArrow->setParentItem(this);
    QPen basePen(Qt::cyan);
    basePen.setWidth(3);
    baseDirectionArrow->setPen(basePen);
    updateBaseArrowDirection();
    // set z value higher than effective arrow
    baseDirectionArrow->setZValue(effectiveDirectionArrow->zValue() + 1);

    // Initially hide the effective arrow - it's only shown on Sim page
    effectiveDirectionArrow->setVisible(false);
}

void MovableUnitItem::setViewPlane(ViewPlane plane)
{
    if (viewPlane == plane)
        return; // No change needed
    viewPlane = plane;
    // Refresh arrows with the new projection
    updateEffectiveArrowDirection();
    updateBaseArrowDirection();
}

MovableUnitItem::~MovableUnitItem() {}
QVariant MovableUnitItem::itemChange(GraphicsItemChange change, const QVariant &value)
{
    if (change == ItemPositionChange && scene())
    {
        QPointF newPos = value.toPointF();
        // Snapping logic, every 10 units
        qreal snapSize = 5.0;
        newPos.setX(qRound(newPos.x() / snapSize) * snapSize);
        newPos.setY(qRound(newPos.y() / snapSize) * snapSize);
        // Update the scene position
        return newPos;
    }
    return QGraphicsEllipseItem::itemChange(change, value);
}

void MovableUnitItem::setEffectiveQuaternion(const Quaternion &q)
{
    if (effectiveQuaternion.w == q.w && effectiveQuaternion.x == q.x && effectiveQuaternion.y == q.y && effectiveQuaternion.z == q.z)
        return; // Avoid unnecessary updates
    effectiveQuaternion = q;
    updateEffectiveArrowDirection();
}

void MovableUnitItem::setBaseQuaternion(const Quaternion &q)
{
    if (baseQuaternion.w == q.w && baseQuaternion.x == q.x && baseQuaternion.y == q.y && baseQuaternion.z == q.z)
        return; // Avoid unnecessary updates
    baseQuaternion = q;
    updateBaseArrowDirection();
}

void MovableUnitItem::setFuturePosition(const QPointF &pos)
{
    futurePosition = pos;
    hasFuturePosition = true;
    update();
}

void MovableUnitItem::clearFuturePosition()
{
    hasFuturePosition = false;
    update();
}

void MovableUnitItem::paint(QPainter *painter,
                            const QStyleOptionGraphicsItem *option,
                            QWidget *widget)
{
    // Draw future position
    if (hasFuturePosition)
    {
        QPen ghostPen(Qt::white);
        ghostPen.setStyle(Qt::DashLine);
        ghostPen.setWidth(1);
        painter->setPen(ghostPen);
        QBrush ghostBrush = this->brush();
        QColor ghostColor = ghostBrush.color();
        ghostColor.setAlpha(20);
        ghostBrush.setColor(ghostColor);
        painter->setBrush(ghostBrush);
        QPointF localFuturePos = mapFromScene(futurePosition);
        painter->drawEllipse(localFuturePos.x() - rect().width() / 2,
                             localFuturePos.y() - rect().height() / 2,
                             rect().width(),
                             rect().height());
    }

    // Draw the main ellipse
    QStyleOptionGraphicsItem opt = *option;
    opt.state &= ~QStyle::State_Selected;
    QGraphicsEllipseItem::paint(painter, &opt, widget);
}

void MovableUnitItem::updateEffectiveArrowDirection()
{
    if (!effectiveDirectionArrow)
        return;

    QQuaternion qEff = toQQuaternion(effectiveQuaternion);
    QVector3D baseDir(arrowLengthEffective, 0.0, 0.0);
    QVector3D rotatedDir = qEff.rotatedVector(baseDir);

    qreal dx = 0.0, dy = 0.0;
    // --- Project the 3D rotated vector onto the selected 2D plane ---
    switch (viewPlane)
    {
    case ViewPlane::XY:
        dx = rotatedDir.x();
        dy = -rotatedDir.y(); // Y flip for scene coords
        break;
    case ViewPlane::XZ:
        dx = rotatedDir.x();
        dy = -rotatedDir.z(); // Project Z to screen's Y, flip for natural view
        break;
    case ViewPlane::YZ:
        dx = rotatedDir.y();  // Project Y to screen's X
        dy = -rotatedDir.z(); // Project Z to screen's Y, flip
        break;
    }

    effectiveDirectionArrow->setLine(0, 0, dx, dy);
}

void MovableUnitItem::updateBaseArrowDirection()
{
    if (!baseDirectionArrow)
        return;

    QQuaternion qBase = toQQuaternion(baseQuaternion);
    QVector3D baseDir(arrowLengthBase, 0.0, 0.0);
    QVector3D rotatedDir = qBase.rotatedVector(baseDir);

    qreal dx = 0.0, dy = 0.0;
    // --- Project the 3D rotated vector onto the selected 2D plane ---
    switch (viewPlane)
    {
    case ViewPlane::XY:
        dx = rotatedDir.x();
        dy = -rotatedDir.y(); // Y flip for scene coords
        break;
    case ViewPlane::XZ:
        dx = rotatedDir.x();
        dy = -rotatedDir.z(); // Project Z to screen's Y, flip
        break;
    case ViewPlane::YZ:
        dx = rotatedDir.y();  // Project Y to screen's X
        dy = -rotatedDir.z(); // Project Z to screen's Y, flip
        break;
    }

    baseDirectionArrow->setLine(0, 0, dx, dy);
}
