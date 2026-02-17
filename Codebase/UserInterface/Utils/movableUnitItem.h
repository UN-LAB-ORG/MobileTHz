// movableUnitItem.h
#ifndef MOVABLEUNITITEM_H
#define MOVABLEUNITITEM_H

#include <QGraphicsEllipseItem>
#include <QVariant>
#include "directionArrowItem.h"
#include "structDefinition.h"
#include <qquaternion.h>

class DirectionArrowItem;

enum class ViewPlane
{
    XY,
    XZ,
    YZ
};

inline QQuaternion toQQuaternion(const Quaternion &q)
{
    return QQuaternion(q.w, q.x, q.y, q.z);
}

inline Quaternion toQuaternionStruct(const QQuaternion &q)
{
    return {q.scalar(), q.x(), q.y(), q.z()};
}
// -------------------------------------------------------------------

class MovableUnitItem : public QGraphicsEllipseItem
{
public:
    MovableUnitItem(qreal x, qreal y, qreal width, qreal height, QGraphicsItem *parent = nullptr);
    ~MovableUnitItem();

    QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;
    void setUnitIndex(int index) { unitIndex = index; }
    int getUnitIndex() const { return unitIndex; }

    void setUnitName(const std::string &name) { unitName = name; }
    std::string getUnitName() const { return unitName; }

    // Effective Orientation (Base * Rotary)
    void setEffectiveQuaternion(const Quaternion &q);
    Quaternion getEffectiveQuaternion() const { return effectiveQuaternion; }

    // Base Orientation (From Environment / User Input on Config Page)
    void setBaseQuaternion(const Quaternion &q);
    Quaternion getBaseQuaternion() const { return baseQuaternion; }

    // --- Setter for the view plane ---
    void setViewPlane(ViewPlane plane);

    // Make arrows accessible (e.g., to hide/show or set interactivity)
    DirectionArrowItem *getBaseArrow() const { return baseDirectionArrow; }
    DirectionArrowItem *getEffectiveArrow() const { return effectiveDirectionArrow; }

    void setFuturePosition(const QPointF &pos);
    void clearFuturePosition();
    qreal getWidth() const { return width; }

protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

private:
    void updateBaseArrowDirection();
    void updateEffectiveArrowDirection();

    int unitIndex;
    std::string unitName;
    Quaternion effectiveQuaternion; // Stores Base * Rotary)
    Quaternion baseQuaternion;      // Stores Base orientation

    QPointF futurePosition;
    bool hasFuturePosition;

    DirectionArrowItem *effectiveDirectionArrow; // Shows Base * Rotary orientation
    DirectionArrowItem *baseDirectionArrow;      // Shows Base orientation

    qreal arrowLengthEffective;
    qreal arrowLengthBase;
    qreal width;

    // --- Member to store the current view plane ---
    ViewPlane viewPlane = ViewPlane::XY;
};

#endif // MOVABLEUNITITEM_H
