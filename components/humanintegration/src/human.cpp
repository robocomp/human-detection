#include "human.h"
#include <QtCore>
#include <QGraphicsSceneMouseEvent>

Human::Human(const QRectF &r, QColor color_, QPointF pos, QGraphicsScene *scene) : 
	QGraphicsEllipseItem(r) ,  color(color_)
{
	setFlag(ItemIsMovable);
    setFlag(ItemSendsGeometryChanges);
    setFlag(QGraphicsItem::ItemIsFocusable);
    setCacheMode(DeviceCoordinateCache);
    setAcceptHoverEvents(true);
	QPixmap pixmap = QPixmap::fromImage(QImage("/home/robocomp/robocomp/components/robocomp-tests/elasticpath/src/person.png")).scaled(800,400);
    pixmapItem = new QGraphicsPixmapItem( pixmap);
	ellipseItem = new QGraphicsEllipseItem(r);
	ellipseItem->setParentItem(this);
	ellipseItem->setPen(QColor(0,0,0,0));  //transparent
	ellipseItem->setBrush(QColor(0,0,0,0));  //transparent
	pixmapItem->setParentItem(ellipseItem);
	pixmapItem->setPos(this->x()-pixmap.width()/2, this->y()-pixmap.height()/2);
	this->setPos(pos);
	this->color.setAlpha(80);
	this->setZValue(10);
	scene->addItem(this);
};
