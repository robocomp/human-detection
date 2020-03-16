#include "human.h"

Human::Human(const QRectF &r, QColor color_, QPointF pos, float angle, QGraphicsScene *scene_) 
{
	scene = scene_;
	setFlag(ItemIsMovable);
    setFlag(ItemSendsGeometryChanges);
    setFlag(QGraphicsItem::ItemIsFocusable);
    setCacheMode(DeviceCoordinateCache);
    setAcceptHoverEvents(true);
	QPixmap pixmap = QPixmap::fromImage(QImage("/home/robocomp/robocomp/components/robocomp-tests/elasticpath/src/person.png")).scaled(800,400);
    pixmapItem = new QGraphicsPixmapItem( pixmap);
	ellipseItem = new QGraphicsEllipseItem(r);
	ellipseItem->setParentItem(this);
	ellipseItem->setPen(QPen(QBrush(QColor(color)),20));  //transparent
	ellipseItem->setBrush(QColor(color));  //transparent
	pixmapItem->setParentItem(ellipseItem);
	pixmapItem->setPos(pos.x()-pixmap.width()/2, pos.y()-pixmap.height()/2);
	//pixmapItem->setPos(ellipseItem->rect().center().x(), pos.y()-ellipseItem->rect().center().y());
	//ellipseItem->setPos(-ellipseItem->rect().center().x(), -ellipseItem->rect().center().y());
	this->setPos(pos);
	this->setRotation(angle);
	this->color.setAlpha(80);
	this->setZValue(10);
	scene->addItem(this);
};

Human::~Human()
{
	delete pixmapItem;
	delete ellipseItem;
}

void Human::initialize(const QPointF &pos, float ang)
{
 	//Initial position
	x.setZero();
	x.x() = pos.x();
	x.y() = pos.y();
	x.theta() = ang;

	this->setPos(pos);
}
void Human::update(float x, float y, float ang)
{
	this->setPos(QPointF(x,y));
	this->setRotation(qRadiansToDegrees(ang)+180);
}
