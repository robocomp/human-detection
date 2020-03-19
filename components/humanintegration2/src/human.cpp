#include "human.h"

Human::Human(int ncameras, const QRectF &r, QPointF pos, float angle, QGraphicsScene *scene_) 
{
	scene = scene_;
	setFlag(ItemIsMovable);
    setFlag(ItemSendsGeometryChanges);
    setFlag(QGraphicsItem::ItemIsFocusable);
    setCacheMode(DeviceCoordinateCache);
    setAcceptHoverEvents(true);
	QPixmap pixmap = QPixmap::fromImage(QImage("/home/robocomp/robocomp/components/robocomp-tests/elasticpath/src/person.png")).scaled(800,400);
    pixmapItem = new QGraphicsPixmapItem( pixmap);
	pixmapItem->setParentItem(this);
	pixmapItem->setPos(pos.x()-pixmap.width()/2, pos.y()-pixmap.height()/2);

	
	for (int i=0;i<ncameras;i++)
	{
		QGraphicsEllipseItem *ellipseItem = new QGraphicsEllipseItem(r);
		ellipseItem->setPen(QPen(QBrush(QColor(colors[i])),20));  //transparent
		ellipseItem->setBrush(QColor(colors[i]));  //transparent
		cameraPose_list.append(ellipseItem);
		scene->addItem(ellipseItem);
	}
	
	this->setZValue(10);
	scene->addItem(this);
};

Human::~Human()
{
	delete pixmapItem;
	while (not cameraPose_list.isEmpty())
		delete cameraPose_list.takeFirst();
}

void Human::update(int cameraId, float x, float y, float ang)
{
//	cameraPose_list[cameraId-1]->setPos(QPointF(x,y));
	
	pixmapItem->setPos(x, y);
	pixmapItem->setRotation(qRadiansToDegrees(ang)+180);
}
