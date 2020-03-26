#include "human.h"

Human::Human(int id_, int ncameras, const QRectF &r, QPointF pos, float angle, QGraphicsScene *scene_) 
{
	scene = scene_;
	id = id_;
	setFlag(ItemIsMovable);
    setFlag(ItemSendsGeometryChanges);
    setFlag(QGraphicsItem::ItemIsFocusable);
    setCacheMode(DeviceCoordinateCache);
    setAcceptHoverEvents(true);
	pixmap = QPixmap::fromImage(QImage("/home/robocomp/robocomp/components/robocomp-tests/elasticpath/src/person.png")).scaled(800,400);
    pixmapItem = new QGraphicsPixmapItem( pixmap);
	pixmapItem->setTransformOriginPoint(pixmapItem->boundingRect().center());
	pixmapItem->setParentItem(this);
	pixmapHalfSizeX = pixmap.width()/2;
	pixmapHalfSizeY = pixmap.height()/2;
	scene->addItem(pixmapItem);
	
	ellipseHalfSizeX = r.width()/2;
	ellipseHalfSizeY = r.height()/2;

	for (int i=0;i<ncameras;i++)
	{
		cameraPose cPose;
		//ellipse
		cPose.ellipse = new QGraphicsEllipseItem(r);
		cPose.ellipse->setTransformOriginPoint(cPose.ellipse->boundingRect().center());
		cPose.ellipse->setPen(QPen(QBrush(QColor(colors[i])),20));  //transparent
		cPose.ellipse->setBrush(QColor(colors[i]));  //transparent
		cPose.ellipse->setZValue(20);
		scene->addItem(cPose.ellipse);
		//text
		cPose.text = new QGraphicsTextItem(QString::number(id));
		cPose.text->setTransformOriginPoint(cPose.text->boundingRect().center());
		cPose.text->setDefaultTextColor("black");
		cPose.text->setFont(QFont("Times", 150));
		cPose.text->setZValue(30);
		scene->addItem(cPose.text);
		cameraPose_list.append(cPose);
		
	}
	
	this->setZValue(10);
};

Human::~Human()
{
	delete pixmapItem;
	while (not cameraPose_list.isEmpty())
	{
		auto cPose = cameraPose_list.takeFirst();
		delete cPose.ellipse;
		delete cPose.text;
	}
}

void Human::update(int cameraId, float x, float y, float ang)
{
//	qDebug()<<"update"<<cameraId<<x<<y<<ang<<cameraPose_list.size();
	cameraPose_list[cameraId-1].ellipse->setPos(x-ellipseHalfSizeX, y - ellipseHalfSizeY);
	cameraPose_list[cameraId-1].text->setPos(x-ellipseHalfSizeX/2, y - ellipseHalfSizeY);

	pixmapItem->setPos(x-pixmapHalfSizeX, y-pixmapHalfSizeY);
	pixmapItem->setRotation(qRadiansToDegrees(ang)+180);
}

