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
/*	pixmap = QPixmap::fromImage(QImage("/home/robocomp/robocomp/components/robocomp-tests/elasticpath/src/person.png")).scaled(800,400);
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
		cPose.text->setFont(QFont("Times", 120));
		cPose.text->setZValue(30);
		scene->addItem(cPose.text);
		cameraPose_list.append(cPose);
		
	}
*/
	pixmap = QPixmap::fromImage(QImage("/home/robocomp/robocomp/components/robocomp-tests/elasticpath/src/person.png")).scaled(600,300);
	pixmapHalfSizeX = pixmap.width()/2;
	pixmapHalfSizeY = pixmap.height()/2;
	ellipseHalfSizeX = r.width()/2;
	ellipseHalfSizeY = r.height()/2;
	for (int i=0;i<3;i++)
	{
		humanModel newModel;
		//ellipse
		newModel.ellipse = new QGraphicsEllipseItem(r);
		newModel.ellipse->setTransformOriginPoint(newModel.ellipse->boundingRect().center());
		newModel.ellipse->setPen(QPen(QBrush(QColor(colors[i])),20));  
		newModel.ellipse->setBrush(QColor(colors[i]));  
		newModel.ellipse->setZValue(30);
		scene->addItem(newModel.ellipse);
		//pixmap

		newModel.pixmap = new QGraphicsPixmapItem( pixmap);
		newModel.pixmap->setTransformOriginPoint(newModel.pixmap->boundingRect().center());
		newModel.pixmap->setParentItem(this);
		newModel.pixmap->setZValue(20);
		scene->addItem(newModel.pixmap);
		models.append(newModel);
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
	while (not models.isEmpty())
	{
		auto model = models.takeFirst();
		delete model.ellipse;
		delete model.pixmap;
	}
}

void Human::update(int cameraId, float x, float y, float ang)
{
//	qDebug()<<"update"<<cameraId<<x<<y<<ang<<cameraPose_list.size();
	cameraPose_list[cameraId-1].ellipse->setPos(x-ellipseHalfSizeX, y - ellipseHalfSizeY);
	cameraPose_list[cameraId-1].text->setPos(x-ellipseHalfSizeX/2, y - ellipseHalfSizeY);
}

void Human::updateHuman(float x, float y, float ang)
{
qDebug()<<"PERSONUPDATE"<<x<<y<<ang;
	pixmapItem->setPos(x-pixmapHalfSizeX, y-pixmapHalfSizeY);
	if (not isnan(ang))
		pixmapItem->setRotation(qRadiansToDegrees(ang)+180);
}

void Human::updateGroundTruth(float x, float y, float ang)
{
	updateN(0, x, y, ang);
}
void Human::updateGNN(float x, float y, float ang)
{
	updateN(1, x, y, ang);
}
void Human::updateCameraMedian(float x, float y, float ang)
{
	updateN(2, x, y, ang);
}

void Human::updateN(int n, float x, float y, float ang)
{
	models[n].pixmap->setPos(x-pixmapHalfSizeX, y-pixmapHalfSizeY);
	models[n].pixmap->setRotation(qRadiansToDegrees(ang)+180);
	models[n].ellipse->setPos(x-ellipseHalfSizeX, y - ellipseHalfSizeY);
}