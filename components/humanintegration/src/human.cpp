#include "human.h"
#include <QtCore>
#include <QGraphicsSceneMouseEvent>

Human::Human(const QRectF &r, QColor color_, QPointF pos, float angle, QGraphicsScene *scene_) 
{
	scene = scene_;
	setFlag(ItemIsMovable);
    setFlag(ItemSendsGeometryChanges);
    setFlag(QGraphicsItem::ItemIsFocusable);
    setCacheMode(DeviceCoordinateCache);
    setAcceptHoverEvents(true);
	//QPixmap pixmap = QPixmap::fromImage(QImage("/home/robocomp/robocomp/components/robocomp-tests/elasticpath/src/person.png")).scaled(800,400);
    //pixmapItem = new QGraphicsPixmapItem( pixmap);
	ellipseItem = new QGraphicsEllipseItem(r);
	ellipseItem->setParentItem(this);
	ellipseItem->setPen(QPen(QBrush(QColor(color)),20));  //transparent
	ellipseItem->setBrush(QColor(color));  //transparent
	//pixmapItem->setParentItem(ellipseItem);
	//pixmapItem->setPos(ellipseItem->rect().center().x(), pos.y()-ellipseItem->rect().center().y());
	ellipseItem->setPos(-ellipseItem->rect().center().x(), -ellipseItem->rect().center().y());
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
	x.x() = pos.x();
	x.y() = pos.y();
	x.theta() = ang;
	
	//Covarianza sys
	Kalman::Covariance<State> sC(3,3);
	sC.setZero();
	// sC(0,0) = 0.9;
	// sC(1,1) = 0.9;
	// sC(2,2) = 0.9;

	sC(0,0) = 0.00001;
	sC(1,1) = 0.00001;
	sC(2,2) = 0.00001;
	sC(0,0) = 0.0001;
	sC(1,1) = 0.0001;
	sC(2,2) = 0.01;
	sys.setCovariance(sC);
	
	//positionModel
	Kalman::Covariance<PositionMeasurement> pC(2,2);
	pC.setZero();
	pC(0,0) = 0.01;
	pC(1,1) = 0.01;
	pm.setCovariance(pC);
	
	//orientationModel
	Kalman::Covariance<OrientationMeasurement> oC(1,1);
	oC(0,0) = 0.01;
	om.setCovariance(oC);

	// Init filters with true system state
	//ukf = Kalman::UnscentedKalmanFilter<State>(1);
	ekf.init(x);

	this->setPos(pos);
}
void Human::update(float x, float y, float ang)
{
	static QPointF pos_antEKF;

	State observed_state;
	observed_state.x() = x;
	observed_state.y() = y;
	observed_state.theta() = qDegreesToRadians(ang);

    auto x_ekf = ekf.predict(sys);
	//	auto x_ukf = ukf.predict(sys, u);
	OrientationMeasurement orientation = om.h(observed_state);
	x_ekf = ekf.update(om, orientation);
//	x_ukf = ukf.update(om, orientation);
	PositionMeasurement position = pm.h(observed_state);
	x_ekf = ekf.update(pm, position);
//	x_ukf = ukf.update(pm, position);

	scene->addLine(QLineF(pos_antEKF, QPointF(x_ekf.x(),x_ekf.y())), QPen(QColor("Blue"), 50));
	pos_antEKF = QPointF(x_ekf.x(),x_ekf.y());
	this->setPos(QPointF(x_ekf.x(),x_ekf.y()));
	this->setRotation(qRadiansToDegrees(x_ekf.theta()));
	//this->setRotation(ang);
	
	qDebug()<< "filter:" << qRadiansToDegrees(x_ekf.theta());
	
	//this->setPos(x,y);
	
	//qDebug() << QPointF(x_ekf.x(),x_ekf.y());
}
