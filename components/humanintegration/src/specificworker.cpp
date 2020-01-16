/*
 *    Copyright (C)2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <QGridLayout>
#include <QDesktopWidget>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	emit t_compute_to_finalize();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		this->params = params;
	}
	catch (const std::exception &e)
	{
		qFatal("Error reading config params");
	}
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = std::make_shared<InnerModel>(innermodel_path);
	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }

	defaultMachine.start();

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	//Load World
	initializeWorld();

	resize(QDesktopWidget().availableGeometry(this).size() * 0.6);
	scene.setSceneRect(dimensions.HMIN, dimensions.VMIN, dimensions.WIDTH, dimensions.HEIGHT);
	graphicsView->setViewport(new QGLWidget);
	graphicsView->scale(1, -1);
	graphicsView->setScene(&scene);
	graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio);

	this->Period = period;
	timer.start(100);
	emit this->t_initialize_to_compute();

}
/////////////////////////////////////////////////
void SpecificWorker::compute()
{
	// match de lo que llega con lo que hay en el estado
	// para cada camara ..
	for(auto &cam : cameraList)
	{
		if( const auto &[success, observed_people] = cam.pop(); success == true )
		{
			//transformo a coordenadas del mundo y calculo pose
			auto observed_model_people = transformToWorld(observed_people);
			//para todas las pers. observadas por esta cámara ...
			for(auto &ob_p : observed_model_people)
			{
				//compruebo si ya están en el mundo: identidad
				for(auto &mo_p : model_people)
				{
					//check for maximum distance of 300 mms
					if((QVec::vec3(mo_p.x,mo_p.y,mo_p.z) - QVec::vec3(ob_p.x,ob_p.y,ob_p.z)).norm2() < 300)
					{
						qDebug() << "cool, recognized person!" << model_people.size();
						// mark person from observed_world_people as recognized
						ob_p.matched = true;
						mo_p.tiempo_no_visible = 0;  //poner timestamp
						mo_p.human->setPos(QPointF(ob_p.x,ob_p.z));
					}
				}
			}
			// añadir aquellos que se ha visto pero no se han reconocido entre los existentes
			for(const auto &mo_p : observed_model_people)
			{
				if(mo_p.matched == false)
				{
					ModelPerson mp;
					mp.x=0; mp.y=0; mp.z=0;
					mp.angle = 0;
					mp.tiempo_no_visible = 0;
					mp.matched=false;
					mp.human = new Human(QRectF(0, 0, 100, 100), QColor(Qt::green), QPointF(mo_p.x, mo_p.z), &scene);
					model_people.push_back(mp);
				}
			}
			//borrado de la lista interna que no se hayan visto en MAX_AUSENTE segundos
			for(auto &mo_p : model_people)
			{
				if(mo_p.tiempo_no_visible > MAX_AUSENTE)
				{
					//marcamos el sujeto para borrado

				};
				//borrado físico
			}
		}
	}
}

SpecificWorker::ModelPeople SpecificWorker::transformToWorld(const RoboCompHumanCameraBody::PeopleData &observed_people)
{
	ModelPeople res;
	//innerModel->transform("world", QVec::vec3(0,0,2000), "world_camera_1").print("prueba");
	for(const auto &obs_person : observed_people.peoplelist)
	{
		QVec left_s, right_s;
		QVec wj;
		for(const auto &[name, key] : obs_person.joints)
		{
			//qDebug() << "claves" << QString::fromStdString(name);
			//qDebug() << QString::fromStdString(name) ; QVec::vec3(key.x, key.y, key.z).print("key");
			wj = innerModel->transform("world", QVec::vec3(1000*key.x, 1000*key.y, 1000*key.z), "world_camera_" + QString::number(observed_people.cameraId));
			
			if(name=="right_shoulder")
				left_s = wj;
			if(name=="left_shoulder")
				right_s = wj;
		}
		//qDebug() << __FUNCTION__;
		((left_s + right_s)/(T)2.).print("media");
		res.push_back( { wj.x(), wj.y(), wj.z(), 0.0, 0} );
	}	
	return res;
} 

//////////////////////////////////////////////////
void SpecificWorker::sm_compute()
{
	//std::cout<<"Entered state compute"<<std::endl;
	compute();
}

void SpecificWorker::sm_initialize()
{
	//std::cout<<"Entered initial state initialize"<<std::endl;
}

void SpecificWorker::sm_finalize()
{
	//std::cout<<"Entered final state finalize"<<std::endl;
}

//load world model from file
void SpecificWorker::initializeWorld()
{
	QString val;
	QFile file(QString::fromStdString(params["World"].value));
	if (not file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		qDebug() << "Error reading world file, check config params:" << QString::fromStdString(params["World"].value);
		exit(-1);
	}
	val = file.readAll();
	file.close();
	QJsonDocument doc = QJsonDocument::fromJson(val.toUtf8());
	QJsonObject jObject = doc.object();
	QVariantMap mainMap = jObject.toVariantMap();
	//load dimensions
	QVariantMap dim = mainMap[QString("dimensions")].toMap();
	dimensions = Dimensions{dim["LEFT"].toFloat(), dim["BOTTOM"].toFloat(), dim["WIDTH"].toFloat(), dim["HEIGHT"].toFloat()};

	//load tables
	QVariantMap tables = mainMap[QString("tables")].toMap();
	for (auto &t : tables)
	{
		QVariantList object = t.toList();
		auto box = scene.addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("SandyBrown")), QBrush(QColor("SandyBrown")));
		box->setPos(object[4].toFloat(), object[5].toFloat());
		box->setRotation(object[6].toFloat());
		boxes.push_back(box);
	}
	//load roundtables
	QVariantMap rtables = mainMap[QString("roundTables")].toMap();
	for (auto &t : rtables)
	{
		QVariantList object = t.toList();
		auto box = scene.addEllipse(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Khaki")), QBrush(QColor("Khaki")));
		box->setPos(object[4].toFloat(), object[5].toFloat());
		boxes.push_back(box);
	}
	//load walls
	QVariantMap walls = mainMap[QString("walls")].toMap();
	for (auto &t : walls)
	{
		QVariantList object = t.toList();
		auto box = scene.addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Brown")));
		box->setPos(object[4].toFloat(), object[5].toFloat());
		//box->setRotation(object[6].toFloat()*180/M_PI2);
		boxes.push_back(box);
	}

	//load points
	QVariantMap points = mainMap[QString("points")].toMap();
	for (auto &t : points)
	{
		QVariantList object = t.toList();
		auto box = scene.addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Brown")));
		box->setPos(object[4].toFloat(), object[5].toFloat());
		//box->setRotation(object[6].toFloat()*180/M_PI2);
		boxes.push_back(box);
	}
	//load boxes
	QVariantMap cajas = mainMap[QString("boxes")].toMap();
	for (auto &t : cajas)
	{
		QVariantList object = t.toList();
		auto box = scene.addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Orange")));
		box->setPos(object[4].toFloat(), object[5].toFloat());
		//box->setRotation(object[6].toFloat()*180/M_PI2);
		box->setFlag(QGraphicsItem::ItemIsMovable);
		boxes.push_back(box);
	}
}

/////////////////////////////////77
//// STUB
///////////////////////////////////

void SpecificWorker::HumanCameraBody_newPeopleData(PeopleData people)
{
	if(people.cameraId +1 > (int)cameraList.size())
		cameraList.resize(people.cameraId + 1);
	
	cameraList[people.cameraId].push(people);

	//qDebug() << "Inserting in " << people.cameraId << people.peoplelist.size();
}

