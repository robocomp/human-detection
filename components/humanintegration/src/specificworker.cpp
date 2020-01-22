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
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = std::make_shared<InnerModel>(innermodel_path);
	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }
	this->params = params;
	
	
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

// delete-create version
void SpecificWorker::compute()
{
	//add one per camera
	for(auto &cam : cameraList)
	{
		if( const auto &[success, observed_people] = cam.pop(); success == true )
		{
			//Delete camera people
			model_people.erase(std::remove_if(model_people.begin(), model_people.end(), [this, observed_people](auto &mo_p) 
				{ 
					if( observed_people.cameraId == mo_p.cameraId)
					{
						scene.removeItem(mo_p.human);
						return true;
					}
					else return false;
				}), model_people.end());

			//transformar a coordenadas del mundo y calculo pose
			auto observed_model_people = transformToWorld(observed_people);
			// añadir a cámara
			for(const auto &mo_p : observed_model_people)
			{
				QVec::vec3(mo_p.x,mo_p.y,mo_p.z).print("p");
				qDebug() << observed_people.cameraId;
				ModelPerson mp;
				mp.x=mo_p.x; mp.y=mo_p.y; mp.z=mo_p.z;
				mp.angle = mp.angle;
				mp.cameraId = observed_people.cameraId;
				QString color;
				if(observed_people.cameraId==1) color = "Green";
				if(observed_people.cameraId==2) color = "Blue";
				if(observed_people.cameraId==3) color = "Red";
				mp.human = new Human(QRectF(0, 0, 200, 200), QColor(color), QPointF(mo_p.x, mo_p.z), &scene);
				model_people.push_back(mp);
			}
		}
	}
}

/////////////////////////////////////////////////
// void SpecificWorker::compute()
// {
// 	// match de lo que llega con lo que hay en el estado
// 	// para cada camara ..
// 	for(auto &cam : cameraList)
// 	{
// 		if( const auto &[success, observed_people] = cam.pop(); success == true )
// 		{
// 			//transformo a coordenadas del mundo y calculo pose
// 			auto observed_model_people = transformToWorld(observed_people);
// 			//para todas las pers. observadas por esta cámara ...
// 			for(auto &ob_p : observed_model_people)
// 			{
// 				QVec::vec3(ob_p.x,ob_p.y,ob_p.z).print("p");
// 				//compruebo si ya están en el mundo: identidad
// 				for(auto &mo_p : model_people)
// 				{
// 					//check for maximum distance of 500 mm
// 					if((QVec::vec3(mo_p.x,mo_p.y,mo_p.z) - QVec::vec3(ob_p.x,ob_p.y,ob_p.z)).norm2() < 900)
// 					{
// 						qDebug() << "cool, recognized person!" << model_people.size();
// 						// mark person from observed_world_people as recognized
// 						ob_p.matched = true;
// 						mo_p.tiempo_no_visible = std::chrono::system_clock::now();  //poner timestamp
// 						mo_p.human->setPos(QPointF(ob_p.z,ob_p.x));
// 						mo_p.human->setRotation(ob_p.angle);
// 					}
// 				}
// 			}
// 			// añadir aquellos que se ha visto pero no se han reconocido entre los existentes
// 			for(const auto &mo_p : observed_model_people)
// 			{
// 				if(mo_p.matched == false)
// 				{
// 					ModelPerson mp;
// 					mp.x=mo_p.x; mp.y=mo_p.y; mp.z=mo_p.z;
// 					mp.angle = mp.angle;
// 					mp.tiempo_no_visible = std::chrono::system_clock::now();
// 					mp.matched=false;
// 					mp.human = new Human(QRectF(0, 0, 100, 100), QColor(Qt::green), QPointF(mo_p.z, mo_p.x), &scene);
// 					model_people.push_back(mp);
// 				}
// 			}
// 			//borrado de la lista interna que no se hayan visto en MAX_AUSENTE segundos
// 			model_people.erase(std::remove_if(model_people.begin(), model_people.end(), [this](auto &mo_p) 
// 				{ 
// 					if( std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-mo_p.tiempo_no_visible).count() > MAX_AUSENTE)
// 					{
// 						scene.removeItem(mo_p.human);
// 						return true;
// 					}
// 					else return false;
// 				}), model_people.end());
// 		}
// 	}
// }

std::tuple<bool, float> SpecificWorker::getOrientation(const RoboCompHumanCameraBody::Person &ob_p)
{
	std::pair<std::string, std::string> sh_pair{"left_shoulder", "right_shoulder"};
	std::pair<std::string, std::string> hip_pair{"left_hip", "right_hip"};
	std::vector<std::pair<std::string, std::string>> pair_list{sh_pair, hip_pair};
	for(const auto &pair : pair_list)
	{
		if(ob_p.joints.count(pair.first)>0 and ob_p.joints.count(pair.second)>0)
		{
			auto first = QPointF(ob_p.joints.at(pair.first).x, ob_p.joints.at(pair.first).z);
			auto second = QPointF(ob_p.joints.at(pair.second).x, ob_p.joints.at(pair.second).z);
			auto line =QLineF(first, second).normalVector();	
			return std::make_tuple(true, line.angle());
		}
	}
	return std::make_tuple(false, 0.0);
}

SpecificWorker::ModelPeople SpecificWorker::transformToWorld(const RoboCompHumanCameraBody::PeopleData &observed_people)
{
	ModelPeople res;
	for(const auto &obs_person : observed_people.peoplelist)
	{
		QVec left_s, right_s, wj;
		QVec acum = QVec::zeros(3);
		if(obs_person.joints.count("left_shoulder") > 0)   //test with one shoulder
		{
			auto key = obs_person.joints.at("left_shoulder");
			wj = innerModel->transform("world", QVec::vec3(key.x, key.y, key.z), "world_camera_" + QString::number(observed_people.cameraId));
			wj.print("wj");
			res.push_back( { obs_person.id, wj.x(), wj.y(), wj.z(), 0.0, std::chrono::time_point<std::chrono::system_clock>(), false, nullptr, false} );
			break;
		}

		// for(const auto &[name, key] : obs_person.joints)
		// {
		// 	wj = innerModel->transform("world", QVec::vec3(key.x, key.y, key.z), "world_camera_" + QString::number(observed_people.cameraId));
		// 	acum += wj;
		// }
		// auto [success, angle_degrees] = getOrientation(obs_person);
		// if(obs_person.joints.size() > 0)   
		// {
		// 	compute mean of projected joints coordinates on the floor
		// 	acum = acum/(T)obs_person.joints.size();
		// 	if( acum.x() != 0.0 and acum.z() != 0.0)
		// 	 	res.push_back( { obs_person.id, acum.x(), acum.y(), acum.z(), angle_degrees, std::chrono::time_point<std::chrono::system_clock>(), false, nullptr, false} );
		// }
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
	//int x_offset = -3200;
	//int y_offset = 1850;
	int x_offset = 0;
	int y_offset = 0;
	//load roundtables
	QVariantMap rtables = mainMap[QString("roundTables")].toMap();
	for (auto &t : rtables)
	{
		QVariantList object = t.toList();
		auto box = scene.addEllipse(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Khaki")), QBrush(QColor("Black")));
		box->setPos(object[4].toFloat(), object[5].toFloat());
		boxes.push_back(box);
	}
	//load tables
	QVariantMap tables = mainMap[QString("tables")].toMap();
	for (auto &t : tables)
	{
		QVariantList object = t.toList();
		auto box = scene.addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("SandyBrown")), QBrush(QColor("SandyBrown")));
		box->setPos(object[4].toFloat()+x_offset, object[5].toFloat()+y_offset);
		//box->setPos(object[4].toFloat(), object[5].toFloat());
		box->setRotation(object[6].toFloat());
		boxes.push_back(box);
	}
	//load walls
	QVariantMap walls = mainMap[QString("walls")].toMap();
	for (auto &t : walls)
	{
		QVariantList object = t.toList();
		auto box = scene.addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Brown")));
		box->setPos(object[4].toFloat()+x_offset, object[5].toFloat()+y_offset);
	
		box->setRotation(object[6].toFloat());
		boxes.push_back(box);
	}

	//load points
	QVariantMap points = mainMap[QString("points")].toMap();
	for (auto &t : points)
	{
		QVariantList object = t.toList();
		auto box = scene.addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Brown")));
		box->setPos(object[4].toFloat()+x_offset, object[5].toFloat()+y_offset);
		//box->setPos(object[4].toFloat(), object[5].toFloat());
		//box->setRotation(object[6].toFloat()*180/M_PI2);
		boxes.push_back(box);
	}
	//load boxes
	QVariantMap cajas = mainMap[QString("boxes")].toMap();
	for (auto &t : cajas)
	{
		QVariantList object = t.toList();
		auto box = scene.addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Orange")));
		box->setPos(object[4].toFloat()+x_offset, object[5].toFloat()+y_offset);
		//box->setPos(object[4].toFloat(), object[5].toFloat());
		//box->setRotation(object[6].toFloat()*180/M_PI2);
		box->setFlag(QGraphicsItem::ItemIsMovable);
		boxes.push_back(box);
	}
	/////////////
	//AXIS
	scene.addLine(0,0,200,0,QPen(QBrush(QColor("red")),20));
	scene.addLine(0,0,0,200,QPen(QBrush(QColor("blue")),20));
	
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


// if(model_people.size()>0)
// 				model_people.erase(std::remove_if(model_people.begin(), model_people.end(), [this](auto &mo_p)
// 				{ if(mo_p.matched==false)
// 					{
// 						scene.removeItem(mo_p.human);
// 						return true;
// 					}
// 				})); 