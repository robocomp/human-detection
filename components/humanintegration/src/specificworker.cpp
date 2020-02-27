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
	outfile << "]}";
	outfile.close();
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

	resize(QDesktopWidget().availableGeometry(this).size() * 0.6);
	scene.setSceneRect(dimensions.HMIN, dimensions.VMIN, dimensions.WIDTH, dimensions.HEIGHT);
	graphicsView->setViewport(new QGLWidget);
	graphicsView->scale(1, -1);
	graphicsView->setScene(&scene);
	graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio);

	//Load World
	initializeWorld();

	//Create one human
		//wait for a measure to initialize
	// auto res = cameraList[0].pop();
	// while(std::get<bool>(res) == false){ res = cameraList[0].pop(); }
	// auto pe = transformToWorld(std::get<1>(res))[0];
	human_one.x = 0; human_one.y = 0; human_one.z = 0;
	human_one.angle = 0.0;
	human_one.cameraId = 1;
	QString color;
	if(human_one.cameraId==1) color = "Green";
	if(human_one.cameraId==2) color = "Blue";
	if(human_one.cameraId==3) color = "Red";
	human_one.human = new Human(QRectF(0, 0, 200, 200), QColor(color), QPointF(0, 0), 0, &scene);
	human_one.human->initialize(QPointF(0,0), 0.f);

	outfile.open("human_data.txt", std::ios_base::out); // append instead of overwrite
	outfile << "{  " << std::endl << "\"data_set\"" << ":[";

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
			//transformar a coordenadas del mundo y calculo pose
			auto observed_model_people = transformToWorld(observed_people);
			//update human
			for(const auto &op : observed_model_people)
			{
				// write to file
				QJsonObject jsonObject;
				jsonObject["cameraId"] = observed_people.cameraId;
				jsonObject["timestamp"] = observed_people.timestamp;
				QJsonArray gval{ -op.gtruth_y * 1000.f, op.gtruth_z * 1000.f, op.gtruth_x * 1000.f, op.gtruth_angle };
				if(fabs(-op.gtruth_y)< 0.1 and fabs(op.gtruth_z)<0.1 and fabs(op.gtruth_x)<0.1)
				{	
					qDebug() << "SHIT";
					continue;
				}
				jsonObject["ground_truth"] = gval;
				QJsonArray wval{ op.x, op.y, op.z, qDegreesToRadians(op.angle) };
				jsonObject["world"] = wval;

				QJsonObject jsonJoints;
				for(const auto &[name, key] : op.joints)
				{			
					QJsonArray jval{ key.x, key.y, key.z, key.i, key.j, key.score }; 
					jsonJoints[QString::fromStdString(name)] = jval;
				}
				jsonObject["joints"] = jsonJoints;
				QJsonDocument jsonDoc(jsonObject);
				QString strJson(jsonDoc.toJson(QJsonDocument::Compact));
				outfile << strJson.toStdString();
				outfile << ",\n";

				//update view
				std::cout<<"calculado "<<degreesToRadians(op.angle)<<" simulador "<<op.gtruth_angle<<std::endl;
				human_one.human->update(op.x, op.z, degreesToRadians(op.angle));
				//human_one.human->update(-op.gtruth_y * 1000.f, op.gtruth_x * 1000.f, op.angle);
			}
		}
	}
}

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
			return std::make_tuple(true, line.angle()+90);
		}
	}
	return std::make_tuple(false, 0.0);
}

float SpecificWorker::degreesToRadians(const float angle_)
{	
	float angle = angle_ * 2*M_PI / 360;
	if(angle > M_PI)
   		return angle - M_PI*2;
	else if(angle < -M_PI)
   		return angle + M_PI*2;
	else return angle;
}

std::tuple<bool, float, float> SpecificWorker::getPosition(std::vector<float> &acum_x, std::vector<float> &acum_z, const RoboCompHumanCameraBody::Person &ob_p)
{
	//QVec acum = QVec::zeros(3);
	// compute position as the median of projected joints coordinates on the floor
	if(acum_x.size() == 0 or acum_z.size() == 0)
			return std::make_tuple(false, 0.0, 0.0);

	size_t n = acum_x.size() / 2;
    std::nth_element(acum_x.begin(), acum_x.begin() + n, acum_x.end());
	std::nth_element(acum_z.begin(), acum_z.begin() + n, acum_z.end());
    float median_x = acum_x[n];	
	float median_z = acum_z[n];	
	//acum = acum/(T)obs_person.joints.size();
	return std::make_tuple(true, median_x, median_z);
}

SpecificWorker::ModelPeople SpecificWorker::transformToWorld(const RoboCompHumanCameraBody::PeopleData &observed_people)
{
	ModelPeople res;
	for(const auto &obs_person : observed_people.peoplelist)
	{
		//QVec left_s, right_s, wj;
	
		std::vector<float> acum_x, acum_z;
		QVec wj;
		ModelPerson person;
		for(const auto &[name, key] : obs_person.joints)
		{
			//qDebug() << "Trans [" << key.x << key.y << key.z << "]";
			wj = innerModel->transform("world", QVec::vec3(key.x*1000, key.y*1000, key.z*1000), "world_camera_" + QString::number(observed_people.cameraId));
			//wj = innerModel->transform("world", QVec::vec3(-key.y*1000.f, key.z*1000.f, key.x*1000.f), "world_camera_" + QString::number(observed_people.cameraId));
			
			acum_x.push_back(wj.x());
			acum_z.push_back(wj.z());
			person.joints[name] = { wj.x(), wj.y(), wj.z(), key.i, key.j, key.score};
		}
	
		if(obs_person.joints.size() > 0)   
		{
			// compute orientation
			auto [success_r, angle_degrees] = getOrientation(obs_person);
			// compute position
			auto [success_p, median_x, median_z] = getPosition(acum_x, acum_z, obs_person);

			person.id = obs_person.id;
			person.x = median_x;
			person.y = 0; 
			person.z = median_z;
			person.angle = angle_degrees; 
			person.tiempo_no_visible = std::chrono::time_point<std::chrono::system_clock>();
			person.matched = false; 
			person.human = nullptr; 
			person.to_delete = false;
			person.cameraId = observed_people.cameraId;
			person.gtruth_x = obs_person.x;
			person.gtruth_y = obs_person.y;
			person.gtruth_z = obs_person.z;
			person.gtruth_angle = obs_person.rz;
		}
		res.push_back( person ); 
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
		auto box = scene.addEllipse(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Khaki")), QBrush(QColor("Khaki")));
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
	QTransform t;
  	//t.translate(3200, -1850);
  	t.rotate(-90);
 	//t.translate(-3200, 1850);
	for(auto &item : boxes)
	{	
    	item->setPos(t.map(item->pos()));
    	item->setRotation(item->rotation() + 90);
		item->setPos(item->pos() + QPointF(1850,3200));
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
	const auto &p = people.peoplelist[0];
	//qDebug() << " sub [" << p.x << p.y << p.z << "]";
	if(fabs(p.x< 0.1) and fabs(p.y<0.1) and fabs(p.z<0.1))
		qDebug() << "SHIT ABAJO";
				
}


// if(model_people.size()>0)
// 				model_people.erase(std::remove_if(model_people.begin(), model_people.end(), [this](auto &mo_p)
// 				{ if(mo_p.matched==false)
// 					{
// 						scene.removeItem(mo_p.human);
// 						return true;
// 					}
// 				})); 















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


	//Delete camera people
			// model_people.erase(std::remove_if(model_people.begin(), model_people.end(), [this, observed_people](auto &mp) 
			// 	{ 
			// 		if( observed_people.cameraId == mp.cameraId)
			// 		{
			// 			scene.removeItem(mp.human);
			// 			delete mp.human;
			// 			return true;
			// 		}
			// 		else return false;
			// 	}), model_people.end());


			// añadir a cámara
			// for(const auto &mo_p : observed_model_people)
			// {
			// 	ModelPerson mp;
			// 	mp.x=mo_p.x; mp.y=mo_p.y; mp.z=mo_p.z;
			// 	mp.angle = mo_p.angle;
			// 	mp.cameraId = observed_people.cameraId;
			// 	QString color;
			// 	if(observed_people.cameraId==1) color = "Green";
			// 	if(observed_people.cameraId==2) color = "Blue";
			// 	if(observed_people.cameraId==3) color = "Red";
			// 	mp.human = new Human(QRectF(0, 0, 200, 200), QColor(color), QPointF(mp.x, mp.z), mp.angle, &scene);
			// 	model_people.push_back(mp);
			// }


				// Only left_shoulder
		// if(obs_person.joints.count("left_shoulder") > 0)   //test with one shoulder
		// {
		// 	auto key = obs_person.joints.at("left_shoulder");
		// 	wj = innerModel->transform("world", QVec::vec3(key.x, key.y, key.z), "world_camera_" + QString::number(observed_people.cameraId));
		// 	wj.print("wj");
		// 	res.push_back( { obs_person.id, wj.x(), wj.y(), wj.z(), 0.0, std::chrono::time_point<std::chrono::system_clock>(), false, nullptr, false} );
		// 	break;
		// }