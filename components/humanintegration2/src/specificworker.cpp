/*
 *    Copyright (C)2020 by YOUR NAME HERE
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
	human_one.x = 0; human_one.y = 0; human_one.z = 0;
	human_one.angle = 0.0;
	human_one.cameraId = 1;
	QString color;
	if(human_one.cameraId==1) color = "Green";
	if(human_one.cameraId==2) color = "Blue";
	if(human_one.cameraId==3) color = "Red";
	human_one.human = new Human(QRectF(0, 0, 200, 200), QColor(color), QPointF(0, 0), 0, &scene);
	human_one.human->initialize(QPointF(0,0), 0.f);

	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{

	//add one per camera
	for(auto &cam : cameraList)
	{
		if( const auto &[success, observed_people] = cam.pop(); success == true )
		{
			//transformar a coordenadas del mundo y calculo pose
			auto observed_model_people = transformToWorld(observed_people);
			for(const auto &op : observed_model_people)
			{

				//update view
				human_one.human->update(op.x, op.z, degreesToRadians(op.angle));
			}
		}
	}
}

float SpecificWorker::degreesToRadians(const float angle_)
{	
	if (angle_ == 99999)
		return 99999;
	float angle = angle_ * 2*M_PI / 360;
	if(angle > M_PI)
   		return angle - M_PI*2;
	else if(angle < -M_PI)
   		return angle + M_PI*2;
	else return angle;
}

std::tuple<bool, float, float> SpecificWorker::getPosition(std::vector<float> &acum_x, std::vector<float> &acum_z)
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

std::tuple<bool, float> SpecificWorker::getOrientation(const ModelPerson &ob_p)
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
			return std::make_tuple(true, 90-line.angle());
		}
	}
	return std::make_tuple(false, 99999);
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
			wj = innerModel->transform("world", QVec::vec3(key.x, key.y, key.z), "world_camera_" + QString::number(observed_people.cameraId));
			
			acum_x.push_back(wj.x());
			acum_z.push_back(wj.z());
			person.joints[name] = { wj.x(), wj.y(), wj.z(), key.i, key.j, key.score};
		}
	
		if(person.joints.size() > 0)   
		{
			// compute orientation
			auto [success_r, angle_degrees] = getOrientation(person);
			if (not success_r) //using last computed angle
			{
				angle_degrees = last_computed_angle[observed_people.cameraId];
			}
			else{
				last_computed_angle[observed_people.cameraId] = angle_degrees;
			}
			// compute position
			auto [success_p, median_x, median_z] = getPosition(acum_x, acum_z);

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




void SpecificWorker::HumanCameraBody_newPeopleData(PeopleData people)
{
	qDebug()<<"newPeople"<<people.cameraId;
	if(people.cameraId +1 > (int)cameraList.size())
		cameraList.resize(people.cameraId + 1);
	cameraList[people.cameraId].push(people);
	const auto &p = people.peoplelist[0];
	qDebug() << " sub [" << p.x << p.y << p.z << "]";
	if(fabs(p.x< 0.1) and fabs(p.y<0.1) and fabs(p.z<0.1))
		qDebug() << "SHIT ABAJO";

}


