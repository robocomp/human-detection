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
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
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
		auto [success, peopledata] = cam.pop();
		if(success)	 // hay algo en la cola
		{
			//transformo a coordenadas del mundo y calculo pose
			auto observed_world_people = transformToWorld(peopledata);
			// para todas las pers. observadas por esta cámara ...
			for(const auto &ob_p : observed_world_people)
			{
				std::for_each(std::begin(personList), std::end(personList),[ob_p](auto &rp) 
				{  
					if((QVec::vec3(rp.x,rp.y,rp.z) - QVec::vec3(ob_p.x,ob_p.y,ob_p.z)).norm2())
						qDebug() << "cool";
				});
			}
		}
	}
	// añadir

	//borrado

}

SpecificWorker::RealPeople SpecificWorker::transformToWorld(const RoboCompHumanCameraBody::PeopleData &peopledata)
{
	RealPeople res;
innerModel->transform("world", QVec::vec3(0,0,2000), "wall_camera_1").print("prueba");
	for(const auto &obs_person : peopledata.peoplelist)
	{
		QVec left_s, right_s;
		for(const auto &[name, key] : obs_person.joints)
		{
			//qDebug() << "claves" << QString::fromStdString(name);
	//		QVec wj = innerModel->transform("world", QVec::vec3(key.x, key.y, key.z), "wall_camera_" + QString::number(peopledata.cameraId));
			QVec wj = innerModel->transform("world", QVec::vec3(1000.*key.x, 1000.*key.y, 1000.*key.z), "wall_camera_1");
			if(name=="right_shoulder")
				left_s = wj;
			if(name=="left_shoulder")
				right_s = wj;
		}
		((left_s + right_s)/(T)2.).print("media");
		//RealPerson rp = { wj.x(), wj.y(), wj.z(), 0.0, 0};
		//res.push_back(rp);
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





void SpecificWorker::HumanCameraBody_newPeopleData(PeopleData people)
{
	if(people.cameraId +1 > (int)cameraList.size())
		cameraList.resize(people.cameraId + 1);
	
	cameraList[people.cameraId].push(people);

	qDebug() << "Inserting in " << people.cameraId << people.peoplelist.size();
}

