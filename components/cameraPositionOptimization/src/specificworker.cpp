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
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innermodel = std::make_shared<InnerModel>(innermodel_path);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }



	


	return true;
}



void SpecificWorker::updateCameraPosition(string camera, QVec values)
{
	Rot3DOX crx (-values.rx()); //***********RX inverse SIGN************
	Rot3DOY cry (values.ry());
	Rot3DOZ crz (values.rz());
	QMat crZYX = crz * cry * crx;

	RTMat cam = RTMat();
	cam.setR(crZYX);
	cam.setTr(values.x(), values.y(), values.z());
	cam = cam.invert();

	innermodel->getNode<InnerModelTransform>(camera)->setR(cam.getR());
	innermodel->getNode<InnerModelTransform>(camera)->setTr(cam.getTr());
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	//initialize cameras
	cameras.push_back(QVec::vec6(-88.5111083984, -294.114929199, 3636.78735352, 0.00779854133725, -0.731414854527, 1.56042146683));
	cameras.push_back(QVec::vec6(217.058700562, -190.08354187, 4365.43603516, 0.881269991398, -0.0238653570414, -3.06511044502));
	cameras.push_back(QVec::vec6(-25.1116256714, 288.730499268, 4390.10351562, -0.775234341621, -0.0307027455419, -0.00459992652759));
	updateCameraPosition("cam1Translation", cameras.at(0));
	updateCameraPosition("cam2Translation", cameras.at(1));
	updateCameraPosition("cam3Translation", cameras.at(2));


	//init compute
	this->Period = period;
	timer.start(Period);
}

//if random change do not decrease error
void SpecificWorker::restoreCameraValues()
{
	cameras[cameraChanged] = savedCamera;
	std::string cameraName = "cam" + std::to_string(cameraChanged+1) + "Translation";
	updateCameraPosition(cameraName, savedCamera);
}

//apply random value change to random camera at random position
void SpecificWorker::randomCameraChange()
{
	cameraChanged = randomValue(0, 2); //select camera to update
	savedCamera = cameras.at(cameraChanged); //save actual camera values

	int pos = randomValue(0, 6); //select value to change
	float factor = randomValue(-5, 5)/100.f; // factor to apply
	QVec newValues = savedCamera;
	newValues[pos] += factor * newValues[pos];
	std::string cameraName = "cam" + std::to_string(cameraChanged+1) + "Translation";
	updateCameraPosition(cameraName, newValues);

	cameras.at(cameraChanged) = newValues;
}

int SpecificWorker::randomValue(int min, int max)
{
	std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_int_distribution<> distr(min, max);
	return distr(eng);
}

void SpecificWorker::compute()
{
	int lines = 1;
	float dist_error = 0.f;
	std::ifstream infile("april.txt");
	std::string line;
	while (std::getline(infile, line))
	{ 
		QStringList list = QString::fromStdString(line).split(QRegExp("\\s+"), QString::SkipEmptyParts);
		qDebug()<<"Frame"<<list[0];
		QVec p1 = converToWorld(list[1], list[2].toFloat(), list[3].toFloat(), list[4].toFloat(), list[5].toFloat(), list[6].toFloat(), list[7].toFloat());
		QVec p2 = converToWorld(list[8], list[9].toFloat(), list[10].toFloat(), list[11].toFloat(), list[12].toFloat(), list[13].toFloat(), list[14].toFloat());

		if (list.size() <= 15) // Only two cameras
		{
			float error = euclidean3D_distance(p1, p2);
			qDebug()<< "distance error"<<error;
			dist_error += error;
		}
		else //three cameras
		{
			QVec p3 = converToWorld(list[15], list[16].toFloat(), list[17].toFloat(), list[18].toFloat(), list[19].toFloat(), list[20].toFloat(), list[21].toFloat());
			float d1_2 = euclidean3D_distance(p1, p2);
			float d1_3 = euclidean3D_distance(p1, p3);
			float d2_3 = euclidean3D_distance(p2, p3);
			dist_error += (d1_2 + d1_3 + d2_3)/3.f;
			
		}
		lines++;


	}
	qDebug()<<"lines "<<lines<<" dist_error "<<dist_error/lines;
	exit(0);

}

QVec SpecificWorker::converToWorld(QString camera, float tx, float ty, float tz, float rx, float ry, float rz)
{
	QVec p = QVec::vec6(tx, ty, tz, rx, ry, rz);
	return innermodel->transform("world", p, camera);
}

float SpecificWorker::euclidean3D_distance(const QVec &p1, const QVec &p2)
{
	float xSqr = (p1.x() - p2.x()) * (p1.x() - p2.x());
	float ySqr = (p1.y() - p2.y()) * (p1.y() - p2.y());
	float zSqr = (p1.z() - p2.z()) * (p1.z() - p2.z());
	return sqrt(xSqr + ySqr + zSqr);
}




