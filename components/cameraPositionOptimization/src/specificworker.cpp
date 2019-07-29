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

void SpecificWorker::setCameraPositions()
{
	/*CAM1*/
	Rot3DOX c1rx (-0.00779854133725); //***********RX inverse SIGN************
	Rot3DOY c1ry (-0.731414854527);
	Rot3DOZ c1rz (1.56042146683);
	QMat c1rZYX = c1rz * c1ry * c1rx;

	cam1 = RTMat();
	cam1.setR(c1rZYX);
	cam1.setTr(-88.5111083984, -294.114929199, 3636.78735352);
	cam1 = cam1.invert();

	innermodel->getNode("cam1Translation")->setR(cam1.getR());
	innermodel->getNode("cam1Translation")->setTr(cam1.getTr());

	/*CAM2*/
	Rot3DOX c2rx (-0.881269991398); //***********RX inverse SIGN************
	Rot3DOY c2ry (-0.0238653570414);
	Rot3DOZ c2rz (-3.06511044502);
	QMat c2rZYX = c2rz * c2ry *c2rx;

	cam2 = RTMat();
	cam2.setR(c2rZYX);
	cam2.setTr(217.058700562, -190.08354187, 4365.43603516);
	cam2 = cam2.invert();

	innermodel->getNode("cam2Translation")->setR(cam2.getR());
	innermodel->getNode("cam2Translation")->setTr(cam2.getTr());

	/*CAM3*/
	Rot3DOX c3rx (0.775234341621); //***********RX inverse SIGN************
	Rot3DOY c3ry (-0.0307027455419);
	Rot3DOZ c3rz (-0.00459992652759);
	QMat c3rZYX = c3rz * c3ry *c3rx;

	cam3 = RTMat();
	cam3.setR(c3rZYX);
	cam3.setTr(-25.1116256714, 288.730499268, 4390.10351562);
	cam3 = cam3.invert();

	innermodel->getNode("cam3Translation")->setR(cam3.getR());
	innermodel->getNode("cam3Translation")->setTr(cam3.getTr());
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	setCameraPositions();
	this->Period = period;
	timer.start(Period);
	
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




