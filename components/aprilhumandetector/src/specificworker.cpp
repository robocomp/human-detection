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
	for(int i=0;i < ncameras; i++)
		videoCapture[i].release();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	config_params = params;
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innermodel = std::make_shared<InnerModel>(innermodel_path);
		ncameras = QString::fromStdString(params["numCameras"].value).toInt();
	}
	catch(std::exception e) { qFatal("Error reading config params"); }

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	////april
	aprilImage.frmt.width = QString::fromStdString(config_params["width"].value).toInt();
	aprilImage.frmt.height = QString::fromStdString(config_params["height"].value).toInt();
	aprilImage.data.resize(aprilImage.frmt.width * aprilImage.frmt.height *3 );

	//initialize cameras
	updateCameraPosition("cam1Translation", QVec::vec6(-88.5111083984, -294.114929199, 3636.78735352, 0.00779854133725, -0.731414854527, 1.56042146683));
	updateCameraPosition("cam2Translation", QVec::vec6(217.058700562, -190.08354187, 4365.43603516, 0.881269991398, -0.0238653570414, -3.06511044502));
	updateCameraPosition("cam3Translation", QVec::vec6(-25.1116256714, 288.730499268, 4390.10351562, -0.775234341621, -0.0307027455419, -0.00459992652759));
	initVideo();

	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::initVideo()
{
	for(int i=0;i < ncameras; i++)
	{
		std::string s = QString::number(i).toStdString();
		std::string source = config_params["camera.Params_" + s +".source"].value;
		cv::VideoCapture cam = cv::VideoCapture(source);
		cv::Mat frame;
		frames.push_back(frame);
		RoboCompAprilTagsServer::tagsList people;
		peopleList.push_back(people);
		if (cam.isOpened() == false)
		{
			std::cout<<"Error opening camera: "<< source << " check camera and config file" << std::endl;
			exit(-1);
		}
		cam.set(CV_CAP_PROP_BUFFERSIZE, 0);
		videoCapture.push_back(cam);
	}
	timeStamp.resize(ncameras);

}

void SpecificWorker::compute()
{

	//get frames
	for(int i=0; i<ncameras; i++)
	{
		readFrame(i, frames[i]);
	}
	//compute april
	for(int i=0;i< ncameras; i++)
	{
		computeAprilPosition(frames[i], i);
		cv::imshow(std::to_string(i), frames[i]);
	}
	//publish to human
	for(int i=0;i<ncameras; i++)
	{
		RoboCompHumanPose::personList people;
		for (auto tag: peopleList[i])
		{
			RoboCompHumanPose::PersonType person;
			person.id = tag.id;
			person.pos.x = tag.tx;
			person.pos.z = tag.tz;
			people.push_back(person);
		}
		try
		{
			RoboCompHumanPose::humansDetected humanD;
			humanD.idCamera = i;
			humanD.timeStamp = timeStamp[i];
			humanD.humanList = people;
			humanpose_pubproxy->obtainHumanPose(humanD);
		}
		catch(const Ice::Exception& e)
		{
			std::cerr << e.what() << std::endl;
		}
	}
}



void SpecificWorker::readFrame(int camera, cv::Mat &frame)
{
	auto t0 = std::chrono::high_resolution_clock::now();
	try{
		videoCapture[camera].grab();
		videoCapture[camera].retrieve(frame);

		timeStamp[camera] =  t0.time_since_epoch() / std::chrono::milliseconds(1);
	}catch(...)
	{
		std::cout<<"Error reading frame from camera: "<<camera<<std::endl;
	}
}

QVec SpecificWorker::updateCameraPosition(string camera, QVec values)
{
	Rot3DOX crx (values.rx());
	Rot3DOY cry (values.ry());
	Rot3DOZ crz (values.rz());
	QMat crZYX = crz * cry * crx;

	RTMat cam = RTMat();
	cam.setR(crZYX);
	cam.setTr(values.x(), values.y(), values.z());
	cam = cam.invert();

	innermodel->getNode<InnerModelTransform>(camera)->setR(cam.getR());
	innermodel->getNode<InnerModelTransform>(camera)->setTr(cam.getTr());
	innermodel->cleanupTables();

	return innermodel->transformS6D("world", camera);		
}


RoboCompAprilTagsServer::tagsList SpecificWorker::computeAprilPosition(cv::Mat frame, int id_camera)
{
	std::string s = QString::number(id_camera).toStdString();
	std::string camera = config_params["camera.Params_" + s +".name"].value;
	try
	{
		memcpy(&aprilImage.data[0], &frame.data[0], aprilImage.frmt.width * aprilImage.frmt.height*3);
		peopleList[id_camera] = apriltagsserver_proxy->getAprilTags(aprilImage, 350, 1000, 1000);

		if(peopleList[id_camera].size() > 0)
		{
			for (auto tag: peopleList[id_camera])
			{
				QVec p = QVec::vec3(tag.tx, tag.ty, tag.tz);
				QVec w = innermodel->transform("world", p, QString::fromStdString(camera));

				std::cout<<"Position "<<tag.tx <<";"<< tag.ty << ";" << tag.tz <<";"<<std::endl;
				std::cout<<"World "<< w.x() <<";"<< w.y() << ";" << w.z() <<";"<<std::endl;
				//update position
				tag.tx = w.x();
				tag.ty = w.y();
				tag.tz = w.z();
			}
		}

	}
	catch(const Ice::Exception& e)
	{
		std::cerr <<"Error connecting to ArpilTags: "<< e.what() << std::endl;
	}
	return peopleList[id_camera];
}