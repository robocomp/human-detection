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
#include <chrono>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{
	skeleton = SKELETON_CONNECTIONS({
		{"left_ankle","left_knee"},
		{"left_knee","left_hip"},
		{"right_ankle", "right_knee"},
		{"right_knee", "right_hip"},
		{"left_hip", "right_hip"},
		{"left_shoulder", "left_hip"},
		{"right_shoulder", "right_hip"},
		{"left_shoulder", "right_shoulder"},
		{"left_shoulder", "left_elbow"},
		{"right_shoulder", "right_elbow"},
		{"left_elbow", "left_wrist"},
		{"right_elbow", "right_wrist"},		
		{"left_eye", "right_eye"},		
		{"nose", "left_eye"},
		{"nose", "right_eye"},
		{"left_eye", "left_ear"},
		{"right_eye", "right_ear"},
		{"left_ear", "left_shoulder"},
		{"right_ear", "right_shoulder"}});
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	camcv.release();
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	pMOG2 = cv::createBackgroundSubtractorMOG2();
	size_t erosion_size = 2;
	kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, 
										cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                        cv::Point( erosion_size, erosion_size ) );
	
	//cam.run();
	camcv.open(0);

	this->Period = 50;
	//timer.setSingleShot(true);
	timer.start(Period);
	
}

void SpecificWorker::compute()
{
	//auto [ret, frame] = cam.read();	//access without copy
	cv::Mat frame; bool ret = true;
	camcv >> frame;  

	if(ret == false) return;
	pMOG2->apply(frame, fgMaskMOG2);
	//cv::erode(fgMaskMOG2, erode, kernel);
	//cv::dilate(erode, dilate, kernel);
	cv::countNonZero(fgMaskMOG2);
	// if count > TH 
	RoboCompPeopleServer::TImage img;
	img.image.assign(frame.data, frame.data + (frame.total() * frame.elemSize()));
	img.width = frame.cols;
	img.height = frame.rows;
	img.depth = 3;
	try
	{
		scale = 0.5;
		auto people = peopleserver_proxy->processImage(img, scale);
		drawBody(frame, people);
		//	go from feet upwards
		//		compute floor position
		// publish results
	}
	catch(const Ice::Exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	
	
//	cv::imshow("Camara sala de reuniones", frame);
	//qDebug() << "compute" << frame.rows << frame.cols;
	cvWaitKey(1);
}

void SpecificWorker::drawBody(cv::Mat frame, const RoboCompPeopleServer::People &people)
{
	for(auto &p : people)
	{
		qDebug()<<"person"<<p.id;
		for (auto &connection: skeleton)
		{
			auto j1 = &p.joints.at(std::get<0>(connection));
			auto j2 = &p.joints.at(std::get<1>(connection));
			if (j1->score > 0)
				cv::circle(frame,cv::Point(j1->x/scale, j1->y/scale),10,cv::Scalar(0,0,255));
			if (j2->score > 0)
				cv::circle(frame,cv::Point(j2->x/scale, j2->y/scale),10,cv::Scalar(0,0,255));
			if (j1->score > 0 and j2->score > 0)
				cv::line(frame, cv::Point(j1->x/scale, j1->y/scale), cv::Point(j2->x/scale, j2->y/scale), cv::Scalar(0,255,0), 2);
		}
	}
	cv::imshow("Camara IP", frame);
	cvWaitKey(1);
}


void SpecificWorker::CameraSimple_getImage(RoboCompCameraSimple::TImage &im)
{
//implementCODE

}


