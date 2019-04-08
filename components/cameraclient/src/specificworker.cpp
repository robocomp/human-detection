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
	cam.run();

	this->Period = 50;
	//timer.setSingleShot(true);
	timer.start(Period);
	
}

void SpecificWorker::compute()
{
	//qDebug() << "compute";
	auto [ret, frame] = cam.read();	//access without copy
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
		auto people = peopleserver_proxy->processImage(img, 0.5);
		//	go from feet upwards
		//		compute floor position
		// publish results
		for (auto person: people)
		{
			
		}
	}
	catch(const Ice::Exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	
	
	cv::imshow("Camara sala de reuniones", frame);
	//qDebug() << "compute" << frame.rows << frame.cols;
	cvWaitKey(1);
}

void SpecificWorker::CameraSimple_getImage(RoboCompCameraSimple::TImage &im)
{
//implementCODE

}


