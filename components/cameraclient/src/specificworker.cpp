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
	if(!cam.empty())
	{
		cv::Mat &frame = cam.front();	
		pMOG2->apply(frame, fgMaskMOG2);
        cv::erode(fgMaskMOG2, erode, kernel);
        cv::dilate(erode, dilate, kernel);
		qDebug() << cv::countNonZero(dilate);
		cv::imshow("Camara sala de reuniones", erode);
		//qDebug() << "compute" << frame.rows << frame.cols;
        cvWaitKey(1);
		cam.pop();
	}
}

void SpecificWorker::CameraSimple_getImage(TImage &im)
{
//implementCODE

}


