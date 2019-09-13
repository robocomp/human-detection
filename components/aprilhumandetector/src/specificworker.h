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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <opencv2/opencv.hpp>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void initVideo();
	void readFrame(int camera, cv::Mat &frame);
	RoboCompAprilTagsServer::tagsList computeAprilPosition(cv::Mat frame, int id_camera);
	QVec updateCameraPosition(string camera, QVec values);

public slots:
	void compute();
	void initialize(int period);
private:
	RoboCompCommonBehavior::ParameterList config_params;
	std::shared_ptr<InnerModel> innermodel;
	std::vector<cv::VideoCapture> videoCapture;
	std::vector<RoboCompAprilTagsServer::tagsList> peopleList;
	std::vector<long> timeStamp;
	std::vector<cv::Mat> frames;
	int ncameras = 0;
	RoboCompAprilTagsServer::Image aprilImage;
};

#endif
