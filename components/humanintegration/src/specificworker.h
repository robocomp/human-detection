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
#include <queue>
// 2D drawing
#include <QGraphicsScene>
#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <QGraphicsPolygonItem>
#include <QGLWidget>
#include "human.h"

#define NCAMERAS 2
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void HumanCameraBody_newPeopleData(PeopleData people);

	//Interchange thread safe buffer
	struct SafeBuffer
	{
		std::queue<RoboCompHumanCameraBody::PeopleData> peopledata;
		mutable std::mutex mymutex; 

		void push(const PeopleData &detected_people)
		{
			std::lock_guard<std::mutex> lock(mymutex);
			peopledata.push(detected_people);
		}
		std::tuple<bool,PeopleData> pop()
		{
			std::lock_guard<std::mutex> lock(mymutex);
			if(peopledata.empty() == true)
				return std::make_tuple(false, PeopleData());
			auto detected_people = peopledata.front();
			peopledata.pop();
			return std::make_tuple(true, detected_people);
		}
	};
	
	//data type for people in the model
	struct ModelPerson
	{
		int id;
		float x,y,z;
		float angle;
		std::chrono::time_point<std::chrono::system_clock> tiempo_no_visible;
		bool matched = false;
		Human *human;
		bool to_delete = false;
		int cameraId;
	};


public slots:
	void compute();
	void initialize(int period);
	//Specification slot methods State Machine
	void sm_compute();
	void sm_initialize();
	void sm_finalize();

//--------------------
private:
	const int MAX_AUSENTE = 2; //secs
	std::vector<std::string> COCO_IDS{"nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow",
            "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee",
            "left_ankle", "right_ankle"};

	std::shared_ptr<InnerModel> innerModel;
	std::deque<SafeBuffer> cameraList;
	using ModelPeople = std::vector<ModelPerson>;
	ModelPeople model_people;										// people in the model
	ModelPeople transformToWorld(const RoboCompHumanCameraBody::PeopleData &observed_people);
	RoboCompCommonBehavior::ParameterList params;
	std::tuple<bool, float> getOrientation(const RoboCompHumanCameraBody::Person &ob_p);

	// 2D draw
	struct Dimensions 		// Size of the world
	{
		float HMIN = -3000, VMIN = -3000, WIDTH = 6000, HEIGHT = 6000;
	};
	void initializeWorld();
	QGraphicsScene scene;
	Dimensions dimensions;
	std::vector<QGraphicsItem *> boxes; //Obstacles

};

#endif
