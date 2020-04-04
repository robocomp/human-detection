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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H


#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <queue>
#include <cmath>
// 2D drawing

#include <QDesktopWidget>
#include <QGraphicsScene>
#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <QGraphicsPolygonItem>
#include <QGLWidget>

#include <QJsonObject>
#include <QJsonArray>
#include <cassert>


#include "human.h"

class PythonCall;


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:

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
		uint size()
		{
			std::lock_guard<std::mutex> lock(mymutex);
			return peopledata.size();
		}
	};

	//data type for people in the model
	struct ModelPerson
	{
		struct KeyPoint
		{
			float x;
			float y;
			float z;
			int i;
			int j;
			float score;
		};
		int id;
		qint64 timestamp;
		float x,y,z;
		float angle;
		std::chrono::system_clock::time_point tiempo_no_visible;
		int viewedTimes = 0;
		bool matched = false;
		Human *human;
		bool to_delete = false;
		int cameraId;
		float gtruth_x, gtruth_y, gtruth_z, gtruth_angle;  // ground truth
		std::map<std::string, KeyPoint> joints; 
	};
	using ModelPeople = std::vector<ModelPerson>;
	//data for GNN access
	struct GNNData
	{
		qint64 timestamp;
		int cameraId;
		std::map<std::string, ModelPerson::KeyPoint> joints; 
	};
	using ModelGNN = std::vector<GNNData>;
	std::map<int, ModelGNN> gnnData;
	PythonCall *pythonCall;

	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void update_person(ModelPerson *p_old, ModelPerson p_new);
	void add_people(ModelPeople plist);
	void removePeople();
	int computeDistance(const ModelPerson &p_old, const ModelPerson &p_new);
	void clearMatchedPeople();	
	void HumanCameraBody_newPeopleData(PeopleData people);
	void joinPeople();
	void writeGNNFile(ModelGNN model);

public slots:
	void compute();
	void initialize(int period);
private:
	std::vector<std::string> COCO_IDS{"nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow",
            "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee",
            "left_ankle", "right_ankle"};

	std::shared_ptr<InnerModel> innerModel;
	std::deque<SafeBuffer> cameraList;
	ModelPeople model_people;										// people in the model
	ModelPeople transformToWorld(const RoboCompHumanCameraBody::PeopleData &observed_people);
	RoboCompCommonBehavior::ParameterList params;
	std::tuple<bool, float> getOrientation(const ModelPerson &ob_p);
	std::tuple<bool, float, float> getPosition(std::vector<float> &acum_x, std::vector<float> &acum_z);
	float degreesToRadians(const float angle_);
	std::map<int, float> last_computed_angle;

	// 2D draw
	struct Dimensions 		// Size of the world
	{
		float HMIN = -3000, VMIN = -3000, WIDTH = 6000, HEIGHT = 6000;
	};
	void initializeWorld();
	QGraphicsScene scene;
	Dimensions dimensions;
	std::vector<QGraphicsItem *> boxes; //Obstacles
	int newId = 1;
	const int MINDISTANCE = 1000;  //Distance to assume to person data are the same 
	const int MAXTIME = 2000; //Maximum time elapsed without seen a person before deleted
	const int MINFRAMES = 10;
};

#include "pythonCall.h"

#endif
