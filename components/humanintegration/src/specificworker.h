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

#define NCAMERAS 2

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void HumanCameraBody_newPeopleData(PeopleData people);


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
	
	struct RealPerson
	{
		float x,y,z;
		float angle;
		long tiempo_no_visible;
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
	std::shared_ptr<InnerModel> innerModel;
	std::deque<SafeBuffer> cameraList;
	std::vector<RealPerson> personList;
	using RealPeople = std::vector<SpecificWorker::RealPerson>;
	RealPeople transformToWorld(const RoboCompHumanCameraBody::PeopleData &peopledata);
};

#endif
