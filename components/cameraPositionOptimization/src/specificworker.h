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

#include <fstream>
#include <iostream>
#include <random>

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
private:
	int cameraChanged;
	QVec savedCamera;
	std::vector<QVec> cameras;
	float lambda, nu;
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	QVec converToWorld(QString camera, float tx, float ty, float tz, float rx, float ry, float rz);
	void updateCameraPosition(string camera, QVec values);
	void randomCameraChange();

	int randomValue(int min, int max);
	void restoreCameraValues();
	float euclidean3D_distance(const QVec &p1, const QVec &p2);
	float compute_distance(const QVec &p1, const QVec &p2);

public slots:
	void compute();
	void initialize(int period);
private:
	std::shared_ptr<InnerModel> innermodel;

};

#endif
