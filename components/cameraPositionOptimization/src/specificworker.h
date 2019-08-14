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

#include <cppitertools/itertools.hpp>
#include <fstream>
#include <iostream>
#include <random>
#include "ceres/ceres.h"
#include <genericworker.h>
#include <innermodel/innermodel.h>


using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class SpecificWorker : public GenericWorker
{
Q_OBJECT

public:

	struct CostFunctor 
		{
			CostFunctor(std::shared_ptr<InnerModel> innermodel, const std::string camera_A, const QVec &mark_A, const std::string camera_B, const QVec &mark_B) 
			: innermodel(innermodel), camera_A(camera_A), mark_A(mark_A), camera_B(camera_B), mark_B(mark_B) {}
			bool operator()(const double* const mut_cam_A, const double* const mut_cam_B, double* residuals) const 
			{
				// is rescaled in the copy by *1000
				updateCameraPosition(camera_A, mut_cam_A);
				updateCameraPosition(camera_B, mut_cam_B);
				QVec pA = innermodel->transformS("world", mark_A, camera_A);
				QVec pB = innermodel->transformS("world", mark_B, camera_B);

				QVec res = pA-pB;
				residuals[0] = res[0];
				residuals[1] = res[1];
				residuals[2] = res[2];

				return true;
			}
			void updateCameraPosition(const std::string camera, const double* const c) const
			{
				std::string tcam;
				if(camera == "camera1") tcam = "cam1Translation";
				if(camera == "camera2") tcam = "cam2Translation";
				if(camera == "camera3") tcam = "cam3Translation";
				innermodel->updateTransformValuesS(tcam, c[0], c[1], c[2], c[3], c[4], c[5]);
				innermodel->cleanupTables();
			}
			std::shared_ptr<InnerModel> innermodel;
			std::string camera_A;
			QVec mark_A;
			std::string camera_B;
			QVec mark_B;
			
		};


		SpecificWorker(TuplePrx tprx);
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		QVec converToWorld(QString camera, float tx, float ty, float tz, float rx, float ry, float rz);
		QVec updateCameraPosition(string camera, QVec values);
		void randomCameraChange();
		int randomValue(int min, int max);
		void restoreCameraValues();
		//float compute_distance(const QVec &p1, const QVec &p2);

public slots:
	void compute();
	void initialize(int period);
private:
	std::shared_ptr<InnerModel> innermodel;
	void createList();
	std::list<std::tuple<std::string, QVec, std::string, QVec>> measurements; //camA, mut indexA markA, camB, mut_indexB, markB
	std::vector<std::tuple<QString, int>> camera_list;
	std::vector<QVec> mark_list;
	std::vector<double *> mutable_marks;
	std::vector<double *> mutable_cameras;
	std::vector<std::string> t_camera_names, camera_names;
	std::map<std::string, std::tuple<std::string, QVec, double * >> cameras_map; //cam : {t_cam, mutable_camera_for_CERES }
	int cameraChanged;
	QVec savedCamera;
	std::vector<QVec> cameras;

};

#endif
