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
#include <chrono>
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
			CostFunctor(std::shared_ptr<InnerModel> innermodel, 
						std::map<std::string, std::tuple<std::string, QVec, double * >> cameras_map,
						const std::string camera_A, const QVec &mark_A, const std::string camera_B, const QVec &mark_B) 
						: innermodel(innermodel), cameras_map(cameras_map), camera_A(camera_A), mark_A(mark_A), camera_B(camera_B), mark_B(mark_B) {}
			bool operator()(const double* const mut_cam_A, const double* const mut_cam_B, double* residuals) const 
			//bool operator()(const double* const mut_cam_A, double* residuals) const 
			{
				const double *pA = mut_cam_A; 
				const double *pB = mut_cam_B;
				//const double *pB = std::get<double *>(cameras_map.at(camera_B));
				innermodel->updateTransformValuesS(std::get<std::string>(cameras_map.at(camera_A)), 
								pA[0]*1000., pA[1]*1000., pA[2]*1000., pA[3], pA[4], pA[5]);
				innermodel->updateTransformValuesS(std::get<std::string>(cameras_map.at(camera_B)), 
								pB[0]*1000., pB[1]*1000., pB[2]*1000., pB[3], pB[4], pB[5]);
				QVec rA = innermodel->transformS("world", mark_A, camera_A);
				QVec rB = innermodel->transformS("world", mark_B, camera_B);
				QVec res = (rA-rB);

				residuals[0] = res[0]/1000.;
				residuals[1] = res[1]/1000.;
				residuals[2] = res[2]/1000.;
				
				return true;
			}
			std::shared_ptr<InnerModel> innermodel;
			std::map<std::string, std::tuple<std::string, QVec, double * >> cameras_map;
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
	std::map<std::string, std::tuple<std::string, QVec, double * >> cameras_map; //cam : {t_cam, mutable_camera_for_CERES }
	int cameraChanged;
	QVec savedCamera;
	std::vector<QVec> cameras;

};

#endif
