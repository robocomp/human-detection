/*
 * Copyright 2018 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef HUMAN_H
#define HUMAN_H

#include <QGraphicsItem>
#include <QGraphicsScene>
#include <random>
#include <chrono>

//kalman
#include "SystemModel.hpp"
#include "OrientationMeasurementModel.hpp"
#include "PositionMeasurementModel.hpp"
#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>
using namespace KalmanExamples;
typedef float T;
// Some type shortcuts
typedef Robot1::State<T> State;
typedef Robot1::Control<T> Control;
typedef Robot1::SystemModel<T> SystemModel;
typedef Robot1::PositionMeasurement<T> PositionMeasurement;
typedef Robot1::OrientationMeasurement<T> OrientationMeasurement;
typedef Robot1::PositionMeasurementModel<T> PositionModel;
typedef Robot1::OrientationMeasurementModel<T> OrientationModel;

class Human : public QObject, public QGraphicsEllipseItem
{     
	Q_OBJECT
	public:
		Human(const QRectF &r, QColor color_, QPointF pos, float angle, QGraphicsScene *scene_);  
		~Human();
		void initialize(const QPointF &pos, float ang);
		void update(float x, float y, float ang);

	private:
		QGraphicsPixmapItem* pixmapItem;
		QGraphicsEllipseItem *ellipseItem;
		float degreesToRadians(const float angle);
		QGraphicsPolygonItem *polygon_item = nullptr;
		QColor color;
		QGraphicsScene *scene;

		//kalman
		 Robot1::State<T> x, x_ekf;
		Control u;
		SystemModel sys;
		PositionModel pm;
		OrientationModel om;
		Kalman::ExtendedKalmanFilter<State> predictor;
		Kalman::ExtendedKalmanFilter<State> ekf;
		Kalman::UnscentedKalmanFilter<State> ukf;
		

		// Random number generation (for noise simulation)
    	std::default_random_engine generator;
		std::normal_distribution<float> noise;
};

#endif // HUMAN_H
