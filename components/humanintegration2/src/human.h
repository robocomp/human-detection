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
#include <QtCore>
#include <random>
#include <chrono>


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
		QGraphicsPolygonItem *polygon_item = nullptr;
		QColor color;
		QGraphicsScene *scene;

};

#endif // HUMAN_H
