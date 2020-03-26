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
	struct cameraPose
	{
		QGraphicsEllipseItem *ellipse;
		QGraphicsTextItem *text;
	};

	Q_OBJECT
	public:
		Human(int id, int ncameras, const QRectF &r, QPointF pos, float angle, QGraphicsScene *scene_);  
		~Human();
		void update(int cameraID, float x, float y, float ang);
	private:
	    QList<QString> colors = {"red", "green", "blue", "yellow", "orange"};
		QGraphicsPixmapItem* pixmapItem;
		QList<cameraPose> cameraPose_list;
		QGraphicsPolygonItem *polygon_item = nullptr;
		QColor color;
		QGraphicsScene *scene;
		QGraphicsEllipseItem *ellipseItem;
		QPixmap pixmap;
		int ellipseHalfSizeX;
		int ellipseHalfSizeY;
		int pixmapHalfSizeX;
		int pixmapHalfSizeY;
		int id;
};

#endif // HUMAN_H
