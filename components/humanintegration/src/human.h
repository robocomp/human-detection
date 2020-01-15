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


class Human : public QObject, public QGraphicsEllipseItem
{     
	Q_OBJECT
	public:
		Human(const QRectF &r, QColor color_, QPointF pos);  
		void setPolygon(QGraphicsPolygonItem *poly)				{ polygon_item = poly; }
		QGraphicsPolygonItem * getPolygon() const				{ return polygon_item;}
		void updatePolygon(QPolygonF poly);
		qreal rotation() const;
	private:
		QGraphicsPixmapItem* pixmapItem;
		Qt::MouseButton mouseButton;
		QGraphicsEllipseItem *ellipseItem;
		float degreesToRadians(const float angle);
		QGraphicsPolygonItem *polygon_item = nullptr;
		QColor color;
		
};

#endif // HUMAN_H
