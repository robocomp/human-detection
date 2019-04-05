import json
import os
import random
import signal
import sys

from PySide2.QtCore import QFile, QIODevice, QJsonDocument, QRectF, Qt, QPointF
from PySide2.QtGui import QBrush, QColor, QPen, QFont, QPainterPath, QPolygonF
from PySide2.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QGraphicsTextItem, \
	QGraphicsPolygonItem, QGraphicsPathItem
from numpy import diff
from pyside2uic.properties import QtCore

CURRENT_FILE_PATH = os.path.dirname(__file__)

class HumanCircle(QGraphicsEllipseItem):
	def __init__(self,x,y, height, width, human_id, parent=None):
		super(HumanCircle, self).__init__(x,y, height, width, parent)
		self._id = human_id

	def paint(self, painter, option, widget):
		super(HumanCircle, self).paint(painter, option, widget)
		painter.drawText(painter, Qt.AlignHCenter | Qt.AlignVCenter, self._id)


	def drawText(self, painter, flags, text, boundingRect =0):
		size = 32767.0
		x, y = self.boundingRect().center()
		corner = QPointF(x, y - size)
		if flags & Qt.AlignHCenter:
			corner.setX(corner.x() -size / 2.0)
		elif flags & Qt.AlignRight:
			corner.setX(corner.x() - size)
		if flags & Qt.AlignVCenter:
			corner.setY(corner.y() + size / 2.0)
		elif flags & Qt.AlignTop:
			corner.setY(corner.y() + size)
		else:
			flags |= Qt.AlignBottom
		rect = QRectF(corner.x(), corner.y(), size, size)
		pen = painter.setPen(painter.brush().color().inverse())
		painter.save()
		painter.drawText(rect, flags, text, boundingRect)
		painter.restore()



class HumanVisualizationWidget(QGraphicsView):
	def __init__(self, parent=None):
		super(HumanVisualizationWidget, self).__init__(parent)
		self._scene = QGraphicsScene()
		self.setScene(self._scene)
		# circle = QGraphicsEllipseItem( 10, 10, 10 ,10)
		# self._scene.addItem(circle)
		self._boxes = []
		self._humans = {}

	def load_json_world(self, file):

		if not os.path.isfile(file):
			print("Error reading world file, check config params:", file)
			return False

		with open(file, "r") as read_file:
			json_data = json.load(read_file)

		types_colors = {
			"tables": "SandyBrown",
			"roundTables": "Khaki",
			"walls": "Brown",
			"points": "Blue"}

		for type, color in types_colors.items():
			if type in json_data:
				tables = json_data[type]
				for object in tables.values():
					rect = QRectF(-float(object[2]) / 2, -float(object[3]) / 2, float(object[2]), float(object[3]))
					border = QPen(QColor(color))
					fill = QBrush(QColor(color))
					box = self._scene.addRect(rect, border, fill)
					box.setPos(float(object[4]), float(object[5]))
					box.setRotation(float(object[6]))
					self._boxes.append(box)
		self.clear()
		for item in self._boxes:
			self._scene.addItem(item)

	def load_json_world(self, file):

		if not os.path.isfile(file):
			print("Error reading world file, check config params:", file)
			return False

		with open(file, "r") as read_file:
			json_data = json.load(read_file)

			polygon_points = []
		paths_count = 0
		for item in json_data:
			if 'json_geometry' in item:
				geometry = item['json_geometry']
				if geometry['type'] == 'Polygon':
					for coord in geometry['coordinates'][0]:

						if isinstance(coord, list) and (
								(isinstance(coord, list) and len(coord) == 2) or (len(coord) == 3 and coord[3] == 0)):
							current_point = QPointF(coord[0], coord[1])
							polygon_points.append(current_point)
						else:
							print("Unknown coord", geometry["coordinates"][0])
					polygon = QPolygonF(polygon_points)
					path = QPainterPath()
					path.addPolygon(polygon)
					contour = QGraphicsPathItem(path)
					# r = lambda: random.randint(0, 255)
					# next_color = '#%02X%02X%02X' % (r(), r(), r())
					contour.setPen(QPen(QColor("red"), 0.1))

					contour.setBrush(QBrush(Qt.transparent))
					# if paths_count == 4:
					print(item['json_featuretype'])
					self._scene.addItem(contour)
					paths_count += 1
		self.update()

	def clear(self):
		for item in self._scene.items():
			self._scene.removeItem(item)

	def resizeEvent(self, event):
		# skip initial entry
		self.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)
		self._scene.setSceneRect(self._scene.itemsBoundingRect())
		super(HumanVisualizationWidget, self).resizeEvent(event)

	def add_human_by_pos(self, id, pos):
		x, y = pos
		human = QGraphicsEllipseItem(0, 0, 200, 200)
		human.setBrush(QBrush(Qt.black, style=Qt.SolidPattern))
		human_text = QGraphicsTextItem(str(pos))
		font = QFont("Helvetica [Cronyx]", 40, QFont.Bold)
		human_text.setFont(font)
		human_text.setParentItem(human)
		human.setPos(pos[0], pos[1])
		self._humans[id] = human
		human.setZValue(30)
		self._scene.addItem(human)

	def move_human(self, id, pos):
		x, y = pos
		human = self._humans[id]
		human.setPos(x, y)

	def set_human_color(self, id, color):
		if id in self._humans:
			self._humans[id].setBrush(color)


if __name__ == '__main__':
	app = QApplication(sys.argv)
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	h_v = HumanVisualizationWidget()
	h_v.show()
	h_v.load_json_world(os.path.join(CURRENT_FILE_PATH, "..", "resources", "prueba.json"))
	# h_v.add_human_by_pos(0, (30,30))
	# h_v.move_human(0, (1000, -1000))
	app.exec_()
