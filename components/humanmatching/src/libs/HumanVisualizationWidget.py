import json
import os
import signal
import sys

from PySide2.QtCore import QFile, QIODevice, QJsonDocument, QRectF, Qt, QPointF
from PySide2.QtGui import QBrush, QColor, QPen, QFont, QPainterPath
from PySide2.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QGraphicsTextItem, \
	QGraphicsPolygonItem
from pyside2uic.properties import QtCore

CURRENT_FILE_PATH = os.path.dirname(__file__)


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

	def load_geojson_world(self, file):

		if not os.path.isfile(file):
			print("Error reading world file, check config params:", file)
			return False

		with open(file, "r") as read_file:
			json_data = json.load(read_file)

		polygon_points = []
		for item in json_data:
			for feature in item['features']:
				for coord in feature['geometry']['coordinates'][0]:
					polygon_points.append(QPointF(coord[0], coord[1]))
			poligon = QGraphicsPolygonItem(polygon_points)
			self._scene.addItem(poligon)
			self._boxes



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
		human = QGraphicsEllipseItem(x,y, 200,200)
		human.setBrush(QBrush(Qt.red, style = Qt.SolidPattern))
		human_text = QGraphicsTextItem(str(pos))
		font = QFont("Helvetica [Cronyx]", 40, QFont.Bold)
		human_text.setFont(font)
		human_text.setParentItem(human)
		self._humans[id] = human
		human.setZValue(30)
		self._scene.addItem(human)

	def move_human(self, id, pos):
		x, y = pos
		human = self._humans[id]
		human.setPos(x, y)





if __name__ == '__main__':
	app = QApplication(sys.argv)
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	h_v = HumanVisualizationWidget()
	h_v.show()
	h_v.load_geojson_world(os.path.join(CURRENT_FILE_PATH, "..", "resources", "p00_01.json"))
	h_v.add_human_by_pos(0, (30,30))
	h_v.move_human(0, (1000, -1000))
	app.exec_()
