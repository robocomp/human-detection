#
# Copyright (C) 2019 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#
import copy
import datetime
import random
from Queue import Queue, Empty

import numpy
from PySide2.QtCore import QSize, Qt
from PySide2.QtGui import QColor

from genericworker import *
import networkx as nx
from libs.QNetworkxGraph.QNetworkxGraph import QNetworkxController
from libs.HumanVisualizationWidget import HumanVisualizationWidget
from PySide2.QtWidgets import QVBoxLayout, QHBoxLayout, QSlider, QLabel, QCheckBox, QLCDNumber, QSpinBox

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

ABS_THR = 500
REL_THR = 250

PERSON_ALIVE_TIME = 20000

CURRENT_FILE_PATH = os.path.dirname(__file__)

class Position:
	def __init__(self):
		self._x = -1
		self._y = -1
		self._z = -1

	@property
	def x(self):
		return  self._x

	@property
	def y(self):
		return self._y

	@property
	def z(self):
		return self._z

	@x.setter
	def x(self, value):
		if isinstance(value,[float, int]):
			self._x = value
		else:
			raise TypeError

	@y.setter
	def y(self, value):
		if isinstance(value, [float, int]):
			self._y = value
		else:
			raise TypeError

	@z.setter
	def z(self, value):
		if isinstance(value, [float, int]):
			self._z = value
		else:
			raise TypeError

class Person:
	def __init__(self):
		self._person_id = -1
		self._pos = Position()
		self._previous_pos = Position()
		self._rot = 0
		self._color = "black"
		self._cameras = []
		self._velocity = []
		self._last_time_detected = -1

	@property
	def person_id(self):
		return self._person_id

	@property
	def pos(self):
		return self._pos

	@property
	def rot(self):
		return self._rot

	@property
	def color(self):
		return self._color

	@property
	def cameras(self):
		return self._cameras

	@person_id.setter
	def person_id(self, p_id):
		self._person_id  = p_id

	@pos.setter
	def pos(self, value):
		self._previous_pos = self._pos
		if isinstance(value, list) and not isinstance(value, basestring):
			if len(value) > 0:
				self._pos.x = value[0]
			if len(value) > 1:
				self._pos.y = value[1]
			if len(value) > 2:
				self._pos.z = value[2]
			if len(value) > 3:
				raise IndexError
		elif isinstance(value, Position):
			self._pos = value
		else:
			raise TypeError

	def is_active(self):
		now = datetime.datetime.now()
		time_diff = self._last_time_detected - now
		if time_diff > PERSON_ALIVE_TIME:
			return False
		else:
			return True




class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 500
		self._noise_factor = 1000
		self.timer.start(self.Period)
		self._matching_graph = nx.Graph()
		self._current_person_list = []
		self._next_person_list = []
		self._main_layout = QVBoxLayout()
		self.setLayout(self._main_layout)
		self.widget_graph = QNetworkxController()
		self._main_layout.addWidget(self.widget_graph.graph_widget)

		self._noise_checkbox = QCheckBox("Apply noise")
		self._noise_slider = QSlider(QtCore.Qt.Horizontal)
		self._noise_slider.setMinimum(0)
		self._noise_slider.setMaximum(1000)
		self._noise_slider.setTickInterval(100)
		self._noise_layout = QHBoxLayout()
		self._noise_layout.addWidget(self._noise_checkbox)
		self._noise_layout.addWidget(self._noise_slider)
		self._noise_slider.sliderMoved.connect(self.set_noise_factor)

		self._noise_factor_lcd_layout = QHBoxLayout()
		self._lcds_layout = QHBoxLayout()

		self._noise_factor_label = QLabel("Noise factor:")
		self._noise_factor_lcd = QSpinBox()
		self._noise_factor_lcd.setReadOnly(True)
		self._noise_factor_lcd.setRange(0,10000)
		self._noise_factor_lcd_layout.addWidget(self._noise_factor_label)
		self._noise_factor_lcd_layout.addWidget(self._noise_factor_lcd)


		self._min_max_noise_layout = QHBoxLayout()
		self._min_max_label = QLabel("Min, max noise added:")
		self._min_noise = QSpinBox()
		self._min_noise.setReadOnly(True)
		self._min_noise.setSuffix("mm")
		self._min_noise.setRange(-5000, 5000)
		self._max_noise = QSpinBox()
		self._max_noise.setReadOnly(True)
		self._max_noise.setSuffix("mm")
		self._max_noise.setRange(-5000, 5000)
		self._min_max_noise_layout.addWidget(self._min_max_label)
		self._min_max_noise_layout.addWidget(self._min_noise)
		self._min_max_noise_layout.addWidget(self._max_noise)


		self._lcds_layout.addLayout(self._noise_factor_lcd_layout)
		# self._lcds_layout.addWidget(self._min_max_label)
		self._lcds_layout.addStretch()
		self._lcds_layout.addLayout(self._min_max_noise_layout)


		# self._noise_layout.addLayout(self._lcds_layout)


		self._maps_layout = QHBoxLayout()
		self._main_layout.addLayout(self._maps_layout)

		self._main_layout.addLayout(self._noise_layout)
		self._main_layout.addLayout(self._lcds_layout)

		self._first_view = HumanVisualizationWidget()
		self._first_view.setMinimumSize(QSize(400, 400))
		self._first_view.setWindowTitle("Current view")
		self._first_view.load_custom_json_world(os.path.join(CURRENT_FILE_PATH, "resources", "autonomy.json"))
		self._maps_layout.addWidget(self._first_view)

		self._second_view = HumanVisualizationWidget()
		self._second_view.setMinimumSize(QSize(400, 400))
		self._second_view.setWindowTitle("Next view")
		self._second_view.load_custom_json_world(os.path.join(CURRENT_FILE_PATH, "resources", "autonomy.json"))
		# self._arriving_view.show()
		self._maps_layout.addWidget(self._second_view)

		self._update_views = False
		self._detection_queue = Queue()

	def __del__(self):
		print 'SpecificWorker destructor'

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		try:
			humansFromCam = self._detection_queue.get_nowait()
			if len(self._next_person_list) == 0:
				print("obtainHumanPose: First humans detected")
				self._next_person_list = humansFromCam.humanList

			# print self._current_person_list
			else:
				print("obtainHumanPose: New humans detected")
				# copy
				self._current_person_list = self._next_person_list[:]
				self._next_person_list = humansFromCam.humanList[:]
				# self._update_current_person_list_view()
				self._update_person_list_view(self._current_person_list, self._first_view)
				self._update_person_list_view(self._next_person_list, self._second_view)

				self.calculate_matching(humansFromCam)


		except Empty as e:
			# print("No new detection")
			pass

		return True

	def calculate_matching(self, input):
		self._matching_graph.clear()
		self.widget_graph.clear()
		camera_id = input.idCamera
		new_persons_list = input.humanList
		current_person_list = self._current_person_list
		if self._noise_checkbox.isChecked():
			new_persons_list = self.add_noise(new_persons_list)
			current_person_list = self.add_noise(current_person_list)
			self._update_person_list_view(current_person_list, self._first_view)
			self._update_person_list_view(new_persons_list, self._second_view)
		print("Person list input: ", new_persons_list)
		for detected_person in new_persons_list:
			for existing_person in current_person_list:
				print("Indexes:", detected_person.id," ", existing_person.id)
				dist = self._calculate_person_distance(detected_person, existing_person)
				print("Distance persons: ", dist)
				if dist < ABS_THR:
					print("adding node")
					self._matching_graph.add_node(str(detected_person.id)+"_"+str(existing_person.id), person1=existing_person, person2=detected_person)
		print("Nodes added to graph: ", self._matching_graph.number_of_nodes())
		for node1_id, node1 in self._matching_graph.nodes.data():
			for node2_id, node2 in self._matching_graph.nodes.data():
				if node1 != node2:
					d0 = self._calculate_person_distance(node1["person1"], node2["person1"])
					d1 = self._calculate_person_distance(node1["person2"], node2["person2"])
					if numpy.fabs(d0 - d1) < REL_THR:
						print("Adding edges ", node1_id, node2_id)
						self._matching_graph.add_edge(node1_id, node2_id)

		self.widget_graph.set_graph(self._matching_graph)
		self.widget_graph.graph_widget.show()

		result = nx.find_cliques(self._matching_graph)
		max_nodes = -1
		max_clique = []
		for r in result:
			if len(r) > max_nodes:
				max_nodes = len(r)
				max_clique = r
			print("Nodes in result: ", r)

		for node_id in max_clique:
			node = self._matching_graph.nodes[node_id]
			if node_id in self._matching_graph.nodes:
				print("Node %s relates %d in T is with %d in T+1. Moving from pos (%d, %d) to (%d, %d)"%(node_id, node["person1"].id, node["person2"].id, node["person1"].pos.x, node["person1"].pos.z, node["person2"].pos.x, node["person2"].pos.z))
				r = lambda: random.randint(0, 255)
				next_color = '#%02X%02X%02X' % (r(), r(), r())

				self._first_view.set_human_color(node["person1"].id, QColor(next_color))
				self._second_view.set_human_color(node["person2"].id, QColor(next_color))

		return result

	# def _update_current_person_list_view(self):
	# 	self._update_person_list_view(self._current_person_list, self._first_view)
	# 	pass
	#
	# def _update_new_person_list_view(self, list):
	# 	self._update_person_list_view(list, self._second_view)

	def _update_person_list_view(self, person_list, view):
		view.clear()
		print("Updating %d persons on view %s"%(len(person_list), str(view)))
		for person in person_list:
			view.add_human_by_pos(person.id, (person.pos.x, person.pos.z))


	def _calculate_person_distance(self, p1, p2):
		a = numpy.array((p1.pos.x, p1.pos.z))
		b = numpy.array((p2.pos.x, p2.pos.z))
		dist_a_b = numpy.sqrt(numpy.sum((a - b) ** 2))
		return dist_a_b

	# def _ice2Persons(self, humansFromCam):
	# 	camera_id = input.idCamera
	# 	detection_list = input.humanList
	#
	# 	for detected in detection_list:
	# 		new_person = Person()
	# 		new_person.

	def add_noise(self, persons_list):
		new_humans_detected = copy.deepcopy(persons_list)
		mu, sigma = 0, 0.1  # mean and standard deviation
		s = numpy.random.normal(mu, sigma, len(new_humans_detected)*2)
		s = s*self._noise_factor
		self._min_noise.setValue(min(s, key=abs))
		self._max_noise.setValue(max(s, key=abs))
		print("Noises vector %s"%str(s))
		for index, detected_person in enumerate(new_humans_detected):
			print("Person %d, %d"%(detected_person.pos.x, detected_person.pos.z))
			detected_person.pos.x += s[index*2]
			detected_person.pos.z += s[index*2+1]
			print("Person with noise %d, %d" % (detected_person.pos.x, detected_person.pos.z))
		return new_humans_detected

	def set_noise_factor(self, value):
		self._noise_factor = value*10
		self._noise_factor_lcd.setValue(self._noise_factor)


	def obtainHumanPose(self, humansFromCam):
		self._detection_queue.put(humansFromCam)


