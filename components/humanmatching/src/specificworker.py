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
import time
from Queue import Queue, Empty

import numpy
from PySide2.QtCore import QSize, Qt, QState, QFinalState, QStateMachine, QTimer, SIGNAL, Signal
from PySide2.QtGui import QColor

from genericworker import *
import networkx as nx

from kalman import KalmanTracker
from libs.QNetworkxGraph.QNetworkxGraph import QNetworkxController
from libs.HumanVisualizationWidget import HumanVisualizationWidget
from PySide2.QtWidgets import QVBoxLayout, QHBoxLayout, QSlider, QLabel, QCheckBox, QLCDNumber, QSpinBox
import logging


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

# logging.basicConfig(level=logging.DEBUG)
# logging.basicConfig(level=logging.DEBUG)
# logging.basicConfig()
# root = logging.getLogger()
logger = logging.getLogger(__name__)

# create a file handler
file_handler = logging.FileHandler('humanmatching.log')
file_handler.setLevel(logging.DEBUG)

terminal_handler = logging.StreamHandler(sys.stdout)
terminal_handler.setLevel(logging.DEBUG)

# create a logging format
formatter = logging.Formatter('%(asctime)s - %(name)s - %(funcName)s - %(levelname)s - %(message)s')


file_handler.setFormatter(formatter)
terminal_handler.setFormatter(formatter)

# add the file_handlers to the logger
logger.addHandler(file_handler)
logger.addHandler(terminal_handler)


ABS_THR = 500
REL_THR = 250

PERSON_ALIVE_TIME = 20000

CURRENT_FILE_PATH = os.path.dirname(__file__)

def random_hexrgb():
	r = lambda: random.randint(0, 255)
	rand_color = '#%02X%02X%02X' % (r(), r(), r())
	return rand_color

class Position2D:
	def __init__(self, x=-1, y=-1):
		self.x = x
		self.y = y


	@property
	def x(self):
		return  self._x

	@property
	def y(self):
		return self._y


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

	def __repr__(self):
		return '(%f, %f)'%(self.x, self.y)


class Person:
	def __init__(self):
		self._person_id = -1
		self._pos = Position2D()
		# self._position_history = Queue(5)
		# self._rot = 0
		self.tracker = KalmanTracker()
		self._color = random_hexrgb()
		self._cameras = []
		self._velocity = []
		self._last_time_detected = -1
		self._last_time_predicted = -1

	@property
	def person_id(self):
		return self._person_id

	@property
	def pos(self):
		return self._pos

	# @property
	# def rot(self):
	# 	return self._rot

	@property
	def color(self):
		return self._color

	@property
	def cameras(self):
		return self._cameras

	@person_id.setter
	def person_id(self, p_id):
		self._person_id  = p_id

	# @pos.setter
	# def pos(self, value):
	# 	self._position_history.put(self._pos)
	# 	if isinstance(value, list) and not isinstance(value, basestring):
	# 		if len(value) > 0:
	# 			self._pos.x = value[0]
	# 		if len(value) > 1:
	# 			self._pos.y = value[1]
	# 		if len(value) > 2:
	# 			self._pos.z = value[2]
	# 		if len(value) > 3:
	# 			raise IndexError
	# 	elif isinstance(value, Position):
	# 		self._pos = value
	# 	else:
	# 		raise TypeError

	def is_active(self):
		current_time = datetime.datetime.now()
		time_diff = self._last_time_detected - current_time
		if time_diff > PERSON_ALIVE_TIME:
			return False
		else:
			return True

	def predict(self):
		if self._last_time_predicted != -1:
			current_time = datetime.datetime.now()
			miliseconds_dt = (self._last_time_detected-current_time).total_seconds() * 1000
			predicted, cov =self.tracker.predict_with_time_diff(miliseconds_dt)
			self._pos = Position2D(float(predicted[0]), float(predicted[2]))
			self._last_time_predicted = current_time

	def update_position(self, position):
		current_time = datetime.datetime.now()
		self._last_time_detected = current_time
		self._last_time_predicted = current_time
		predicted, cov =self.tracker.update([position.x, position.y])
		self._pos = Position2D(float(predicted[0]), float(predicted[2]))


	# Reset the tracker to this position
	def initialice_tracker(self, position):
		current_time = datetime.datetime.now()
		self._last_time_detected = current_time
		self._last_time_predicted = current_time
		self.tracker.init_to_position(position.x, position.y)
		self._pos = position


	def detection_delta_time(self):
		current_time = datetime.datetime.now()
		delta_time = (current_time-self._last_time_detected).total_seconds() * 1000
		return delta_time

	def __repr__(self):
		return 'Person (%d) at %s'%(self.person_id, str(self.pos))








class SpecificWorker(GenericWorker):

	new_humans_signal = Signal()
	first_humans_updated_signal = Signal()

	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 500
		self._noise_factor = 1000
		self.timer.start(self.Period)
		self._matching_graph = nx.Graph()
		self._current_person_list = []
		self._next_person_list = []


		self.widget_graph = QNetworkxController(self.ui._graph_view)
		self.ui._noise_slider.sliderMoved.connect(self.set_noise_factor)
		self.ui._first_view.load_custom_json_world(os.path.join(CURRENT_FILE_PATH, "resources", "autonomy.json"))
		self.ui._second_view.load_custom_json_world(os.path.join(CURRENT_FILE_PATH, "resources", "autonomy.json"))

		self._update_views = False
		self._detection_queue = Queue()

		self._state_machine = QStateMachine()
		self._create_state_machine()
		self._state_machine.start()
		self._test_timer.start(1000/30)

	def __del__(self):
		logger.info('SpecificWorker destructor')

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):

		return True

	def _create_state_machine(self):

		self._test_timer = QTimer()

		self._initial_state = QState()
		self._initial_state.entered.connect(self._initial_state_method)

		self._first_person_state = QState()
		self._first_person_state.entered.connect(self._first_person_state_method)

		self._tracking_state = QState()

		self._prediction_state = QState(self._tracking_state)
		self._prediction_state.entered.connect(self._prediction_state_method)
		self._data_association_state = QState(self._tracking_state)
		self._data_association_state.entered.connect(self._data_association_state_method)
		self._data_update_state = QState(self._tracking_state)
		self._data_update_state.entered.connect(self._data_update_state_method)

		self._tracking_state.setInitialState(self._prediction_state)


		self._final_state = QFinalState()

		self._state_machine.addState(self._initial_state)
		self._state_machine.addState(self._first_person_state)
		self._state_machine.addState(self._tracking_state)
		# self._state_machine.addState(self._prediction_state)
		# self._state_machine.addState(self._data_association_state)
		# self._state_machine.addState(self._data_update_state)
		self._state_machine.setInitialState(self._initial_state)


		self._initial_state.addTransition(self, SIGNAL('new_humans_signal()'), self._first_person_state)
		self._first_person_state.addTransition(self, SIGNAL('first_humans_updated_signal()'), self._tracking_state)
		self._prediction_state.addTransition(self.timer, SIGNAL('timeout()'), self._prediction_state)
		self._prediction_state.addTransition(self, SIGNAL('new_humans_signal()'), self._data_association_state)
		self._data_association_state.addTransition(self._test_timer, SIGNAL('timeout()'), self._data_update_state)

		# self._data_association_state.addTransition(self._test_timer, SIGNAL('timeout()'), self._data_update_state)
		self._data_update_state.addTransition(self._test_timer, SIGNAL('timeout()'), self._prediction_state)
		logger.debug("State machine created")

	def _initial_state_method(self):
		logger.debug("entered")

	def _first_person_state_method(self):
		logger.debug("entered")
		try:
			humansFromCam = self._detection_queue.get_nowait()
			# First time detection
			if len(self._next_person_list) == 0:
				logger.debug("obtainHumanPose: First %d humans detected"%len(humansFromCam.humanList))
				self._current_person_list = self._cam_humans_2_person_list(humansFromCam)
				self.first_humans_updated_signal.emit()
			# print self._current_person_list
			else:
				logger.warning("We should not be here. self._next_person_list is not empty")
				#TODO: emit signal to go back
		except Empty as e:
			logger.warn("Exception. No new detection but unexpected state.")
			# TODO: emit signal to go back

	# def _detection_state_method(self):
	# 	logger.debug("entered")
	# 	try:
	# 		humansFromCam = self._detection_queue.get_nowait()
	# 		# First time detection
	# 		if len(self._next_person_list) == 0:
	# 			logger.debug("obtainHumanPose: First humans detected")
	# 			for cam_person in humansFromCam.humanList:
	# 				# 	 struct Pose3D
	# 				# 	{
	# 				# 		float x;
	# 				# 		float z;
	# 				# 		float ry;
	# 				# 		bool posGood;
	# 				# 		bool rotGood;
	# 				# 		int confidence = 0;
	# 				# 	};
	# 				#
	# 				# 	struct PersonType
	# 				# 	{
	# 				# 		int id;
	# 				# 		Pose3D pos;
	# 				# 	};
	#
	# 				detected_person = Person()
	# 				detected_person.person_id = cam_person.id
	# 				detected_person.cameras.append(humansFromCam.idCamera)
	# 				detected_person.initialice_tracker(Position2D(cam_person.pos.x, cam_person.pos.z))
	# 				self._current_person_list.append(detected_person)
	#
	# 		# print self._current_person_list
	# 		else:
	# 			logger.debug("obtainHumanPose: New humans detected")
	# 			# copy
	# 			self._current_person_list = self._next_person_list[:]
	# 			self._next_person_list = humansFromCam.humanList[:]
	# 			# self._update_current_person_list_view()
	# 			self._update_person_list_view(self._current_person_list, self.ui._first_view)
	# 			self._update_person_list_view(self._next_person_list, self.ui._second_view)
	#
	# 			max_clique = self.calculate_matching(humansFromCam)
	# 			for node_id in max_clique:
	# 				if node_id in self._matching_graph.nodes:
	# 					node = self._matching_graph.nodes[node_id]
	# 					logger.debug("Node %s relates %d in T is with %d in T+1. Moving from pos (%d, %d) to (%d, %d)",
	# 								 node_id, node["person1"].id, node["person2"].id, node["person1"].pos.x,
	# 								 node["person1"].pos.z, node["person2"].pos.x, node["person2"].pos.z)
	#
	# 					next_color = random_hexrgb()
	#
	# 					self.ui._first_view.set_human_color(node["person1"].id, QColor(next_color))
	# 					self.ui._second_view.set_human_color(node["person2"].id, QColor(next_color))
	#
	#
	# 	except Empty as e:
	# 		logger.info("No new detection")

	def _prediction_state_method(self):
		logger.debug("entered. %d persons "%len(self._current_person_list))
		for person in self._current_person_list:
			person.predict()
			logger.debug("Person %d predicted at (%s) " % (person.person_id, str(person.pos)))
			self._update_person_list_view(self._current_person_list, self.ui._first_view)


	def _data_association_state_method(self):
		logger.debug("entered. New humans detected")
		try:
			humansFromCam = self._detection_queue.get_nowait()

			# self._update_current_person_list_view()
			# self._update_person_list_view(self._current_person_list, self.ui._first_view)
			# self._update_person_list_view(self._next_person_list, self.ui._second_view)

			# Translate the incoming ice structure to a person list
			new_persons_list = self._cam_humans_2_person_list(humansFromCam)
			self._update_person_list_view(new_persons_list, self.ui._second_view)

			max_clique = self.calculate_clique_matching(self._current_person_list, new_persons_list)
			for node_id in max_clique:
				if node_id in self._matching_graph.nodes:
					node = self._matching_graph.nodes[node_id]
					old_person = node["person1"]
					new_person = node["person2"]
					logger.debug("Node %s relates %d in T is with %d in T+1. Moving from pos (%d, %d) to (%d, %d)",
								 node_id, old_person.person_id, new_person.person_id, old_person.pos.x,
								 old_person.pos.y, new_person.pos.x, new_person.pos.y)
					old_person.update_position(new_person.pos)
					#
					# next_color = random_hexrgb()
					#
					# self.ui._first_view.set_human_color(old_person.person_id, QColor(next_color))
					# self.ui._second_view.set_human_color(new_person.person_id, QColor(next_color))
			# if len(max_clique)>0:
			#
			# 	#TODO: Do this on a different state
			# 	#self.people_to_update.emit()
		except Empty as e:
				logger.warn("Exception. No new detection but unexpected state.")

	def _cam_humans_2_person_list(self, humansFromCam):
		new_person_list = []
		for cam_person in humansFromCam.humanList:
			detected_person = Person()
			#TODO: is the camera id the one we assume for the person? How we update this?
			detected_person.person_id = cam_person.id
			detected_person.cameras.append(humansFromCam.idCamera)
			detected_person.initialice_tracker(Position2D(cam_person.pos.x, cam_person.pos.z))
			new_person_list.append(detected_person)
		return new_person_list


	def _data_update_state_method(self):
		logger.debug("entered")

	def calculate_clique_matching(self, input1, input2):
		self._matching_graph.clear()
		# TODO: Move graphic to its own methods
		self.widget_graph.clear()
		# camera_id = input.idCamera
		current_person_list = input1
		new_persons_list = input2
		# if self.ui._noise_checkbox.isChecked():
		# 	new_persons_list = self.add_noise(new_persons_list)
		# 	current_person_list = self.add_noise(current_person_list)
		# 	self._update_person_list_view(current_person_list, self.ui._first_view)
		# 	self._update_person_list_view(new_persons_list, self.ui._second_view)
		logger.debug("Person list input: %s", str(new_persons_list))
		for detected_person in new_persons_list:
			for existing_person in current_person_list:
				logger.debug("Indexes: %d %d", detected_person.person_id, existing_person.person_id)
				dist = self._calculate_person_distance(detected_person, existing_person)
				logger.debug("Distance persons: %d", dist)
				if dist < ABS_THR:
					logger.debug("adding node")
					self._matching_graph.add_node(str(detected_person.person_id)+"_"+str(existing_person.person_id), person1=existing_person, person2=detected_person)
		logger.debug("Nodes added to graph: %d", self._matching_graph.number_of_nodes())
		for node1_id, node1 in self._matching_graph.nodes.data():
			for node2_id, node2 in self._matching_graph.nodes.data():
				if node1 != node2:
					d0 = self._calculate_person_distance(node1["person1"], node2["person1"])
					d1 = self._calculate_person_distance(node1["person2"], node2["person2"])
					if numpy.fabs(d0 - d1) < REL_THR:
						logger.debug("Adding edges %s %s", str(node1_id), str(node2_id))
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
			logger.debug("Nodes in result: %s", str(r))

		return max_clique

	#TODO: Deprecated, remove when new is working
	# def calculate_matching(self, input):
	#
	# 	self._matching_graph.clear()
	# 	self.widget_graph.clear()
	# 	camera_id = input.idCamera
	# 	new_persons_list = input.humanList
	# 	current_person_list = self._current_person_list
	# 	if self.ui._noise_checkbox.isChecked():
	# 		new_persons_list = self.add_noise(new_persons_list)
	# 		current_person_list = self.add_noise(current_person_list)
	# 		self._update_person_list_view(current_person_list, self.ui._first_view)
	# 		self._update_person_list_view(new_persons_list, self.ui._second_view)
	# 	logger.debug("Person list input: %s", str(new_persons_list))
	# 	for detected_person in new_persons_list:
	# 		for existing_person in current_person_list:
	# 			logger.debug("Indexes: %d %d", detected_person.id, existing_person.id)
	# 			dist = self._calculate_person_distance(detected_person, existing_person)
	# 			logger.debug("Distance persons: %d", dist)
	# 			if dist < ABS_THR:
	# 				logger.debug("adding node")
	# 				self._matching_graph.add_node(str(detected_person.id)+"_"+str(existing_person.id), person1=existing_person, person2=detected_person)
	# 	logger.debug("Nodes added to graph: %d", self._matching_graph.number_of_nodes())
	# 	for node1_id, node1 in self._matching_graph.nodes.data():
	# 		for node2_id, node2 in self._matching_graph.nodes.data():
	# 			if node1 != node2:
	# 				d0 = self._calculate_person_distance(node1["person1"], node2["person1"])
	# 				d1 = self._calculate_person_distance(node1["person2"], node2["person2"])
	# 				if numpy.fabs(d0 - d1) < REL_THR:
	# 					logger.debug("Adding edges %s %s", str(node1_id), str(node2_id))
	# 					self._matching_graph.add_edge(node1_id, node2_id)
	#
	# 	self.widget_graph.set_graph(self._matching_graph)
	# 	self.widget_graph.graph_widget.show()
	#
	# 	result = nx.find_cliques(self._matching_graph)
	# 	max_nodes = -1
	# 	max_clique = []
	# 	for r in result:
	# 		if len(r) > max_nodes:
	# 			max_nodes = len(r)
	# 			max_clique = r
	# 		logger.debug("Nodes in result: %s", str(r))
	#
	# 	return max_clique

	# def _update_current_person_list_view(self):
	# 	self._update_person_list_view(self._current_person_list, self.ui._first_view)
	# 	pass
	#
	# def _update_new_person_list_view(self, list):
	# 	self._update_person_list_view(list, self.ui._second_view)

	def _update_person_list_view(self, person_list, view):
		view.clear()
		# logger.debug("Updating %d persons on view %s", len(person_list), str(view))
		for person in person_list:
			view.add_human_by_pos(person.person_id, (person.pos.x, person.pos.y))
			color = QColor(person._color)
			darker_factor = 100+int(person.detection_delta_time()/200)
			color = color.darker(darker_factor)
			logger.debug("setting color %s darker factor %d"%(color.name(), darker_factor))
			view.set_human_color(person.person_id, color)



	def _calculate_person_distance(self, p1, p2):
		a = numpy.array((p1.pos.x, p1.pos.y))
		b = numpy.array((p2.pos.x, p2.pos.y))
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
		self.ui._min_noise.setValue(min(s, key=abs))
		self.ui._max_noise.setValue(max(s, key=abs))
		logger.debug("Noises vector %s",str(s))
		for index, detected_person in enumerate(new_humans_detected):
			logger.debug("Person %d, %d", detected_person.pos.x, detected_person.pos.z)
			detected_person.pos.x += s[index*2]
			detected_person.pos.z += s[index*2+1]
			logger.debug("Person with noise %d, %d", detected_person.pos.x, detected_person.pos.z)
		return new_humans_detected

	def set_noise_factor(self, value):
		self._noise_factor = value*10
		self.ui._noise_factor_lcd.setValue(self._noise_factor)


	def obtainHumanPose(self, humansFromCam):
		self._detection_queue.put(humansFromCam)
		self.new_humans_signal.emit()


