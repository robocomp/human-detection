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
import logging

from classes.clique import calculate_clique_matching
from classes.state_machine import HumanMatchingStateMachine

logger = logging.getLogger(__name__)

import copy
from Queue import Queue, Empty

import numpy
from PySide2.QtCore import Signal
from PySide2.QtGui import QColor

from genericworker import *

from libs.QNetworkxGraph.QNetworkxGraph import QNetworkxController
from classes.person import Position2D, Person

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

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
app = QtWidgets.QApplication(sys.argv)

CURRENT_FILE_PATH = os.path.dirname(__file__)



class SpecificWorker(GenericWorker):

	new_humans_signal = Signal()
	first_humans_updated_signal = Signal()

	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)

		self.timer.timeout.connect(self.compute)
		self.Period = 500
		self._noise_factor = 1000
		self.timer.start(self.Period)
		self._current_person_list = []
		self._next_person_list = []


		self.widget_graph = QNetworkxController(self.ui._graph_view)
		self.ui._noise_slider.sliderMoved.connect(self.set_noise_factor)
		self.ui._first_view.load_custom_json_world(os.path.join(CURRENT_FILE_PATH, "resources", "autonomy.json"))
		self.ui._second_view.load_custom_json_world(os.path.join(CURRENT_FILE_PATH, "resources", "autonomy.json"))

		self._update_views = False
		self._detection_queue = Queue()
		self._state_machine = HumanMatchingStateMachine(self)
		self._state_machine._initial_state.addTransition(self.new_humans_signal, self._state_machine._first_person_state)
		self._state_machine._prediction_state.addTransition(self.new_humans_signal,
														 self._state_machine._data_association_state)
		self._state_machine.start()


	def __del__(self):
		logger.info('SpecificWorker destructor')

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):

		return True


	def initial_state_entered(self):
		logger.debug("entered")

	def first_person_state_entered(self):
		logger.debug("entered")
		try:
			humansFromCam = self._detection_queue.get_nowait()
			# First time detection
			if len(self._next_person_list) == 0:
				logger.debug("obtainHumanPose: First %d humans detected"%len(humansFromCam.humanList))
				self._current_person_list = self._cam_humans_2_person_list(humansFromCam)
				self._state_machine.first_person_to_tracking.emit()
			# print self._current_person_list
			else:
				logger.warning("We should not be here. self._next_person_list is not empty")
				#TODO: emit signal to go back
		except Empty as e:
			logger.warn("Exception. No new detection but unexpected state.")
			# TODO: emit signal to go back

	# ef _detection_state_entered(self):
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

	def prediction_state_entered(self):
		logger.debug("entered. %d persons "%len(self._current_person_list))
		for person in self._current_person_list:
			person.predict()
			logger.debug("Person %d predicted at (%s) " % (person.person_id, str(person.pos)))
			self._update_person_list_view(self._current_person_list, self.ui._first_view)


	def data_association_state_entered(self):
		logger.debug("entered. New humans detected")
		try:
			humansFromCam = self._detection_queue.get_nowait()

			# self._update_current_person_list_view()
			# self._update_person_list_view(self._current_person_list, self.ui._first_view)
			# self._update_person_list_view(self._next_person_list, self.ui._second_view)

			# Translate the incoming ice structure to a person list
			new_persons_list = self._cam_humans_2_person_list(humansFromCam)
			self._update_person_list_view(new_persons_list, self.ui._second_view)
			self.widget_graph.clear()
			max_clique, matching_graph = calculate_clique_matching(self._current_person_list, new_persons_list)
			self.widget_graph.set_graph(matching_graph)
			self.widget_graph.graph_widget.show()
			for node_id in max_clique:
				if node_id in matching_graph.nodes:
					node = matching_graph.nodes[node_id]
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
			self._state_machine.data_association_to_data_update.emit()
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


	def data_update_state_entered(self):
		logger.debug("entered")
		self._state_machine.data_update_to_prediction.emit()



	def _update_person_list_view(self, person_list, view):
		view.clear()
		# logger.debug("Updating %d persons on view %s", len(person_list), str(view))
		for person in person_list:
			view.add_human_by_pos(person.person_id, (person.pos.x, person.pos.y))
			color = QColor(person._color)
			darker_factor = 100+int(person.detection_delta_time()/200)
			color = color.darker(darker_factor)
			# logger.debug("setting color %s darker factor %d"%(color.name(), darker_factor))
			view.set_human_color(person.person_id, color)

	#TODO: Deprecated. Not used anymore.
	#TODO: Remove fom UI or implement noise as input option
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


