#!/usr/bin/python
# -*- coding: utf-8 -*-
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
from PySide2.QtCore import Signal, QTimer
from PySide2.QtGui import QColor

from genericworker import *

from libs.QNetworkxGraph.QNetworkxGraph import QNetworkxController
from classes.person import Position2D, Person

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


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
		self.new_humans_signal.connect(self.tracking_initializationtofirst_person_state)
		self.new_humans_signal.connect(self.predictiontodata_association)
		self.human_matching_machine.start()

	def __del__(self):
		logger.info('SpecificWorker destructor')

	def setParams(self, params):
		return True

	# =============== Slots methods for State Machine ===================
# ===================================================================
	#
	# sm_initialize
	#
	@QtCore.Slot()
	def sm_initialize(self):
		print("Entered state initialize")
		self.initializetohuman_frames_tracking.emit()

	#
	# sm_cameras_matching
	#
	@QtCore.Slot()
	def sm_cameras_matching(self):
		print("Entered state cameras_matching")
		pass

	#
	# sm_human_frames_tracking
	#
	@QtCore.Slot()
	def sm_human_frames_tracking(self):
		print("Entered state human_frames_tracking")
		pass

	#
	# sm_finalize
	#
	@QtCore.Slot()
	def sm_finalize(self):
		print("Entered state finalize")
		pass

	#
	# sm_check_new_data
	#
	@QtCore.Slot()
	def sm_check_new_data(self):
		print("Entered state check_new_data")
		pass

	#
	# sm_cameras_clique
	#
	@QtCore.Slot()
	def sm_cameras_clique(self):
		print("Entered state cameras_clique")
		pass

	#
	# sm_results_update
	#
	@QtCore.Slot()
	def sm_results_update(self):
		print("Entered state results_update")
		pass

	#
	# sm_end_camera_matching
	#
	@QtCore.Slot()
	def sm_end_camera_matching(self):
		print("Entered state end_camera_matching")
		pass

	#
	# sm_tracking_initialization
	#
	@QtCore.Slot()
	def sm_tracking_initialization(self):
		print("Entered state tracking_initialization")
		pass

	#
	# sm_first_person_state
	#
	@QtCore.Slot()
	def sm_first_person_state(self):
		logger.debug("entered sm_first_person_state")
		try:
			humansFromCam = self._detection_queue.get_nowait()
			# First time detection
			if len(self._next_person_list) == 0:
				logger.debug("obtainHumanPose: First %d humans detected"%len(humansFromCam.humanList))
				self._current_person_list = self._cam_humans_2_person_list(humansFromCam)
				self.first_person_statetotracking_state.emit()
			# print self._current_person_list
			else:
				logger.warning("We should not be here. self._next_person_list is not empty")
				#TODO: emit signal to go back
		except Empty as e:
			logger.warn("Exception. No new detection but unexpected state.")
			# TODO: emit signal to go back

	#
	# sm_tracking_state
	#
	@QtCore.Slot()
	def sm_tracking_state(self):
		print("Entered state tracking_state")
		pass

	#
	# sm_end_camera_matching
	#
	@QtCore.Slot()
	def sm_end_camera_matching(self):
		print("Entered state end_camera_matching")
		pass

	#
	# sm_prediction
	#
	@QtCore.Slot()
	def sm_prediction(self):
		print("Entered sm_prediction %d persons "%len(self._current_person_list))
		for person in self._current_person_list:
			person.predict()
			logger.debug("Person %d predicted at (%s) " % (person.person_id, str(person.pos)))
			self._update_person_list_view(self._current_person_list, self.ui._first_view)
		QTimer.singleShot(self.Period, self.predictiontoprediction)
		#
	# sm_data_association
	#
	@QtCore.Slot()
	def sm_data_association(self):
		logger.debug("entered sm_data_association. New humans detected")
		try:
			humansFromCam = self._detection_queue.get_nowait()

		except Empty as e:
			logger.warn("Exception. No new detection but unexpected state.")
			self.data_associationtodata_update.emit()
		else:
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
					logger.debug("Node %s relates %d in T  with %d in T+1. Moving from pos (%d, %d) to (%d, %d)",
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
			self.data_associationtodata_update.emit()

	#
	# sm_data_update
	#
	@QtCore.Slot()
	def sm_data_update(self):
		logger.debug("entered sm_data_update")
		self.data_updatetoprediction.emit()


# =================================================================
# =================================================================


	@QtCore.Slot()
	def compute(self):

		return True

###----------- STATE MACHINE METHODS START ------------------------

	# def initial_state_entered(self):
	# 	logger.debug("entered")

	# def first_person_state_entered(self):
	# 	logger.debug("entered")
	# 	try:
	# 		humansFromCam = self._detection_queue.get_nowait()
	# 		# First time detection
	# 		if len(self._next_person_list) == 0:
	# 			logger.debug("obtainHumanPose: First %d humans detected"%len(humansFromCam.humanList))
	# 			self._current_person_list = self._cam_humans_2_person_list(humansFromCam)
	# 			self._state_machine.first_person_to_tracking.emit()
	# 		# print self._current_person_list
	# 		else:
	# 			logger.warning("We should not be here. self._next_person_list is not empty")
	# 			#TODO: emit signal to go back
	# 	except Empty as e:
	# 		logger.warn("Exception. No new detection but unexpected state.")
	# 		# TODO: emit signal to go back

	# def _detection_state_entered(self):
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

	# def prediction_state_entered(self):
	# 	logger.debug("entered. %d persons "%len(self._current_person_list))
	# 	for person in self._current_person_list:
	# 		person.predict()
	# 		logger.debug("Person %d predicted at (%s) " % (person.person_id, str(person.pos)))
	# 		self._update_person_list_view(self._current_person_list, self.ui._first_view)


	# def data_association_state_entered(self):
	# 	logger.debug("entered. New humans detected")
	# 	try:
	# 		humansFromCam = self._detection_queue.get_nowait()
    #
	# 		# self._update_current_person_list_view()
	# 		# self._update_person_list_view(self._current_person_list, self.ui._first_view)
	# 		# self._update_person_list_view(self._next_person_list, self.ui._second_view)
    #
	# 		# Translate the incoming ice structure to a person list
	# 		new_persons_list = self._cam_humans_2_person_list(humansFromCam)
	# 		self._update_person_list_view(new_persons_list, self.ui._second_view)
	# 		self.widget_graph.clear()
	# 		max_clique, matching_graph = calculate_clique_matching(self._current_person_list, new_persons_list)
	# 		self.widget_graph.set_graph(matching_graph)
	# 		self.widget_graph.graph_widget.show()
	# 		for node_id in max_clique:
	# 			if node_id in matching_graph.nodes:
	# 				node = matching_graph.nodes[node_id]
	# 				old_person = node["person1"]
	# 				new_person = node["person2"]
	# 				logger.debug("Node %s relates %d in T is with %d in T+1. Moving from pos (%d, %d) to (%d, %d)",
	# 							 node_id, old_person.person_id, new_person.person_id, old_person.pos.x,
	# 							 old_person.pos.y, new_person.pos.x, new_person.pos.y)
	# 				old_person.update_position(new_person.pos)
	# 				#
	# 				# next_color = random_hexrgb()
	# 				#
	# 				# self.ui._first_view.set_human_color(old_person.person_id, QColor(next_color))
	# 				# self.ui._second_view.set_human_color(new_person.person_id, QColor(next_color))
	# 		# if len(max_clique)>0:
	# 		#
	# 		# 	#TODO: Do this on a different state
	# 		# 	#self.people_to_update.emit()
	# 		self._state_machine.data_association_to_data_update.emit()
	# 	except Empty as e:
	# 			logger.warn("Exception. No new detection but unexpected state.")


	# def data_update_state_entered(self):
	# 	logger.debug("entered")
	# 	self._state_machine.data_update_to_prediction.emit()

###----------- STATE MACHINE METHODS END ------------------------

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

	def update_noise_values(self, noise_vector):
		"""Function to create a gausian noise vector"""

		self.ui._min_noise.setValue(min(noise_vector, key=abs))
		self.ui._max_noise.setValue(max(noise_vector, key=abs))
		logger.debug("Noises vector %s", str(noise_vector))

	def set_noise_factor(self, value):
		"""Qt Slot to update the noise factor value and visualization on slider change"""

		self._noise_factor = value*10
		self.ui._noise_factor_lcd.setValue(self._noise_factor)

	#
	# obtainHumanPose
	#
	def obtainHumanPose(self, humansFromCam):
		"""Component entry point for the detected human poses"""
		logger.debug("obtainHumanPose")
		self._detection_queue.put(humansFromCam)
		if (len(humansFromCam.humanList)!=0):
			print (humansFromCam)
			self.new_humans_signal.emit()


