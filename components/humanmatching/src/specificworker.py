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
import random
from Queue import Queue, Empty

import numpy
from PySide2.QtCore import QSize
from PySide2.QtGui import QColor

from genericworker import *
import networkx as nx
from libs.QNetworkxGraph.QNetworkxGraph import QNetworkxController
from libs.HumanVisualizationWidget import HumanVisualizationWidget
from PySide2.QtWidgets import QVBoxLayout, QHBoxLayout

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

ABS_THR = 500
REL_THR = 500

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 500
		self.timer.start(self.Period)
		self._matching_graph = nx.Graph()
		self._current_person_list = []
		self._next_person_list = []
		self._main_layout = QVBoxLayout()
		self.setLayout(self._main_layout)
		self.widget_graph = QNetworkxController()
		self._main_layout.addWidget(self.widget_graph.graph_widget)

		self._maps_layout = QHBoxLayout()
		self._main_layout.addLayout(self._maps_layout)
		self._first_view = HumanVisualizationWidget()
		self._first_view.setMinimumSize(QSize(400, 400))
		self._first_view.setWindowTitle("Current view")
		self._maps_layout.addWidget(self._first_view)

		self._second_view = HumanVisualizationWidget()
		self._second_view.setMinimumSize(QSize(400, 400))
		self._second_view.setWindowTitle("Next view")
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
		print 'SpecificWorker.compute...'
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
			print("No new detection")

		return True

	def calculate_matching(self, input):
		self._matching_graph.clear()
		self.widget_graph.clear()
		camera_id = input.idCamera
		persons_list = input.humanList
		print("Person list input: ", persons_list)
		for detected_person in persons_list:
			for existing_person in self._current_person_list:
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
		for r in result:
			# print("Nodes in result: ", r)
			for node_id in r:
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


	def obtainHumanPose(self, humansFromCam):
		self._detection_queue.put(humansFromCam)


