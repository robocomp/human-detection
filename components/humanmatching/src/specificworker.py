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
import numpy

from genericworker import *
import networkx as nx

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

ABS_THR = 10
REL_THR = 10

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		self._matching_graph = nx.Graph()
		self._current_person_list = None

	def __del__(self):
		print 'SpecificWorker destructor'

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		return True

	def calculate_matching(self, input):
		camera_id = input.idCamera
		persons_list = input.humanList
		for index_detected, detected_person in enumerate(persons_list):
			for index_existing, existing_person in enumerate(self._current_person_list):
				dist = self._calculate_person_distance(detected_person, existing_person)
				if dist < ABS_THR:
					self._matching_graph.add_node(str(index_detected)+str(index_existing), person1=detected_person, person2=existing_person)

		for node1_id, node1 in self._matching_graph.nodes.data():
			for node2_id, node2 in self._matching_graph.nodes.data():
				if node1 != node2:
					d0 = self._calculate_person_distance(node1.person1, node2.person1)
					d1 = self._calculate_person_distance(node1.person2, node2.person2)
					if numpy.fabs(d0 - d1) < REL_THR:
						self._matching_graph.add_edge(node1_id, node2_id)
		result = nx.make_max_clique_graph(self._matching_graph)
		return result


	def _calculate_person_distance(self, p1, p2):
		a = numpy.array((p1.pos.x, p1.pos.z))
		b = numpy.array((p2.pos.x, p2.pos.z))
		dist_a_b = numpy.sqrt(numpy.sum((a - b) ** 2))
		return dist_a_b


	def obtainHumanPose(self, humansFromCam):
		if self._current_person_list is None:
			self._current_person_list = humansFromCam.humanList
			# print self._current_person_list
		else:
			self.calculate_matching(humansFromCam)
