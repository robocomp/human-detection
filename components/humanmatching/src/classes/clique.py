#
# Copyright (C) 2019 by Esteban Martinena
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
import logging

logger = logging.getLogger(__name__)

import networkx as nx
import numpy


ABS_THR = 500
REL_THR = 250


def calculate_person_distance( p1, p2):
	a = numpy.array((p1.pos.x, p1.pos.y))
	b = numpy.array((p2.pos.x, p2.pos.y))
	dist_a_b = numpy.sqrt(numpy.sum((a - b) ** 2))
	return dist_a_b

def calculate_clique_matching(input1, input2, noise_vector =[]):
	matching_graph = nx.Graph()
	# camera_id = input.idCamera
	current_person_list = input1
	new_persons_list = input2
	# TODO: This should be moved out of the clique algorithm
	if len(noise_vector) > 0:
		if len(noise_vector)== len(input2)*2:
			new_persons_list = add_noise_to_person_list(new_persons_list, noise_vector)
		else:
			logger.warning("Noise vector must have 2 times the size of person list")
	# if self.ui._noise_checkbox.isChecked():
	# 	new_persons_list = self.add_noise(new_persons_list)
	# 	current_person_list = self.add_noise(current_person_list)
	# 	self._update_person_list_view(current_person_list, self.ui._first_view)
	# 	self._update_person_list_view(new_persons_list, self.ui._second_view)
	logger.debug("Person list input1: %s", str(current_person_list))
	logger.debug("Person list input2: %s", str(new_persons_list))
	for detected_person in new_persons_list:
		for existing_person in current_person_list:
			logger.debug("Indexes: %d %d", detected_person.person_id, existing_person.person_id)
			dist = calculate_person_distance(detected_person, existing_person)
			logger.debug("Distance persons: %d", dist)
			if dist < ABS_THR:
				logger.debug("adding node")
				matching_graph.add_node(str(detected_person.person_id ) +"_ " +str(existing_person.person_id), person1=existing_person, person2=detected_person)
	logger.debug("Nodes added to graph: %d", matching_graph.number_of_nodes())
	for node1_id, node1 in matching_graph.nodes.data():
		for node2_id, node2 in matching_graph.nodes.data():
			if node1 != node2:
				d0 = calculate_person_distance(node1["person1"], node2["person1"])
				d1 = calculate_person_distance(node1["person2"], node2["person2"])
				if numpy.fabs(d0 - d1) < REL_THR:
					logger.debug("Adding edges %s %s", str(node1_id), str(node2_id))
					matching_graph.add_edge(node1_id, node2_id)

	result = nx.find_cliques(matching_graph)
	max_nodes = -1
	max_clique = []
	for r in result:
		if len(r) > max_nodes:
			max_nodes = len(r)
			max_clique = r
		logger.debug("Nodes in result: %s", str(r))

	return max_clique, matching_graph

#Noise vector need to have 2 times the size of person_list (2 coordinates)
def add_noise_to_person_list(person_list, noise_vector):
	"""Add noise to the x and y coordinates of the """

	person_list_copy = copy.deepcopy(person_list)
	for index, detected_person in enumerate(person_list):
		logger.debug("Person %d, %d", detected_person.pos.x, detected_person.pos.z)
		detected_person.pos.x += noise_vector[index * 2]
		detected_person.pos.z += noise_vector[index * 2 + 1]
		logger.debug("Person with noise %d, %d", detected_person.pos.x, detected_person.pos.z)
	return person_list_copy


def create_noise_vector(self, vector_size, mu=0, sigma = 0.1, scale_factor=1):
		"""Function to create a gausian noise vector"""

		noise_vector = numpy.random.normal(mu, sigma, vector_size)
		noise_vector = noise_vector * scale_factor

		return noise_vector