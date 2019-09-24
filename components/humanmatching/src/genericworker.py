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

import logging
import sys, Ice, os
from PySide2 import QtWidgets, QtCore

logger = logging.getLogger(__name__)

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'

preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ --all /opt/robocomp/interfaces/"
Ice.loadSlice(preStr+"CommonBehavior.ice")
import RoboCompCommonBehavior

additionalPathStr = ''
icePaths = [ '/opt/robocomp/interfaces' ]
try:
	SLICE_PATH = os.environ['SLICE_PATH'].split(':')
	for p in SLICE_PATH:
		icePaths.append(p)
		additionalPathStr += ' -I' + p + ' '
	icePaths.append('/opt/robocomp/interfaces')
except:
	print 'SLICE_PATH environment variable was not exported. Using only the default paths'
	pass

ice_HumanPose = False
for p in icePaths:
	if os.path.isfile(p+'/HumanPose.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"HumanPose.ice"
		Ice.loadSlice(wholeStr)
		ice_HumanPose = True
		break
if not ice_HumanPose:
	print 'Couln\'t load HumanPose'
	sys.exit(-1)
from RoboCompHumanPose import *


from humanposeI import *

try:
	from ui_mainUI import *
except:
	print "Can't import UI file. Did you run 'make'?"
	sys.exit(-1)


class GenericWorker(QtWidgets.QWidget):

	kill = QtCore.Signal()
#Signals for State Machine
	initializetohuman_frames_tracking = QtCore.Signal()
	initializetocameras_matching = QtCore.Signal()
	cameras_matchingtofinalize = QtCore.Signal()
	human_frames_trackingtofinalize = QtCore.Signal()
	check_new_datatocameras_clique = QtCore.Signal()
	cameras_cliquetoresults_update = QtCore.Signal()
	results_updatetoend_camera_matching = QtCore.Signal()
	tracking_initializationtofirst_person_state = QtCore.Signal()
	first_person_statetotracking_state = QtCore.Signal()
	results_updatetoend_camera_matching = QtCore.Signal()
	predictiontoprediction = QtCore.Signal()
	predictiontodata_association = QtCore.Signal()
	data_associationtodata_update = QtCore.Signal()
	data_updatetoprediction = QtCore.Signal()

#-------------------------

	def __init__(self, mprx):
		super(GenericWorker, self).__init__()


		self.ui = Ui_guiDlg()
		self.ui.setupUi(self)
		self.show()


		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
		self.Period = 30
		self.timer = QtCore.QTimer(self)

#State Machine
		self.human_matching_machine= QtCore.QStateMachine()
		self.cameras_matching_state = QtCore.QState(self.human_matching_machine)
		self.human_frames_tracking_state = QtCore.QState(self.human_matching_machine)
		self.initialize_state = QtCore.QState(self.human_matching_machine)

		self.finalize_state = QtCore.QFinalState(self.human_matching_machine)



		self.cameras_clique_state = QtCore.QState(self.cameras_matching_state)
		self.results_update_state = QtCore.QState(self.cameras_matching_state)
		self.check_new_data_state = QtCore.QState(self.cameras_matching_state)

		self.end_camera_matching_state = QtCore.QFinalState(self.cameras_matching_state)



		self.first_person_state_state = QtCore.QState(self.human_frames_tracking_state)
		self.tracking_state_state = QtCore.QState(self.human_frames_tracking_state)
		self.tracking_initialization_state = QtCore.QState(self.human_frames_tracking_state)

		self.end_camera_matching_state = QtCore.QFinalState(self.human_frames_tracking_state)



		self.data_association_state = QtCore.QState(self.tracking_state_state)
		self.data_update_state = QtCore.QState(self.tracking_state_state)
		self.prediction_state = QtCore.QState(self.tracking_state_state)



#------------------
#Initialization State machine
		self.initialize_state.addTransition(self.initializetohuman_frames_tracking, self.human_frames_tracking_state)
		self.initialize_state.addTransition(self.initializetocameras_matching, self.cameras_matching_state)
		self.cameras_matching_state.addTransition(self.cameras_matchingtofinalize, self.finalize_state)
		self.human_frames_tracking_state.addTransition(self.human_frames_trackingtofinalize, self.finalize_state)
		self.check_new_data_state.addTransition(self.check_new_datatocameras_clique, self.cameras_clique_state)
		self.cameras_clique_state.addTransition(self.cameras_cliquetoresults_update, self.results_update_state)
		self.results_update_state.addTransition(self.results_updatetoend_camera_matching, self.end_camera_matching_state)
		self.tracking_initialization_state.addTransition(self.tracking_initializationtofirst_person_state, self.first_person_state_state)
		self.first_person_state_state.addTransition(self.first_person_statetotracking_state, self.tracking_state_state)
		self.results_update_state.addTransition(self.results_updatetoend_camera_matching, self.end_camera_matching_state)
		self.prediction_state.addTransition(self.predictiontoprediction, self.prediction_state)
		self.prediction_state.addTransition(self.predictiontodata_association, self.data_association_state)
		self.data_association_state.addTransition(self.data_associationtodata_update, self.data_update_state)
		self.data_update_state.addTransition(self.data_updatetoprediction, self.prediction_state)


		self.cameras_matching_state.entered.connect(self.sm_cameras_matching)
		self.human_frames_tracking_state.entered.connect(self.sm_human_frames_tracking)
		self.initialize_state.entered.connect(self.sm_initialize)
		self.finalize_state.entered.connect(self.sm_finalize)
		self.check_new_data_state.entered.connect(self.sm_check_new_data)
		self.end_camera_matching_state.entered.connect(self.sm_end_camera_matching)
		self.cameras_clique_state.entered.connect(self.sm_cameras_clique)
		self.results_update_state.entered.connect(self.sm_results_update)
		self.tracking_initialization_state.entered.connect(self.sm_tracking_initialization)
		self.end_camera_matching_state.entered.connect(self.sm_end_camera_matching)
		self.first_person_state_state.entered.connect(self.sm_first_person_state)
		self.tracking_state_state.entered.connect(self.sm_tracking_state)
		self.prediction_state.entered.connect(self.sm_prediction)
		self.data_association_state.entered.connect(self.sm_data_association)
		self.data_update_state.entered.connect(self.sm_data_update)

		self.human_matching_machine.setInitialState(self.initialize_state)
		self.cameras_matching_state.setInitialState(self.check_new_data_state)
		self.human_frames_tracking_state.setInitialState(self.tracking_initialization_state)
		self.tracking_state_state.setInitialState(self.prediction_state)

#------------------

#Slots funtion State Machine
	@QtCore.Slot()
	def sm_cameras_matching(self):
		print "Error: lack sm_cameras_matching in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_human_frames_tracking(self):
		print "Error: lack sm_human_frames_tracking in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_initialize(self):
		print "Error: lack sm_initialize in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_finalize(self):
		print "Error: lack sm_finalize in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_cameras_clique(self):
		print "Error: lack sm_cameras_clique in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_results_update(self):
		print "Error: lack sm_results_update in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_check_new_data(self):
		print "Error: lack sm_check_new_data in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_end_camera_matching(self):
		print "Error: lack sm_end_camera_matching in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_first_person_state(self):
		print "Error: lack sm_first_person_state in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_tracking_state(self):
		print "Error: lack sm_tracking_state in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_tracking_initialization(self):
		print "Error: lack sm_tracking_initialization in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_end_camera_matching(self):
		print "Error: lack sm_end_camera_matching in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_data_association(self):
		print "Error: lack sm_data_association in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_data_update(self):
		print "Error: lack sm_data_update in Specificworker"
		sys.exit(-1)

	@QtCore.Slot()
	def sm_prediction(self):
		print "Error: lack sm_prediction in Specificworker"
		sys.exit(-1)


#-------------------------
	@QtCore.Slot()
	def killYourSelf(self):
		rDebug("Killing myself")
		self.kill.emit()

	# \brief Change compute period
	# @param per Period in ms
	@QtCore.Slot(int)
	def setPeriod(self, p):
		print "Period changed", p
		Period = p
		timer.start(Period)
