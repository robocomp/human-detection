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
import sys, Ice, os
from PySide2 import QtWidgets, QtCore

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
	print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
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
	print('SLICE_PATH environment variable was not exported. Using only the default paths')
	pass

ice_PeopleServer = False
class GenericWorker(QtCore.QObject):

	kill = QtCore.Signal()
#Signals for State Machine
	initialize_videotoprocessing_video = QtCore.Signal()
	initialize_videotofinalize_video = QtCore.Signal()
	processing_videotofinalize_video = QtCore.Signal()
	reading_framestoreading_frames = QtCore.Signal()
	reading_framestocounting = QtCore.Signal()
	countingtoreading_frames = QtCore.Signal()

#-------------------------

	def __init__(self, mprx):
		super(GenericWorker, self).__init__()		
		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
		self.Period = 30
		self.timer = QtCore.QTimer(self)

#State Machine
		self.peopleCounterMachine= QtCore.QStateMachine()
		self.processing_video_state = QtCore.QState(self.peopleCounterMachine)
		self.initialize_video_state = QtCore.QState(self.peopleCounterMachine)
		self.finalize_video_state = QtCore.QFinalState(self.peopleCounterMachine)

		self.reading_frames_state = QtCore.QState(self.processing_video_state)
		self.counting_state = QtCore.QState(self.processing_video_state)


#------------------
#Initialization State machine
		
		self.initialize_video_state.addTransition(self.initialize_videotoprocessing_video, self.processing_video_state)
		self.initialize_video_state.addTransition(self.initialize_videotofinalize_video, self.finalize_video_state)
		self.processing_video_state.addTransition(self.processing_videotofinalize_video, self.finalize_video_state)
		self.reading_frames_state.addTransition(self.reading_framestocounting, self.counting_state)
		self.reading_frames_state.addTransition(self.reading_framestoreading_frames, self.reading_frames_state)
		self.counting_state.addTransition(self.countingtoreading_frames, self.reading_frames_state)
		
		self.processing_video_state.entered.connect(self.sm_processing_video)
		self.initialize_video_state.entered.connect(self.sm_initialize_video)
		self.finalize_video_state.entered.connect(self.sm_finalize_video)
		self.reading_frames_state.entered.connect(self.sm_reading_frames)
		self.counting_state.entered.connect(self.sm_counting)

		self.peopleCounterMachine.setInitialState(self.initialize_video_state)
		self.processing_video_state.setInitialState(self.reading_frames_state)

#------------------

#Slots funtion State Machine
	@QtCore.Slot()
	def sm_processing_video(self):
		print("Error: lack sm_processing_video in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_initialize_video(self):
		print("Error: lack sm_initialize_video in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_finalize_video(self):
		print("Error: lack sm_finalize_video in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_counting(self):
		print("Error: lack sm_detecting in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_reading_frames(self):
		print("Error: lack sm_reading_frames in Specificworker")
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
		print("Period changed", p)
		Period = p
		timer.start(Period)
