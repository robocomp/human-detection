#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2020 by YOUR NAME HERE
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

from genericworker import *

import pyrealsense2 as rs
import json
import time
import math

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 30
		

	def initialize(self):
		# realsense configuration
		try:
			self.pipeline = rs.pipeline()
			config = rs.config()
			config.enable_stream(rs.stream.pose)
			cfg = self.pipeline.start(config)
		except Exception as e:
			print("Error initializing camera")
			print(e)
			sysexit(-1)
		#open write file
		self.outfile = open('realsense_pose.txt', 'w')
		self.outfile.write('"data_set":[')
		self.timer.start(self.Period)


	def __del__(self):
		print('SpecificWorker destructor')
		if self.pipeline:
			self.pipeline.stop()
		if self.outfile:
			self.outfile.write("]}")
			self.outfile.close()

	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print("Error reading config params")
		self.initialize()
		return True

	@QtCore.Slot()
	def compute(self):
		print('SpecificWorker.compute...')
		
		frames = self.pipeline.wait_for_frames()
		
		# Fetch pose frame
		pose = frames.get_pose_frame()
		if pose:
			# Print some of the pose data to the terminal
			data = pose.get_pose_data()
			print("Frame #{}".format(pose.frame_number))
			print("Position: {}".format(data.translation))
			print("Velocity: {}".format(data.velocity))
			print("Acceleration: {}".format(data.acceleration))
			print("Rotation yaw: ",self.quaternion2euler(data),"\n")
			self.outfile.write('{"cameraId":0,"timestamp":'
				+str(time.time()*1000) 
				+',"ground_truth":['
				+str(data.translation.x*1000)+','
				+str(data.translation.y*1000)+','
				+str(data.translation.z*1000)+","
				+str(self.quaternion2euler(data))+"]}")
			self.outfile.write(",\n")
		return True


	def quaternion2euler(self, data):
		w = data.rotation.w
		x = -data.rotation.z
		y = data.rotation.x
		z = -data.rotation.y

		pitch =  -math.asin(2.0 * (x*z - w*y))
		roll  =  math.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z)
		yaw   =  math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z)
		
		return yaw
