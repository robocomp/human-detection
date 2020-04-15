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
import traceback
import torch
import numpy as np
import cv2
from openpifpaf.network import nets
from openpifpaf import decoder, show, transforms
import argparse
import time
import PIL
from PySide2.QtCore import QMutexLocker
import math 
import b0RemoteApi
import threading

COCO_IDS = ["nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow",
            "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee",
            "left_ankle", "right_ankle"]
SKELETON_CONNECTIONS = [("left_ankle", "left_knee"),
                        ("left_knee", "left_hip"),
                        ("right_ankle", "right_knee"),
                        ("right_knee", "right_hip"),
                        ("left_hip", "right_hip"),
                        ("left_shoulder", "left_hip"),
                        ("right_shoulder", "right_hip"),
                        ("left_shoulder", "right_shoulder"),
                        ("left_shoulder", "left_elbow"),
                        ("right_shoulder", "right_elbow"),
                        ("left_elbow", "left_wrist"),
                        ("right_elbow", "right_wrist"),
                        ("left_eye", "right_eye"),
                        ("nose", "left_eye"),
                        ("nose", "right_eye"),
                        ("left_eye", "left_ear"),
                        ("right_eye", "right_ear"),
                        ("left_ear", "left_shoulder"),
                        ("right_ear", "right_shoulder")]




# openpifpaf configuration
class Args:
	source = 0
	checkpoint = None
	basenet = None
	dilation = None
	dilation_end = None
	headnets = ['pif', 'paf']
	dropout = 0.0
	quad = 1
	pretrained = False
	keypoint_threshold = None
	seed_threshold = 0.2
	force_complete_pose = False
	debug_pif_indices = []
	debug_paf_indices = []
	connection_method = 'max'
	fixed_b = None
	pif_fixed_scale = None
	profile_decoder = None
	instance_threshold = 0.05
	device = torch.device(type="cuda")
	disable_cuda = False
	scale = 1
	key_point_threshold = 0.05
	head_dropout = 0.0
	head_quad = 0
	default_kernel_size = 1
	default_padding = 0
	default_dilation = 1
	head_kernel_size = 1
	head_padding = 0
	head_dilation = 0
	cross_talk = 0.0
	two_scale = False
	multi_scale = False
	multi_scale_hflip = False
	paf_th = 0.1
	pif_th = 0.1
	decoder_workers = None
	experimental_decoder = False
	extra_coupling = 0.0


def processPifPaf(processor, img, scale, pifResult):
	image = cv2.resize(img, None, fx=scale, fy=scale)
	image_pil = PIL.Image.fromarray(image)
	processed_image_cpu, _, __ = transforms.EVAL_TRANSFORM(image_pil, [], None)
	processed_image = processed_image_cpu.contiguous().to(non_blocking=True).cuda()
	fields = processor.fields(torch.unsqueeze(processed_image, 0))[0]
	keypoint_sets, _ = processor.keypoint_sets(fields)
	pifResult.append(keypoint_sets)


def getDepth2(image, i, j, simulation):
	OFFSET = 19
	x = i-OFFSET
	y = j-OFFSET
	x_max = np.minimum(i + OFFSET, image.shape[0])
	y_max = np.minimum(j + OFFSET, image.shape[1])
	image_roi = image[y:y_max, x: x_max]
	min = cv2.reduce(image_roi, -1, cv2.REDUCE_MIN)
	if simulation:
		return np.min(min) * 1000
	return np.min(min)


	#return median depth value
def getDepth(image, i, j, simulation):
	OFFSET = 19
	values = []
	for xi in range(i-OFFSET, i+OFFSET):
		for xj in range(j-OFFSET, j+OFFSET):
			if math.isnan(image[xj, xi]):
				print("NAN value")
			else:
				if image[xj, xi] > 0.0:
					values.append(image[xj, xi])
	if not values:
		print("Not values")
		return 0
	else:
		if simulation:
			return np.min(values) * 1000  # VREP to mm
		return np.min(values)


if __name__ == '__main__':
	mat = np.full((480, 640), 150.0)
	mat[240][320] = 1
	mat[240][321] = 10
	min1 = getDepth(mat, 310, 230, False)
	min2 = getDepth2(mat, 310, 230, False)
	print(min1, min2)


def processDescriptor(image, depth, simulation, scale, keypoint_sets, focal, descriptor_size, desResult):
	#image descriptors
	grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	orb_extractor = cv2.ORB_create()
	peoplelist = []
	# create joint dictionary
	for id, p in enumerate(keypoint_sets):
		person = RoboCompHumanCameraBody.Person()
		person.id = id
		person.joints = dict()
		for pos, joint in enumerate(p):
			if float(joint[2]) > 0.5:
				keypoint = RoboCompHumanCameraBody.KeyPoint()
				keypoint.i = int(joint[0] / scale)
				keypoint.j = int(joint[1] / scale)
				keypoint.score = float(joint[2])

				ki = keypoint.i - 320
				kj = 240 - keypoint.j
				#pdepth = float(getDepth(depth, keypoint.i, keypoint.j, simulation))
				pdepth = float(getDepth2(depth, keypoint.i, keypoint.j, simulation))
				#print("depth", pdepth, pdepth2)
				if pdepth < 10000 and pdepth > 0:
					keypoint.z = pdepth
					keypoint.x = ki * keypoint.z / focal
					keypoint.y = kj * keypoint.z / focal

					# descriptors
					desKeypoint = cv2.KeyPoint(keypoint.i, keypoint.j, descriptor_size, -1)
					kp, des = orb_extractor.compute(grey, [desKeypoint])
					cv2.drawKeypoints(grey, kp, grey, color=(255, 0, 0), flags=0)
					if type(des).__module__ == np.__name__:
						keypoint.floatdesclist = des.tolist()

					person.joints[COCO_IDS[pos]] = keypoint
				else:
					print("Incorrect depth")
		if len(person.joints) > 5:
			peoplelist.append(person)
	desResult.append(peoplelist)



class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.params = {}
		self.cameraid = 0
		self.adepth = []
		self.bdepth = []
		self.acolor = []
		self.bcolor = []
		self.viewimage = False
		self.peoplelist = []
		self.timer.timeout.connect(self.compute)
		self.Period = 50
		self.contFPS = 0
		self.descriptor_size = 10
		self.akeypoint_sets = []
		self.bkeypoint_sets = []
		self.bill_pos = [99999, 99999, 99999]
		self.bill_ori = [0.0, 0.0, 0.0]
		self.processor = None

	def __del__(self):
		print('SpecificWorker destructor')

	def setParams(self, params):
		self.params = params
		self.cameraid = int(self.params["cameraid"])
		self.verticalflip = "true" in self.params["verticalflip"]
		self.horizontalflip = "true" in self.params["horizontalflip"]
		self.viewimage = "true" in self.params["viewimage"]
		self.simulation = "true" in self.params["simulation"]
		if self.simulation:
			self.focal = 462 #VREP
		else:
			self.focal = 617 #REALSENSE
		self.initialize()
		self.timer.start(self.Period)
		return True

	def initialize(self):
		args = Args()
		model, _ = nets.factory_from_args(args)
		model = model.to(args.device)
		self.processor = decoder.factory_from_args(args, model)

		if self.simulation:
			self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApiAddOn')
			self.bill = self.client.simxGetObjectHandle('Bill_base#1', self.client.simxServiceCall())

		self.start = time.time()

	@QtCore.Slot()
	def compute(self):
		start = time.time()
#		print("Timer intialization")
		try:
			color_, depth_ = self.camerargbdsimple_proxy.getAll()
			if (len(color_.image) == 0) or (len(depth_.depth) == 0):
				print ('Error retrieving images!')
		except Ice.Exception:
			print("Error connecting to camerargbd")
			return
		t1 = time.time()
#		print("Receive images:", ((t1 - start)*1000 ))
		if self.simulation:
			p_success, self.bill_pos = self.client.simxGetObjectPosition(self.bill[1], -1, self.client.simxServiceCall())
			o_success, self.bill_ori = self.client.simxGetObjectOrientation(self.bill[1], -1, self.client.simxServiceCall())
			if p_success == False or o_success == False or all(np.abs(x) < 0.1 for x in self.bill_pos):
				print("Error reading pose")
				return
		#print("pose", self.bill_pos, self.bill_ori)
		t2 = time.time()
#		print("Read bill vrep:", ((t2 - t1) * 1000))
		self.width = color_.width
		self.adepth = np.frombuffer(depth_.depth, dtype=np.float32).reshape(depth_.height, depth_.width)
		self.acolor = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
		t3 = time.time()
#		print("Convert images:", ((t3 - t2) * 1000))

		if self.horizontalflip:
			self.acolor = cv2.flip(self.acolor, 1)
			self.adepth = cv2.flip(self.adepth, 1)
		if self.verticalflip:
			self.acolor = cv2.flip(self.acolor, 0)
			self.adepth = cv2.flip(self.adepth, 0)
		t4 = time.time()
#		print("Flip images:", ((t4 - t3)) * 1000)
		scale = 0.5

		pifResults = []
		p1 = threading.Thread(target=processPifPaf, args=(self.processor, self.acolor, 0.5, pifResults))
		p1.start()

		desResults = []

		p2 = threading.Thread(target=processDescriptor, args=(self.bcolor, self.bdepth, self.simulation, 0.5,
															  self.bkeypoint_sets, self.focal, self.descriptor_size,
															  desResults))


		if len(self.bcolor) > 0:
			p2.start()
			p2.join()
			self.peoplelist = desResults[0]

		p1.join()
		self.bkeypoint_sets = pifResults[0]


		t5 = time.time()
#		print("OpenPifPaf completo:", ((t5 - t4)) * 1000)
		self.publishData()
		t6 = time.time()
#		print("Publish:", ((t6 - t5)) * 1000)
		if self.viewimage and len(self.bcolor) > 0:
			self.drawImage(self.bcolor, self.peoplelist)
			cv2.imshow("Color frame", self.bcolor)
			cv2.waitKey(1)



		# swap data
		self.acolor, self.bcolor = self.bcolor, self.acolor
		self.adepth, self.bdepth = self.bdepth, self.adepth
		self.akeypoint_sets, self.bkeypoint_sets = self.bkeypoint_sets, self.akeypoint_sets

		if time.time() - self.start > 1:
			print("FPS:", self.contFPS)
			self.start = time.time()
			self.contFPS = 0
		self.contFPS += 1

		print("Total time:", ((time.time() - start) * 1000))
		return True

	def drawImage(self, image, peoplelist):
		# draw
		for person in peoplelist:
			for name1, name2 in SKELETON_CONNECTIONS:
				try:
					joint1 = person.joints[name1]
					joint2 = person.joints[name2]
					if joint1.score > 0.5:
						cv2.circle(image, (joint1.i, joint1.j), 10, (0, 0, 255))
					if joint2.score > 0.5:
						cv2.circle(image, (joint2.i, joint2.j), 10, (0, 0, 255))
					if joint1.score > 0.5 and joint2.score > 0.5:
						cv2.line(image, (joint1.i, joint1.j), (joint2.i, joint2.j), (0, 255, 0), 2)
				except:
					pass

	# roi containing joints detected
	def getRoi(self, person):
		roi = RoboCompHumanCameraBody.TImage()
		points = [[joint.i, joint.j] for key, joint in person.joints.items()]
		points = np.array(points, dtype=np.float32)
		x, y, w, h = cv2.boundingRect(points)
		roi.width = w
		roi.height = h
		roi.image = self.bcolor[x:x+w, y:y+h].tobytes()
		return roi

######################################
##### PUBLISHER
######################################

	def publishData(self):
		people = RoboCompHumanCameraBody.PeopleData()
		people.cameraId = self.cameraid
		people.timestamp = int(round(time.time() * 1000))
		people.peoplelist = self.peoplelist
		if len(people.peoplelist) > 0:
			people.peoplelist[0].x = self.bill_pos[0] * 1000.0
			people.peoplelist[0].y = self.bill_pos[1] * 1000.0
			people.peoplelist[0].z = self.bill_pos[2] * 1000.0
			people.peoplelist[0].rx = self.bill_ori[0]
			people.peoplelist[0].ry = self.bill_ori[1]
			people.peoplelist[0].rz = self.bill_ori[2]
			for person in people.peoplelist:
				person.roi = self.getRoi(person)

			try:
				self.humancamerabody_proxy.newPeopleData(people)
			except:
				print("Error on camerabody data publication")
				traceback.print_exc()
