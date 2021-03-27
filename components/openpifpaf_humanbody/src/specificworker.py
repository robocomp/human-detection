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
import matplotlib.pyplot as plt
from openpifpaf import decoder, show, transforms, network
import time
import PIL
import math
import threading
import dt_apriltags as april
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager

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

def processPifPaf(processor, img, scale, pifResult, args, model):
	preprocess = transforms.Compose([
		transforms.NormalizeAnnotations(),
		transforms.CenterPadTight(16),
		transforms.EVAL_TRANSFORM,
	])
	image = cv2.resize(img, None, fx=scale, fy=scale)
	image_pil = PIL.Image.fromarray(image)
	#processed_image_cpu, _, __ = transforms.EVAL_TRANSFORM(image_pil, [], None)
	#processed_image = processed_image_cpu.contiguous().to(non_blocking=True).cuda()
	meta = {
		'hflip': False,
		'offset': np.array([0.0, 0.0]),
		'scale': np.array([1.0, 1.0]),
		'valid_area': np.array([0.0, 0.0, image_pil.size[0], image_pil.size[1]]),
	}
	processed_image, _, meta = preprocess(image_pil, [], meta)
	#fields = processor.fields(torch.unsqueeze(processed_image, 0))[0]
	preds = processor.batch(model, torch.unsqueeze(processed_image, 0), device=args.device)[0]
	#keypoint_sets, _ = processor.keypoint_sets(preds.data)
	pifResult.append(preds)

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

def processDescriptor(image, depth, simulation, scale, keypoint_sets, focal, descriptor_size, desResult):
	#image descriptors
	grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	orb_extractor = cv2.ORB_create()
	peoplelist = []
	height, width = image.shape[:2]
	# create joint dictionary
	for id, p in enumerate(keypoint_sets):
		person = RoboCompHumanCameraBody.Person()
		person.id = id
		person.joints = dict()
		for pos, joint in enumerate(p.data):
			if float(joint[2]) > 0.5:
				keypoint = RoboCompHumanCameraBody.KeyPoint()
				keypoint.i = int(joint[0] / scale)
				keypoint.j = int(joint[1] / scale)
				keypoint.score = float(joint[2])

				ki = keypoint.i - width/2
				kj = height/2 - keypoint.j
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
	def __init__(self, proxy_map, startup_check=False):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.params = {}
		self.cameraid = 0
		self.gt = []
		self.bill = []
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
		self.cameraname = self.params["cameraname"]
		self.do_openpifpaf = "true" in self.params["openpifpaf"]
		if self.simulation:
			#self.focal = 462  # VREP
			self.focal = 443.4
		else:
			self.focal = 617  # REALSENSE

		self.hide()
		self.initialize()
		#self.timer.setSingleShot(True)
		self.timer.start(self.Period)
		return True

	def initialize(self):

		self.peoplelist = []

		#apriltags
		self.detector = april.Detector(searchpath=['apriltags'], families='tag36h11', nthreads=5,
									   quad_decimate=1.0, quad_sigma=0.1, refine_edges=1, decode_sharpening=0.25,
									   debug=0)

		# openpifpaf
		if self.do_openpifpaf:
			self.args = Args()
			model, _ = network.Factory().factory()
			self.model = model.to(self.args.device)
			head_metas = [hn.meta for hn in model.head_nets]
			self.processor = decoder.factory(head_metas)

		self.start = time.time()

	@QtCore.Slot()
	def compute(self):
		try:
			all_ = self.camerargbdsimple_proxy.getAll(self.cameraname)
			rgb = all_.image
			self.adepth = np.frombuffer(all_.depth.depth, dtype=np.float32).reshape(all_.depth.height, all_.depth.width)
			self.acolor = np.frombuffer(all_.image.image, np.uint8).reshape(rgb.height, rgb.width, all_.image.depth)
			if self.horizontalflip:
				self.acolor = cv2.flip(self.acolor, 1)
				self.adepth = cv2.flip(self.adepth, 1)
			if self.verticalflip:
				self.acolor = cv2.flip(self.acolor, 0)
				self.adepth = cv2.flip(self.adepth, 0)
		except Ice.Exception:
			print("Error connecting to camerargbd")
			return

		self.calibrate_with_apriltag(self.acolor, self.focal, rgb.width, rgb.height)

		# openPifPaf threads
		if self.do_openpifpaf:
			scale = 0.5
			pifResults = []
			p1 = threading.Thread(target=processPifPaf, args=(self.processor, self.acolor, 0.5, pifResults, self.args, self.model))
			p1.start()
			desResults = []
			p2 = threading.Thread(target=processDescriptor, args=(self.bcolor, self.bdepth, self.simulation, 0.5, self.bkeypoint_sets, self.focal, self.descriptor_size, desResults))

			if len(self.bcolor) > 0:
				p2.start()
				p2.join()
				self.peoplelist = desResults[0]
			p1.join()
			self.bkeypoint_sets = pifResults[0]

		# draw
		if self.viewimage and len(self.bcolor) > 0:
			self.drawImage(self.bcolor, self.peoplelist)
			plt.imshow(self.bcolor)
			plt.show()
			self.draw_points_in_coppelia(self.peoplelist[0])

		# swap data
		self.acolor, self.bcolor = self.bcolor, self.acolor
		self.adepth, self.bdepth = self.bdepth, self.adepth
		self.akeypoint_sets, self.bkeypoint_sets = self.bkeypoint_sets, self.akeypoint_sets

		# time
		if time.time() - self.start > 1:
			print("FPS:", self.contFPS)
			self.start = time.time()
			self.contFPS = 0
		self.contFPS += 1
		return True

	def draw_points_in_coppelia(self, person):
		names = ['left_shoulder', 'right_shoulder', 'left_hip', 'right_hip']
		for name, joint in person.joints.items():
			try:
				if name in names and joint.score > 0.5:
					try:
						#print(name, joint.x, joint.y, joint.z)
						p = pt.transform(self.tm.get_transform("camera", "world"), np.array([joint.x, -joint.y, joint.z, 1]))
						body_pose = RoboCompCoppeliaUtils.PoseType()
						body_pose.x = p[1]
						body_pose.y = p[0]
						body_pose.z = -p[2]
						self.coppeliautils_proxy.addOrModifyDummy(RoboCompCoppeliaUtils.TargetTypes.Info, name, body_pose)
					except Exception as e:
						print(e)
			except:
				pass

###########################################################################
# Apriltags calibration
###########################################################################
	def calibrate_with_apriltag(self, rgb, focal, width, height):
		self.tm = TransformManager()
		# add transform from world to tag
		#w2tag = pt.transform_from(, [0.0,0.0,0.0])
		#self.tm.add_transform("world", "tag", w2tag)

		grey = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
		tags = self.detector.detect(grey, estimate_tag_pose=True, camera_params=[focal, focal, width/2.0, height/2.0], tag_size=0.275)
		if len(tags) > 0:
			tag = tags[0]
			for idx in range(len(tag.corners)):
				cv2.line(rgb, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)),
						 (0, 255, 0))
				cv2.putText(rgb, str(tag.tag_id),
							org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
							fontFace=cv2.FONT_HERSHEY_SIMPLEX,
							fontScale=0.8,
							color=(0, 0, 255))

			t = tags[0].pose_t.ravel() * 1000.0
			w2c = pt.transform_from(tags[0].pose_R, t)
			#print(w2c)
			#print(tags[0].pose_R, t)
			self.tm.add_transform("world", "camera", w2c)
		else:
			transform = None

###########################################################################
#
###########################################################################
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

		# increase roi size (15% width, 10% height)
		x = int(x - 0.075 * w)
		y = int(y - 0.05 * h)
		h = int(h * 1.1)
		w = int(w * 1.15)

		roi.width = w
		roi.height = h
		roi.image = self.bcolor[y:y+h, x:x+w].tobytes()
		return roi

		# to check roi aspect
		cv2.circle(self.bcolor, (x, y), 10, (0, 0, 255))
		cv2.circle(self.bcolor, (x, y+h), 10, (0, 0, 255))
		cv2.circle(self.bcolor, (x+w, y), 10, (0, 0, 255))
		cv2.circle(self.bcolor, (x+w, y+h), 10, (0, 0, 255))
		cv2.imshow("color", self.bcolor)
		if len(roi.image) > 0:
			cv2.imshow("roi", self.bcolor[y:y+h, x:x+w])
		return roi


######################################
##### PUBLISHER
######################################

	def publishData(self):
		people = RoboCompHumanCameraBody.PeopleData()
		people.cameraId = self.cameraid
		people.timestamp = int(round(time.time() * 1000))
		people.gt = self.gt
		people.peoplelist = self.peoplelist
		if len(people.peoplelist) > 0:
			for person in people.peoplelist:
				person.roi = self.getRoi(person)
			try:
				self.humancamerabody_proxy.newPeopleData(people)
			except:
				print("Error on camerabody data publication")
				traceback.print_exc()
