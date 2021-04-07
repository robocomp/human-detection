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
import queue
import pyrealsense2 as rs


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

peoplelist_queue = queue.SimpleQueue()
openpifpaf_queue = queue.SimpleQueue()
descriptors_queue = queue.SimpleQueue()

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

class Process_Descriptor(threading.Thread):
	def __init__(self, scale, descriptor_size, tm):
		threading.Thread.__init__(self)
		self.scale = scale
		self.descriptor_size = descriptor_size
		self.tm = tm

	def run(self):
		while True:
			[image, depth, focal, keypoint_sets] = descriptors_queue.get(block=True)
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
						keypoint.i = int(joint[0] / self.scale)
						keypoint.j = int(joint[1] / self.scale)
						keypoint.score = float(joint[2])

						ki = keypoint.i - width / 2
						kj = height / 2 - keypoint.j
						pdepth = self.getDepth(depth, keypoint.i, keypoint.j, False)
						if pdepth < 10000 and pdepth > 0:
							keypoint.z = pdepth
							keypoint.x = ki * keypoint.z / focal
							keypoint.y = kj * keypoint.z / focal
							# compute world transform
							p = pt.transform(self.tm.get_transform("camera", "world"),
											 np.array([keypoint.x, -keypoint.y, keypoint.z, 1]))
							keypoint.wx = p[1]
							keypoint.wy = p[0]
							keypoint.wz = -p[2]
							# descriptors
							desKeypoint = cv2.KeyPoint(keypoint.i, keypoint.j, self.descriptor_size, -1)
							kp, des = orb_extractor.compute(grey, [desKeypoint])
							cv2.drawKeypoints(grey, kp, grey, color=(255, 0, 0), flags=0)
							if type(des).__module__ == np.__name__:
								keypoint.floatdesclist = des.tolist()
							person.joints[COCO_IDS[pos]] = keypoint
						else:
							print("Incorrect depth")
				if len(person.joints) > 2:
					peoplelist.append(person)
			peoplelist_queue.put(peoplelist)

	def getDepth(self, image, i, j, median=False):
		OFFSET = 19
		x = i - OFFSET
		y = j - OFFSET
		x_max = np.minimum(i + OFFSET, image.shape[0])
		y_max = np.minimum(j + OFFSET, image.shape[1])
		image_roi = image[y:y_max, x: x_max]
		if median:
			image_roi = cv2.medianBlur(image_roi, 3)
		min = cv2.reduce(image_roi, -1, cv2.REDUCE_MIN)
		return float(np.min(min) * 1000.0)

class OpenPifPaf_Extractor(threading.Thread):
	def __init__(self, processor, scale, args, model):
		threading.Thread.__init__(self)
		self.scale = scale
		self.processor = processor
		self.model = model
		self.args = args

		self.preprocess = transforms.Compose([
			transforms.NormalizeAnnotations(),
			transforms.CenterPadTight(16),
			transforms.EVAL_TRANSFORM,
		])

	def run(self):
		while True:
			[img, depth, focalx] = openpifpaf_queue.get(block=True)
			image = cv2.resize(img, None, fx=self.scale, fy=self.scale)
			image_pil = PIL.Image.fromarray(image)
			meta = {
				'hflip': False,
				'offset': np.array([0.0, 0.0]),
				'scale': np.array([1.0, 1.0]),
				'valid_area': np.array([0.0, 0.0, image_pil.size[0], image_pil.size[1]]),
			}
			processed_image, _, meta = self.preprocess(image_pil, [], meta)
			preds = self.processor.batch(self.model, torch.unsqueeze(processed_image, 0), device=self.args.device)[0]
			descriptors_queue.put([img, depth, focalx, preds])

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map, startup_check=False):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.params = {}
		self.cameraid = 0
		self.gt = []
		self.adepth = []
		self.acolor = []
		self.viewimage = False
		self.timer.timeout.connect(self.compute)
		self.Period = 50
		self.contFPS = 0
		self.descriptor_size = 10
		self.processor = None
		self.tm = TransformManager()
		self.args = Args()
		self.scale = 0.5

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
		self.do_calibrate = "true" in self.params["calibrate"]
		self.do_realsense = "true" in self.params["realsense"]

		self.hide()
		self.initialize()
		#self.timer.setSingleShot(True)
		self.timer.start(self.Period)
		return True

	def initialize(self):

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
			self.desc_processor = Process_Descriptor(scale=self.scale, descriptor_size=self.descriptor_size, tm=self.tm)
			self.desc_processor.daemon = True
			self.desc_processor.start()
			self.openpifpaf_processor = OpenPifPaf_Extractor(self.processor, 0.5, self.args, self.model)
			self.openpifpaf_processor.daemon = True
			self.openpifpaf_processor.start()

		if self.do_realsense:
			# realsense configuration
			try:
				config = rs.config()
				config.enable_device(self.params["device_serial"])
				config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, 30)
				config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)

				self.pointcloud = rs.pointcloud()
				self.pipeline = rs.pipeline()
				cfg = self.pipeline.start(config)
			#            profile = cfg.get_stream(rs.stream.color) # Fetch stream profile for depth stream
			#            intr = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
			#            print (intr.fx, intr.fy)
			#            depth_scale = cfg.get_device().first_depth_sensor().get_depth_scale()
			#            print("Depth Scale is: " , depth_scale)
			#            sys.exit(-1)
			except Exception as e:
				print("Error initializing camera")
				print(e)
				sys.exit(-1)

		self.start = time.time()

	@QtCore.Slot()
	def compute(self):
		if self.do_realsense:
			frames = self.pipeline.wait_for_frames()
			if not frames:
				return
			depthData = frames.get_depth_frame()
			self.adepth = np.asanyarray(depthData.get_data(), dtype=np.float32)
			self.acolor = np.asanyarray(frames.get_color_frame().get_data())

		else:
			try:
				rgb, depth = self.camerargbdsimple_proxy.getAll("cam")
				self.adepth = np.frombuffer(depth.depth, dtype=np.float32).reshape(depth.height, depth.width)
				self.acolor = np.frombuffer(rgb.image, np.uint8).reshape(rgb.height, rgb.width, rgb.depth)
			except Ice.Exception as e:
				print("Error connecting to camerargbd", e)
				return

		if self.horizontalflip:
			self.acolor = cv2.flip(self.acolor, 1)
			self.adepth = cv2.flip(self.adepth, 1)
		if self.verticalflip:
			self.acolor = cv2.flip(self.acolor, 0)
			self.adepth = cv2.flip(self.adepth, 0)


		if self.do_calibrate:
			transform = self.calibrate_with_apriltag(self.acolor, rgb.focalx, rgb.width, rgb.height)
			if len(transform) > 0:
				self.tm.add_transform("world", "camera", transform)
				self.do_calibrate = False  # Do it once
				print("Camera to World transform", self.tm.get_transform("camera", "world"))
			else:
				sys.exit()

		# send data to threads
		openpifpaf_queue.put([self.acolor, self.adepth, rgb.focalx])
		peoplelist = peoplelist_queue.get()

		if self.publishimage:
			im = RoboCompCameraRGBDSimple.TImage()
			im.cameraID = self.cameraid
			im.width = self.width
			im.height = self.height
			im.focalx = self.color_focal_x
			im.focaly = self.color_focal_y
			im.depth = 3
			im.image = self.acolor

			dep = RoboCompCameraRGBDSimple.TDepth()
			dep.cameraID = self.cameraid
			dep.width = self.width
			dep.height = self.height
			dep.focalx = self.depth_focal_x
			dep.focaly = self.depth_focal_y
			# dep.depth = self.adepth

			try:
				dep.alivetime = (time.time() - self.capturetime) * 1000
				im.alivetime = (time.time() - self.capturetime) * 1000
				self.camerargbdsimplepub_proxy.pushRGBD(im, dep)
			except Exception as e:
				print("Error on camerabody data publication")
				print(e)

		# draw
		if self.viewimage and len(self.acolor) > 0 and len(peoplelist) > 0:
			#self.drawImage(self.acolor, peoplelist)
			self.draw_points_in_coppelia(peoplelist[0])
			#plt.imshow(self.bcolor)
			#plt.pause(0.001)

		# time
		if time.time() - self.start > 1:
			print("FPS:", self.contFPS)
			self.start = time.time()
			self.contFPS = 0
		self.contFPS += 1
		return True

	###########################################################################
	# 3D-world
	###########################################################################
	def draw_points_in_coppelia(self, person):
		names = ['left_shoulder', 'right_shoulder', 'left_hip', 'right_hip', "left_elbow", "right_elbow", "left_knee", "right_knee"]
		for name, keypoint in person.joints.items():
			try:
				if name in names and keypoint.score > 0.5:
					try:
						body_pose = RoboCompCoppeliaUtils.PoseType()
						body_pose.x = keypoint.wx
						body_pose.y = keypoint.wy
						body_pose.z = keypoint.wz
						self.coppeliautils_proxy.addOrModifyDummy(RoboCompCoppeliaUtils.TargetTypes.Info, name, body_pose)
					except Exception as e:
						print(e)
			except:
				pass

###########################################################################
# Apriltags calibration
###########################################################################
	def calibrate_with_apriltag(self, rgb, focal, width, height):
		grey = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
		transform = []
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
			transform = pt.transform_from(tags[0].pose_R, t)
		return transform

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

####################################################################################################

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
