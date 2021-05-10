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
import copy
import time
#from PIL import Image
import PIL
import threading
import dt_apriltags as april
#from pytransform3d import rotations as pr
#from pytransform3d import transformations as pt
#from pytransform3d.transform_manager import TransformManager
import queue
import json
import pyrealsense2 as rs

import trt_pose.coco
import trt_pose.models
import torch2trt
from torch2trt import TRTModule
import torchvision.transforms as transforms
from trt_pose.parse_objects import ParseObjects
device = torch.device("cuda")

model = 'resnet'
# model = 'densenet'


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

peoplelist_queue = queue.Queue(1)
openpifpaf_queue = queue.Queue(1)
descriptors_queue = queue.Queue(1)

rgb_width = 0 #probando variables publishimage
rgb_height = 0
rgb_focal_x = 0
rgb_focal_y = 0
rgb_depth = 0


class Process_Descriptor(threading.Thread):
	def __init__(self, scale, descriptor_size, tm, depth_scale):
		threading.Thread.__init__(self)
		self.scale = scale
		self.descriptor_size = descriptor_size
		self.tm = tm
		self.depth_scale = depth_scale

	def run(self):
		while True:
			[image, depth, focal, keypoint_sets, tm, counts, objects, peaks ] = descriptors_queue.get(block=True)
			#grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
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
						pdepth = self.getDepth(depth, keypoint.i, keypoint.j, False) * self.depth_scale
						#print("pdepth " , pdepth)
						if pdepth < 10000 and pdepth > 0:
							keypoint.z = pdepth
							keypoint.x = ki * keypoint.z / focal
							keypoint.y = kj * keypoint.z / focal
							#print(keypoint.x, keypoint.y)
							# compute world transform
							p = pt.transform(tm.get_transform("camera", "world"),
											 np.array([keypoint.x, -keypoint.y, keypoint.z, 1]))
							keypoint.xw = p[1]
							keypoint.yw = p[0]
							keypoint.zw = -p[2]
							
							# descriptors
							#desKeypoint = cv2.KeyPoint(keypoint.i, keypoint.j, self.descriptor_size, -1)
							#kp, des = orb_extractor.compute(grey, [desKeypoint])
							#cv2.drawKeypoints(grey, kp, grey, color=(255, 0, 0), flags=0)
							#if type(des).__module__ == np.__name__:
							#	keypoint.floatdesclist = des.tolist()
							person.joints[COCO_IDS[pos]] = keypoint
						else:
							#print("Incorrect depth")
							pass
				if len(person.joints) > 2:
					peoplelist.append(person)
			peoplelist_queue.put([image, peoplelist])

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

class Skeleton_Extractor(threading.Thread):
	def __init__(self, do_realsense, pipeline, width, height, rs_focal_x, hf, vf, calibrate):
		threading.Thread.__init__(self)

		self.do_realsense = do_realsense
		self.pipeline = pipeline
		self.width = width
		self.height = height
		self.rs_focal_x = rs_focal_x
		self.horizontalflip = hf
		self.verticalflip = vf
		self.do_calibrate = calibrate

		with open('human_pose.json', 'r') as f:
			human_pose = json.load(f)
		self.topology = trt_pose.coco.coco_category_to_topology(human_pose)
		num_parts = len(human_pose['keypoints'])
		num_links = len(human_pose['skeleton'])

		if model == 'resnet':
			MODEL_WEIGHTS = 'resnet18_baseline_att_224x224_A_epoch_249.pth'
			OPTIMIZED_MODEL = 'resnet18_baseline_att_224x224_A_epoch_249_trt.pth'
			self.model = trt_pose.models.resnet18_baseline_att(num_parts, 2 * num_links).cuda().eval()
			self.WIDTH = 224
			self.HEIGHT = 224
		elif model == 'densenet':
			print('------ model = densenet--------')
			MODEL_WEIGHTS = 'densenet121_baseline_att_256x256_B_epoch_160.pth'
			OPTIMIZED_MODEL = 'densenet121_baseline_att_256x256_B_epoch_160_trt.pth'
			self.model = trt_pose.models.densenet121_baseline_att(num_parts, 2 * num_links).cuda().eval()
			self.WIDTH = 256
			self.HEIGHT = 256

		self.X_compress = float(width) / float(self.WIDTH)
		self.Y_compress = float(height) / float(self.HEIGHT)

		data = torch.zeros((1, 3, self.HEIGHT, self.WIDTH)).cuda()
		if os.path.exists(OPTIMIZED_MODEL) == False:
			self.model.load_state_dict(torch.load(MODEL_WEIGHTS))
			self.model_trt = torch2trt.torch2trt(model, [data], fp16_mode=True, max_workspace_size=1<<25)
			torch.save(model_trt.state_dict(), OPTIMIZED_MODEL)

		self.model_trt = TRTModule()
		self.model_trt.load_state_dict(torch.load(OPTIMIZED_MODEL))
		self.mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
		self.std = torch.Tensor([0.229, 0.224, 0.225]).cuda()
		
		#self.tm = TransformManager()
		self.detector = april.Detector(searchpath=['apriltags'], families='tagStandard41h12', nthreads=1,
		 							   quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25,
		 							   debug=0)
		
		# Declare filters
		self.dec_filter = rs.decimation_filter ()   # Decimation - reduces depth frame density
		self.spat_filter = rs.spatial_filter()          # Spatial    - edge-preserving spatial smoothing
		self.temp_filter = rs.temporal_filter()    # Temporal   - reduces temporal noise
		self.hole_filter = rs.hole_filling_filter()

	def preprocess(self, image):
			global device
			image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
			image = PIL.Image.fromarray(image)
			image = transforms.functional.to_tensor(image).to(device)
			image.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
			return image[None, ...]
		
	def run(self):
		parse_objects = ParseObjects(self.topology)
		
		while True:
			if self.do_realsense:
				frames = self.pipeline.wait_for_frames()
				if not frames:
					return
				depthData = frames.get_depth_frame()
				colorData = frames.get_color_frame()
				#filtered = self.dec_filter.process(depthData)
				filtered = self.spat_filter.process(depthData)
				filtered = self.temp_filter.process(filtered)
				filtered = self.hole_filter.process(filtered)
				self.adepth = np.asanyarray(filtered.get_data(), dtype=np.uint16)  
				self.acolor = np.asanyarray(colorData.get_data())
				rgb_width = self.width
				rgb_height = self.height
				rgb_depth = 3
				rgb_focal_x = self.rs_focal_x
				
			if self.horizontalflip:
				self.acolor = cv2.flip(self.acolor, 1)
				self.adepth = cv2.flip(self.adepth, 1)
			if self.verticalflip:
				self.acolor = cv2.flip(self.acolor, 0)
				self.adepth = cv2.flip(self.adepth, 0)
				
			if self.do_calibrate:
				transform = self.calibrate_with_apriltag(self.acolor, rgb_focal_x)
				print(transform)
				if len(transform) > 0:
					self.tm.add_transform("world", "camera", transform)
					self.do_calibrate = False  # Do it once
					print("Camera to World transform", self.tm.get_transform("camera", "world"))
				else:
					print("No AprilTag recognized. Exiting")
					#sys.exit()
					
			# compute SKELETON
			img = cv2.resize(self.acolor, dsize=(self.WIDTH, self.HEIGHT), interpolation=cv2.INTER_AREA)
			data = self.preprocess(img)
			cmap, paf = self.model_trt(data)
			cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
			counts, objects, peaks = parse_objects(cmap, paf)#, cmap_threshold=0.15, link_threshold=0.15)
			#return counts, objects, peaks
					
			descriptors_queue.put([self.acolor, self.adepth, self.rs_focal_x, None, None, counts, objects, peaks, self.WIDTH, self.HEIGHT, self.X_compress, self.Y_compress])

	def calibrate_with_apriltag(self, rgb, focal):
		grey = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
		s = grey.shape
		transform = []
		tags = self.detector.detect(grey, estimate_tag_pose=True, camera_params=[focal, focal, s[0]/2.0, s[1]/2.0], tag_size=0.25)
		print(tags)
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

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map, startup_check=False):
		super(SpecificWorker, self).__init__(proxy_map)
		#self.timer.timeout.connect(self.compute)
		self.params = {}
		self.cameraid = 0
		self.gt = []
		self.viewimage = False
		self.contFPS = 0
		
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
		self.device_serial = self.params["device_serial"]
		self.width = int(self.params["width"])
		self.height = int(self.params["height"])
		self.publishimage = "true" in self.params["publishimage"]

		#self.hide()
		self.initialize()
		#self.timer.setSingleShot(True)
		#self.timer.start(50)
		return True
	
	def get_keypoint(self, humans, hnum, peaks):
		kpoint = []
		human = humans[0][hnum]
		C = human.shape[0]
		for j in range(C):
			k = int(human[j])
			if k >= 0:
				peak = peaks[0][j][k]   # peak[1]:width, peak[0]:height
				peak = (j, float(peak[0]), float(peak[1]))
				kpoint.append(peak)
			else:
				peak = (j, None, None)
				kpoint.append(peak)
		return kpoint
	
	def initialize(self):

		#apriltags
		self.detector = april.Detector(searchpath=['apriltags'], families='tagStandard41h12', nthreads=1,
		 							   quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25,
		 							   debug=0)
		
		#self.detector = april.Detector(searchpath=['apriltags'], families='tag36h11', nthreads=1,
		#							   quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25,
		#							   debug=0)

		self.depth_scale = 1.0
		if self.do_realsense:
		#	# realsense configuration
			try:
				config = rs.config()
				config.enable_device(self.params["device_serial"])
				config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, 30)
				config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)
				#self.pointcloud = rs.pointcloud()
				self.pipeline = rs.pipeline()
				cfg = self.pipeline.start(config)
				profile = cfg.get_stream(rs.stream.color) # Fetch stream profile for depth stream
				intr = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
				self.rs_focal_x = intr.fx
				self.rs_focal_y = intr.fy
				ds = cfg.get_device().first_depth_sensor()
				self.depth_scale = ds.get_depth_scale()
				print("----------------------", self.depth_scale)
			except Exception as e:
				print("Error initializing camera")
				print(e)
				sys.exit(-1)
				
		# openpifpaf
		
		#self.desc_processor = Process_Descriptor(scale=self.scale, descriptor_size=self.descriptor_size, tm=self.tm, depth_scale = self.depth_scale)
		#self.desc_processor.daemon = True
		#self.desc_processor.start()
		self.skeleton_processor = Skeleton_Extractor(self.do_realsense, self.pipeline,  \
				self.width, self.height, self.rs_focal_x, self.horizontalflip, self.verticalflip, self.do_calibrate)
		self.skeleton_processor.daemon = True
		self.skeleton_processor.start()
		
		self.start = time.time()
		self.compute()

	def compute(self):
		with open('human_pose.json', 'r') as f:
			human_pose = json.load(f)
		topology = trt_pose.coco.coco_category_to_topology(human_pose)
		
		while True:
			#image, peoplelist = peoplelist_queue.get()
			image, depth, rs_focal_x, _, _,counts, objects, peaks, WIDTH, HEIGHT, X_compress, Y_compress = descriptors_queue.get()
			
			print(counts)
			print(objects) 
			print(peaks)
			print("------------")
			self.draw_points(image, counts, objects, peaks, WIDTH*X_compress, HEIGHT*Y_compress, topology)
			cv2.imshow(" ", image)
			cv2.waitKey(10)
			
			#peoplelist = []
			#if len(image) > 0 and self.viewimage:
		#		self.drawImage(image, peoplelist)
		#		
		#		cv2.imshow(" ", image)
		#		cv2.waitKey(10)
				
			#if len(peoplelist) > 0:
				##self.draw_points_in_coppelia(peoplelist[0])
				##self.move_avatar_in_coppelia(peoplelist[0])
				#pass
				
			#if self.publishimage:
				#im = RoboCompCameraRGBDSimple.TImage()
				#im.cameraID = self.cameraid
				#height, width = image.shape[:2]
				#im.width = width
				#im.height = height
				#im.focalx = 500
				#im.focaly = 500
				#im.depth = 3
				#im.image = image

				#dep = RoboCompCameraRGBDSimple.TDepth()
				#dep.cameraID = self.cameraid
				#dep.width = self.width
				#dep.height = self.height
				##dep.focalx = self.depth_focal_x
				##dep.focaly = self.depth_focal_y
				##dep.depth = self.adepth

				#try:
					##dep.alivetime = (time.time() - self.capturetime) * 1000
					##im.alivetime = (time.time() - self.capturetime) * 1000
					#self.camerargbdsimplepub_proxy.pushRGBD(im, dep)
				#except Exception as e:
					#print("Error on camerabody data publication")
					#print(e)

			#try:
				#self.publishData(peoplelist)
			#except Exception as e:
				#print("Error on camerabody data publication")
				#print(e)
				
			# time
			if time.time() - self.start > 1:
				print("FPS:", self.contFPS)
				self.start = time.time()
				self.contFPS = 0
			self.contFPS += 1
			
	###########################################################################
	# 3D-world
	###########################################################################
	def draw_points(self, image, object_counts, objects, normalized_peaks, width, height, topology):
		height = image.shape[0]
		width = image.shape[1]
		
		K = topology.shape[0]
		count = int(object_counts[0])
		K = topology.shape[0]
		for i in range(count):
			color = (0, 255, 0)
			obj = objects[0][i]
			C = obj.shape[0]
			for j in range(C):
				k = int(obj[j])
				if k >= 0:
					peak = normalized_peaks[0][j][k]
					x = round(float(peak[1]) * width)
					y = round(float(peak[0]) * height)
					cv2.circle(image, (x, y), 3, color, 2)

			for k in range(K):
				c_a = topology[k][2]
				c_b = topology[k][3]
				if obj[c_a] >= 0 and obj[c_b] >= 0:
					peak0 = normalized_peaks[0][c_a][obj[c_a]]
					peak1 = normalized_peaks[0][c_b][obj[c_b]]
					x0 = round(float(peak0[1]) * width )
					y0 = round(float(peak0[0]) * height )
					x1 = round(float(peak1[1]) * width )
					y1 = round(float(peak1[0]) * height)
					cv2.line(image, (x0, y0), (x1, y1), color, 2)
					
					
	def draw_points_in_coppelia(self, person):
		names = ['left_shoulder', 'right_shoulder', 'left_hip', 'right_hip', "left_elbow", "right_elbow", "left_knee", "right_knee"]
		for name, keypoint in person.joints.items():
			try:
				if name in names and keypoint.score > 0.3:
					try:
						body_pose = RoboCompCoppeliaUtils.PoseType()
						body_pose.x = keypoint.xw
						body_pose.y = keypoint.yw
						body_pose.z = keypoint.zw
						self.coppeliautils_proxy.addOrModifyDummy(RoboCompCoppeliaUtils.TargetTypes.Info, name, body_pose)
					except Exception as e:
						print(e)
			except:
				pass

	###########################################################################
	# AVATAR
	###########################################################################
	def move_avatar_in_coppelia(self, person):
		bill_name = 'Bill_goalDummy'
		left_avatar_x = 0.0
		right_avatar_x = 0.0
		right_avatar_y = 0.0
		left_avatar_y = 0.0
		left_cont = 0
		right_cont = 0
		left_names = ['left_shoulder', 'left_hip', 'left_elbow', 'left_ankle', 'left_knee', 'left_eye', 'left_ear', 'left_wrist']
		right_names = ['right_shoulder', 'right_hip', 'right_elbow', 'right_ankle', 'right_knee', 'right_eye', 'right_ear', 'right_wrist']
		
		for name, keypoint in person.joints.items():
			if name in left_names:
				left_avatar_x += keypoint.xw
				left_avatar_y += keypoint.yw
				left_cont += 1
		if left_cont > 0:
			left_avatar_x /= left_cont
			left_avatar_y /= left_cont
		for name, keypoint in person.joints.items():
			if name in right_names:
				right_avatar_x += keypoint.xw
				right_avatar_y += keypoint.yw
				right_cont += 1
		if right_cont > 0:
			right_avatar_x /= right_cont
			right_avatar_y /= right_cont
		
		if right_cont > 0 and left_cont > 0:
			try:
				body_pose = RoboCompCoppeliaUtils.PoseType()
				body_pose.x = (left_avatar_x + right_avatar_x )/ 2.0
				body_pose.y = (left_avatar_y + right_avatar_y )/ 2.0
				self.coppeliautils_proxy.addOrModifyDummy(RoboCompCoppeliaUtils.TargetTypes.Info, bill_name,  body_pose)
			except Exception as e:
				print(e)

###########################################################################
# Apriltags calibration
###########################################################################
	def calibrate_with_apriltag(self, rgb, focal):
		grey = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
		s = grey.shape
		transform = []
		tags = self.detector.detect(grey, estimate_tag_pose=True, camera_params=[focal, focal, s[0]/2.0, s[1]/2.0], tag_size=0.25)
		print(tags)
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
		roi.image = self.acolor[y:y+h, x:x+w].tobytes()
		return roi

		# to check roi aspect
		cv2.circle(self.acolor, (x, y), 10, (0, 0, 255))
		cv2.circle(self.acolor, (x, y+h), 10, (0, 0, 255))
		cv2.circle(self.acolor, (x+w, y), 10, (0, 0, 255))
		cv2.circle(self.acolor, (x+w, y+h), 10, (0, 0, 255))
		#cv2.imshow("color", self.bcolor)
		#if len(roi.image) > 0:
		#	cv2.imshow("roi", self.bcolor[y:y+h, x:x+w])
		return roi

####################################################################################################

	######################################
##### PUBLISHER
######################################

	def publishData(self, peoplelist):
		people = RoboCompHumanCameraBody.PeopleData()
		people.cameraId = self.cameraid
		people.timestamp = int(round(time.time() * 1000))
		people.gt = self.gt
		people.peoplelist = peoplelist
				
		
		if len(people.peoplelist) > 0:
			#for person in people.peoplelist:
				#person.roi = self.getRoi(person)
			try:
				self.humancamerabody_proxy.newPeopleData(people)
			except:
				print("Error on camerabody data publication")
				traceback.print_exc()
