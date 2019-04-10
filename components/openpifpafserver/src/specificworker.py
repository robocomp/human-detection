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
import argparse
from genericworker import *
from openpifpaf.network import nets
from openpifpaf import decoder, show, transforms
import torch
import cv2
import numpy as np

COCO_IDS=["nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle" ]

class SpecificWorker(GenericWorker):
	processor = None
	src = None

	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 1000
		self.timer.start(self.Period)
		#self.timer.setSingleShot(True)
		self.initialize()

	def __del__(self):
		print('SpecificWorker destructor')

	def initialize(self):
		
		# add args.device
		class Args:
			source = 0
			checkpoint = None
			basenet = None
			dilation = None
			dilation_end = None
			headnets=['pif', 'paf']
			dropout=0.0
			quad=1
			pretrained=False
			keypoint_threshold = None
			seed_threshold = 0.2
			force_complete_pose = False
			debug_pif_indices = []
			debug_paf_indices = []
			connection_method = 'max'
			fixed_b = None
			pif_fixed_scale = None
			profile_decoder = False
			instance_threshold = 0.05
			device = torch.device(type="cuda")
			disable_cuda = False
			scale = 0.5
			key_point_threshold = 0.05

		self.args = Args()
		model, _ = nets.factory_from_args(self.args)
		model = model.to(self.args.device)
		#model.cuda()
		self.processor = decoder.factory_from_args(self.args, model)
		self.src = np.zeros((480, 640, 3), np.uint8)

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print('SpecificWorker.compute...')
		#cv2.imshow("", self.src)
		#cv2.waitKey(1)
		return True

	#
	# SERVANT processImage
	#

	def processImage(self, img, scale):
		print("llega imagen ", img.width, scale)
		scale = 0.5
		self.src = np.frombuffer(img.image, np.uint8).reshape(img.height, img.width, img.depth)
		image = cv2.resize(self.src, None, fx=scale, fy=scale)
		#image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		processed_image_cpu = transforms.image_transform(image.copy())
		processed_image = processed_image_cpu.contiguous().to(non_blocking=True)
		unsqueezed = torch.unsqueeze(processed_image, 0).to(self.args.device)
		fields = self.processor.fields(unsqueezed)[0]
		keypoint_sets, _ = self.processor.keypoint_sets(fields)
		#print("keyPoints", keypoint_sets)

		# # save in ice structure
		people = []
		for p in keypoint_sets:
			joints = {}
			person = Person()
			for pos, joint in enumerate(p):
				keypoint = KeyPoint()
				keypoint.x = joint[0]
				keypoint.y = joint[1]
				keypoint.score = joint[2]
				joints[COCO_IDS[pos]] = keypoint
			person.id = 0
			person.joints = joints
			people.append(person)
		return people

