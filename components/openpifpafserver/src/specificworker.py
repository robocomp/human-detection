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

#COCO_IDS=["nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle" ]

class SpecificWorker(GenericWorker):
	processor = None

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
		#TODO: Change parameters read
		#parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter,)
		#parser.add_argument('--no-colored-connections', dest='colored_connections', default=True, action='store_false', help='do not use colored connections to draw poses')
		#parser.add_argument('--disable-cuda', action='store_true', help='disable CUDA')
		#parser.add_argument('--source', default=0, help='OpenCV source url. Integer for webcams. Or ipwebcam streams.')
		#parser.add_argument('--scale', default=0.1, type=float, help='input image scale factor')
		#nets.cli(parser)
		#decoder.cli(parser, force_complete_pose=False, instance_threshold=0.05)

		# add args.device
		torch.device('cpu')
		if torch.cuda.is_available():
			torch.device('cuda')
		# load model
		#args = parser.parse_args()
		class Args:
			source = 0
			checkpoint = None
			basenet = None
			dilation = None
			dilation_end = None
			headnets=['pif', 'paf']
			dropout=0.0
			quad=1
			no_pretrain=True
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
			device = torch.device('cuda')
			disable_cuda = False
			scale = 0.5
			key_point_threshold = 0.05

		args = Args()
		model, _ = nets.factory(args)
		model = model.to(args.device)
		#self.processors = decoder.factory(args, model)
		decode = decoder.factory(args,model)
		self.processor = decoder.Processor(model, decode)
		self.src = np.zeros( (480, 640, 3), np.uint8)
		
	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print('SpecificWorker.compute...')
		cv2.imshow("", self.src)
		cv2.waitKey(1)
		return True

	#
	# SERVANT processImage
	#
	def processImage(self, img):
		print("llega imagen")
		scale = 0.5
		self.src = np.frombuffer(img.image, np.uint8).reshape( img.height, img.width, img.depth )
		#self.src = np.fromstring(img.image, np.uint8).reshape( img.height, img.width, img.depth )
		#print(src.shape)
		image = cv2.resize(self.src, None, fx=scale, fy=scale)
		#image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		processed_image_cpu = transforms.image_transform(image.copy())
		processed_image = processed_image_cpu.contiguous().to(non_blocking=True)
		fields = self.processor.fields(torch.unsqueeze(processed_image, 0))[0]

		#fields = self.processor.fields(torch.unsqueeze(processed_image, 0))[0]
		#keypoint_sets, _ = self.processors[0].keypoint_sets(fields)
		#print("keyPoints", keypoint_sets)

		# # save in ice structure
		# keypoint = KeyPoint()
		# person = Person()
		# for id, person in enumerate(keypoint_sets):
		# 	joints = TJoints()
		# 	for pos, joint in enumerate(person):
		# 		keypoint.x = joint[0]
		# 		keypoint.y = joint[1]
		# 		keypoint.score = joint[2]
		# 		joints[COCO_ID[pos]] = keypoint
		# 	person.id = id
		# 	person.joints = joints
		# 	people.append(person)
		# return people

