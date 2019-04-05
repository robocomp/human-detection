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


#COCO_IDS=["nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle" ]


class SpecificWorker(GenericWorker):
	processor = None

	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		self.initialize()

	def __del__(self):
		print('SpecificWorker destructor')

	def initialize(self):
		#TODO: Change parameters read
		parser = argparse.ArgumentParser(
			description=__doc__,
			formatter_class=argparse.ArgumentDefaultsHelpFormatter,
		)
		nets.cli(parser)
		decoder.cli(parser, force_complete_pose=False, instance_threshold=0.05)

		# add args.device
		torch.device('cpu')
		if torch.cuda.is_available():
			torch.device('cuda')
		# load model
		model, _ = nets.factory()
		decode = decoder.factory_decode(model)
		self.processor = decoder.Processor(model, decode)


	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print('SpecificWorker.compute...')
		return True

	#
	# processImage
	#
	def processImage(self, img, scale):
		image = cv2.resize(img, None, fx=scale, fy=scale)
		image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		processed_image_cpu = transforms.image_transform(image.copy())
		processed_image = processed_image_cpu.contiguous().to(non_blocking=True)
		fields = self.processor.fields(torch.unsqueeze(processed_image, 0))[0]
		keypoint_sets, _ = self.processor.keypoint_sets(fields)
		print("keyPoints", keypoint_sets)

		# save in ice structure
		keypoint = KeyPoint()
		person = Person()
		for id, person in enumerate(keypoint_sets):
			joints = TJoints()
			for pos, joint in enumerate(person):
				keypoint.x = joint[0]
				keypoint.y = joint[1]
				keypoint.score = joint[2]
				joints[COCO_ID[pos]] = keypoint
			person.id = id
			person.joints = joints
			people.append(person)
		return people

