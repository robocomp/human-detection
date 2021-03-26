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
import json, sys, random, time, bigjson
import numpy as np
import itertools
import cv2
from collections import deque, namedtuple
from scipy import stats
from apartment import Apartment2D
from netforward import NetForwad

JSON_FILE = 'human_data_textured2.txt'
#JSON_FILE = 'human_data_2P_Text_I_L.txt'
#JSON_FILE = 'human_data_textured2_short.txt'
#JSON_FILE = 'human_data_textured2_large.txt'
#JSON_FILE = 'human_data_3P.txt'

KNOWN_PEOPLE = 2
USE_GNN = True

# Hyperparameters
MAX_IDLE_TIME = 3  # secs unseen before deleted
MAX_QUEUE_LENGTH = 50  # obs
MIN_TIME_ACTIVE = 1.2 # secs before accepted as new person
#MIN_DIST = 10 # for KL divergence
MIN_DIST = 0.65 # to accept match using Bat divergence (0 is best)

class Person(object):
	
	def __init__(self, camera, histogram, world, timestamp, pose=[], gtruth=[0,0,0,0], joints=[]):
		self.Epoch = namedtuple('Epoch', 'camera world timestamp histogram pose gtruth joints')
		self.history = deque(maxlen=MAX_QUEUE_LENGTH)
		self.history.append(self.Epoch(camera, world, timestamp, histogram, pose, gtruth, joints))
		self.idletime = time.time()
		self.scene_view = None
		self.time_active = 0
		self.time_created = time.time()

	def last(self):
		return self.history[-1]

	def world(self):
		return self.history[-1].world

	def rot(self):
		return self.history[-1].world[3]

	def update(self, obs):
		self.history.extend(obs.history)
		self.idletime = time.time()
		self.time_active = time.time() - self.time_created

	def addPose(self, pose):
		#self.history[-1].pose = pose
		pass

	def getSuitableSet(self):
		''' Return a past window of consecutive different cameras
		'''
		bag_cameras = []
		bag_samples = []
		for h in reversed(self.history):
			if h.camera in bag_cameras:
				break
			else:
				bag_cameras.append(h.camera)
				bag_samples.append(h)
		saco = []
		for b in bag_samples:
			saco.append({"cameraId": b.camera, "ground_truth": b.gtruth, "joints": b.joints, "timestamp": b.timestamp})
		window = {"data_set":[{'superbody': saco}]}
		return window
		
	def print(self):
		print("Person:-> camera:", self.last().camera, "world:", self.last().world, "history:", len(self.history))

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)

	def __del__(self):
		print('SpecificWorker destructor')

	def setParams(self, params):
		self.data = self.dataIter(file=JSON_FILE, init=0, size=1, end=-1)
		self.dist = []
		self.people = []
		self.alab = Apartment2D(self.ui)
		self.total_epochs = 0
		self.count_epochs = 0
		self.net = NetForwad()

		self.Period = 10
		self.timer.start(self.Period)
		return True

	@QtCore.Slot()
	def compute(self):
		try:
			sample = next(self.data)

			# convert observation into format
			observations = self.obsToPeople(sample[0]) # one sample only
			
			# compute distances to existing people
			matches, new, unseen = self.compareToExistingPeople(observations)
			
			# update coincidences
			for o,p in matches:
				p.update(o)
				if p.time_active > MIN_TIME_ACTIVE:
					if USE_GNN:
						window = p.getSuitableSet()['data_set'][0]
						if window:  
							pose = self.net.forward(window)
							self.alab.movePerson(p.scene_view, [pose[0], -pose[1]], pose[2])
						else:
							pass
					else:
						self.alab.movePerson(p.scene_view, o.world(), o.rot())

			# add new potential candidates
			for n in new:
				n.scene_view = self.alab.addPerson(n.world(), n.rot())
				self.people.append(n)

			# remove unseen
			now = time.time()
			self.people[:] = [p for p in self.people if now - p.idletime < MAX_IDLE_TIME]
				
			total_people = len([p for p in self.people if p.time_active>MIN_TIME_ACTIVE])
			print("Matches:", len(matches), "New:", len(new), "Unseen:", len(unseen), "Total:", total_people)
			
			if total_people != KNOWN_PEOPLE:
				self.count_epochs += 1
			self.total_epochs += 1

		except StopIteration:
			print("End of file")
			result = 100*(self.count_epochs / self.total_epochs)
			print("Percent of epochs with other than ", str(KNOWN_PEOPLE), " people: ", result,  " out of ", self.total_epochs)
			# save result
			# sample hyperparameters as a function of result
			# reset dataIter
			self.timer.stop()
	
	#############################################################################
	def obsToPeople(self, sample):
		camera = sample['cameraId']
		people = sample['people']
		obs_as_people = []
		for i,person in enumerate(people):
			roi = person['roi']
			w = roi['width']
			h = roi['height']
			image = roi['image']
			if h*w*3 == len(image):
				img = np.asarray(image , np.uint8).reshape(h,w,3)
				hist = self.hsvHistogram(img)
			else:
				hist = None
			world = person['world']
			timestamp = sample['timestamp']
			joints = person['joints']
			gtruth = [0,0,0,0]
			obs_as_people.append(Person(camera=camera, histogram=hist, world=world, timestamp=timestamp, joints=joints))
		return obs_as_people

	def compareToExistingPeople(self, observations):
		matches = [] 
		new = []
		unseen = []
		if len(self.people)>0:  # first time
			# for each obervation get the min dist to existing people
			mins = []
			for o in observations:
				mins.append(min([[self.histDist(o, p), p, o]  for p in self.people]))
			for d,p,o in mins:
				if d < MIN_DIST: #and all([dm > MAX_DIST for dm,pm,om in mins if om is not o]):
					matches.append([o, p])
			for p in self.people:
				if all(p is not mp for mo,mp in matches):
					unseen.append(p)
			for o in observations:
				if all(o is not mo for mo,mp in matches):
					new.append(o)
		else: 
			new = observations
		return matches, new, unseen
			
	def histDist(self, obs, person):
		""" Compare obs with the complete p history using histogram differences
		""" 
		dist = np.median([cv2.compareHist(obs.last().histogram, h.histogram, method=cv2.HISTCMP_BHATTACHARYYA) for h in person.history if obs.last().histogram is not None and h.histogram is not None])
		#dist = cv2.compareHist(a, b, method=cv2.HISTCMP_CHISQR_ALT ) 
		#dist = np.median([cv2.compareHist(obs.last().histogram, h.histogram, method=cv2.HISTCMP_KL_DIV) for h in person.history if obs.last().histogram is not None and h.histogram is not None])
		#print("Dist:", dist)
		return dist

	def dataIter(self, file, init=0, size=40, end=-1):
		with open(file, 'r') as f:
			data = json.load(f)["data_set"]
		print("Total evidences: ", len(data))
		end = len(data) if end==-1 else end
		while init+size < end:
			sample = data[init:init+size]    
			#print("Sample elapsed time (ms): ", sample[-1]['timestamp']-sample[0]['timestamp'], " Initial: ", size)
			init = init + size 
			yield sample

	def hsvHistogram(self, img):
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		hist = cv2.calcHist([hsv], [0,1], None, [30,32], [0,180,0,255])
		cv2.normalize(hist, hist)
		return hist

	def extractROIS(self, sample):
		camera = sample['cameraId']
		people = sample['people']
		rois = []
		for i,person in enumerate(people):
			roi = person['roi']
			w = roi['width']
			h = roi['height']
			d = 3
			if h*w*d == len(roi['image']):
				rois.append(np.asarray(roi['image'], np.uint8).reshape(h, w, 3))
		return rois