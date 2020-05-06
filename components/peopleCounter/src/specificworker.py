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
#

import argparse
import os
from datetime import datetime

import cv2
import dlib
import imutils
import numpy as np
import requests
from imutils.video import FPS

import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +"/../resources/")

from centroidtracker import CentroidTracker
from trackableobject import TrackableObject
from genericworker import *
from RoboCompPeopleServer import TImage

class ReadIPStream:
    def __init__(self, url):
        self.stream = requests.get(url, stream=True)

    def read_stream(self):
        bytes = ''
        for chunk in self.stream.iter_content(chunk_size=1024):
            bytes += chunk
            a = bytes.find(b'\xff\xd8')
            b = bytes.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = bytes[a:b + 2]
                bytes = bytes[b + 2:]
                if len(jpg) > 0:
                    img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    return True, img


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        # self.Period = 2000
        # self.timer.start(self.Period)

        self.available_trackers = ["dlib", "mosse", "csrt", "kcf", "medianflow"]

        self.classes = ["background", "aeroplane", "bicycle", "bird", "boat",
                        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                        "sofa", "train", "tvmonitor"]

        self.opencv_trackers = {
            "csrt": cv2.TrackerCSRT_create,
            "kcf": cv2.TrackerKCF_create,
            "medianflow": cv2.TrackerMedianFlow_create,
            "mosse": cv2.TrackerMOSSE_create
        }

        self.input = None
        self.save_results = None
        self.selected_tracker = None
        self.model = None
        self.prototxt = None

        self.net = None
        self.fps = None
        self.stream = None
        self.frame = None
        self.rgb = None
        self.rects = []
        self.trackers = None

        self.confidence = 0.5
        self.skip_frames = 5
        self.width = None
        self.height = None
        self.read_ipstream = False
        self.writer = None
        self.ct = CentroidTracker(maxDisappeared=50, maxDistance=100)
        self.trackableObjects = {}
        self.totalFrames = 0
        self.peopleInside = 0
        self.dict_id_position = {}

        if self.selected_tracker != "dlib":
            self.trackers = cv2.MultiTracker_create()
        else:
            self.trackers = []

        self.peopleCounterMachine.start()

    def __del__(self):
        print 'SpecificWorker destructor'

    def setParams(self, params):
        self.input = params["video_input"]
        self.save_results = params["save_results"] == "True"
        self.selected_tracker = params["tracker"]
        self.model = params["model"]
        self.prototxt = params["prototxt"]

        if self.selected_tracker not in self.available_trackers:
            print ("[ERROR] tracker not found")
            exit(-1)

        if self.input.startswith('http'):
            self.read_ipstream = True
            print("readIP")
        else:
            self.read_ipstream = False
            print("videoCapture")
        return True

    # =============== Slots methods for State Machine ===================
    # ===================================================================
    #
    # sm_initialize_video
    #
    @QtCore.Slot()
    def sm_initialize_video(self):
        print("Entered state initialize_video")

        print("[INFO] loading model...")
        self.net = cv2.dnn.readNetFromCaffe(self.prototxt, self.model)

        if self.read_ipstream:
            print("[INFO] starting video stream...")
            try:
                self.stream = ReadIPStream(self.input)
            except:
                print("Error trying readIPStream")
                self.initialize_videotofinalize_video.emit()
        else:
            print("[INFO] opening video file...")
            try:
                self.stream = cv2.VideoCapture(self.input)
            except:
                print("Error trying videoCapture")
                self.initialize_videotofinalize_video.emit()

        self.fps = FPS().start()

        self.initialize_videotoprocessing_video.emit()

    #
    # sm_processing_video
    #
    @QtCore.Slot()
    def sm_processing_video(self):
        print("Entered state processing_video")
        pass

    #
    # sm_finalize_video
    #
    @QtCore.Slot()
    def sm_finalize_video(self):
        print("Entered state finalize_video")

        cv2.destroyAllWindows()

        if self.writer is not None:
            self.writer.release()

        self.fps.stop()
        print ("[INFO] total frames", self.totalFrames)
        print("[INFO] elapsed time: {:.2f}".format(self.fps.elapsed()))
        print("[INFO] approx. FPS: {:.2f}".format(self.fps.fps()))

        exit(-1)
    ## Processing video sub states
    #
    # sm_reading_frames
    #
    @QtCore.Slot()
    def sm_reading_frames(self):
        print("Entered state reading_frames")

        if self.read_ipstream:
            process_image, self.frame = self.stream.read_stream()
        else:
            process_image, self.frame = self.stream.read()

        if self.read_ipstream == False and self.frame is None:
            self.processing_videotofinalize_video.emit()

        if process_image:
            self.frame = imutils.resize(self.frame, width=400)

            self.rgb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

            if self.width is None or self.height is None:
                (self.height, self.width) = self.frame.shape[:2]

            if self.save_results == True and self.writer is None:
                try:
                    current_date = datetime.now()
                    date = datetime.strftime(current_date, "%m%d%H%M")
                    video_dir = os.path.join("results", self.selected_tracker + "_result_" + date + ".mp4")
                    print(video_dir)
                    self.writer = cv2.VideoWriter(video_dir, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10,
                                                  (self.width, self.heights))

                except:
                    print("Error trying to create video writer")

            self.rects = []

            if self.totalFrames % self.skip_frames == 0:
                self.reading_framestodetecting.emit()
            else:
                self.reading_framestotracking.emit()

    #
    # sm_detecting
    #
    @QtCore.Slot()
    def sm_detecting(self):
        print("Entered state detecting")

        if self.selected_tracker == "dlib":
            self.trackers = []
        else:
            self.trackers = cv2.MultiTracker_create()

        ###Deteccion con red neuronal
        # blob = cv2.dnn.blobFromImage(self.frame, 0.007843, (self.width, self.height), 127.5)
        # self.net.setInput(blob)
        # detections = self.net.forward()
        #
        # for i in np.arange(0, detections.shape[2]):
        #
        #     conf = detections[0, 0, i, 2]
        #
        #     if conf > self.confidence:
        #
        #         idx = int(detections[0, 0, i, 1])
        #         if self.classes[idx] != "person":
        #             continue
        #
        #         box = detections[0, 0, i, 3:7] * np.array([self.width, self.height, self.width, self.height])
        #         (startX, startY, endX, endY) = box.astype("int")

                # if self.selected_tracker == "dlib":
                #
                #     tracker = dlib.correlation_tracker()
                #     rect = dlib.rectangle(startX, startY, endX, endY)
                #     tracker.start_track(self.rgb, rect)
                #     self.trackers.append(tracker)
                #
                # else:
                #     (x, y, w, h) = (startX, startY, endX - startX, endY - startY)
                #     tracker = self.opencv_trackers[self.selected_tracker]()
                #     self.trackers.add(tracker, self.frame, (x, y, w, h))
                #
                # self.rects.append((startX, startY, endX, endY))

        ##Deteccion con openPifPaf
        im = TImage()
        im.image = self.frame.data
        im.height, im.width, im.depth = self.frame.shape
        people = self.peopleserver_proxy.processImage(im, 1)

        for p in people:
            joints = p.joints
            list = []
            for jointname,pose in joints.items():
               if pose.score > 0:
                   list.append((int(pose.x),int(pose.y)))
                   cv2.circle(self.frame, (int(pose.x), int(pose.y)), 2, (0, 0, 255), -1)

            x,y,w,h = cv2.boundingRect(cv2.UMat(np.asarray(list)))
            if self.selected_tracker == "dlib":
                tracker = dlib.correlation_tracker()
                rect = dlib.rectangle(x, y, x+w, y+h)
                tracker.start_track(self.rgb, rect)
                self.trackers.append(tracker)

            else:
                tracker = self.opencv_trackers[self.selected_tracker]()
                self.trackers.add(tracker, self.frame, (x, y, w, h))

            self.rects.append((x, y, x+w, y+h))



        #recorrer personas que devuelve, paara cada una almacenar los joints con score >0
        #para cada persona calcular el bounding rect que contiene los puntos de su esqueleto (habia que haceru na doble transformacion para calcular el bounding rect)
        #teniendo los rectangulos detectados para cada persona dependiendo del tracking añadirlos

        self.detectingtoupdate.emit()

    #
    # sm_tracking
    #
    @QtCore.Slot()
    def sm_tracking(self):
        print("Entered state tracking")

        if self.selected_tracker == "dlib":
            for tracker in self.trackers:
                tracker.update(self.rgb)
                pos = tracker.get_position()

                startX = int(pos.left())
                startY = int(pos.top())
                endX = int(pos.right())
                endY = int(pos.bottom())

                self.rects.append((startX, startY, endX, endY))

        else:
            (success, boxes) = self.trackers.update(self.frame)
            if success:
                for box in boxes:
                    (x, y, w, h) = [int(v) for v in box]
                    self.rects.append((x, y, x + w, y + h))

        self.trackingtoupdate.emit()

    #
    # sm_update
    #
    @QtCore.Slot()
    def sm_update(self):
        print("Entered state update")

        cv2.line(self.frame, (0, self.height // 2), (self.width, self.height // 2), (0, 255, 255), 2)

        objects = self.ct.update(self.rects)

        # loop over the tracked objects
        for (objectID, centroid) in objects.items():

            to = self.trackableObjects.get(objectID, None)

            # if there is no existing trackable object, create one
            if to is None:
                to = TrackableObject(objectID, centroid)

            y = [c[1] for c in to.centroids]
            direction = centroid[1] - np.mean(y)
            to.centroids.append(centroid)

            if centroid[1] < self.height // 2:  # El centroide esta en la parte superior FUERA
                try:
                    isOutside = self.dict_id_position[objectID]
                    if not isOutside:
                        self.peopleInside -= 1
                        # to.counted = True
                        self.dict_id_position[objectID] = True

                except:
                    self.dict_id_position[objectID] = True


            else:  # El centroide eesta en la inferior DENTRO
                try:
                    isOutside = self.dict_id_position[objectID]
                    if isOutside:
                        self.peopleInside += 1
                        # to.counted = True
                        self.dict_id_position[objectID] = False

                except:
                    self.dict_id_position[objectID] = False

            self.trackableObjects[objectID] = to

            text = "ID {}".format(objectID)
            cv2.putText(self.frame, text, (centroid[0] - 10, centroid[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(self.frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

        for rect in self.rects:
            (startX, startY, endX, endY) = rect
            cv2.rectangle(self.frame, (startX, startY), (endX, endY),
                          (0, 255, 0), 2)

        info = [
            ("People inside", self.peopleInside),
            ("Tracker", self.selected_tracker),
        ]

        for (i, (k, v)) in enumerate(info):
            text = "{}: {}".format(k, v)
            cv2.putText(self.frame, text, (10, self.height - ((i * 20) + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        if self.writer is not None:
            self.writer.write(self.frame)

        cv2.imshow("Frame", self.frame)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # Escape
            self.processing_videotofinalize_video.emit()

        if key == ord('s'):
            currentDate = datetime.now()
            date = datetime.strftime(currentDate, "%m%d%H%M")
            photoname = os.path.join("results", "captura_" + date + ".jpg")
            print("[SAVING]", photoname)
            cv2.imwrite(photoname, self.frame)

        self.totalFrames += 1
        self.fps.update()

        self.updatetoreading_frames.emit()

# =================================================================
# =================================================================
