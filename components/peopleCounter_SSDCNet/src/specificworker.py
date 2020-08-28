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

import torch.nn as nn
import torch
import torch.nn.functional as F
from torch.utils.data import  DataLoader
import torch.optim as optim
import cv2
from PIL import Image
import os
import numpy as np
from time import time, sleep
import math
import pandas as pd
import csv
from load_data import Countmap_Dataset
from Network.SSDCNet import SSDCNet_classify
from Val import test_phase
import queue, threading
import matplotlib.pyplot as plt
import imutils
from genericworker import *

def test(frame, opt, rgb, transform_test, num_workers, label_indice, model_path):
    img = Image.fromarray(frame)
    testset = Countmap_Dataset(img,rgb,transform=transform_test,\
    if_test=True, IF_loadmem=opt['IF_savemem_test'])
    testloader = DataLoader(testset, batch_size=opt['test_batch_size'],
                        shuffle=False, num_workers=num_workers)
    # init networks
    label_indice = torch.Tensor(label_indice)
    class_num = len(label_indice)+1
    div_times = opt['div_times']
    net = SSDCNet_classify(class_num,label_indice,div_times=div_times,\
            frontend_name='VGG16',block_num=5,\
            IF_pre_bn=False,IF_freeze_bn=False,load_weights=True,\
            psize=opt['psize'],pstride = opt['pstride'],parse_method ='maxp').cuda()

    # test the min epoch
    mod_path='best_epoch.pth' 
    mod_path=os.path.join(opt['model_path'] ,mod_path)
    if os.path.exists(mod_path):
        all_state_dict = torch.load(mod_path)
        net.load_state_dict(all_state_dict['net_state_dict'])
        tmp_epoch_num = all_state_dict['tmp_epoch_num']
        log_save_path = os.path.join(model_path,'log-epoch-min[%d]-%s.txt'%(tmp_epoch_num+1,opt['parse_method']) )
        # test
        test_log, count = test_phase(opt,net,testloader,log_save_path=log_save_path)
        return count

def KalmanFilter(count_k_1, P_k_1, count, R=0.1):
    count_prior_k = count_k_1
    P_prior_k = P_k_1
    K_k = P_prior_k / (P_prior_k + R)
    count_k = count_prior_k + K_k * (count - count_prior_k)
    P_k = (1 - K_k) * P_prior_k
    return count_k, P_k

def Moving_avg(count, c_queue, window=20):
    if(len(c_queue)<=window):
        count = mean(c_queue)
    else:
        c_queue.pop(0)
        count = mean(c_queue)
    return round(count)

def majorityElement(count_list):
    x = np.unique(np.round(count_list))
    if(len(x)==len(count_list) and np.count_nonzero(x)==len(count_list)):
        return np.round(np.mean(x))
    m = -1
    i = 0
    ind = -1
    for j in range(len(count_list)):
        if i == 0:
            m = count_list[j]
            i = 1
            ind = j
        elif m == count_list[j]:
            i = i + 1
        else:
            i = i - 1
    return m
    
def mean(nums):
    return float(sum(nums)) / max(len(nums), 1)

class ReadIPStream:
    def __init__(self, url):
        self.stream = requests.get(url, stream=True)

    def read_stream(self):
        msg = bytes('', encoding = 'UTF-8')
        for chunk in self.stream.iter_content(chunk_size=1024):
            msg += chunk
            a = msg.find(b'\xff\xd8')
            b = msg.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = msg[a:b + 2]
                msg = msg[b + 2:]
                if len(jpg) > 0:
                    img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    return True, img


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.model_idxlist = {'model1':0,'model2':1,'model3':2}
        self.model_list = ['model1', 'model2', 'model3']    
        self.model_max = [[22], [7], [8]]
        self.vidcap = None
        self.fps = None
        self.peopleCounterMachine.start()
        self.opt = dict()




    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        self.model_choose = [self.model_idxlist[params["model"]]]
        for di in self.model_choose:
            self.opt['model'] = self.model_list[di]
            self.opt['max_list'] = self.model_max[di]
            self.opt['num_workers'] = 0
            self.opt['IF_savemem_train'] = False
            self.opt['IF_savemem_test'] = False
            self.opt['test_batch_size'] = 1
            self.opt['psize'],self.opt['pstride'] = 64,64
            self.opt['div_times'] = 2
            parse_method_dict = {0:'maxp'}
            self.opt['parse_method'] = parse_method_dict[0]
            self.opt['max_num'] = self.opt['max_list'][0]
            self.opt['partition'] = 'two_linear'
            self.opt['step'] = 0.5
            self.opt['model_path'] = os.path.join('model',params["model"])

            self.model_path = self.opt['model_path'] 
            self.num_workers = self.opt['num_workers']
            self.transform_test = []

            self.opt['start_webcam'] = False
            self.opt['read_ipstream'] = None
            self.opt['filter'] = params["filter"]
            self.stitch = True
            self.opt['read_ipstream']=[]
            self.opt['video']=[]
            if(params["stitch"]=='False'):
                self.stitch = False
            if params["video"]=='None':
                self.opt['start_webcam'] = True
            else:
                videos = params['video'].split()
                for video in videos:
                    if video.startswith('http'):
                        self.opt['read_ipstream'].append(True)
                        self.opt['video'].append(video)

                    else:
                        self.opt['read_ipstream'].append(False)
                        self.opt['video'].append(video)
            self.videos = self.opt['video']
            self.filter_method = params["filter"]
            
            label_indice = np.arange(self.opt['step'],self.opt['max_num']+self.opt['step'],self.opt['step'])
            add = np.array([1e-6,0.05,0.10,0.15,0.20,0.25,0.30,0.35,0.40,0.45]) 
            label_indice = np.concatenate( (add,label_indice) )
            self.label_indice = label_indice
            self.opt['class_num'] = label_indice.size+1
            self.save_results=params['save_results']
            self.skip_frames = int(params["skip_frames"])
            self.total_frames = 0
            self.rgb = None
            self.num_views = None
            self.exit_flag = None
            self.shape = (1024, 768)
            self.frame = None
            self.frames=None
            self.writer = None
            self.count = 0
            self.start_flag = None
            self.count_k_1 = None
            self.P_k_1 = None
            self.P_k = None
            self.c_queue = None
            self.vote=None
            self.count_list=None
    # =============== Slots methods for State Machine ===================
    # ===================================================================
    #
    # sm_initialize_video
    #
    @QtCore.Slot()
    def sm_initialize_video(self):
        print("Entered state initialize_video")
        self.vidcap = []
        if not self.opt['start_webcam']:
            for read_ipstream, video in zip(self.opt['read_ipstream'], self.videos):
                if not read_ipstream:
                    print('[INFO]Loading video from file...')
                else:
                    print('[INFO]Loading from the given URL...')
                try:
                    vc = cv2.VideoCapture(video)
                    self.vidcap.append(vc)
                except:
                    print("Error trying videoCapture")
                    self.initialize_videotofinalize_video.emit()
        else:
            print("[INFO] starting video stream...")
            try:
                vc = cv2.VideoCapture(0)
                self.vidcap.append(vc)
            except:
                print("Error trying videoCapture through web cam")
                self.initialize_videotofinalize_video.emit()
        self.fps = FPS().start()
        self.num_views = len(self.vidcap)

        if(self.stitch):
            self.start_flag = np.ones((1), dtype=bool)
        else:
            self.start_flag = np.ones((self.num_views), dtype=bool)

            self.vote=True
            self.count_list = np.zeros(self.num_views)
            if self.filter_method=='kf':
                self.count_k_1 = np.zeros(self.num_views)
                self.P_k_1 = np.ones(self.num_views)
                self.P_k = np.zeros(self.num_views)
            elif self.filter_method=='mavg':
                self.c_queue = np.empty((self.num_views,),dtype=object)

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
        print ("[INFO] total frames", self.total_frames)
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
        self.rgb = np.zeros([self.num_views, 3])
        self.frames =[]   
        for i in range(self.num_views):
            vc = self.vidcap[i].read()[1]
            if vc is None:
                print("End of video feed or error in streaming")
                self.processing_videotofinalize_video.emit()
            self.frame = vc
            color = cv2.mean(self.frame)
            self.rgb[i]+=np.array([color[2], color[1], color[0]])
            self.total_frames +=1

        if self.save_results == True and self.writer is None:
            try:
                current_date = datetime.now()
                date = datetime.strftime(current_date, "%m%d%H%M")
                video_dir = os.path.join("results", + "result_" + date + ".mp4")
                print(video_dir)
                self.writer = cv2.VideoWriter(video_dir, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10,
                                              (self.shape))
            except:
                print("Error trying to create video writer")
        if self.total_frames % self.skip_frames == 0:
            for i in range(self.num_views):
                self.frames.append(self.vidcap[i].read()[1])
            self.reading_framestocounting.emit()
        else:
            self.reading_framestoreading_frames.emit()    
    #
    # sm_counting
    #
    @QtCore.Slot()
    def sm_counting(self):
        print("Entered state counting")
        self.frame = None
        if len(self.frames)==0:
            self.processing_videotofinalize_video.emit()
        if(self.stitch):
            if(self.num_views==1):
                self.frame = self.frames[0]
            elif(self.num_views>1):
                if imutils.is_cv3() :
                    stitcher = cv2.createStitcher() 
                else:
                    stitcher = cv2.Stitcher_create()
                (status, self.frame) = stitcher.stitch(self.frames)
            if self.frame is not None:
                rgb = np.sum(self.rgb, axis=0)/(self.skip_frames * 256 * self.num_views)
                self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                self.frame = cv2.resize(self.frame, self.shape)
                self.count = test(self.frame, self.opt, rgb, self.transform_test, self.num_workers, self.label_indice, self.model_path)
                if self.filter_method=='kf':
                    if self.start_flag[0]==False:
                        self.count, self.P_k = KalmanFilter(self.count_k_1, self.P_k_1, self.count)
                        self.count_k_1, self.P_k_1 = self.count, self.P_k
                    else:
                        self.count_k_1 = self.count
                        self.P_k_1 = 1
                        
                        self.start_flag[0] = False
                    if self.filter_method=='mavg':
                        if not self.start_flag[0]:
                            self.c_queue.append(self.count)
                            self.count = Moving_avg(self.count, self.c_queue)
                        else:
                            self.c_queue = []
                            self.c_queue.append(self.count)
                            self.start_flag[0] = False
        if(self.vote and self.num_views>1):
            for i in range(len(self.frames)):
                img = cv2.cvtColor(self.frames[i], cv2.COLOR_BGR2RGB)
                cv2.imwrite('IMG_'+str(i)+'.jpg',img)
                img = cv2.resize(img, self.shape)
                if(i==0):
                    self.frame=img
                self.rgb[i] = self.rgb[i]/(self.skip_frames * 256 )
                self.count_list[i] = test(img, self.opt, self.rgb[i], self.transform_test, self.num_workers, self.label_indice, self.model_path)
                if self.filter_method=='kf':
                    if not self.start_flag[i]:
                        self.count_list[i], self.P_k[i] = KalmanFilter(self.count_k_1[i], self.P_k_1[i], self.count_list[i])
                        self.count_k_1[i], self.P_k_1[i] = self.count_list[i], self.P_k[i]
                    else:
                        self.count_k_1[i]=self.count_list[i]
                        self.P_k_1[i] = 1
                        self.start_flag[i] = False
                elif self.filter_method=='mavg':
                    if not self.start_flag[i]:
                        self.c_queue[i].append(self.count_list[i])
                        self.count_list[i] = Moving_avg(self.count_list[i], self.c_queue[i])
                    else:
                        self.c_queue[i] =[]
                        self.c_queue[i].append(self.count_list[i])
                        self.start_flag[i] = False
            self.count = majorityElement(self.count_list)
        if self.frame is not None:
            cv2.putText(self.frame, 'No.of People: '+str(round(self.count)), (50, 50),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow('frame ',self.frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                self.processing_videotofinalize_video.emit()
        self.countingtoreading_frames.emit()
