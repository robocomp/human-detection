import json, sys, pickle
import random, time
import numpy as np
import pprint
import itertools
import cv2

FILE = 'human_data.txt'

def dataIter(init=0, size=40, end=-1):
    with open(FILE, 'r') as f:
        raw = f.read()
    data = json.loads(raw)["data_set"]
    total = len(data)
    print("Total evidences: ", total)
    if end == -1:
        end = len(data)
    while init+size < end:
        sample = data[init:init+size]
        # obs = []
        # for s in sample:
        #     for person in s['people']:
        #         w = person['roi']['width']
        #         h = person['roi']['height']
        #         d = 3
        #         hist = dict()
        #         if h*w*d == len(person['roi']['image']):
        #             roi = np.asarray(person['roi']['image'], np.uint8).reshape(h,w,3)
        #             hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        #             hh = cv2.calcHist([hsv], [0], None, [10], [0, 256])[:,0]
        #             hs = cv2.calcHist([hsv], [1], None, [10], [0, 256])[:,0]
        #             hv = cv2.calcHist([hsv], [2], None, [10], [0, 256])[:,0]
        #             hist = np.concatenate([hh, hs, hv])
        #             obs.append({'timestamp':s['timestamp'], 'world':person['world'], 'roi':hist}) 

        print("Sample elapsed time (ms): ", sample[-1]['timestamp']-sample[0]['timestamp'], " Initial: ", size)
        init = init + size 
        yield sample

cv2.namedWindow('Cam_1', cv2.WINDOW_NORMAL)
cv2.namedWindow('Cam_2', cv2.WINDOW_NORMAL)
cv2.namedWindow('Cam_3', cv2.WINDOW_NORMAL)

data = dataIter(0, 1, -1)
while True:
    print("step")
    sample = next(data)[0]
    camera = sample['cameraId']
    person = sample['people'][0]
    w = person['roi']['width']
    h = person['roi']['height']
    d = 3
    if h*w*d == len(person['roi']['image']):
        roi = np.asarray(person['roi']['image'], np.uint8).reshape(w, h, 3)
        cv2.imshow('Cam_'+str(camera), roi)
    cv2.waitKey(50)

cv2.destroyAllWindows()