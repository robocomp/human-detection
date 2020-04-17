import json, sys, pickle, random, time
import numpy as np
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
    people = sample['people']
    rois = []
    for i,person in enumerate(people):
        roi = person['roi']
        w = roi['width']
        h = roi['height']
        d = 3
        if h*w*d == len(roi['image']):
            rois.append(np.asarray(roi['image'], np.uint8).reshape(w, h, 3))
    if len(rois) > 0:
        final = np.zeros((max([r.shape[0] for r in rois]), sum([r.shape[1] for r in rois]), 3), np.uint8)
        wt=0
        for img in rois:
            h, w = img.shape[:2]
            final[:h, wt:wt+w] = img
            wt = w
        cv2.imshow('Cam_'+str(camera), final)
    cv2.waitKey(50)

cv2.destroyAllWindows()