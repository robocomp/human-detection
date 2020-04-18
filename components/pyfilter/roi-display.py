import json, sys, pickle, random, time, bigjson
import numpy as np
import itertools
import cv2

FILE = 'human_data_textured2.txt'

def dataIter(init=0, size=40, end=-1):
    with open(FILE, 'rb') as f:
        raw = f.read()
    data = json.loads(raw)["data_set"]

    total = len(data)
    print("Total evidences: ", total)
    if end == -1:
        end = len(data)
    while init+size < end:
        sample = data[init:init+size]    
        #print("Sample elapsed time (ms): ", sample[-1]['timestamp']-sample[0]['timestamp'], " Initial: ", size)
        init = init + size 
        yield sample

def hsvHistogram(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0,1], None, [30,32], [0,180,0,255])
    cv2.normalize(hist, hist)
    return hist

def earthmoverdistance(hist1,hist2,h_bins,s_bins):
    #Define number of rows
    numRows = h_bins*s_bins
    sig1 = np.array((numRows, 3), dtype=float)
    sig2 = np.array((numRows, 3), dtype=float)    
    for h in range(h_bins):
        for s in range(s_bins): 
            bin_val = cv2.QueryHistValue_2D(hist1, h, s)
            
            
            cv2.Set2D(sig1, h*s_bins+s, 0, cv2.Scalar(bin_val))
            cv2.Set2D(sig1, h*s_bins+s, 1, cv2.Scalar(h))
            cv2.Set2D(sig1, h*s_bins+s, 2, cv2.Scalar(s))

            bin_val = cv2.QueryHistValue_2D(hist2, h, s)
            cv2.Set2D(sig2, h*s_bins+s, 0, cv2.Scalar(bin_val))
            cv2.Set2D(sig2, h*s_bins+s, 1, cv2.Scalar(h))
            cv2.Set2D(sig2, h*s_bins+s, 2, cv2.Scalar(s))
    #This is the important line were the OpenCV EM algorithm is called
    return cv2.CalcEMD2(sig1,sig2,cv2.CV_DIST_L2)

def extractROIS(sample):
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

cv2.namedWindow('Cam_1', cv2.WINDOW_NORMAL)
cv2.namedWindow('Cam_2', cv2.WINDOW_NORMAL)
cv2.namedWindow('Cam_3', cv2.WINDOW_NORMAL)

data = dataIter(0, 2, -1)
dist = []
while True:
    try:
        sample = next(data)
        rois0 = extractROIS(sample[0])
        rois1 = extractROIS(sample[1])
        
        if len(rois0) == 2 and len(rois1) == 2:
            h1 = [hsvHistogram(r) for r in rois0]
            h2 = [hsvHistogram(r) for r in rois1]
            for a,b in itertools.product(h1,h2):
                #dist = cv2.compareHist(a, b, method=cv2.HISTCMP_BHATTACHARYYA ) 
                #dist = cv2.compareHist(a, b, method=cv2.HISTCMP_CHISQR_ALT ) 
                dist.append( cv2.compareHist(a, b, method=cv2.HISTCMP_KL_DIV ) )
                
            #print("----------------------")

            # final = np.zeros((max([r.shape[0] for r in rois]), sum([r.shape[1] for r in rois]), 3), np.uint8)
            # wt=0
            # for img in rois:
            #     h, w = img.shape[:2]
            #     final[:h, wt:wt+w] = img
            #     wt = w
            # cv2.imshow('Cam_'+str(camera), final)
            # cv2.waitKey(5)
    except StopIteration:
        break
print(len(dist))



cv2.destroyAllWindows()