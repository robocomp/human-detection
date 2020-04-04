#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import sys
import math
import calibration2
import trackerapi
import math
from getkey import getkey, keys
import pickle


def xxx_degrees(a):
    def xxx_degrees_item(a):
        while a > 180:
            a -= 360
        while a < -180:
            a += 360
        return a
    if isinstance(a, list):
        return [xxx_degrees_item(x) for x in a]
    return xxx_degrees_item(a)

def rads2degrees(a):
    if a < 0:
        a += 2.*math.pi
    return a*180/math.pi

def main():
	
	with open("dataGNN.json", 'r') as f:
		raw = f.read()
	raw = list(raw)
	f.close()

	raws = ''.join(raw)
	data = json.loads(raws)['data_set']

	
	params = pickle.load(open('calibration.prms', 'rb'), fix_imports=True)
	
	model = trackerapi.TrackerAPI('.', None)
	
	test_dataset = calibration2.CalibrationDataset(data[0], 'run', '1')
	
	results = model.predictOneGraph(test_dataset)[0]
	
	x_nn = results[0].item()*4000
	z_nn = results[2].item()*4000
	a_nn = rads2degrees(math.atan2(results[3].item()/0.7, results[4].item()/0.7))

	return [x_nn, z_nn, a_nn]

