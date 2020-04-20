#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import calibration2
import trackerapi

class NetForwad(object):
    def __init__(self):
        self.model = trackerapi.TrackerAPI('.', None)
   
    def forward(self, data):
        test_dataset = calibration2.CalibrationDataset(data, 'run', '1')
        results = self.model.predictOneGraph(test_dataset)[0]
        x_nn = results[0].item()*4000
        z_nn = results[2].item()*4000
        a_nn = self.rads2degrees(math.atan2(results[3].item()/0.7, results[4].item()/0.7))
        return [x_nn, z_nn, a_nn]

    def xxx_degrees(self, a):
        def xxx_degrees_item(a):
            while a > 180:
                a -= 360
            while a < -180:
                a += 360
            return a
        if isinstance(a, list):
            return [xxx_degrees_item(x) for x in a]
        return xxx_degrees_item(a)

    def rads2degrees(self, a):
        if a < 0:
            a += 2.*math.pi
        return a*180/math.pi