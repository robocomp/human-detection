#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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

import sys, Ice, os


ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'

Ice.loadSlice("-I ./src/ --all ./src/CameraRGBDSimple.ice")
import RoboCompCameraRGBDSimple
Ice.loadSlice("-I ./src/ --all ./src/HumanCameraBody.ice")
import RoboCompHumanCameraBody

class ImgType(list):
    def __init__(self, iterable=list()):
        super(ImgType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(ImgType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(ImgType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(ImgType, self).insert(index, item)

setattr(RoboCompCameraRGBDSimple, "ImgType", ImgType)

class DepthType(list):
    def __init__(self, iterable=list()):
        super(DepthType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(DepthType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(DepthType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(DepthType, self).insert(index, item)

setattr(RoboCompCameraRGBDSimple, "DepthType", DepthType)

class DescriptorFloat(list):
    def __init__(self, iterable=list()):
        super(DescriptorFloat, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, float)
        super(DescriptorFloat, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, float)
        super(DescriptorFloat, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, float)
        super(DescriptorFloat, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "DescriptorFloat", DescriptorFloat)

class DescriptorByte(list):
    def __init__(self, iterable=list()):
        super(DescriptorByte, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(DescriptorByte, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(DescriptorByte, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(DescriptorByte, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "DescriptorByte", DescriptorByte)

class DescByteList(list):
    def __init__(self, iterable=list()):
        super(DescByteList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHumanCameraBody.DescriptorByte)
        super(DescByteList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHumanCameraBody.DescriptorByte)
        super(DescByteList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHumanCameraBody.DescriptorByte)
        super(DescByteList, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "DescByteList", DescByteList)

class DescFloatList(list):
    def __init__(self, iterable=list()):
        super(DescFloatList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHumanCameraBody.DescriptorFloat)
        super(DescFloatList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHumanCameraBody.DescriptorFloat)
        super(DescFloatList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHumanCameraBody.DescriptorFloat)
        super(DescFloatList, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "DescFloatList", DescFloatList)

class ImgType(list):
    def __init__(self, iterable=list()):
        super(ImgType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(ImgType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(ImgType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(ImgType, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "ImgType", ImgType)

class GroundTruth(list):
    def __init__(self, iterable=list()):
        super(GroundTruth, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHumanCameraBody.TGroundTruth)
        super(GroundTruth, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHumanCameraBody.TGroundTruth)
        super(GroundTruth, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHumanCameraBody.TGroundTruth)
        super(GroundTruth, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "GroundTruth", GroundTruth)

class People(list):
    def __init__(self, iterable=list()):
        super(People, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHumanCameraBody.Person)
        super(People, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHumanCameraBody.Person)
        super(People, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHumanCameraBody.Person)
        super(People, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "People", People)


class GenericWorker(object):


    def __init__(self, mprx):
        super(GenericWorker, self).__init__()

        self.camerargbdsimple_proxy = mprx["CameraRGBDSimpleProxy"]
        self.humancamerabody_proxy = mprx["HumanCameraBodyPub"]


    def killYourSelf(self):
        rDebug("Killing myself")
        self.kill.emit()

    def setPeriod(self, p):
        pass
