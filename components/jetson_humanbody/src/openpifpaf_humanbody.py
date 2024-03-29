#!/usr/bin/env python3
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
#

# \mainpage RoboComp::openpifpaf_humanbody
#
# \section intro_sec Introduction
#
# Some information about the component...
#
# \section interface_sec Interface
#
# Descroption of the interface provided...
#
# \section install_sec Installation
#
# \subsection install1_ssec Software depencences
# Software dependences....
#
# \subsection install2_ssec Compile and install
# How to compile/install the component...
#
# \section guide_sec User guide
#
# \subsection config_ssec Configuration file
#
# <p>
# The configuration file...
# </p>
#
# \subsection execution_ssec Execution
#
# Just: "${PATH_TO_BINARY}/openpifpaf_humanbody --Ice.Config=${PATH_TO_CONFIG_FILE}"
#
# \subsection running_ssec Once running
#
#
#

import sys
import traceback
import IceStorm
import time
import os
import copy
import argparse
from termcolor import colored
# Ctrl+c handling
import signal


from specificworker import *

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)
 
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('iceconfigfile', nargs='?', type=str, default='etc/config')
    parser.add_argument('--startup-check', action='store_true')

    args = parser.parse_args()

    ic = Ice.initialize(args.iceconfigfile)
    status = 0
    mprx = {}
    parameters = {}
    for i in ic.getProperties():
        parameters[str(i)] = str(ic.getProperties().getProperty(i))

    # Topic Manager
    proxy = ic.getProperties().getProperty("TopicManager.Proxy")
    obj = ic.stringToProxy(proxy)
    try:
        topicManager = IceStorm.TopicManagerPrx.checkedCast(obj)
    except Ice.ConnectionRefusedException as e:
        print(colored('Cannot connect to rcnode! This must be running to use pub/sub.', 'red'))
        exit(1)


    # Create a proxy to publish CameraRGBSSIMPLE topic
    topic = False
    try:
        topic = topicManager.retrieve("CameraRGBDSimplePub")
    except:
        pass
    while not topic:
        try:
            topic = topicManager.retrieve("CameraRGBDSimplePub")
        except IceStorm.NoSuchTopic:
            try:
                topic = topicManager.create("CameraRGBDSimplePub")
            except:
                print('Another client created the HumanCameraBody topic? ...')
    pub = topic.getPublisher().ice_oneway()
    camerargbdsimplepubTopic = RoboCompCameraRGBDSimplePub.CameraRGBDSimplePubPrx.uncheckedCast(pub)
    mprx["CameraRGBDSimplePubPub"] = camerargbdsimplepubTopic



    # Create a proxy to publish a HumanCameraBody topic
    topic = False
    try:
        topic = topicManager.retrieve("HumanCameraBody")
    except:
        pass
    while not topic:
        try:
            topic = topicManager.retrieve("HumanCameraBody")
        except IceStorm.NoSuchTopic:
            try:
                topic = topicManager.create("HumanCameraBody")
            except:
                print('Another client created the HumanCameraBody topic? ...')
    pub = topic.getPublisher().ice_oneway()
    humancamerabodyTopic = RoboCompHumanCameraBody.HumanCameraBodyPrx.uncheckedCast(pub)
    mprx["HumanCameraBodyPub"] = humancamerabodyTopic

    if status == 0:
        worker = SpecificWorker(mprx, args.startup_check)
        worker.setParams(parameters)
    else:
        print("Error getting required connections, check config file")
        sys.exit(-1)

    adapter = ic.createObjectAdapter('CameraRGBDSimple')
    adapter.add(camerargbdsimpleI.CameraRGBDSimpleI(worker), ic.stringToIdentity('camerargbdsimple'))
    adapter.activate()
    
    signal.signal(signal.SIGINT, signal_handler)
    

    if ic:
        # try:
        ic.destroy()
        # except:
        #     traceback.print_exc()
        #     status = 1
