from datetime import datetime
import urllib
import argparse
import numpy as np
import os
import requests
import imutils

import cv2


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


# Puerta principal
# stream = ReadIPStream('http://10.253.247.34:88/cgi-bin/CGIStream.cgi?cmd=GetMJStream&usr=guest&pwd=smpt00')
# name_door = "principal"

## Puerta de arriba
# name_door = "arriba"
# stream = ReadIPStream('http://10.253.247.35:88/cgi-bin/CGIStream.cgi?cmd=GetMJStream&usr=guest&pwd=smpt00')

## Puerta trasera
stream = ReadIPStream('http://user:cotilla$@10.253.247.22:5900/mjpg/video.mjpg')
name_door = "trasera"

# name_door = "livingroom"
# vs = cv2.VideoCapture('rtsp://camera3:opticalflow3@10.253.247.39:88/videoMain')

bytes = ''

saving_dir = "videos"

currentDate = datetime.now()
date = datetime.strftime(currentDate, "%m%d%H%M")

if not os.path.isdir(saving_dir):
    os.mkdir(saving_dir)

video_dir = os.path.join(saving_dir,  name_door + "_" + date  + ".mp4")

try:
    out = cv2.VideoWriter(video_dir, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (640, 480))
    print("[SAVING]", video_dir)
except:
    print("Fail openning video writer")


while True:
    ret,frame = stream.read_stream()
    # ret,frame = vs.read()
    frame = cv2.resize(frame, (640,480))

    if ret:
        out.write(frame)
        cv2.imshow("Frame", frame)

        if cv2.waitKey(1) == 27:  # if user hit esc
            exit(0)  # exit program
