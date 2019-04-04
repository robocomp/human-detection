import cv2
import numpy as np

cam = cv2.VideoCapture(1)

while True:
    ret, frame = cam.read()
    cv2.imshow("Hola",frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # wait for ESC key to exit
        break
cam.release()
cv2.destroyAllWindows()

