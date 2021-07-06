import cv2
import numpy as np

cap = cv2.VideoCapture("rtsp://admin:admin@192.168.178.179")

while(True):
    ret, frame = cap.read()
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()