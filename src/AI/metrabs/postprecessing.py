#!/usr/bin/env python3
import numpy as np
import cv2

i=6000
image = cv2.imread('./TedAllein&TraurigAfter/Ted'+ str(i) +'.png')
height, width, channels = image.shape
out = cv2.VideoWriter('./Ted_after.avi', cv2.VideoWriter_fourcc(*'XVID'), 20.0, (int(width), int(height)))
while (True):
    image = cv2.imread('./ArturAlleineTraurigAfter/Ted'+ str(i) +'.png')
    out.write(image)
    cv2.imshow("Evaluation", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    i+=1
    #if i > 107:
    #    break
