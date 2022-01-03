import numpy as np
import cv2

cap = cv2.VideoCapture("data/videos/video.mp4")

ret = True
frame_num = 0
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

while(ret):
    ret, frame = cap.read()
    if not ret:
        if frame_num != total_frames:
            print(f"error at frame {frame_num + 1}")
        else: 
            print("success")
        break

    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    frame_num += 1

print(f"{frame_num}/{total_frames}") 
cap.release()
cv2.destroyAllWindows()