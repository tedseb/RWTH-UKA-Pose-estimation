import cv2
import numpy as np
from src.config import COLOR_RANGES

MODE = 0           #0 for webcame and 1 for loadinf an image
COLOR_PRESET = 1

if MODE == 0:
    cap = cv2.VideoCapture("./data/10_5Rund5_10L.avi")
else:
    img = cv2.imread("./data/image.jpg")

def nothing(arg): pass

def priorLocations(frame):
    frame0 = frame[50:200,200:350]                                                                                     #Python matrix --> [y,x]
    frame1 = frame[50:200,840:990]
    return [frame0, frame1]

def only_color(frame, color_ranges, morph):
    h, s, v, h1, s1, v1 = color_ranges
    kernel = np.ones((3, 3), np.uint8)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([h, s, v])
    upper = np.array([h1, s1, v1])
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    return res, mask

color_low, color_high = COLOR_RANGES[COLOR_PRESET][2], COLOR_RANGES[COLOR_PRESET][3]
cv2.namedWindow('image')
cv2.createTrackbar('h', 'image', color_low[0], 255, nothing)
cv2.createTrackbar('s', 'image', color_low[1], 255, nothing)
cv2.createTrackbar('v', 'image', color_low[2], 255, nothing)
cv2.createTrackbar('h1', 'image', color_high[0], 255, nothing)
cv2.createTrackbar('s1', 'image', color_high[1], 255, nothing)
cv2.createTrackbar('v1', 'image', color_high[2], 255, nothing)
cv2.createTrackbar('morph', 'image', 0, 10, nothing)

while True:
    if MODE == 0:
        ret, img = cap.read()

        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, img = cap.read()

    h = cv2.getTrackbarPos('h', 'image')
    s = cv2.getTrackbarPos('s', 'image')
    v = cv2.getTrackbarPos('v', 'image')
    h1 = cv2.getTrackbarPos('h1', 'image')
    s1 = cv2.getTrackbarPos('s1', 'image')
    v1 = cv2.getTrackbarPos('v1', 'image')
    morph = cv2.getTrackbarPos('morph', 'image')

    img_mask, mask = only_color(img.copy(), (h, s, v, h1, s1, v1), morph)

    cv2.imshow('img', img_mask)
    cv2.imshow('image', mask)
    pressed_key = cv2.waitKey(16) & 0xFF
    if pressed_key == ord('q'):
        break

print(f"Result {COLOR_RANGES[COLOR_PRESET][0]}: [{h},{s},{v}], [{h1},{s1},{v1}]")

if MODE == 0:
    cap.release()

cv2.destroyAllWindows()
