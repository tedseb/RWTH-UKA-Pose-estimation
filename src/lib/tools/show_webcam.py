import cv2
#out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 25, (1280, 720))

def show_webcam():
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    while True:
        ret_val, img = cam.read()
        if not ret_val:
            print("No image")
            return
        #img = cv2.resize(img, (1200, 900))
        #print(img.shape)
        cv2.imshow('my webcam', img)
        key = cv2.waitKey(1)
        if key == 27 or key == ord('q'): 
            break  # esc to quit
        #out.write(img)
    #out.close()
    cv2.destroyAllWindows()


def main():
    show_webcam()


if __name__ == '__main__':
    main()