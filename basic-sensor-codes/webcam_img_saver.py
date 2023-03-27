"""
Simply display the contents of the webcam with optional mirroring using OpenCV 
via the new Pythonic cv2 interface.  Press <esc> to quit.
"""

import cv2
import os
import shutil
import time
import numpy as np

def ask_to_delete_dir(dir_name, force=False):
    if os.path.exists(dir_name) and os.path.isdir(dir_name):
        if not force:
            reply = input("Directory already exists! Delete directory "+ dir_name + "? [y/[n]] ")
            if reply in ['y', 'Y', 'yes']:
                shutil.rmtree(dir_name)
            else:
                print("Not deleting anything.")
                # sys.exit()
        else:
            print("Deleted directory " + dir_name)
            shutil.rmtree(dir_name)

def show_webcam(mirror=False, save_imgs=False):
    if save_imgs:
        ask_to_delete_dir("./images")
        os.mkdir("./images")

    cam = cv2.VideoCapture(0)
    counter = 0
    start_fps = time.time()
    start_delay = time.time()
    while True:
        ret_val, img = cam.read()
        if mirror: 
            img = cv2.flip(img, 1)
        cv2.imshow('my webcam', img)
        if save_imgs:
            end_delay = time.time()
            if end_delay - start_delay > 1:
                start_delay = end_delay
                cv2.imwrite(os.path.join("./images" , str(counter) + '.jpg'), img)
                print("saving img")
        
        key = cv2.waitKey(1)
        if key == 27: 
            cv2.destroyAllWindows()
            break  # esc to quit
        counter += 1
        if counter % 10 == 0:
            end_fps = time.time()
            print("fps:", np.round(10/(end_fps - start_fps), 1))
            start_fps = end_fps
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # show_webcam()
    show_webcam(mirror=False, save_imgs=True)