import numpy as np

import cv2
from imutils.video import VideoStream

from vision.aruco_detector import ArucoDetector
from vision.block_detector import BlockDetector

class ObjectPositioner:

    def __init__(self, frame):
        self.frame = frame


    def in_view(self) -> bool:
        """Returns wether the camera detects the object or not
        """
        raise NotImplementedError()

    def debug_image(rectangle, obj_bounds, image):
        # Draw debug image
        if image is not None:
            top_right, bottom_right, bottom_left, top_left = obj_bounds
            top_left_rect, bottom_right_rect = np.array(rectangle, dtype=int)
            
            # Draw object bounds
            color = (0,200,0)
            image = cv2.rectangle(image, bottom_right, top_left, color)
            # Draw rect
            color = (200,0,200)
            image = cv2.rectangle(image, top_left_rect, bottom_right_rect, color)
            cv2.imwrite("in_rect.png", image)

    def in_rect(rectangle, obj_bounds, image = None) -> bool:
        """Returns wether the object is inside a specific rectangle of the camera view

        Params:
            rectangle: ((int,int),(int,int)) -> The rectangle to check if the object is inside.
                                                (Top left corner (x, y), bottom right corner (x, y)).
        """

        top_right, bottom_right, bottom_left, top_left = obj_bounds
        top_left_rect, bottom_right_rect = np.array(rectangle)

        ObjectPositioner.debug_image(rectangle, obj_bounds, image)

        return (top_left > top_left_rect).all() and (bottom_right < bottom_right_rect).all()

    def to_rect(rectangle, obj_bounds, image = None):
        """Returns a vector which defines how the view must be modified to fit object into a rectangle.
        In case the object is bigger as the 
        
        Param: ((int,int),(int,int)) -> The rectangle where the object should be placed in.
                                        (Top left corner (x, y), bottom right corner (x, y)).
        """

        top_right, bottom_right, bottom_left, top_left = obj_bounds
        top_left_rect, bottom_right_rect = np.array(rectangle)

        vec = [0, 0]

        for i, (c, c_rect) in enumerate(zip(top_left, top_left_rect)):
            if (c < c_rect):
                vec[i] = c - c_rect

        for i, (c, c_rect) in enumerate(zip(bottom_right, bottom_right_rect)):
            if (c > c_rect):
                if vec[i] == 0:
                    vec[i] = c - c_rect
                else:
                    return [np.linalg.norm(bottom_right - top_left) / np.linalg.norm(bottom_right_rect - top_left_rect)]

        ObjectPositioner.debug_image(rectangle, obj_bounds, image)

        return vec

class ArucoPositioner(ObjectPositioner):

    def __init__(self, frame, aruco_type):
        super().__init__(frame)
        self.aruco_type = aruco_type


    def in_view(self) -> bool:
        """Returns wether the camera detects an aruco marker or not
        """

        detector = ArucoDetector()
        return len(detector.detect(self.frame, self.aruco_type)[0]) > 0

class BlockPositioner(ObjectPositioner):

    def __init__(self, frame):
        super().__init__(frame)


    def in_view(self) -> bool:
        """Returns wether the camera detects a block or not
        """

        detector = BlockDetector()
        return len(detector.detect(self.frame)[0]) > 0

