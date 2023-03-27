from distutils.filelist import translate_pattern
import numpy as np
import cv2
import imutils

class ArucoDetector:
    """The job of the ArucoDetector is to detec aruco markers.
    """
    def __init__(self):
        # Aruco parameters used by cv2.aruco.detectMarkers() for detection
        self.camera_matrix = np.array([[819.614890, 0.000000, 293.182081], [0.000000, 825.737837, 250.299296], [0.000000, 0.000000, 1.000000]])
        self.dist_coeffs = np.array([-0.043100, 0.162156, 0.000011, -0.005394, 0.000000])

    def detect(self, frame, aruco_type):
        """Detects aruco markers in an given frame.

        Params:
            frame: PiRGBArray -> Frame of an VideoStream.
            aruco_type -> cv2.aruco.DICT... which defines the shape of the aruco code.

        Returns:
            corners: np array -> List of the corners of each detected aruco marker (top_right, bottom_right, bottom_left, top_left).
            ids: list -> The ids of all detected aruco markers.
        """
        factor = frame.shape[1] / 1000
        frame = imutils.resize(frame, width=1000)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, cv2.aruco.Dictionary_get(aruco_type), parameters=parameters)


        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            corners = np.reshape(corners, (len(corners), 4, 2))
            corners = corners * factor
            return corners, ids
        else:
            return [[],[]]

    def detect_locations(self, frame, aurco_type):
        """Detects aruco markers from a frame and calculates the positon of them relative to the robot.

        Args:
            frame (cv2 brg image): Frame 
            aurco_type (): Aruco marker type

        Returns:
            [[float,float,float], [float,float,float], [int]]: Translateion, rotation, id
        """
        corners, ids = self.detect(frame, aurco_type)
        rotations, translations, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 5, self.camera_matrix, self.dist_coeffs)
        if translations is not None and rotations is not None:
            return translations[:,0], rotations[:,0], ids
        else:
            return None, None, None
