import math
import ev3_dc as ev3

from time import sleep

from ekfslam import slam
import driving
import driving.drive
import driving.collision
import driving.navigation

from claw import Claw

from vision.qr_code_img_reading import QrCode
from vision.block_detector import BlockDetector
from vision.positioning import BlockPositioner

import numpy as np
import find_intersection
import create_map
from path_planning.astar import AStarPlanner

import matplotlib.pyplot as plt
from simple_pid import PID
import cv2

NUMBER_OF_BLOCKS = 5

class IntersectionLandmark:
    def __init__(self, pos, radius):
        self.position = pos
        self.radius = radius

class Robot:
    def __init__(self, ev3, cam_id) -> None:
        self.ekf_slam = slam.EKF_SLAM()
        self.block_detector = BlockDetector(720, 9.5)
        self.driver = driving.drive.Driver(ev3, cam_id, self.ekf_slam)
        self.claw = Claw(ev3)
        self.cam_id = cam_id

    def initialize_map(self):
        """Rotates the robot to initialize the map
        """
        for i in range(0, 361, 30):
            self.driver.rotate(30, 0)

    def read_destinaton_from_qr(self):
        """Takes a photo and read data from QR code.
        Retrieves the goal position form that data.

        Returns:
            (x0, y0), (x1, y1): Goal position
        """
        qr_code = QrCode()
        data = []

        while len(data) == 0:
            data = qr_code.read_qr_from_camera(self.cam_id, True)

        #intersectionmarkers = {token[0]:token[1] for marker in data[0].split(",") for token in marker.split("=")}
        # Separate out the indexes from the qr code data - {'MARKER1=XX,MARKER2=XX'}
        ind1 = data[0].find('=')
        ind2 = data[0].find(';') #Todo Change to comma
        ind3 = data[0].rfind('=')

        # Aruco marker 1 details
        qr_code1 = data[0][0:ind1]
        qr_code1_val = data[0][ind1+1:ind2]

        # Aruco marker 2 details
        qr_code2 = data[0][ind2+1:ind3]
        qr_code2_val = data[0][ind3+1:len(data[0])]

        intersectionmarkers = {int(qr_code1):int(qr_code1_val), int(qr_code2):int(qr_code2_val)}

        # Replace marker id by marker posiiton
        for marker, radius in intersectionmarkers.items():
            intersectionmarkers.update({marker : IntersectionLandmark(self.ekf_slam.get_position_of_landmark(marker), radius)})

        markers = list(intersectionmarkers.values())
        x0, y0, x1, y1 = find_intersection.get_intersections(markers[0].position, markers[0].radius, markers[1].position, markers[1].radius)

        return (x0, y0), (x1, y1)

    def generate_map_image(self, destination=None, path=[]):
        """Generates an image from the slam map

        Args:
            destination ([x,y], optional): Where teh robot should drive to. Defaults to None.
            path (list, optional): A path the robot should drive along. Defaults to [].
        """
        x, y = self.ekf_slam.get_landmark_positions()[::2], self.ekf_slam.get_landmark_positions()[1::2]
        ax = create_map.creat_polygon(x, y, self.ekf_slam.landmark_map.keys())

        if destination is not None:
            ax.scatter([destination[0]], [destination[1]], c=["r"])

        x_r, y_r, theta = self.ekf_slam.get_robot_pose()
        ax.scatter([x_r], [y_r], c=["g"])
        x_arr, y_arr = math.sin(theta) * 100, math.cos(theta) * 100
        ax.plot([x_r, x_arr], [y_r, y_arr])

        if len(path) != 0:
            ax.scatter(path[:,0], path[:,1], c=["y"])

        plt.savefig("polygon.png")

    def drive_to_position(self, position):
        """Drives to robot to a specific position on the map.

        Args:
            position ([x,y]): where to drive

        Returns:
            [[x,y], ...]: The path which was used
        """
        x, y = self.ekf_slam.get_landmark_positions()[::2], self.ekf_slam.get_landmark_positions()[1::2]

        print("A-Star input", x, y)

        a_star_planer = AStarPlanner(x, y, 10, 12)

        x_r, y_r, theta = self.ekf_slam.get_robot_pose()
        path = np.array(a_star_planer.planning(x_r, y_r, position[0], position[1]), dtype=float).T

        self.generate_map_image(intersections[0], path)

        print("Path", path)

        self.driver.rotate(-(theta * 180) / np.pi, 0)
        driving.navigation.drive_along(self.driver, path, (x_r, y_r))

        return path

    def search_block(self):
        """Rotates the robot till a block was detected or full turn.

        Returns:
            _type_: Block location in case a block was detected otherwise None
        """
        for _ in range(0, 361, 30):
            cam = cv2.VideoCapture(self.cam_id)
            for _ in range(10):
                _, frame = cam.read()

                block_locations = self.block_detector.detect(frame)
                if len(block_locations) > 0:
                    return block_locations
            cam.release()

            self.driver.rotate(30, 0)
        return None

    def drive_to_nearest_block(self):
        """Drives to the nearest block
        """

        last_block_distance = 0

        # Get correction vector
        cam = cv2.VideoCapture(self.cam_id)
        _, frame = cam.read()
        cam.release()

        frame_size = np.flip(frame.shape[0:2])
        top_left = frame_size * 0.30
        bottom_right = frame_size - top_left


        for _ in range(20):
            pid = PID(0.1, 0.01, 0.05, setpoint=0)
            pid.output_limits = (2, 50)
            # Rotate in block direction
            for _ in range(20):
                cam = cv2.VideoCapture(self.cam_id)
                _, frame = cam.read()
                cam.release()

                block_positions = self.block_detector.detect(frame)
                if len(block_positions) > 0:
                    vec = BlockPositioner.to_rect((top_left, bottom_right), block_positions[0], frame)

                    if vec[0] == 0:
                            break
                    elif len(vec) == 2:
                        angle = pid(vec[0])
                        print("PID_rotation", vec[0], angle)
                        self.driver.rotate(angle, 0)

                        

            # Drive to block
            # Retrieve distence to block
            block_distance = 0
            cam = cv2.VideoCapture(self.cam_id)
            for _ in range(10):
                _, frame = cam.read()

                block_locations = self.block_detector.detect_locations(frame)
                if block_locations is not None:
                    block_distance += block_locations[0][2]
            cam.release()
            block_distance /= 10
            print("Block distance", block_distance)
            
            if block_distance != 0:
                self.driver.drive_distance(block_distance / 2)
                last_block_distance = block_distance
            elif last_block_distance != 0:
                self.driver.drive_distance(last_block_distance / 2)
                break


if __name__ == '__main__':
    with ev3.EV3(protocol=ev3.USB) as my_ev3:
        robot = Robot(my_ev3, 0)
        robot.initialize_map()
        robot.generate_map_image()


        for i in  range(NUMBER_OF_BLOCKS):
            block_locations = robot.search_block()
            if block_locations is not None:
                robot.drive_to_nearest_block()
            robot.claw.grab()
            robot.driver.drive_distance(-35)
            intersections = robot.read_destinaton_from_qr()
            robot.generate_map_image(intersections[0])
            # Check which intersection to use
            path = robot.drive_to_position(intersections[0])
            robot.claw.release()
            robot.driver.drive_distance(-20)
        