from multiprocessing.connection import wait
from time import sleep
import cv2
import ev3_dc as ev3
import math
from . import collision

WHELE_SIZE = 17

class Driver:

    def __init__(self, ev3_obj, cam_id, slam):
        self.ultrasonic = ev3.Ultrasonic(ev3.PORT_3, ev3_obj=ev3_obj)
        self.cam_id = cam_id
        self.ev3_obj = ev3_obj
        self.left_movement = 0
        self.right_movement = 0
        self.ekf_slam = slam

    def drive_distance(self, distance):
        """ Drive for a given distance in a straight line

        Args:
            distance (float): The distance in cm you want to drive
        """

        with ev3.TwoWheelVehicle(2.6, 12, ev3_obj=self.ev3_obj, tracking_callback=self.driving) as vehicle:
            vehicle.drive_straight(distance).start()

            sleep(0.1)

            while vehicle.busy:
                sleep(0.1)

            self.left_movement += vehicle.position.x
            self.right_movement += vehicle.position.x

        self.slam_correction()


    def rotate(self, angle, radius):
        """ Rotate by a given angle using a spcific angle

        Args:
            angle (float): by which the robot should rotate
            radius (float): of the curve
        """

        with ev3.TwoWheelVehicle(2.6, 12, ev3_obj=self.ev3_obj, tracking_callback=self.driving, speed=5) as vehicle:
            vehicle.drive_turn(angle, radius).start()

            sleep(0.1)

            while vehicle.busy:
                sleep(0.1)

            inner = radius - (vehicle._tread / 2)
            outer = inner + vehicle._tread

            inner = math.radians(vehicle.position.o) * inner
            outer = math.radians(vehicle.position.o) * outer

            if angle > 0:
                self.left_movement += inner
                self.right_movement += outer
            else:
                self.left_movement += outer
                self.right_movement += inner

        self.slam_correction()

    def movement(self):
        """ Retunrs the distances each wheele moved since last movement call.

        Returns:
            [float, float]: [distance left wheel moved, distance right wheele moved]
        """

        movement = [self.left_movement, self.right_movement]
        self.left_movement = 0
        self.right_movement = 0
        return movement
                    
    def slam_correction(self):
        """ Calls the corrections step of slam.
        Also it makes a photo and retreives the aruco markers form it.
        """

        cam = cv2.VideoCapture(self.cam_id)
        _, frame = cam.read()
        cam.release()

        movements = self.movement()
        self.ekf_slam.predict(movements)
        arucos = self.ekf_slam.camera_detections(self.ekf_slam.get_robot_pose(), frame)

        for aruco in arucos:
            if aruco[3] not in self.ekf_slam.landmark_map:
                self.ekf_slam.add_landmark(aruco[1], aruco[3])
            self.ekf_slam.correction(aruco[0], aruco[1], aruco[2], aruco[3])