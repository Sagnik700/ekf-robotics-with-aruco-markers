# define imports
import math
import matplotlib.pyplot as plt
import numpy as np

import os
import shutil
import time

from vision.aruco_detector import ArucoDetector
import cv2

import random

from . import utils_math

# define constants
ROBOT_WIDTH = 12  # measure and put

class EKF_SLAM:
    def __init__(self):
        # initialize the mu and covariance with arbiteray starting positions
        #self.mu = np.array([0, 0, 0])
        self.mu = np.array([0, 0, 0.5 * np.pi])
        self.cov = np.zeros((3,3))

        # create an empty landmark map where landmark id and count will be stored as key-value pairs
        self.landmark_map = dict()
        self.total_landmarks = 0

    def camera_detections(sefl, camera_coords, frame):
        """Get landmarks from camera image and return a list where each item is a landmark observation.

        Args:
            camera_coords ([float, float, float]): The camera coordinates. (x, y, alpha)

        Returns:
            [[actual_measurement, world_coords, camera_coords, id], ...]: [description]
        """
        
        detector = ArucoDetector()
        translations, _, ids = detector.detect_locations(frame, cv2.aruco.DICT_ARUCO_ORIGINAL)

        if translations is not None:
            alpha = np.arctan2(translations[:,2], translations[:,0])
            r = np.sqrt(translations[:,0]**2 + translations[:,2]**2)
            actual_measurements = np.array([r, alpha]).T

            psi = camera_coords[2] + alpha
            x_r = np.cos(psi) * r
            y_r = np.sin(psi) * r
            world_coords_robot = np.array([x_r, y_r]).T
            world_coords = camera_coords[:2] + world_coords_robot

            return [[actual_measurement, world_coord, camera_coords, id] for actual_measurement, world_coord, id in zip(actual_measurements, world_coords, ids)]
        else:
            return []


    def get_alpha_angle(self, l: float, r: float) -> float:
        return (r-l) / ROBOT_WIDTH

    def get_radius_capitalR(self, l: float, alpha: float) -> float:
        if alpha != 0:
            return l / alpha
        return 0

    def predict(self, u: tuple[float, float]) -> None:
        """Predict the current location of the robot

        Args:
            u (tuple[float, float]): Current position of the robot
        """
        # get l & r from the movement variable u
        l, r = u
        theta = self.mu[2] #TODO: check if theta is updated correct

        # cov_u = np.array([
        #     [np.var(l), 0],     #TODO: INSERT ACTUAL VARIANCES LATER
        #     [0, np.var(r)]
        # ])
        cov_u = np.array([
            [0.001, 0],     #INSERT ACTUAL VARIANCES LATER
            [0, 0.001]
        ])

        alpha = self.get_alpha_angle(l, r)

        if alpha != 0:
            R = l / alpha
        else:
            R = np.nan
        
        if (l == r):
            # compute G, A, B, C, D
            G = np.array([
                [1, 0, -l * np.sin(theta)],
                [0, 1, l * np.cos(theta)],
                [0, 0, 1]
            ])

            A = (1/2) * (np.cos(theta) + (l/ROBOT_WIDTH) * np.sin(theta))
            B = (1/2) * (np.sin(theta) - (l/ROBOT_WIDTH) * np.cos(theta))
            C = (1/2) * (np.cos(theta) - (l/ROBOT_WIDTH) * np.sin(theta))
            D = (1/2) * (np.sin(theta) + (l/ROBOT_WIDTH) * np.cos(theta))
        else:
            G = np.array([
                [1, 0, ((l/alpha) + ROBOT_WIDTH/2) * (np.cos(theta + alpha) - np.cos(theta))],
                [0, 1, ((l/alpha) + ROBOT_WIDTH/2) * (np.sin(theta + alpha) - np.sin(theta))],
                [0, 0, 1]
            ])

            A = ((ROBOT_WIDTH * r) / np.square(r-l)) * \
                (np.sin(theta + (r-l) / ROBOT_WIDTH) - np.sin(theta)) \
                - (r+l) / (2 * (r-l)) * np.cos(theta + (r-l) / ROBOT_WIDTH)
            B = ((ROBOT_WIDTH * r)/ np.square(r-l)) * \
                (-np.cos(theta + (r-l) / ROBOT_WIDTH) + np.cos(theta)) \
                - (r+l) / (2 * (r-l)) * np.sin(theta + (r-l) / ROBOT_WIDTH)
            C = -((ROBOT_WIDTH * l)/np.square(r-l)) * \
                (np.sin(theta + (r-l) / ROBOT_WIDTH) - np.sin(theta)) \
                + (r+l) / (2 * (r-l)) * np.cos(theta + (r-l) / ROBOT_WIDTH)
            D = -((ROBOT_WIDTH * l)/np.square(r-l)) * \
                (-np.cos(theta + (r-l) / ROBOT_WIDTH) + np.cos(theta)) \
                + (r+l) / (2 * (r-l)) * np.sin(theta + (r-l) / ROBOT_WIDTH)

        # compute V
        V = np.array([
            [A, C],
            [B, D],
            [-(1/ROBOT_WIDTH), (1/ROBOT_WIDTH)]
        ])

        # compute G_hat and V_hat as directed in the EKFSLAM pdf to be used going forward
        if self.total_landmarks != 0:
            N = self.total_landmarks
            G_row = G.shape[0]
            G_col = G.shape[1]
            G_hat = utils_math.assemble_matrix(
                G,
                np.zeros((G_row, 2 * N)),
                np.zeros((2 * N, G_col)),
                np.eye(2 * N)
            )

            V_hat = np.append(V, np.zeros((2 * N, 2)), axis = 0)
            
        else:
            G_hat = G
            V_hat = V
        
        # according to Robot Motion Model of EKFSLAM pdf
        x_y = np.array([
            [self.mu[0]],
            [self.mu[1]]
        ])

        if alpha == 0:
            x_prime_y_prime = x_y + l * np.array([
                [np.cos(theta)],
                [np.sin(theta)]
            ])

            theta_prime = theta
        else:
            x_prime_y_prime = x_y + (R + (ROBOT_WIDTH/2)) * np.array([
                [np.sin(theta + alpha) - np.sin(theta)],
                [(-np.cos(theta + alpha)) - (-np.cos(theta))]
            ])

            theta_prime = (theta + alpha) % (2 * np.pi)

        # Predict new state
        self.mu[:3] = np.append(x_prime_y_prime, theta_prime)

        # Predict new covariance
        self.cov = G_hat.dot(self.cov).dot(G_hat.T) + \
            (V_hat).dot(cov_u).dot(V_hat.T)
            

    def add_landmark(self, world_coords: np.array, id: int) -> None:
        """Adds a single landmarks to slam.

        Args:
            world_coords (np.array): Position of the landmarks
            id (int): Landmark id
        """
        # update the mu
        self.mu = np.append(self.mu, world_coords)

        std_x_i = 200000     # choose standard deviation
        std_y_i = 200000     # very very big, maximum world size (see SLAM pdf)

        cov_i = np.array([
            [std_x_i, 0],
            [0, std_y_i]
        ])

        # update the covariance
        cov_row = self.cov.shape[0]
        cov_col = self.cov.shape[1]
        self.cov = utils_math.assemble_matrix(
            self.cov,
            np.zeros((cov_row,2)),
            np.zeros((2,cov_col)),
            cov_i
        )
                             
        # increase the count of total landmarks found                             
        self.total_landmarks += 1
        # set the landmark number in the map
        self.landmark_map[id] = self.total_landmarks

    def correction(self, actual_measurement, world_coord, camera_coord, id) -> None:#
        """Corrects the map containig landmarks and robot position

        Args:
            actual_measurement (): The landmark measurments form camera
            world_coord ([x,y]): The poistion of the landmark in slam map
            camera_coord ([x,y]): The position of the camera in slam map
            id (int): The landmark id
        """
        #Last work - sent all aruco markers list instead of individual

        # std_r = 0.05
        # std_alpha = 20

        std_r = 0.05
        std_alpha = 20

        # compute Q
        Q = np.array([
            [std_r, 0],
            [0, std_alpha]
        ])
        
        
        # get landmark index number
        landmark_ind = self.landmark_map[id]

        #print('Landmark_Ind', landmark_ind)

        print("Compare", self.mu[2 * landmark_ind + 1], self.mu[2 * landmark_ind + 2], world_coord)

        # compute h(x, y, theta, x_m, y_m) = [[r], [alpha]]
        x_m_minus_x = self.mu[2 * landmark_ind + 1] - self.mu[0]
        y_m_minus_y = self.mu[2 * landmark_ind + 2] - self.mu[1]

        r = np.sqrt(np.square(x_m_minus_x) + np.square(y_m_minus_y))
        alpha = np.arctan2(x_m_minus_x, y_m_minus_y) - self.mu[2]

        h = np.array([r, alpha])

        # compute expected observations
        denom = (np.square(x_m_minus_x) + np.square(y_m_minus_y))
        delta_r_x      = - (x_m_minus_x) / np.sqrt(denom)
        delta_alpha_x  =   (y_m_minus_y) / (denom)
        delta_r_y      = - (y_m_minus_y) / np.sqrt(denom)
        delta_alpha_y  = - (x_m_minus_x) / (denom)
        delta_r_xm     = - delta_r_x
        delta_alpha_xm = - delta_alpha_x
        delta_r_ym     = - delta_r_y
        delta_alpha_ym = - delta_alpha_y
        
        H = np.array([
            [delta_r_x, delta_r_y, 0],
            [delta_alpha_x, delta_alpha_y, -1]
        ])
        
        H_landmark = np.array([
            [delta_r_xm, delta_r_ym],
            [delta_alpha_xm, delta_alpha_ym]
        ])
        
        for i in range(1, self.total_landmarks+1):
            if i != landmark_ind:
                H = np.append(H, np.zeros((2,2)), axis = 1)
            else:
                H = np.append(H, H_landmark, axis = 1)

        # compute Q
        # Q = np.array([
        #     [np.var(r), 0],
        #     [0, np.var(alpha)]
        # ])

        # calculate Kalman gain
        K0 = H.dot(self.cov).dot(H.T)
        K1 = np.linalg.inv(K0 + Q)
        K = self.cov.dot(H.T.dot(K1))

        # calculate difference between expected and real observation
        m_dif_r = actual_measurement[0] - h[0]
        # m_dif_alpha = ((actual_measurement[1] + 2*np.pi) % (2*np.pi)) - ((h[1] + 2*np.pi) % (2*np.pi))
        # m_dif_alpha = m_dif_alpha % (2*np.pi)

        diff = world_coord - camera_coord[:2]
        alpha = np.arctan2(diff[0], diff[1])-camera_coord[2]
        
        m_dif_alpha = ((alpha - h[1]) + np.pi) % (2*np.pi) - np.pi
        m_dif = np.array([m_dif_r, m_dif_alpha])

        # update state vector and covariance matrix
        self.mu = self.mu + K.dot(m_dif)

        # identity matrix should be with size (3+2N)X(3+2N)
        #self.cov = (np.eye(3+(2*self.total_landmarks)) - K.dot(H)).dot(self.cov)
        self.cov = (np.eye(self.cov.shape[0]) - K.dot(H)).dot(self.cov)

        print("Compare End", self.mu[2 * landmark_ind + 1], self.mu[2 * landmark_ind + 2], world_coord)

    def get_robot_pose(self) -> np.array:
        """Returns robot position

        Returns:
            np.array: [x,y,theta]
        """
        return self.mu[:3]

    def get_landmark_positions(self) -> np.array:
        """Returns all landmark positions

        Returns:
            np.array: [[x,y], ...]
        """
        return self.mu[3:]

    def correct_all_landmarks(self, actual_measurement, world_coord, camera_coord):
        """Call correction for each landmark in the map

        Args:
            actual_measurement (): The landmark measurments form camera
            world_coord ([x,y]): The poistion of the landmark in slam map
            camera_coord ([x,y]): The position of the camera in slam map
        """
        for id in self.landmark_map.keys():
            self.correction(actual_measurement, world_coord, camera_coord, id)

    def get_position_of_landmark(self, id):
        """Get the position of a specific landmark

        Args:
            id (int): Landmark id

        Returns:
            [float,float]: [x,y]
        """
        print(self.landmark_map.items())
        print(id)
        if id in self.landmark_map.keys():
            index = self.landmark_map[id]
            return [self.mu[1+index*2], self.mu[2+index*2]]
        else:
            return None
