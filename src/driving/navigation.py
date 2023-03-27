import ev3_dc as ev3
from . import drive
import math
import numpy as np

def drive_to(driver, end_position, start_position = (0, 0)):
    """Navigates a driver to a specific position. The coordinate system is relative to the driver rotation.
       The driver always look into x direction at the start_position.

    Args:
        driver (Driver): This driver is navigated to the end_position.
        end_position ((float, float)): This is the position where the driver should drive to (x, y).
        start_position ((float, float), optional): This is the position where the driver is at th beginning of the navigation (x, y). Defaults to [0, 0].
    """
    position = np.array(end_position) - np.array(start_position)
    degrees = math.atan(position[1]/position[0])*180/math.pi
    driver.rotate(degrees, 0)
    driver.drive_distance(math.sqrt(position[0]**2 + position[1]**2))

def drive_along(driver, path, start_position = (0, 0)):
    """Navigates a driver along a specific path. The coordinate system is relative to the driver rotation.

    Args:
        driver (Driver): This driver is navigated along the path.
        path ([[float,float],...]): List of points which identify the path.
        start_position (tuple, optional): The position where the driver starts driving. Defaults to (0, 0).
    """
    path = path - start_position
    second_last_position = np.array([0,0])
    last_position = np.array([0,0])
    for position in np.array(path):
        new_coord = last_position - second_last_position
        theta = np.arctan2(new_coord[1], new_coord[0])

        absolute_position_in_new_coord = np.array([position[0] * np.cos(theta) + position[1] * np.sin(theta),
                                          position[1] * np.cos(theta) - position[0] * np.sin(theta)])
        last_positon_in_new_coord = np.array([last_position[0] * np.cos(theta) + last_position[1] * np.sin(theta),
                                     last_position[1] * np.cos(theta) - last_position[0] * np.sin(theta)])
        relative_position_in_new_coord = absolute_position_in_new_coord - last_positon_in_new_coord
        drive_to(driver, relative_position_in_new_coord/4)

        second_last_position = last_position
        last_position = position

        print("Relative", relative_position_in_new_coord/4)
