from turtle import distance
from charset_normalizer import detect
import numpy as np
import cv2

def focal_length(measured_distance, real_width, width_in_rf_image):
    """ Auxilary function to finding the focal length from given parameters
    """
    focal_length = (width_in_rf_image * measured_distance) / real_width
    return focal_length

def distance(focal_length, real_size, measured_size):
    """Function to calculate the distance from some known parameters and the measurement

    Args:
        focal_length (double): The focal_length of the camera. You can use focal_length method to find out once.
        real_size (double): The real size of an object (for example height or width)
        measured_size (double): The measured size of the same object

    Returns:
        [double]: The distance to the object
    """
    distance = (focal_length * real_size) / measured_size
    return distance

class BlockDetector:
    """The job of the BlockDetector is to detect block in an image
    """
    def __init__(self, focal_length, block_size):
        self.lower_blue_limit=np.array([100,100,20]) # setting the blue lower limit
        self.upper_blue_limit=np.array([140,255,255]) # setting the blue upper limit
    
        self.lower_right_red_limit=np.array([165,100,20]) # setting the red lower limit (for right spectrum red)
        self.upper_right_red_limit=np.array([180,255,255]) # setting the red upper limit (for right spectrum red)

        self.lower_left_red_limit=np.array([0,100,20]) # setting the red lower limit (for left spectrum red)
        self.upper_left_red_limit=np.array([10,255,255]) # setting the red upper limit (for left spectrum red)

        self.focal_length = focal_length
        self.block_size = block_size

    def contour_close_enough_to_point(self, contour, point, accept_diff):
        difference_to_point = np.abs(np.array(contour) - np.array(point))
        coordinate_in_accept_range = np.less_equal(difference_to_point, accept_diff)
        point_in_accept_range = np.all(coordinate_in_accept_range, 2)
        any_point_accepted = np.any(point_in_accept_range)
        return any_point_accepted    

    def detect(self, frame):
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        blue_mask = cv2.inRange(frame_hsv, self.lower_blue_limit, self.upper_blue_limit)
        red_mask = cv2.inRange(frame_hsv, self.lower_left_red_limit, self.upper_left_red_limit)
        
        cv2.imwrite("original.png", frame)
        cv2.imwrite("blue_mask.png", blue_mask)
        cv2.imwrite("red_mask.png", red_mask)

        #combined_mask = cv2.bitwise_or(blue_mask, red_mask)
        combined_mask = red_mask
        cv2.imwrite("c_mask.png", combined_mask)

        contours, _ = cv2.findContours(combined_mask, 1, 2)
        
        block_rectangles = []
        img = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)

            if w > 20 and h > 40 and h > w*1.3:
                accept_diff = np.array([w*0.1, h*0.1])
                close = [self.contour_close_enough_to_point(contour, [x,y], accept_diff),
                        self.contour_close_enough_to_point(contour, [x+w,y], accept_diff),
                        self.contour_close_enough_to_point(contour, [x+w,y+h], accept_diff),
                        self.contour_close_enough_to_point(contour, [x,y+h], accept_diff)]

                is_contour_rectangle = np.count_nonzero(close) >= 2# and close not in [[True, False, True, False], [False, True, False, True]]

                color = 0
                if is_contour_rectangle:
                    color = (0,200,0)
                    block_rectangles.append([[x+w,y], [x+w,y+h], [x,y+h], [x,y]])
                else:
                    color = (0,0,200)

                img = cv2.rectangle(img,(x,y),(x+w,y+h),color,2)
        
        
        cv2.imwrite("result.png", img)


        return np.array(block_rectangles)

    def detect_locations(self, frame):
        """Detects blocks from a frame and calculates the positon of them relative to the robot.

        Args:
            frame (cv2 brg image): Frame 

        Returns:
            [[float,float,float], [float,float,float], [int]]: Translateion, rotation, id
        """
        corners = self.detect(frame)
        if len(corners) > 0:
            centers = (corners[:,1]-corners[:,3])/2+corners[:,3]
            #Is to the right side of the center positive or to the left? (Check afterwards)
            x_translations = centers[:,0] - (frame.shape[1] / 2)
            z_translations = [distance(self.focal_length, self.block_size, corn[1,1]-corn[3,1]) for corn in corners]

            return np.array([x_translations, centers[:,1], z_translations]).T
        else:
            return None
