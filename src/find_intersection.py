import math

# A Python3 program to check if a given point  
# lies inside a given polygon 
# Refer https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/ 
# for explanation of functions onSegment(), 
# orientation() and doIntersect()  
  
# Define Infinite (Using INT_MAX  
# caused overflow problems)
INT_MAX = 10000
  
# Given three collinear points p, q, r,  
# the function checks if point q lies 
# on line segment 'pr' 
def onSegment(p:tuple, q:tuple, r:tuple) -> bool:
      
    if ((q[0] <= max(p[0], r[0])) &
        (q[0] >= min(p[0], r[0])) &
        (q[1] <= max(p[1], r[1])) &
        (q[1] >= min(p[1], r[1]))):
        return True
          
    return False
  
# To find orientation of ordered triplet (p, q, r). 
# The function returns following values 
# 0 --> p, q and r are collinear 
# 1 --> Clockwise 
# 2 --> Counterclockwise 
def orientation(p:tuple, q:tuple, r:tuple) -> int:
      
    val = (((q[1] - p[1]) * 
            (r[0] - q[0])) - 
           ((q[0] - p[0]) * 
            (r[1] - q[1])))
             
    if val == 0:
        return 0
    if val > 0:
        return 1 # Collinear
    else:
        return 2 # Clock or counterclock
  
def doIntersect(p1, q1, p2, q2):
      
    # Find the four orientations needed for  
    # general and special cases 
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
  
    # General case
    if (o1 != o2) and (o3 != o4):
        return True
      
    # Special Cases 
    # p1, q1 and p2 are collinear and 
    # p2 lies on segment p1q1 
    if (o1 == 0) and (onSegment(p1, p2, q1)):
        return True
  
    # p1, q1 and p2 are collinear and 
    # q2 lies on segment p1q1 
    if (o2 == 0) and (onSegment(p1, q2, q1)):
        return True
  
    # p2, q2 and p1 are collinear and 
    # p1 lies on segment p2q2 
    if (o3 == 0) and (onSegment(p2, p1, q2)):
        return True
  
    # p2, q2 and q1 are collinear and 
    # q1 lies on segment p2q2 
    if (o4 == 0) and (onSegment(p2, q1, q2)):
        return True
  
    return False
  
# Returns true if the point p lies  
# inside the polygon[] with n vertices 
def is_inside_polygon(points:list, p:tuple) -> bool:
      
    n = len(points)
      
    # There must be at least 3 vertices
    # in polygon
    if n < 3:
        return False
          
    # Create a point for line segment
    # from p to infinite
    extreme = (INT_MAX, p[1])
    count = i = 0
      
    while True:
        next = (i + 1) % n
          
        # Check if the line segment from 'p' to  
        # 'extreme' intersects with the line  
        # segment from 'polygon[i]' to 'polygon[next]' 
        if (doIntersect(points[i],
                        points[next], 
                        p, extreme)):
                              
            # If the point 'p' is collinear with line  
            # segment 'i-next', then check if it lies  
            # on segment. If it lies, return true, otherwise false 
            if orientation(points[i], p, 
                           points[next]) == 0:
                return onSegment(points[i], p, 
                                 points[next])
                                   
            count += 1
              
        i = next
          
        if (i == 0):
            break
          
    # Return true if count is odd, false otherwise 
    return (count % 2 == 1)

# Get the intersection points with respect to the radii and coordinates
# of the two circle centers
def get_intersections(pos0, r0, pos1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    x0, y0 = pos0
    x1, y1 = pos1

    d=math.sqrt((x1-x0)**2 + (y1-y0)**2)

    print(x0, y0, x1, y1, d, r0, r1)
    
    # non intersecting
    if d > r0 + r1 :
        print("no intersecting")
        return None
    # One circle within other
    if d < abs(r0-r1):
        print("One circle within other")
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        print("coincident circles")
        return None
    else:
        a=(r0**2-r1**2+d**2)/(2*d)
        h=math.sqrt(r0**2-a**2)
        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d   
        x3=x2+h*(y1-y0)/d     
        y3=y2-h*(x1-x0)/d 

        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d
        
        return (x3, y3, x4, y4)