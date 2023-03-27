import numpy as np                 # v 1.19.2
import matplotlib.pyplot as plt    # v 3.3.2
import re
import find_intersection

def creat_polygon(xs, ys, annotations):
    """Creates an image of an array of landmarks

    Args:
        xs (float): Landmark x coordinates
        ys (float): Landmark y coordinates
        annotations (_type_): _description_

    Returns:
        _type_: _description_
    """

    # Used for debuging

    # Enter x and y coordinates of the aruco marker coordinates, colors and marker annotations
    #xs = [6, 5, 4, 4, 3, 2, 2, 2, 2, 2, 3, 4, 4, 6, 8, 10,
    #      12, 14, 15, 16, 16, 16, 15, 14, 12, 10, 10, 12, 14, 16, 18, 18, 18, 18, 17, 16, 14, 12, 10, 8]
    #ys = [2, 3, 4, 6, 7, 8, 10, 12, 14, 16, 17, 18, 20, 20, 20,
    #      20, 20, 20, 19, 18, 16, 14, 13, 12, 12, 12, 10, 10, 10, 10, 10, 8, 6, 4, 3, 2, 2, 2, 2, 2]
    #annotations = ["B1", "M1", "C1", "D1", "N1", "E1", "A", "B", "F1", "G1", "O1", "C", "P1",
    #               "D", "E", "F", "G", "H", "J1", "I", "I1", "J", "K1", "K", "L", "M", "N", "O", "P", "Q", "R", "H1", "S", "T", "L1", "U", "V", "W", "Z", "A1"]
    
    colors = ['b']
    
    # Select length of axes and the space between tick labels
    xmin, xmax, ymin, ymax = min(xs)-20, max(xs)+20, min(ys)-20, max(ys)+20 
    ticks_frequency = 100
    
    # Plot points
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(xs, ys, c=colors)
    
    for i, label in enumerate(annotations):
        plt.annotate(label, (xs[i], ys[i]))
    
    # Set identical scales for both axes
    ax.set(xlim=(xmin-1, xmax+1), ylim=(ymin-1, ymax+1), aspect='equal')
    
    # Set bottom and left spines as x and y axes of coordinate system
    ax.spines['bottom'].set_position('zero')
    ax.spines['left'].set_position('zero')
    
    # Remove top and right spines
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    
    # Create 'x' and 'y' labels placed at the end of the axes
    ax.set_xlabel('X', size=14, labelpad=-24, x=1.03)
    ax.set_ylabel('Y', size=14, labelpad=-21, y=1.02, rotation=0)
    ax.set_title("Subtask 4 map", fontsize=15)
    
    # Create custom major ticks to determine position of tick labels
    x_ticks = np.arange(xmin, xmax+1, ticks_frequency)
    y_ticks = np.arange(ymin, ymax+1, ticks_frequency)
    ax.set_xticks(x_ticks[x_ticks != 0])
    ax.set_yticks(y_ticks[y_ticks != 0])
    
    # Create minor ticks placed at each integer to enable drawing of minor grid
    # lines: note that this has no effect in this example with ticks_frequency=1
    ax.set_xticks(np.arange(xmin, xmax+1), minor=True)
    ax.set_yticks(np.arange(ymin, ymax+1), minor=True)
    
    # Draw major and minor grid lines
    ax.grid(which='both', color='grey', linewidth=1, linestyle='-', alpha=0.2)
    
    # Draw arrows
    arrow_fmt = dict(markersize=4, color='black', clip_on=False)
    ax.plot((1), (0), marker='>', transform=ax.get_yaxis_transform(), **arrow_fmt)
    ax.plot((0), (1), marker='^', transform=ax.get_xaxis_transform(), **arrow_fmt)
              
    # Make a polygon out of all the aruco marker points in the coordinate system
    #polygon = []
    #for i in range(len(xs)):
    #    polygon.append((xs[i], ys[i]))

    #polygon = np.array(polygon)

    #ax.plot(polygon[:,0], polygon[:,1])
    
    return ax
