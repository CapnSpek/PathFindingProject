import datetime
import os
import sys

import matplotlib.pyplot as plt
from Initialization import Initialization
from RRT_Star import RRTStar
from Cubic_Spline_Interpolation import CubicSplineInterpolator

def Main(track_path):
    init = Initialization(track_path)
    
    finished = False
    serial = 1
    start = 0
    goal = 0
    length_of_track = len(init.x)
    path = []
    
    # while serial * (length_of_track//10) <  750:
    #     serial += 1
        
    # start = (serial-1) * (length_of_track//10)
    # init.x[start] = init.x[start] -12
    # init.y[start] = init.y[start] -12
    
    while finished == False:
        if (serial * (length_of_track//10)) < length_of_track:
            goal = serial * (length_of_track//10)
            print(goal)
        else:
            goal = length_of_track - 1
            finished = True
        
        print(init.x[start], init.y[start], init.x[goal], init.y[goal])
        print(init.x_min(start, goal), init.x_max(start, goal), init.y_min(start, goal), init.y_max(start, goal))
        rrt_star = RRTStar(init.track_name, init.x_left, init.y_left, init.x_right, init.y_right, (init.x[start], init.y[start]), (init.x[goal], init.y[goal]), init.obstacles, x_low=init.x_min(start, goal), x_high=init.x_max(start, goal), y_low=init.y_min(start, goal), y_high=init.y_max(start, goal), serial=serial)
        rrt_star.build_rrt_star()
        rrt_star.plot_rrt_star()
        rrt_star.export_path()
        if rrt_star.reached:
            path += rrt_star.path
            init.x[goal] = rrt_star.goal_discovered[0]
            init.y[goal] = rrt_star.goal_discovered[1]
            #i = start
            #while i < goal-7:
            #    init.obstacles.append((init.x[i], init.y[i]))
            #    i += 1
            start = goal
            #init.obstacles.append((init.x_left[start-7], init.y_left[start-7]))
            #init.obstacles.append((init.x_right[start-7], init.y_right[start-7]))
            serial += 1
    
    
    current_datetime = datetime.datetime.now()
    formatted_datetime = current_datetime.strftime("%Y-%m-%d_%H-%M")
    with open(f'./Data/paths_found/{init.track_name}_ALL_{formatted_datetime}_path.txt', 'w') as file:
        for point in path:
            file.write(str(point) + '\n')
            
    '''
    # Plot the RRT* tree and the path from start to goal
    plt.figure(figsize=(10, 8))
    plt.plot(init.x_left, init.y_left, label='Track Left Boundary', linestyle='--', color='red')
    plt.plot(init.x_right, init.y_right, label='Track Right Boundary', linestyle='--', color='green')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('RRT* Path Planning')
    plt.grid(True)
    # Plot the path from start to goal
    '''
    x_path, y_path = zip(*(path))
    '''
    plt.plot(x_path, y_path, 'g-', label='Unsmoothened full path', linewidth=2)
    plt.legend()
    plt.show()
    '''   
        
        
    # Remove repeated values in path while preserving the order
    path = list(dict.fromkeys(path))
        
    cubicSplineInterpolator = CubicSplineInterpolator(init.track_name, path)
    file_name = os.path.basename(track_path).split('.')[0]
    line_count = 0
    with open(f'./Data/racetrack-database/racelines/{file_name}.csv', 'r') as file:
        for line in file:
            line_count += 1

    print(f"The file {file_name} has {line_count} lines.")
    smoothed_path = cubicSplineInterpolator.smooth_path(num_points=line_count-1)
    cubicSplineInterpolator.export_path()
    
                         
    # Plot the RRT* tree and the path from start to goal
    plt.figure(figsize=(10, 8))
    plt.plot(init.x_left, init.y_left, label='Track Left Boundary', linestyle='--', color='red')
    plt.plot(init.x_right, init.y_right, label='Track Right Boundary', linestyle='--', color='green')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('RRT* Path Planning')
    plt.grid(True)
    # Plot the path from start to goal
    x_path, y_path = zip(*(smoothed_path))
    plt.plot(x_path, y_path, 'g-', label='Smoothened full path', linewidth=2)
    plt.legend()
    plt.show()
    
if __name__ == "__main__":
    Main(sys.argv[1])