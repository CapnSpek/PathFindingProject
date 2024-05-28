import os
import pandas as pd
import numpy as np

import matplotlib.pyplot as plt

class Initialization:
    # Initialize class variables
    x = []  # x-coordinate of track centerline
    y = []  # y-coordinate of track centerline
    w_tr_right = []  # width of track on the right side
    w_tr_left = []  # width of track on the left side
    x_right = []  # x-coordinate of track right boundary
    y_right = []  # y-coordinate of track right boundary
    x_left = []  # x-coordinate of track left boundary
    y_left = []  # y-coordinate of track left boundary
    x_right_obs = []  # x-coordinate of obstacle right boundary
    y_right_obs = []  # y-coordinate of obstacle right boundary
    x_left_obs = []  # x-coordinate of obstacle left boundary
    y_left_obs = []  # y-coordinate of obstacle left boundary
    track_path = None  # path of the track data file
    track_name = None  # name of the track
    nx = []  # x-component of the track normal vector
    ny = []  # y-component of the track normal vector
    obstacles = []  # list of all obstacles

    def __init__(self, track_path):
        self.track_path = track_path
        self.read_track_data()  # Read track data from file
        self.calculate_boundary_points()  # Calculate track boundary points
        self.calculate_obstacle_points()  # Calculate obstacle boundary points
        self.load_total_obstacles()  # Load all obstacles
        self.plot_track()  # Plot the track
        
    def x_min(self, start, goal):
        x_left_cut = self.x_left[start:goal]
        x_right_cut = self.x_right[start:goal]
        x_left_min =  min(x_left_cut)
        x_right_min = min(x_right_cut)
        return min(x_left_min, x_right_min)-5
    
    def x_max(self, start, goal):
        x_left_cut = self.x_left[start:goal]
        x_right_cut = self.x_right[start:goal]
        x_left_max =  max(x_left_cut)
        x_right_max = max(x_right_cut)
        return max(x_left_max, x_right_max)+5
    
    def y_min(self, start, goal):
        y_left_cut = self.y_left[start:goal]
        y_right_cut = self.y_right[start:goal]
        y_left_min =  min(y_left_cut)
        y_right_min = min(y_right_cut)
        return min(y_left_min, y_right_min)-5
    
    def y_max(self, start, goal):
        y_left_cut = self.y_left[start:goal]
        y_right_cut = self.y_right[start:goal]
        y_left_max =  max(y_left_cut)
        y_right_max = max(y_right_cut)
        return max(y_left_max, y_right_max)+5
        
    def read_track_data(self):
        # Read track data from CSV file
        data = pd.read_csv(self.track_path, delimiter=',', encoding='utf-8')
        self.track_name = os.path.basename(self.track_path).split('.')[0]
        print(self.track_name+"asdasd")
        self.x = data['# x_m']  # Get x-coordinate of track centerline
        self.y = data['y_m']  # Get y-coordinate of track centerline
        self.w_tr_right = data['w_tr_right_m']  # Get width of track on the right side
        self.w_tr_left = data['w_tr_left_m']  # Get width of track on the left side
        
    def calculate_boundary_points(self):
        # Calculate track boundary points using gradient and normal vectors
        dx = np.gradient(self.x)
        dy = np.gradient(self.y)
        norm = np.sqrt(dx**2 + dy**2)
        self.nx = dy / norm  # Calculate x-component of the track normal vector
        self.ny = -dx / norm  # Calculate y-component of the track normal vector

        self.x_right = self.x + self.w_tr_right * self.nx  # Calculate x-coordinate of track right boundary
        self.y_right = self.y + self.w_tr_right * self.ny  # Calculate y-coordinate of track right boundary
        self.x_left = self.x - self.w_tr_left * self.nx  # Calculate x-coordinate of track left boundary
        self.y_left = self.y - self.w_tr_left * self.ny  # Calculate y-coordinate of track left boundary
        
    def calculate_obstacle_points(self):
        # Calculate obstacle boundary points by adding a fixed offset to track boundary points
        self.x_right_obs = self.x + (self.w_tr_right - 1) * self.nx
        self.y_right_obs = self.y + (self.w_tr_right - 1) * self.ny
        self.x_left_obs = self.x - (self.w_tr_left - 1) * self.nx
        self.y_left_obs = self.y - (self.w_tr_left - 1) * self.ny
        
    def load_total_obstacles(self):
        self.obstacles = list(zip(self.x_left_obs, self.y_left_obs))
        obstacles_temporary = list(zip(self.x_right_obs, self.y_right_obs))
        self.obstacles.extend(obstacles_temporary)
        
    def plot_track(self):
        # Plot the track and its boundaries
        plt.figure(figsize=(30, 20))
        plt.plot(self.x, self.y, label='Track Centerline', color='blue')
        plt.plot(self.x_right, self.y_right, linestyle='--', label='Track Right Boundary', color='green')
        plt.plot(self.x_left, self.y_left, linestyle='--', label='Track Left Boundary', color='red')
        plt.plot(self.x_right_obs, self.y_right_obs, linestyle='--', label='Obstacle Right Boundary', color='orange')
        plt.plot(self.x_left_obs, self.y_left_obs, linestyle='--', label='Obstacle Left Boundary', color='purple')
        plt.fill_between(self.x_right, self.y_right, self.y_left, color='gray', alpha=0.2)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Race Track')
        plt.legend()
        plt.grid(True)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()
        
    def export_track_boundaries(self):
        # Export track boundaries to a CSV file
        new_data = pd.DataFrame({'x_right': self.x_right, 'y_right': self.y_right, 'x_left': self.x_left, 'y_left': self.y_left})
        new_data.to_csv(f'./Data/track_boundaries/{self.track_name}_track_boundaries.csv', index=False)