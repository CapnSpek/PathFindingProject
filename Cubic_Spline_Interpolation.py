import datetime
import numpy as np
from scipy.interpolate import CubicSpline

class CubicSplineInterpolator:
    def __init__(self, trackname, points):
        """
        Initializes the CubicSplineInterpolator class.
        
        :param trackname: The name of the track.
        :param points: A list of (x, y) tuples representing the path.
        """
        self.track_name = trackname
        self.points = points
        self.path = []

    def smooth_path(self, num_points=5000):
        """
        Smooths a path using cubic spline interpolation.

        :param num_points: Number of points to interpolate for the smoothed path.
        :return: A list of (x, y) tuples representing the smoothed path.
        """
        # Extract x and y coordinates from points
        x = [p[0] for p in self.points]
        y = [p[1] for p in self.points]

        # Create a parameter t based on the length of the path
        t = np.linspace(0, 1, len(self.points))

        # Create cubic spline interpolations
        cs_x = CubicSpline(t, x)
        cs_y = CubicSpline(t, y)

        # Generate new t values for the smoothed path
        t_new = np.linspace(0, 1, num_points)

        # Interpolate x and y values
        x_smooth = cs_x(t_new)
        y_smooth = cs_y(t_new)

        # Combine the smoothed x and y values into a list of tuples
        self.path = list(zip(x_smooth, y_smooth))

        return self.path
    
    def export_path(self):
        # Export path to path.txt
        current_datetime = datetime.datetime.now()
        formatted_datetime = current_datetime.strftime("%Y-%m-%d_%H-%M")
        with open(f'./Data/smoothened_paths/{self.track_name}_{formatted_datetime}_smoothened_path.txt', 'w') as file:
            for point in self.path:
                file.write(str(point) + '\n')
        return self.path
