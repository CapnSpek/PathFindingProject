import numpy as np
import matplotlib.pyplot as plt
import datetime

class RRTStar:
    # Class to implement the RRT* algorithm

    last_added_point = None

    def __init__(self, track_name, x_left, y_left, x_right, y_right, start, goal, obstacles, serial=1, max_iter=7000, goal_radius=12, step_size=10, search_radius=50, 
                 x_low=-600, x_high=600, y_low=-200, y_high=1100):
        # Initialize the RRT* algorithm with the given parameters
        # x_left, y_left: X and Y coordinates of the left boundary of the track
        # x_right, y_right: X and Y coordinates of the right boundary of the track
        # start: Starting point of the path
        # goal: Goal point of the path
        # obstacles: List of obstacles in the environment
        # serial: Serial number of the path
        # max_iter: Maximum number of iterations for the algorithm
        # goal_radius: Radius around the goal point to consider it reached
        # step_size: Distance to extend the tree in each iteration
        # search_radius: Radius to search for neighbors to rewire
        # x_low, x_high: Bounds for the x-coordinate of random points
        # y_low, y_high: Bounds for the y-coordinate of random points
        # path = []: List to store the path from start to goal
        
        self.track_name = track_name
        self.x_left = x_left
        self.y_left = y_left
        self.x_right = x_right
        self.y_right = y_right
        self.start = start
        self.goal = goal
        self.goal_radius = goal_radius
        self.goal_discovered = goal
        self.obstacles = obstacles
        self.serial = serial
        self.max_iter = max_iter
        self.step_size = step_size
        self.search_radius = search_radius
        self.x_low = x_low
        self.x_high = x_high
        self.y_low = y_low
        self.y_high = y_high
        self.tree = {tuple(start): None}
        self.reached = False
        self.reached_again = False
        self.path = []

    def generate_random_point(self):
        # Generate a random point within the specified bounds
        random_point_x = np.random.uniform(low=self.x_low, high=self.x_high, size=1)[0]
        random_point_y = np.random.uniform(low=self.y_low, high=self.y_high, size=1)[0]
        random_point = (random_point_x, random_point_y)
        return tuple(random_point)

    def find_nearest_point(self, point):
        # Find the nearest point in the tree to the given point
        distances = [np.linalg.norm(np.array(point) - np.array(p)) for p in self.tree.keys()]
        nearest_point = list(self.tree.keys())[np.argmin(distances)]
        return nearest_point

    def is_collision_free(self, point1, point2):
        # Check if the line segment between point1 and point2 is collision-free
        if self.is_intersecting(self.obstacles, point1, point2):
            return False
        return True

    def is_intersecting(self, obstacles, point1, point2):
        # Check if the line segment between point1 and point2 intersects with any of the obstacles
        for i in range(len(obstacles)):
            p1 = obstacles[i]
            p2 = obstacles[(i + 1) % len(obstacles)]
            if self.do_segments_intersect(p1, p2, point1, point2):
                return True
        return False

    def do_segments_intersect(self, p1, p2, p3, p4):
        # Check if line segment p1-p2 intersects with line segment p3-p4
        # Here's an example using the cross product method:
        d1 = self.cross_product(p3, p4, p1)
        d2 = self.cross_product(p3, p4, p2)
        d3 = self.cross_product(p1, p2, p3)
        d4 = self.cross_product(p1, p2, p4)
        if d1 * d2 < 0 and d3 * d4 < 0:
            return True
        if d1 == 0 and self.is_point_on_segment(p3, p4, p1):
            return True
        if d2 == 0 and self.is_point_on_segment(p3, p4, p2):
            return True
        if d3 == 0 and self.is_point_on_segment(p1, p2, p3):
            return True
        if d4 == 0 and self.is_point_on_segment(p1, p2, p4):
            return True
        return False

    def cross_product(self, p1, p2, p3):
        # Calculate the cross product of vectors p1p2 and p1p3
        return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])

    def is_point_on_segment(self, p1, p2, p):
        # Check if point p lies on the line segment p1-p2
        return min(p1[0], p2[0]) <= p[0] <= max(p1[0], p2[0]) and min(p1[1], p2[1]) <= p[1] <= max(p1[1], p2[1])

    def extend_tree(self, point):
        # Extend the tree towards the given point
        nearest_point = self.find_nearest_point(point)
        direction = np.array(point) - np.array(nearest_point)
        direction = direction/np.linalg.norm(direction)

        # Calculate the potential new point by stepping towards the given point
        new_point = np.array(nearest_point) + self.step_size * direction
        new_point = tuple(new_point)

        if self.is_collision_free(nearest_point, new_point):
            self.tree[new_point] = nearest_point
            self.last_added_point = new_point
            self.rewire_neighbors(new_point)
            # Check if the new point is within goal-radius distance of the goal
            distance_to_goal = np.linalg.norm(np.array(new_point) - np.array(self.goal))
            if distance_to_goal <= self.goal_radius:
                self.goal_discovered = new_point          
                print("EXISTS")
                self.reached = True
                self.reached_again = True
            return True
        return False

    def rewire_neighbors(self, point):
        # Rewire the neighbors of the given point if a shorter path is found
        for neighbor in self.tree.keys():
            if neighbor == point:
                continue
            if np.linalg.norm(np.array(neighbor) - np.array(point)) > self.search_radius:
                continue
            if self.is_collision_free(neighbor, point):
                cost = self.get_cost(neighbor) + np.linalg.norm(np.array(neighbor) - np.array(point))
                if cost < self.get_cost(point):
                    self.tree[point] = neighbor

    def get_cost(self, point):
        # Calculate the cost of reaching the given point from the start point
        cost = 0
        current_point = point
        while current_point != self.start:
            parent = self.tree[current_point]
            cost += np.linalg.norm(np.array(current_point) - np.array(parent))
            current_point = parent
        return cost

    def build_rrt_star(self):
        # Build the RRT* tree
        for i in range(self.max_iter):
            random_point = self.generate_random_point()
            self.extend_tree(random_point)
            if self.reached and self.reached_again:
              print("Total iterations: " +  str(i))
              self.reached_again = False

    def plot_rrt_star(self):
        # Plot the RRT* tree and the path from start to goal
        '''
        plt.figure(figsize=(10, 8))
        for point, parent in self.tree.items():
            if parent is not None:
                plt.plot([point[0], parent[0]], [point[1], parent[1]], 'k-', linewidth=0.5)
        plt.plot(self.x_left, self.y_left, label='Track Left Boundary', linestyle='--', color='red')
        plt.plot(self.x_right, self.y_right, label='Track Right Boundary', linestyle='--', color='green')
        plt.plot(*zip(*self.obstacles), 'r-', label='Obstacles', linewidth=2)
        plt.plot(self.start[0], self.start[1], 'go', markersize=5, label='Start')
        plt.plot(self.goal[0], self.goal[1], 'bo', markersize=5, label='Goal')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('RRT* Path Planning')
        plt.grid(True)
        '''
        # Plot the path from start to goal
        if self.reached:
           self.get_path()
           # x_path, y_path = zip(*(self.path))
           # plt.plot(x_path, y_path, 'g-', label='Segment path', linewidth=2)        
        #plt.legend()
        #plt.show()

    def get_path(self):
        # Get the path from start to goal
        current_point = self.goal_discovered
        while current_point != self.start:
            self.path.append(current_point)
            current_point = self.tree[current_point]
        self.path.append(self.start)
        self.path.reverse()
        
    def export_path(self):
        # Export path to path.txt
        current_datetime = datetime.datetime.now()
        formatted_datetime = current_datetime.strftime("%Y-%m-%d_%H-%M")
        with open(f'./Data/paths_found/{self.track_name}_{self.serial}_{formatted_datetime}_path.txt', 'w') as file:
            for point in self.path:
                file.write(str(point) + '\n')
        return self.path