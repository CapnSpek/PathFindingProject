import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Function to calculate the Euclidean distance between two points
def euclidean_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Calculate total distance for a raceline
def total_distance(raceline):
    return sum(euclidean_distance(raceline[i], raceline[i+1]) for i in range(len(raceline) - 1))

# Calculate cumulative distance along the path
def cumulative_distance(raceline):
    distances = [0]
    for i in range(1, len(raceline)):
        distances.append(distances[-1] + euclidean_distance(raceline[i-1], raceline[i]))
    return np.array(distances)

# Read raceline1 from file
raceline1 = pd.read_csv('./Data/racetrack-database/racelines/Shanghai.csv')
# Read raceline2 from file
raceline2 = []
with open('./Data/smoothened_paths/Shanghai_2024-05-28_08-57_smoothened_path.txt', 'r') as file:
    for line in file:
        line = line.strip().strip('()')
        x, y = line.split(',')
        raceline2.append((float(x), float(y)))

# Convert raceline1 to a list of tuples
raceline1 = [(row['# x_m'], row['y_m']) for _, row in raceline1.iterrows()]

# Ensure raceline1 and raceline2 have the same length
if len(raceline1) != len(raceline2):
    min_len = min(len(raceline1), len(raceline2))
    raceline1 = raceline1[:min_len]
    raceline2 = raceline2[:min_len]

# Calculate total distances
total_distance_raceline1 = total_distance(raceline1)
total_distance_raceline2 = total_distance(raceline2)

# Prepare data for cumulative distance calculation
raceline1_x, raceline1_y = zip(*raceline1)
raceline2_x, raceline2_y = zip(*raceline2)

# Calculate cumulative distances
distance_raceline1 = cumulative_distance(raceline1)
distance_raceline2 = cumulative_distance(raceline2)

# Create a DataFrame to display results
df = pd.DataFrame({
    'Point': list(range(len(raceline1))),
    'Raceline 1': raceline1,
    'Raceline 2': raceline2,
    'Distance Raceline 1': distance_raceline1,
    'Distance Raceline 2': distance_raceline2
})

# Display the results
print(df)
print(f"\nTotal distance for Raceline 1: {total_distance_raceline1} units")
print(f"Total distance for Raceline 2: {total_distance_raceline2} units")

# Plot the racelines for visual comparison
plt.figure(figsize=(14, 7))

plt.plot(raceline1_x, raceline1_y, 'g-', linewidth=2, label='Raceline 1')
plt.plot(raceline2_x, raceline2_y, 'r-', linewidth=2, label='Raceline 2')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Comparison of Racelines')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()