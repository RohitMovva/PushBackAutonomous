import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Arrow

# Lists to store the data
actual_x = []
actual_y = []
actual_theta = []
goal_x = []
goal_y = []
goal_theta = []
timestamps = []

# Read and parse the log file
with open('robot_log_19700101_000000.txt', 'r') as file:
    t = 0
    for line in file:
        if 'Pose:' in line:
            t += 0.025  # Assuming 25ms between each line
            # Extract pose data (actual position)
            parts = line.split()
            actual_x.append(float(parts[2]))
            actual_y.append(float(parts[3]))
            actual_theta.append(float(parts[4]))
            # Extract timestamp
            time_str = parts[0][1:-1]  # Remove brackets
            timestamps.append(t)
        elif 'Goal:' in line:
            # Extract goal data
            parts = line.split()
            goal_x.append(float(parts[2]))
            goal_y.append(float(parts[3]))
            goal_theta.append(float(parts[4]))

# Convert lists to numpy arrays
actual_x = np.array(actual_x)
actual_y = np.array(actual_y)
actual_theta = np.array(actual_theta)
goal_x = np.array(goal_x)
goal_y = np.array(goal_y)
goal_theta = np.array(goal_theta)

# Create the animation
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_aspect('equal')

# Set the plot limits with some padding
x_min = min(min(actual_x), min(goal_x)) - 5
x_max = max(max(actual_x), max(goal_x)) + 5
y_min = min(min(actual_y), min(goal_y)) - 5
y_max = max(max(actual_y), max(goal_y)) + 5

# Initialize plot elements
actual_line, = ax.plot([], [], 'b-', label='Actual Path', linewidth=2)
goal_line, = ax.plot([], [], 'r--', label='Goal Path', linewidth=2)
actual_point, = ax.plot([], [], 'bo', markersize=10)
goal_point, = ax.plot([], [], 'ro', markersize=10)

# Robot orientation arrows
actual_arrow = None
goal_arrow = None

def init():
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.grid(True)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('Robot Path Animation')
    ax.legend()
    return actual_line, goal_line, actual_point, goal_point

def animate(i):
    global actual_arrow, goal_arrow
    
    # Update path lines
    actual_line.set_data(actual_x[:i+1], actual_y[:i+1])
    goal_line.set_data(goal_x[:i+1], goal_y[:i+1])
    
    # Update current position points
    actual_point.set_data([actual_x[i]], [actual_y[i]])
    goal_point.set_data([goal_x[i]], [goal_y[i]])
    
    # Remove previous arrows
    if actual_arrow:
        actual_arrow.remove()
    if goal_arrow:
        goal_arrow.remove()
    
    # Add new orientation arrows
    arrow_length = 2.0
    actual_arrow = ax.arrow(actual_x[i], actual_y[i],
                          arrow_length * np.cos(actual_theta[i]),
                          arrow_length * np.sin(actual_theta[i]),
                          head_width=0.5, head_length=0.8, fc='blue', ec='blue')
    goal_arrow = ax.arrow(goal_x[i], goal_y[i],
                        arrow_length * np.cos(goal_theta[i]),
                        arrow_length * np.sin(goal_theta[i]),
                        head_width=0.5, head_length=0.8, fc='red', ec='red')
    
    return actual_line, goal_line, actual_point, goal_point, actual_arrow, goal_arrow

# Create animation
anim = FuncAnimation(fig, animate, init_func=init, frames=len(actual_x),
                    interval=25, blit=True, repeat=True)

plt.show()