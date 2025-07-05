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

# Calculate movement direction vectors
# We need at least 2 points to compute direction
velocity_x = np.zeros_like(actual_x)
velocity_y = np.zeros_like(actual_y)
movement_theta = np.zeros_like(actual_theta)
is_moving_backwards = np.zeros_like(actual_theta, dtype=bool)

# Calculate velocities and movement direction for frames after the first one
for i in range(1, len(actual_x)):
    # Calculate velocity components
    velocity_x[i] = actual_x[i] - actual_x[i-1]
    velocity_y[i] = actual_y[i] - actual_y[i-1]
    
    # Calculate velocity magnitude
    velocity_magnitude = np.sqrt(velocity_x[i]**2 + velocity_y[i]**2)
    
    # If at rest, use the robot's orientation angle
    if velocity_magnitude < 0.01:
        # If at rest, use the robot's orientation angle
        movement_theta[i] = actual_theta[i]
        is_moving_backwards[i] = False
    else:
        # Calculate movement direction angle
        if velocity_x[i] != 0 or velocity_y[i] != 0:  # Avoid division by zero
            movement_theta[i] = np.arctan2(velocity_y[i], velocity_x[i])
            
            # Determine if moving backwards by comparing movement direction with robot orientation
            # Normalize the difference between angles to [-pi, pi]
            angle_diff = ((movement_theta[i] - actual_theta[i] + np.pi) % (2 * np.pi)) - np.pi
            is_moving_backwards[i] = abs(angle_diff) > np.pi/2

# For the first frame, use the robot's orientation since we can't determine velocity
if len(actual_x) > 0:
    # For first frame, default to orientation
    movement_theta[0] = actual_theta[0]
    is_moving_backwards[0] = False
    
    # Only copy from second frame if we have more than one frame
    if len(actual_x) > 1:
        velocity_x[0] = velocity_x[1]
        velocity_y[0] = velocity_y[1]

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

# Robot orientation and movement arrows
actual_orientation_arrow = None
goal_arrow = None
movement_arrow = None

def init():
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.grid(True)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('Robot Path Animation')
    ax.legend(loc='upper left')
    return actual_line, goal_line, actual_point, goal_point

def animate(i):
    global actual_orientation_arrow, goal_arrow, movement_arrow
    
    # Update path lines
    actual_line.set_data(actual_x[:i+1], actual_y[:i+1])
    goal_line.set_data(goal_x[:i+1], goal_y[:i+1])
    
    # Update current position points
    actual_point.set_data([actual_x[i]], [actual_y[i]])
    goal_point.set_data([goal_x[i]], [goal_y[i]])
    
    # Remove previous arrows
    if actual_orientation_arrow:
        actual_orientation_arrow.remove()
    if goal_arrow:
        goal_arrow.remove()
    if movement_arrow:
        movement_arrow.remove()
    
    # Add new orientation arrows
    arrow_length = 2.0
    
    # Robot orientation arrow (blue)
    actual_orientation_arrow = ax.arrow(actual_x[i], actual_y[i],
                          arrow_length * np.cos(actual_theta[i]),
                          arrow_length * np.sin(actual_theta[i]),
                          head_width=0.5, head_length=0.8, fc='blue', ec='blue')
    
    # Goal orientation arrow (red)
    goal_arrow = ax.arrow(goal_x[i], goal_y[i],
                        arrow_length * np.cos(goal_theta[i]),
                        arrow_length * np.sin(goal_theta[i]),
                        head_width=0.5, head_length=0.8, fc='red', ec='red')
    
    # Movement direction arrow (green)
    movement_direction = movement_theta[i]
    
    # If moving backwards, flip the arrow direction by 180 degrees
    if is_moving_backwards[i]:
        movement_direction = (movement_direction + np.pi) % (2 * np.pi)
    
    movement_arrow = ax.arrow(actual_x[i], actual_y[i],
                           arrow_length * np.cos(movement_direction),
                           arrow_length * np.sin(movement_direction),
                           head_width=0.5, head_length=0.8, fc='green', ec='green')
    
    return actual_line, goal_line, actual_point, goal_point, actual_orientation_arrow, goal_arrow, movement_arrow

# Create animation
anim = FuncAnimation(fig, animate, init_func=init, frames=len(actual_x),
                    interval=10, blit=True, repeat=False)

plt.show()