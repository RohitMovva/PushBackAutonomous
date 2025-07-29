import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Arrow
import re
from datetime import datetime

# Lists to store the data
goal_x = []
goal_y = []
goal_theta = []
left_wheel_vel = []
right_wheel_vel = []
timestamps = []

# Time format pattern
time_pattern = re.compile(r'\[(\d{2}:\d{2}:\d{2}\.\d{3})\]')

# Function to convert timestamp to seconds
def timestamp_to_seconds(timestamp):
    dt = datetime.strptime(timestamp, '%H:%M:%S.%f')
    return dt.hour * 3600 + dt.minute * 60 + dt.second + dt.microsecond/1000000

# Read and parse the log file
with open('../logs/square.txt', 'r') as file:
    for line in file:
        # Extract timestamp
        time_match = time_pattern.search(line)
        if time_match:
            timestamp = time_match.group(1)
            seconds = timestamp_to_seconds(timestamp)
            
            if 'Goal:' in line:
                # Extract goal pose data
                parts = line.split('Goal:')[1].strip().split()
                if len(parts) >= 3:
                    goal_x.append(float(parts[0]))
                    goal_y.append(float(parts[1]))
                    goal_theta.append(float(parts[2]))
                    timestamps.append(seconds)
            
            elif 'Left wheel velocities:' in line:
                # Extract left wheel velocity (the last value)
                parts = line.split('Left wheel velocities:')[1].strip().split()
                if len(parts) >= 1:
                    left_wheel_vel.append(float(parts[-1]))
            
            elif 'Right wheel velocities:' in line:
                # Extract right wheel velocity (the last value)
                parts = line.split('Right wheel velocities:')[1].strip().split()
                if len(parts) >= 1:
                    right_wheel_vel.append(float(parts[-1]))

# Convert lists to numpy arrays
goal_x = np.array(goal_x)
goal_y = np.array(goal_y)
goal_theta = np.array(goal_theta)
timestamps = np.array(timestamps)

# Convert wheel velocities to numpy arrays
left_wheel_vel = np.array(left_wheel_vel)
right_wheel_vel = np.array(right_wheel_vel)

# Ensure we have equal number of goal poses and wheel velocities
min_length = min(len(goal_x), len(left_wheel_vel), len(right_wheel_vel))
goal_x = goal_x[:min_length]
goal_y = goal_y[:min_length]
goal_theta = goal_theta[:min_length]
left_wheel_vel = left_wheel_vel[:min_length]
right_wheel_vel = right_wheel_vel[:min_length]
timestamps = timestamps[:min_length]

# Robot parameters (adjust these based on your robot)
wheel_radius = 2.75/2  # meters
wheel_separation = 12.7  # meters

# Calculate linear and angular velocities from wheel velocities
linear_vel = (right_wheel_vel + left_wheel_vel) * wheel_radius / 2
angular_vel = (right_wheel_vel - left_wheel_vel) * wheel_radius / wheel_separation

# Calculate time deltas for integration
time_deltas = np.zeros_like(timestamps)
time_deltas[1:] = timestamps[1:] - timestamps[:-1]

# Simulate robot movement based on velocities
simulated_x = np.zeros_like(goal_x)
simulated_y = np.zeros_like(goal_y)
simulated_theta = np.zeros_like(goal_theta)

# Initialize with the first goal position
simulated_x[0] = goal_x[0]
simulated_y[0] = goal_y[0]
simulated_theta[0] = goal_theta[0]

# Integrate velocities to get simulated positions
for i in range(1, len(timestamps)):
    dt = time_deltas[i]
    
    # Update position based on linear velocity and current heading
    simulated_x[i] = simulated_x[i-1] + linear_vel[i-1] * np.cos(simulated_theta[i-1]) * dt
    simulated_y[i] = simulated_y[i-1] + linear_vel[i-1] * np.sin(simulated_theta[i-1]) * dt
    
    # Update orientation based on angular velocity
    simulated_theta[i] = simulated_theta[i-1] + angular_vel[i-1] * dt

# Create the animation
fig, ax = plt.subplots(figsize=(12, 10))
ax.set_aspect('equal')

# Set the plot limits with some padding
all_x = np.concatenate((goal_x, simulated_x))
all_y = np.concatenate((goal_y, simulated_y))
x_min = min(all_x) - 5
x_max = max(all_x) + 5
y_min = min(all_y) - 5
y_max = max(all_y) + 5

# Initialize plot elements
goal_line, = ax.plot([], [], 'r--', label='Goal Path', linewidth=2)
goal_point, = ax.plot([], [], 'ro', markersize=8)
sim_line, = ax.plot([], [], 'g-', label='Wheel-Based Path', linewidth=2)
sim_point, = ax.plot([], [], 'go', markersize=8)

# Robot orientation arrows
goal_arrow = None
sim_arrow = None

# Time text display
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
error_text = ax.text(0.02, 0.90, '', transform=ax.transAxes)
wheel_text = ax.text(0.02, 0.85, '', transform=ax.transAxes)

def init():
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.grid(True)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('Robot Goal Path vs. Wheel-Velocity-Integrated Path')
    ax.legend()
    goal_line.set_data([], [])
    goal_point.set_data([], [])
    sim_line.set_data([], [])
    sim_point.set_data([], [])
    time_text.set_text('')
    error_text.set_text('')
    wheel_text.set_text('')
    return goal_line, goal_point, sim_line, sim_point, time_text, error_text, wheel_text

def animate(i):
    global goal_arrow, sim_arrow
    
    # Update path lines
    goal_line.set_data(goal_x[:i+1], goal_y[:i+1])
    sim_line.set_data(simulated_x[:i+1], simulated_y[:i+1])
    
    # Update current position points
    goal_point.set_data([goal_x[i]], [goal_y[i]])
    sim_point.set_data([simulated_x[i]], [simulated_y[i]])
    
    # Remove previous arrows
    if goal_arrow:
        goal_arrow.remove()
    if sim_arrow:
        sim_arrow.remove()
    
    # Add new orientation arrows
    arrow_length = 2.0
    goal_arrow = ax.arrow(goal_x[i], goal_y[i],
                          arrow_length * np.cos(goal_theta[i]),
                          arrow_length * np.sin(goal_theta[i]),
                          head_width=0.5, head_length=0.8, fc='red', ec='red')
    
    sim_arrow = ax.arrow(simulated_x[i], simulated_y[i],
                         arrow_length * np.cos(simulated_theta[i]),
                         arrow_length * np.sin(simulated_theta[i]),
                         head_width=0.5, head_length=0.8, fc='green', ec='green')
    
    # Update text information
    time_text.set_text(f'Time: {timestamps[i]:.2f} s')
    
    # Calculate and display position error
    pos_error = np.sqrt((goal_x[i] - simulated_x[i])**2 + (goal_y[i] - simulated_y[i])**2)
    angle_error = np.abs(np.arctan2(np.sin(goal_theta[i] - simulated_theta[i]), 
                                    np.cos(goal_theta[i] - simulated_theta[i])))
    error_text.set_text(f'Position Error: {pos_error:.2f} m, Angle Error: {angle_error:.2f} rad')
    
    # Display wheel velocities
    wheel_text.set_text(f'Left: {left_wheel_vel[i]:.2f} m/s, Right: {right_wheel_vel[i]:.2f} m/s')
    
    return goal_line, goal_point, sim_line, sim_point, time_text, error_text, wheel_text, goal_arrow, sim_arrow

# Create animation
anim = FuncAnimation(fig, animate, init_func=init, frames=len(goal_x),
                    interval=25, blit=True, repeat=True)

plt.tight_layout()
plt.show()