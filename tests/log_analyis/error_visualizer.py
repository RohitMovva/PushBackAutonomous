import matplotlib.pyplot as plt
import numpy as np
import re
from datetime import datetime

def parse_timestamp(timestamp_str):
    """Convert timestamp string to seconds since start"""
    # Extract time components from [HH:MM:SS.mmm] format
    time_match = re.match(r'\[(\d{2}):(\d{2}):(\d{2})\.(\d{3})\]', timestamp_str)
    if time_match:
        hours, minutes, seconds, milliseconds = map(int, time_match.groups())
        total_seconds = hours * 3600 + minutes * 60 + seconds + milliseconds / 1000.0
        return total_seconds
    return None

def normalize_angle(angle):
    """Normalize angle to [-180, 180] degree range"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def calculate_angular_error(goal_angle, pose_angle):
    """Calculate angular error with proper wrapping, converting to degrees"""
    error = np.degrees(goal_angle - pose_angle)
    return normalize_angle(error)

def calculate_positional_error(goal_x, goal_y, pose_x, pose_y):
    """Calculate Euclidean distance between goal and pose"""
    return np.sqrt((goal_x - pose_x)**2 + (goal_y - pose_y)**2)

def parse_log_file(filename):
    """Parse the log file and extract relevant data"""
    timestamps = []
    angular_errors = []
    positional_errors = []
    
    current_timestamp = None
    current_pose = None
    current_goal = None
    start_time = None
    
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            
            # Extract timestamp
            timestamp_match = re.match(r'(\[\d{2}:\d{2}:\d{2}\.\d{3}\])', line)
            if timestamp_match:
                current_timestamp = parse_timestamp(timestamp_match.group(1))
                if start_time is None:
                    start_time = current_timestamp
            
            # Extract Pose data
            if '[LOG] Pose:' in line:
                pose_match = re.search(r'Pose: ([-\d.]+) ([-\d.]+) ([-\d.]+)', line)
                if pose_match:
                    current_pose = [float(x) for x in pose_match.groups()]
            
            # Extract Goal data
            elif '[LOG] Goal:' in line:
                goal_match = re.search(r'Goal: ([-\d.]+) ([-\d.]+) ([-\d.]+)', line)
                if goal_match:
                    current_goal = [float(x) for x in goal_match.groups()]
            
            # When we have both pose and goal data, calculate errors
            if current_pose is not None and current_goal is not None and current_timestamp is not None:
                # Calculate errors
                pos_error = calculate_positional_error(
                    current_goal[0], current_goal[1], 
                    current_pose[0], current_pose[1]
                )
                ang_error = calculate_angular_error(current_goal[2], current_pose[2])
                
                # Store data (convert timestamp to seconds since start)
                timestamps.append(current_timestamp - start_time)
                positional_errors.append(pos_error)
                angular_errors.append(ang_error)
                
                # Reset for next iteration
                current_pose = None
                current_goal = None
    
    return timestamps, angular_errors, positional_errors

def plot_errors(filename):
    """Main function to parse data and create plots"""
    timestamps, angular_errors, positional_errors = parse_log_file(filename)
    
    if not timestamps:
        print("No data found in the log file!")
        return
    
    # Create subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # Plot angular error
    ax1.plot(timestamps, angular_errors, 'b-', linewidth=1.5, label='Angular Error')
    ax1.set_ylabel('Angular Error (degrees)')
    ax1.set_title('Robot Tracking Errors Over Time')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Plot positional error
    ax2.plot(timestamps, positional_errors, 'r-', linewidth=1.5, label='Positional Error')
    ax2.set_ylabel('Positional Error (inches)')
    ax2.set_xlabel('Time (seconds)')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Adjust layout and show
    plt.tight_layout()
    plt.show()
    
    # Print some statistics
    print(f"Data points: {len(timestamps)}")
    print(f"Time range: {timestamps[0]:.3f} to {timestamps[-1]:.3f} seconds")
    print(f"Average positional error: {np.mean(positional_errors):.4f}")
    print(f"Max positional error: {np.max(positional_errors):.4f}")
    print(f"Average angular error: {np.mean(np.abs(angular_errors)):.4f} degrees")
    print(f"Max angular error: {np.max(np.abs(angular_errors)):.4f} degrees")

if __name__ == "__main__":
    plot_errors('../logs/default.log')