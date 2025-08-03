import re
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime, timedelta

def extract_theta_data(log_file):
    """
    Extract timestamp, actual theta, and goal theta from robot log file.
    
    Args:
        log_file (str): Path to the log file
        
    Returns:
        tuple: Lists of timestamps, actual thetas, and goal thetas
    """
    timestamps = []
    actual_thetas = []
    goal_thetas = []
    
    # Regular expressions to match pose and goal lines with new format
    pose_pattern = re.compile(r'\[(\d+:\d+:\d+\.\d+)\] \[LOG\] Pose: ([-\d\.]+) ([-\d\.]+) ([-\d\.]+)')
    goal_pattern = re.compile(r'\[(\d+:\d+:\d+\.\d+)\] \[LOG\] Goal: ([-\d\.]+) ([-\d\.]+) ([-\d\.]+)')
    
    pose_data = {}  # Dictionary to store pose data by timestamp
    goal_data = {}  # Dictionary to store goal data by timestamp
    
    with open(log_file, 'r') as file:
        for line in file:
            pose_match = pose_pattern.match(line.strip())
            goal_match = goal_pattern.match(line.strip())
            
            if pose_match:
                time_str = pose_match.group(1)
                time_obj = datetime.strptime(time_str, '%H:%M:%S.%f')
                actual_theta = float(pose_match.group(4))  # Third value is theta
                pose_data[time_str] = (time_obj, actual_theta)
            
            elif goal_match:
                time_str = goal_match.group(1)
                time_obj = datetime.strptime(time_str, '%H:%M:%S.%f')
                goal_theta = float(goal_match.group(4))  # Third value is theta
                goal_data[time_str] = (time_obj, goal_theta)
    
    # Match pose and goal data by timestamp
    for time_str in pose_data:
        if time_str in goal_data:
            time_obj, actual_theta = pose_data[time_str]
            _, goal_theta = goal_data[time_str]
            
            timestamps.append(time_obj)
            actual_thetas.append(actual_theta)
            goal_thetas.append(goal_theta)
    
    # Sort by timestamp to ensure proper ordering
    if timestamps:
        sorted_data = sorted(zip(timestamps, actual_thetas, goal_thetas))
        timestamps, actual_thetas, goal_thetas = zip(*sorted_data)
        timestamps = list(timestamps)
        actual_thetas = list(actual_thetas)
        goal_thetas = list(goal_thetas)
    
    return timestamps, actual_thetas, goal_thetas

def plot_theta_data(timestamps, actual_thetas, goal_thetas):
    """
    Create and save a plot of actual vs goal theta values over time.
    
    Args:
        timestamps (list): List of datetime objects
        actual_thetas (list): List of actual theta values
        goal_thetas (list): List of goal theta values
    """
    plt.figure(figsize=(12, 6))
    
    # Plot actual and goal theta values
    plt.plot(timestamps, actual_thetas, label='Actual Theta', color='blue', marker='o', linestyle='-', markersize=4)
    plt.plot(timestamps, goal_thetas, label='Goal Theta', color='red', marker='x', linestyle='--', markersize=4)
    
    # Add error/difference line
    theta_diff = [actual - goal for actual, goal in zip(actual_thetas, goal_thetas)]
    if theta_diff:
        # Normalize theta differences to be within [-pi, pi]
        for i in range(len(theta_diff)):
            while theta_diff[i] > 3.14159:
                theta_diff[i] -= 2 * 3.14159
            while theta_diff[i] < -3.14159:
                theta_diff[i] += 2 * 3.14159
    plt.plot(timestamps, theta_diff, label='Theta Error (Actual - Goal)', color='green', linestyle=':', alpha=0.7)
    
    # Format the plot
    plt.xlabel('Time')
    plt.ylabel('Theta (radians)')
    plt.title('Robot Theta: Actual vs Goal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Format x-axis to show time properly
    plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
    plt.gca().xaxis.set_major_locator(mdates.SecondLocator(interval=1))
    plt.xticks(rotation=45)
    
    # Add horizontal line at y=0 for reference
    plt.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
    
    # Adjust layout and save
    plt.tight_layout()
    plt.savefig('robot_theta_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()

def calculate_statistics(actual_thetas, goal_thetas):
    """
    Calculate statistics about theta values and differences.
    
    Args:
        actual_thetas (list): List of actual theta values
        goal_thetas (list): List of goal theta values
        
    Returns:
        dict: Dictionary containing calculated statistics
    """
    if not actual_thetas or not goal_thetas:
        return {'error': 'No data available for statistics'}
    
    theta_diff = [abs(actual - goal) for actual, goal in zip(actual_thetas, goal_thetas)]
    theta_diff_signed = [actual - goal for actual, goal in zip(actual_thetas, goal_thetas)]
    
    # Normalize theta values to be within [-pi, pi]
    for i in range(len(theta_diff)):
        while theta_diff[i] > 3.14159:
            theta_diff[i] -= 2 * 3.14159
        while theta_diff[i] < -3.14159:
            theta_diff[i] += 2 * 3.14159
    for i in range(len(theta_diff_signed)):
        while theta_diff_signed[i] > 3.14159:
            theta_diff_signed[i] -= 2 * 3.14159
        while theta_diff_signed[i] < -3.14159:
            theta_diff_signed[i] += 2 * 3.14159
    stats = {
        'min_error': min(theta_diff),
        'max_error': max(theta_diff),
        'avg_error': sum(theta_diff) / len(theta_diff),
        'avg_signed_error': sum(theta_diff_signed) / len(theta_diff_signed),
        'total_measurements': len(actual_thetas),
        'actual_theta_range': (min(actual_thetas), max(actual_thetas)),
        'goal_theta_range': (min(goal_thetas), max(goal_thetas))
    }
    
    return stats

def extract_additional_data(log_file):
    """
    Extract additional useful data from the log file for analysis.
    
    Args:
        log_file (str): Path to the log file
        
    Returns:
        dict: Dictionary containing additional extracted data
    """
    velocities = []
    accelerations = []
    voltages = []
    
    velocity_pattern = re.compile(r'\[(\d+:\d+:\d+\.\d+)\] \[LOG\] Velocity: ([-\d\.]+), Acceleration: ([-\d\.]+)')
    voltage_pattern = re.compile(r'\[(\d+:\d+:\d+\.\d+)\] \[LOG\] Voltages: ([-\d\.]+) ([-\d\.]+)')
    
    with open(log_file, 'r') as file:
        for line in file:
            velocity_match = velocity_pattern.match(line.strip())
            voltage_match = voltage_pattern.match(line.strip())
            
            if velocity_match:
                velocity = float(velocity_match.group(2))
                acceleration = float(velocity_match.group(3))
                velocities.append(velocity)
                accelerations.append(acceleration)
            
            elif voltage_match:
                left_voltage = float(voltage_match.group(2))
                right_voltage = float(voltage_match.group(3))
                voltages.append((left_voltage, right_voltage))
    
    return {
        'velocities': velocities,
        'accelerations': accelerations,
        'voltages': voltages
    }

def main():
    """Main function to run the analysis and visualization."""
    log_file = "../logs/default.txt"
    
    try:
        # Extract theta data
        timestamps, actual_thetas, goal_thetas = extract_theta_data(log_file)
        
        if not timestamps:
            print(f"No valid theta data found in {log_file}.")
            print("Please check that the log file contains 'Pose:' and 'Goal:' entries with the expected format.")
            return
        
        # Calculate statistics
        stats = calculate_statistics(actual_thetas, goal_thetas)
        
        # Print statistics
        print(f"=== Theta Data Analysis Complete ===")
        print(f"Total measurements: {stats['total_measurements']}")
        print(f"Minimum error: {stats['min_error']:.6f} radians")
        print(f"Maximum error: {stats['max_error']:.6f} radians")
        print(f"Average error (absolute): {stats['avg_error']:.6f} radians")
        print(f"Average error (signed): {stats['avg_signed_error']:.6f} radians")
        print(f"Actual theta range: {stats['actual_theta_range'][0]:.6f} to {stats['actual_theta_range'][1]:.6f} radians")
        print(f"Goal theta range: {stats['goal_theta_range'][0]:.6f} to {stats['goal_theta_range'][1]:.6f} radians")
        
        # Create visualization
        plot_theta_data(timestamps, actual_thetas, goal_thetas)
        print("Visualization saved as 'robot_theta_analysis.png'")
        
        # Extract additional data for summary
        additional_data = extract_additional_data(log_file)
        if additional_data['velocities']:
            print(f"\n=== Additional Data Summary ===")
            print(f"Velocity samples: {len(additional_data['velocities'])}")
            print(f"Voltage samples: {len(additional_data['voltages'])}")
        
    except FileNotFoundError:
        print(f"Error: Log file '{log_file}' not found.")
        print("Please ensure the log file exists in the current directory.")
    except Exception as e:
        print(f"Error during analysis: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()