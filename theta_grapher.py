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
    
    # Regular expressions to match pose and goal lines
    pose_pattern = re.compile(r'\[(\d+:\d+:\d+\.\d+)\] Pose: .+ .+ ([-\d\.]+)')
    goal_pattern = re.compile(r'\[(\d+:\d+:\d+\.\d+)\] Goal: .+ .+ ([-\d\.]+)')
    
    with open(log_file, 'r') as file:
        lines = file.readlines()
        
        # Process lines in pairs (assuming pose and goal entries come in pairs)
        i = 0
        while i < len(lines) - 1:
            pose_match = pose_pattern.match(lines[i])
            goal_match = goal_pattern.match(lines[i+1])
            
            if pose_match and goal_match:
                # Extract timestamp from pose line
                time_str = pose_match.group(1)
                time_obj = datetime.strptime(time_str, '%H:%M:%S.%f')
                
                # Extract theta values
                actual_theta = float(pose_match.group(2))
                goal_theta = float(goal_match.group(2))
                
                timestamps.append(time_obj)
                actual_thetas.append(actual_theta)
                goal_thetas.append(goal_theta)
                
                i += 2  # Move to the next pair
            else:
                i += 1  # Move to the next line if pattern doesn't match
    
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
    plt.plot(timestamps, theta_diff, label='Theta Error (Actual - Goal)', color='green', linestyle=':', alpha=0.7)
    
    # Format the plot
    plt.xlabel('Time')
    plt.ylabel('Theta (radians)')
    plt.title('Robot Theta: Actual vs Goal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Format x-axis to show time properly
    plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
    
    # Add horizontal line at y=0 for reference
    plt.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
    
    # Adjust layout and save
    plt.tight_layout()
    plt.savefig('robot_theta_analysis.png')
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
    theta_diff = [abs(actual - goal) for actual, goal in zip(actual_thetas, goal_thetas)]
    
    stats = {
        'min_error': min(theta_diff),
        'max_error': max(theta_diff),
        'avg_error': sum(theta_diff) / len(theta_diff) if theta_diff else 0,
        'total_measurements': len(actual_thetas)
    }
    
    return stats

def main():
    """Main function to run the analysis and visualization."""
    log_file = "robot_log_19700101_000000.txt"
    
    try:
        # Extract data
        timestamps, actual_thetas, goal_thetas = extract_theta_data(log_file)
        
        if not timestamps:
            print(f"No valid data found in {log_file}.")
            return
        
        # Calculate statistics
        stats = calculate_statistics(actual_thetas, goal_thetas)
        
        # Print statistics
        print(f"Data Analysis Complete - {stats['total_measurements']} measurements processed")
        print(f"Minimum Error: {stats['min_error']:.6f} radians")
        print(f"Maximum Error: {stats['max_error']:.6f} radians")
        print(f"Average Error: {stats['avg_error']:.6f} radians")
        
        # Create visualization
        plot_theta_data(timestamps, actual_thetas, goal_thetas)
        print("Visualization saved as 'robot_theta_analysis.png'")
        
    except FileNotFoundError:
        print(f"Error: Log file '{log_file}' not found.")
    except Exception as e:
        print(f"Error during analysis: {e}")

if __name__ == "__main__":
    main()