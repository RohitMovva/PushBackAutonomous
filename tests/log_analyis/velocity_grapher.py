import re
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import os

def parse_log_file(file_path):
    """
    Parse the robot log file to extract timestamps and velocity data.
    
    Args:
        file_path (str): Path to the log file
        
    Returns:
        tuple: Four lists containing mp_times, mp_data, ramsete_times, ramsete_data
    """
    # Initialize data structures
    mp_times = []
    mp_data = []
    ramsete_times = []
    ramsete_data = []
    
    # Compile regex patterns for faster matching
    mp_pattern = re.compile(r'\[(\d{2}:\d{2}:\d{2}\.\d{3})\] MP Output: ([-\d\.]+) ([-\d\.]+)')
    ramsete_pattern = re.compile(r'\[(\d{2}:\d{2}:\d{2}\.\d{3})\] Ramsete Output: ([-\d\.]+) ([-\d\.]+)')
    
    # Parse the log file
    with open(file_path, 'r') as file:
        for line in file:
            # Check for MP Output lines
            mp_match = mp_pattern.search(line)
            if mp_match:
                time_str = mp_match.group(1)
                linear_vel = float(mp_match.group(2))
                angular_vel = float(mp_match.group(3))
                
                # Convert time string to seconds
                time_obj = datetime.strptime(time_str, '%H:%M:%S.%f')
                seconds = time_obj.hour * 3600 + time_obj.minute * 60 + time_obj.second + time_obj.microsecond / 1000000
                
                mp_times.append(seconds)
                mp_data.append((linear_vel, angular_vel))
                continue
            
            # Check for Ramsete Output lines
            ramsete_match = ramsete_pattern.search(line)
            if ramsete_match:
                time_str = ramsete_match.group(1)
                linear_vel = float(ramsete_match.group(2))
                angular_vel = float(ramsete_match.group(3))
                
                # Convert time string to seconds
                time_obj = datetime.strptime(time_str, '%H:%M:%S.%f')
                seconds = time_obj.hour * 3600 + time_obj.minute * 60 + time_obj.second + time_obj.microsecond / 1000000
                
                ramsete_times.append(seconds)
                ramsete_data.append((linear_vel, angular_vel))
    
    return mp_times, mp_data, ramsete_times, ramsete_data

def create_velocity_plots(mp_times, mp_data, ramsete_times, ramsete_data, output_path="velocity_plots.png"):
    """
    Create and save plots comparing motion profile and ramsete velocities.
    
    Args:
        mp_times (list): List of timestamps for motion profile data
        mp_data (list): List of (linear_vel, angular_vel) tuples for motion profile
        ramsete_times (list): List of timestamps for ramsete data
        ramsete_data (list): List of (linear_vel, angular_vel) tuples for ramsete
        output_path (str): Path to save the output plot
    """
    # Extract linear and angular velocities
    mp_linear = [data[0] for data in mp_data]
    mp_angular = [data[1] for data in mp_data]
    ramsete_linear = [data[0] for data in ramsete_data]
    ramsete_angular = [data[1] for data in ramsete_data]
    
    # Normalize time to start at 0
    if mp_times and ramsete_times:
        # min_time = min(min(mp_times), min(ramsete_times))
        mp_times = [t for t in mp_times]
        ramsete_times = [t for t in ramsete_times]
    
    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Plot linear velocities
    ax1.plot(mp_times, mp_linear, 'b-', label='Motion Profile', linewidth=2)
    ax1.plot(ramsete_times, ramsete_linear, 'r--', label='Ramsete Controller', linewidth=2)
    ax1.set_title('Linear Velocity Comparison', fontsize=16)
    ax1.set_xlabel('Time (seconds)', fontsize=12)
    ax1.set_ylabel('Linear Velocity (m/s)', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.legend(fontsize=12)
    
    # Plot angular velocities
    ax2.plot(mp_times, mp_angular, 'b-', label='Motion Profile', linewidth=2)
    ax2.plot(ramsete_times, ramsete_angular, 'r--', label='Ramsete Controller', linewidth=2)
    ax2.set_title('Angular Velocity Comparison', fontsize=16)
    ax2.set_xlabel('Time (seconds)', fontsize=12)
    ax2.set_ylabel('Angular Velocity (rad/s)', fontsize=12)
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.legend(fontsize=12)
    
    # Adjust layout and save
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    print(f"Plot saved to {output_path}")
    
    # Show the plot
    plt.show()

def main():
    """
    Main function to run the script.
    """
    # Log file path (in the same directory as this script)
    log_file = "robot_log_19700101_000000.txt"
    output_path = "velocity_plots.png"
    
    # Check if the log file exists
    if not os.path.exists(log_file):
        print(f"Error: Log file '{log_file}' not found in the current directory.")
        return
    
    print(f"Parsing log file: {log_file}")
    mp_times, mp_data, ramsete_times, ramsete_data = parse_log_file(log_file)
    
    print(f"Found {len(mp_data)} motion profile data points and {len(ramsete_data)} ramsete data points")
    
    if not mp_data or not ramsete_data:
        print("Error: No data found in the log file. Please check the file format.")
        return
    
    create_velocity_plots(mp_times, mp_data, ramsete_times, ramsete_data, output_path)

if __name__ == "__main__":
    main()