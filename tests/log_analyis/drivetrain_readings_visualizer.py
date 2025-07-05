import re
import matplotlib.pyplot as plt
from datetime import datetime

def parse_time(time_str):
    """Convert time string '[HH:MM:SS.mmm]' to seconds."""
    # Extract the time part without brackets
    time_part = time_str.strip('[]')
    # Parse the time
    time_obj = datetime.strptime(time_part, '%H:%M:%S.%f')
    # Convert to total seconds
    total_seconds = (time_obj.hour * 3600 + 
                     time_obj.minute * 60 + 
                     time_obj.second +
                     time_obj.microsecond / 1000000)
    return total_seconds

def analyze_log_file(log_file_path):
    """Extract time and position data from log file."""
    # Patterns to match the required lines
    left_pattern = re.compile(r'(\[\d{2}:\d{2}:\d{2}\.\d{3}\]) Left Positions: ([\d\.\s]+)')
    right_pattern = re.compile(r'(\[\d{2}:\d{2}:\d{2}\.\d{3}\]) Right Positions: ([\d\.\s]+)')
    
    # Data storage
    times = []
    left_avgs = []
    right_avgs = []
    
    with open(log_file_path, 'r') as file:
        for line in file:
            # Check for left positions
            left_match = left_pattern.search(line)
            if left_match:
                time_str = left_match.group(1)
                positions_str = left_match.group(2)
                
                # Convert time to seconds for plotting
                time_in_seconds = parse_time(time_str)
                
                # Extract and average the position values
                positions = [float(pos) for pos in positions_str.strip().split()]
                avg_position = sum(positions) / len(positions)
                
                times.append(time_in_seconds)
                left_avgs.append(avg_position)
                continue
            
            # Check for right positions
            right_match = right_pattern.search(line)
            if right_match:
                time_str = right_match.group(1)
                positions_str = right_match.group(2)
                
                # Extract and average the position values
                positions = [float(pos) for pos in positions_str.strip().split()]
                avg_position = sum(positions) / len(positions)
                
                right_avgs.append(avg_position)
    
    # Make sure we have matching data
    min_len = min(len(left_avgs), len(right_avgs))
    if min_len < len(times):
        times = times[:min_len]
    left_avgs = left_avgs[:min_len]
    right_avgs = right_avgs[:min_len]
    
    return times, left_avgs, right_avgs

def plot_positions(times, left_avgs, right_avgs):
    """Create a plot of the average positions over time."""
    plt.figure(figsize=(12, 6))
    
    # Adjust the times to start from 0
    if times:
        start_time = times[0]
        times = [t - start_time for t in times]
    
    plt.plot(times, left_avgs, label='Left Positions Avg', color='blue', marker='o', linestyle='-', markersize=4)
    plt.plot(times, right_avgs, label='Right Positions Avg', color='red', marker='x', linestyle='-', markersize=4)
    
    plt.xlabel('Time (seconds)')
    plt.ylabel('Average Position')
    plt.title('Average Left vs Right Positions Over Time')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    
    # Add a tight layout to ensure everything fits
    plt.tight_layout()
    
    # Save the plot to a file
    plt.savefig('position_analysis.png')
    
    # Display the plot
    plt.show()

if __name__ == "__main__":
    import sys
    log_file_path = 'robot_log_19700101_000000.txt'
    
    try:
        times, left_avgs, right_avgs = analyze_log_file(log_file_path)
        
        if not times:
            print("No position data found in the log file.")
            sys.exit(1)
        
        print(f"Found {len(times)} matching data points.")
        plot_positions(times, left_avgs, right_avgs)
        
    except Exception as e:
        print(f"Error analyzing log file: {e}")
        sys.exit(1)