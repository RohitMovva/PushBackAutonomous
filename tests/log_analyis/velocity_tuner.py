import pandas as pd
import matplotlib.pyplot as plt
import re
from datetime import datetime
import numpy as np
import mplcursors  # Add this import for cursor snapping and tooltips
import matplotlib.dates as mdates

def parse_data(filename):
    # Separate lists for velocities and voltages
    vel_data = {
        'timestamp': [],
        'left_velocity': [],
        'right_velocity': []
    }
    volt_data = {
        'timestamp': [],
        'voltage': []
    }
    
    # Read and parse the file
    with open(filename, 'r') as file:
        for line in file:
            # Extract timestamp and velocities
            vel_match = re.match(r'\[(\d+:\d+:\d+\.\d+)\] Right wheel velocities: ([-\d.]+) ([-\d.]+)', line)
            volt_match = re.match(r'\[(\d+:\d+:\d+\.\d+)\] Voltages: ([-\d.]+) ([-\d.]+)', line)
            
            if vel_match:
                vel_data['timestamp'].append(vel_match.group(1))
                vel_data['left_velocity'].append(float(vel_match.group(2)))
                vel_data['right_velocity'].append(float(vel_match.group(3)))
                
            elif volt_match:
                volt_data['timestamp'].append(volt_match.group(1))
                volt_data['voltage'].append(float(volt_match.group(3)))
    
    # Create separate dataframes
    vel_df = pd.DataFrame(vel_data)
    volt_df = pd.DataFrame(volt_data)
    
    # Convert timestamp strings to datetime objects
    # Ensure we're parsing with the correct precision
    vel_df['datetime'] = pd.to_datetime(vel_df['timestamp'], format='%H:%M:%S.%f')
    volt_df['datetime'] = pd.to_datetime(volt_df['timestamp'], format='%H:%M:%S.%f')
    
    return vel_df, volt_df

def plot_data(vel_df, volt_df, plot_integral=False):
    fig, ax1 = plt.subplots(figsize=(12, 8))
    
    # Create second y-axis for voltage
    ax2 = ax1.twinx()
    
    # Lists to store plot lines for cursor snapping
    plot_lines = []
    
    if plot_integral:
        # Calculate integrals (cumulative sum * time step)
        # Calculate time differences in seconds for more accurate integration
        time_diffs = np.diff(vel_df['datetime'].values).astype('timedelta64[ms]').astype(float) / 1000
        # Add a default time step for the first point
        time_diffs = np.insert(time_diffs, 0, 0.027)
        
        # Calculate integrals using actual time differences
        left_integral = np.cumsum(vel_df['left_velocity'] * time_diffs)
        right_integral = np.cumsum(vel_df['right_velocity'] * time_diffs)
        
        # Plot integrals on left y-axis using timestamps
        line1 = ax1.plot(vel_df['datetime'], left_integral, label='Goal Distance', color='blue')[0]
        line2 = ax1.plot(vel_df['datetime'], right_integral, label='Actual Distance', color='red')[0]
        plot_lines.extend([line1, line2])
        ax1.set_ylabel('Distance (velocity * time)', fontsize=12)
        title = 'Wheel Distance and Voltage Over Time'
    else:
        # Plot velocities on left y-axis using timestamps
        line1 = ax1.plot(vel_df['datetime'], vel_df['left_velocity'], label='Goal', color='blue')[0]
        line2 = ax1.plot(vel_df['datetime'], vel_df['right_velocity'], label='Actual', color='red')[0]
        plot_lines.extend([line1, line2])
        ax1.set_ylabel('Velocity', fontsize=12)
        title = 'Wheel Velocities and Voltage Over Time'
    
    # Plot voltage on right y-axis with its own timestamps
    line3 = ax2.plot(volt_df['datetime'], volt_df['voltage'], label='Voltage', color='green', alpha=0.5)[0]
    plot_lines.append(line3)
    ax2.set_ylabel('Voltage', color='green', fontsize=12)
    ax2.tick_params(axis='y', labelcolor='green')
    
    # Format the x-axis to show timestamps with precision up to a hundredth of a second
    # Create a custom formatter that shows time with 2 decimal places
    class PreciseTimeFormatter(mdates.DateFormatter):
        def __call__(self, x, pos=0):
            dt = mdates.num2date(x)
            return dt.strftime('%H:%M:%S.%f')[:-4]  # Keep only 2 decimal places
    
    ax1.xaxis.set_major_formatter(PreciseTimeFormatter('%H:%M:%S.%f'))
    
    # Adjust tick spacing based on data range
    time_range = (vel_df['datetime'].max() - vel_df['datetime'].min()).total_seconds()
    if time_range < 10:  # For very short time ranges
        ax1.xaxis.set_major_locator(mdates.SecondLocator(interval=1))
    elif time_range < 60:
        ax1.xaxis.set_major_locator(mdates.SecondLocator(interval=5))
    else:
        ax1.xaxis.set_major_locator(mdates.SecondLocator(interval=15))
    
    plt.xticks(rotation=45)
    
    # Set title and labels
    plt.title(title, fontsize=14)
    ax1.set_xlabel('Time (HH:MM:SS.xx)', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.7)
    
    # Add legends
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
    
    # Add cursor snapping and tooltips
    cursor = mplcursors.cursor(plot_lines, hover=True)
    
    @cursor.connect("add")
    def on_add(sel):
        # Get the index of the selected point
        idx = int(round(sel.target.index))
        
        # Get the x and y values
        x = sel.target[0]
        y = sel.target[1]
        
        # Format the timestamp for display with precision up to a hundredth of a second
        timestamp = mdates.num2date(x).strftime('%H:%M:%S.%f')[:-4]  # Keep only 2 decimal places
        
        # Determine which line was selected to provide appropriate label
        if sel.artist == line1:
            if plot_integral:
                label = f"Goal Distance: {y:.2f}\nTime: {timestamp}"
            else:
                label = f"Goal Velocity: {y:.2f}\nTime: {timestamp}"
        elif sel.artist == line2:
            if plot_integral:
                label = f"Actual Distance: {y:.2f}\nTime: {timestamp}"
            else:
                label = f"Actual Velocity: {y:.2f}\nTime: {timestamp}"
        elif sel.artist == line3:
            label = f"Voltage: {y:.2f}\nTime: {timestamp}"
        
        sel.annotation.set_text(label)
    
    # Generate timestamp for filename
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    plot_type = 'distance' if plot_integral else 'velocity'
    filename = f'wheel_{plot_type}_voltage_{timestamp}.png'
    
    # Save the plot
    plt.tight_layout()
    # plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Plot saved as: {filename}")
    
    # Show the plot
    plt.show()

# Main execution
if __name__ == "__main__":
    # Make sure to install mplcursors if not already installed
    # You can install it using: pip install mplcursors

    vel_df, volt_df = parse_data('../logs/square.txt')

    # Create both velocity and integral plots with voltage
    plot_data(vel_df, volt_df, plot_integral=False)  # Velocity plot
    plot_data(vel_df, volt_df, plot_integral=True)   # Distance plot
