import pandas as pd
import matplotlib.pyplot as plt
import re
from datetime import datetime
import numpy as np

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
            vel_match = re.match(r'\[(\d+:\d+:\d+\.\d+)\] Left wheel velocities: ([-\d.]+) ([-\d.]+)', line)
            volt_match = re.match(r'\[(\d+:\d+:\d+\.\d+)\] Voltages: ([-\d.]+) ([-\d.]+)', line)
            
            if vel_match:
                vel_data['timestamp'].append(vel_match.group(1))
                vel_data['left_velocity'].append(float(vel_match.group(2)))
                vel_data['right_velocity'].append(float(vel_match.group(3)))
                
            elif volt_match:
                volt_data['timestamp'].append(volt_match.group(1))
                volt_data['voltage'].append(float(volt_match.group(2)))
    
    # Create separate dataframes
    vel_df = pd.DataFrame(vel_data)
    volt_df = pd.DataFrame(volt_data)
    
    return vel_df, volt_df

def plot_data(vel_df, volt_df, plot_integral=False):
    fig, ax1 = plt.subplots(figsize=(12, 8))
    
    # Create second y-axis for voltage
    ax2 = ax1.twinx()
    
    if plot_integral:
        # Calculate integrals (cumulative sum * time step)
        time_step = 0.027
        left_integral = np.cumsum(vel_df['left_velocity']) * time_step
        right_integral = np.cumsum(vel_df['right_velocity']) * time_step
        
        # Plot integrals on left y-axis
        ax1.plot(range(len(left_integral)), left_integral, label='Goal Distance', color='blue')
        ax1.plot(range(len(right_integral)), right_integral, label='Actual Distance', color='red')
        ax1.set_ylabel('Distance (velocity * time)', fontsize=12)
        title = 'Wheel Distance and Voltage Over Time'
    else:
        # Plot velocities on left y-axis
        ax1.plot(range(len(vel_df)), vel_df['left_velocity'], label='Goal', color='blue')
        ax1.plot(range(len(vel_df)), vel_df['right_velocity'], label='Actual', color='red')
        ax1.set_ylabel('Velocity', fontsize=12)
        title = 'Wheel Velocities and Voltage Over Time'
    
    # Plot voltage on right y-axis with its own x-coordinates
    ax2.plot(range(len(volt_df)), volt_df['voltage'], label='Voltage', color='green', alpha=0.5)
    ax2.set_ylabel('Voltage', color='green', fontsize=12)
    ax2.tick_params(axis='y', labelcolor='green')
    
    # Set title and labels
    plt.title(title, fontsize=14)
    ax1.set_xlabel('Sample Number', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.7)
    
    # Add legends
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
    
    # Generate timestamp for filename
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    plot_type = 'distance' if plot_integral else 'velocity'
    filename = f'wheel_{plot_type}_voltage_{timestamp}.png'
    
    # Save the plot
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Plot saved as: {filename}")
    
    # Show the plot
    plt.show()

# Main execution
if __name__ == "__main__":

    vel_df, volt_df = parse_data('logs/robot_log_19700101_000000.txt')
    
    # Create both velocity and integral plots with voltage
    plot_data(vel_df, volt_df, plot_integral=False)  # Velocity plot
    plot_data(vel_df, volt_df, plot_integral=True)   # Distance plot
# if __name__ == "__main__":
