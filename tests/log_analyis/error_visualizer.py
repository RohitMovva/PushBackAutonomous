import matplotlib.pyplot as plt
import numpy as np
import re
from typing import List, Tuple

def parse_robot_logs(log_content: str) -> dict:
    """
    Parse robot log data and extract trajectory points, poses, and goals.
    
    Args:
        log_content: Raw log file content as string
    
    Returns:
        Dictionary containing parsed data arrays
    """
    lines = log_content.strip().split('\n')
    
    # Data storage
    timestamps = []
    trajectory_points = []
    poses = []  # [x, y, theta]
    goals = []  # [x, y, theta]
    imu_headings = []
    headings = []
    
    current_timestamp = None
    current_point = None
    current_pose = None
    current_goal = None
    current_imu_heading = None
    current_heading = None
    
    for line in lines:
        line = line.strip()
        if not line:
            continue
            
        # Extract timestamp
        timestamp_match = re.search(r'\[(\d{2}:\d{2}:\d{2}\.\d{3})\]', line)
        if timestamp_match:
            current_timestamp = timestamp_match.group(1)
        
        # Extract trajectory point
        if 'Processing trajectory point' in line:
            point_match = re.search(r'Processing trajectory point (\d+)', line)
            if point_match:
                current_point = int(point_match.group(1))
        
        # Extract IMU heading
        elif 'IMU Heading:' in line:
            heading_match = re.search(r'IMU Heading: ([-\d.]+)', line)
            if heading_match:
                current_imu_heading = float(heading_match.group(1))
        
        # Extract heading
        elif line.startswith('[') and 'Heading:' in line:
            heading_match = re.search(r'Heading: ([-\d.]+)', line)
            if heading_match:
                current_heading = float(heading_match.group(1))
        
        # Extract pose (x, y, theta)
        elif 'Pose:' in line:
            pose_match = re.search(r'Pose: ([-\d.]+) ([-\d.]+) ([-\d.]+)', line)
            if pose_match:
                current_pose = [float(pose_match.group(1)), 
                              float(pose_match.group(2)), 
                              float(pose_match.group(3))]
        
        # Extract goal (x, y, theta)
        elif 'Goal:' in line:
            goal_match = re.search(r'Goal: ([-\d.]+) ([-\d.]+) ([-\d.]+)', line)
            if goal_match:
                current_goal = [float(goal_match.group(1)), 
                              float(goal_match.group(2)), 
                              float(goal_match.group(3))]
                
                # When we have a complete set, store it
                if all(x is not None for x in [current_timestamp, current_point, 
                                             current_pose, current_goal, 
                                             current_imu_heading, current_heading]):
                    timestamps.append(current_timestamp)
                    trajectory_points.append(current_point)
                    poses.append(current_pose)
                    goals.append(current_goal)
                    imu_headings.append(current_imu_heading)
                    headings.append(current_heading)
    
    return {
        'timestamps': timestamps,
        'trajectory_points': np.array(trajectory_points),
        'poses': np.array(poses),
        'goals': np.array(goals),
        'imu_headings': np.array(imu_headings),
        'headings': np.array(headings)
    }

def calculate_errors(poses: np.ndarray, goals: np.ndarray, headings: np.ndarray) -> dict:
    """
    Calculate various error metrics for robot trajectory tracking.
    
    Args:
        poses: Array of [x, y, theta] poses
        goals: Array of [x, y, theta] goals
        headings: Array of heading values
    
    Returns:
        Dictionary containing error calculations
    """
    # Position errors
    position_errors = poses[:, :2] - goals[:, :2]  # [dx, dy]
    
    # Normal (along-track) and cross-track errors
    # Calculate path direction at each point
    if len(goals) > 1:
        # Path direction vector (tangent to path)
        path_directions = np.diff(goals[:, :2], axis=0)
        # Pad with last direction for consistent array size
        path_directions = np.vstack([path_directions, path_directions[-1]])
        
        # Normalize path directions
        path_lengths = np.linalg.norm(path_directions, axis=1)
        path_lengths[path_lengths == 0] = 1  # Avoid division by zero
        path_unit_vectors = path_directions / path_lengths[:, np.newaxis]
        
        # Cross-track error (perpendicular to path)
        cross_track_errors = []
        normal_errors = []
        
        for i, (pos_error, path_unit) in enumerate(zip(position_errors, path_unit_vectors)):
            # Cross-track error (dot product with perpendicular vector)
            perpendicular = np.array([-path_unit[1], path_unit[0]])
            cross_track_error = np.dot(pos_error, perpendicular)
            cross_track_errors.append(cross_track_error)
            
            # Normal (along-track) error
            normal_error = np.dot(pos_error, path_unit)
            normal_errors.append(normal_error)
    else:
        # Fallback for single point
        cross_track_errors = [0]
        normal_errors = [np.linalg.norm(position_errors[0])]
    
    # Heading errors
    heading_errors = poses[:, 2] - goals[:, 2]
    
    # Normalize heading errors to [-π, π]
    heading_errors = np.arctan2(np.sin(heading_errors), np.cos(heading_errors))
    
    # Total position error magnitude
    total_position_errors = np.linalg.norm(position_errors, axis=1)
    
    return {
        'position_errors': position_errors,
        'cross_track_errors': np.array(cross_track_errors),
        'normal_errors': np.array(normal_errors),
        'heading_errors': heading_errors,
        'total_position_errors': total_position_errors
    }

def visualize_robot_errors(log_file_path: str):
    """
    Create comprehensive visualization of robot tracking errors.
    
    Args:
        log_file_path: Path to the log file
    """
    # Read log file
    with open(log_file_path, 'r') as f:
        log_content = f.read()
    
    # Parse data
    data = parse_robot_logs(log_content)
    
    if len(data['poses']) == 0:
        print("No valid data found in log file")
        return
    
    # Calculate errors
    errors = calculate_errors(data['poses'], data['goals'], data['headings'])
    
    # Create visualization
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('Robot Trajectory Tracking Error Analysis', fontsize=16, fontweight='bold')
    
    trajectory_points = data['trajectory_points']
    
    # 1. Trajectory Plot
    ax = axes[0, 0]
    ax.plot(data['goals'][:, 0], data['goals'][:, 1], 'b-', linewidth=2, label='Reference Path', alpha=0.7)
    ax.plot(data['poses'][:, 0], data['poses'][:, 1], 'r-', linewidth=2, label='Actual Path', alpha=0.7)
    ax.scatter(data['goals'][0, 0], data['goals'][0, 1], color='green', s=100, marker='o', label='Start', zorder=5)
    ax.scatter(data['goals'][-1, 0], data['goals'][-1, 1], color='red', s=100, marker='s', label='End', zorder=5)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('Robot Trajectory')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # 2. Cross-track Error
    ax = axes[0, 1]
    ax.plot(trajectory_points, errors['cross_track_errors'], 'g-', linewidth=2)
    ax.fill_between(trajectory_points, errors['cross_track_errors'], alpha=0.3, color='green')
    ax.set_xlabel('Trajectory Point')
    ax.set_ylabel('Cross-track Error')
    ax.set_title('Cross-track Error Over Time')
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='black', linestyle='--', alpha=0.5)
    
    # Add statistics
    mean_cross = np.mean(np.abs(errors['cross_track_errors']))
    max_cross = np.max(np.abs(errors['cross_track_errors']))
    ax.text(0.02, 0.98, f'Mean |Error|: {mean_cross:.3f}\nMax |Error|: {max_cross:.3f}', 
            transform=ax.transAxes, verticalalignment='top', 
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # 3. Normal (Along-track) Error
    ax = axes[0, 2]
    ax.plot(trajectory_points, errors['normal_errors'], 'orange', linewidth=2)
    ax.fill_between(trajectory_points, errors['normal_errors'], alpha=0.3, color='orange')
    ax.set_xlabel('Trajectory Point')
    ax.set_ylabel('Normal Error')
    ax.set_title('Normal (Along-track) Error Over Time')
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='black', linestyle='--', alpha=0.5)
    
    # Add statistics
    mean_normal = np.mean(np.abs(errors['normal_errors']))
    max_normal = np.max(np.abs(errors['normal_errors']))
    ax.text(0.02, 0.98, f'Mean |Error|: {mean_normal:.3f}\nMax |Error|: {max_normal:.3f}', 
            transform=ax.transAxes, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # 4. Heading Error
    ax = axes[1, 0]
    heading_errors_deg = np.degrees(errors['heading_errors'])
    ax.plot(trajectory_points, heading_errors_deg, 'purple', linewidth=2)
    ax.fill_between(trajectory_points, heading_errors_deg, alpha=0.3, color='purple')
    ax.set_xlabel('Trajectory Point')
    ax.set_ylabel('Heading Error (degrees)')
    ax.set_title('Heading Error Over Time')
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='black', linestyle='--', alpha=0.5)
    
    # Add statistics
    mean_heading = np.mean(np.abs(heading_errors_deg))
    max_heading = np.max(np.abs(heading_errors_deg))
    ax.text(0.02, 0.98, f'Mean |Error|: {mean_heading:.2f}°\nMax |Error|: {max_heading:.2f}°', 
            transform=ax.transAxes, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # 5. Total Position Error
    ax = axes[1, 1]
    ax.plot(trajectory_points, errors['total_position_errors'], 'red', linewidth=2)
    ax.fill_between(trajectory_points, errors['total_position_errors'], alpha=0.3, color='red')
    ax.set_xlabel('Trajectory Point')
    ax.set_ylabel('Total Position Error')
    ax.set_title('Total Position Error Over Time')
    ax.grid(True, alpha=0.3)
    
    # Add statistics
    mean_total = np.mean(errors['total_position_errors'])
    max_total = np.max(errors['total_position_errors'])
    rms_total = np.sqrt(np.mean(errors['total_position_errors']**2))
    ax.text(0.02, 0.98, f'Mean Error: {mean_total:.3f}\nMax Error: {max_total:.3f}\nRMS Error: {rms_total:.3f}', 
            transform=ax.transAxes, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # 6. Error Distribution
    ax = axes[1, 2]
    ax.hist(errors['cross_track_errors'], bins=20, alpha=0.7, label='Cross-track', color='green')
    ax.hist(errors['normal_errors'], bins=20, alpha=0.7, label='Normal', color='orange')
    ax.hist(heading_errors_deg, bins=20, alpha=0.7, label='Heading (°)', color='purple')
    ax.set_xlabel('Error Value')
    ax.set_ylabel('Frequency')
    ax.set_title('Error Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Print summary statistics
    print("\n" + "="*60)
    print("ROBOT TRAJECTORY TRACKING ERROR SUMMARY")
    print("="*60)
    print(f"Total trajectory points analyzed: {len(trajectory_points)}")
    print(f"Trajectory point range: {trajectory_points[0]} to {trajectory_points[-1]}")
    print("\nCROSS-TRACK ERROR:")
    print(f"  Mean absolute error: {np.mean(np.abs(errors['cross_track_errors'])):.4f}")
    print(f"  Maximum absolute error: {np.max(np.abs(errors['cross_track_errors'])):.4f}")
    print(f"  RMS error: {np.sqrt(np.mean(errors['cross_track_errors']**2)):.4f}")
    print(f"  Standard deviation: {np.std(errors['cross_track_errors']):.4f}")
    
    print("\nNORMAL (ALONG-TRACK) ERROR:")
    print(f"  Mean absolute error: {np.mean(np.abs(errors['normal_errors'])):.4f}")
    print(f"  Maximum absolute error: {np.max(np.abs(errors['normal_errors'])):.4f}")
    print(f"  RMS error: {np.sqrt(np.mean(errors['normal_errors']**2)):.4f}")
    print(f"  Standard deviation: {np.std(errors['normal_errors']):.4f}")
    
    print("\nHEADING ERROR:")
    print(f"  Mean absolute error: {np.mean(np.abs(heading_errors_deg)):.2f}°")
    print(f"  Maximum absolute error: {np.max(np.abs(heading_errors_deg)):.2f}°")
    print(f"  RMS error: {np.sqrt(np.mean(heading_errors_deg**2)):.2f}°")
    print(f"  Standard deviation: {np.std(heading_errors_deg):.2f}°")
    
    print("\nTOTAL POSITION ERROR:")
    print(f"  Mean error: {np.mean(errors['total_position_errors']):.4f}")
    print(f"  Maximum error: {np.max(errors['total_position_errors']):.4f}")
    print(f"  RMS error: {np.sqrt(np.mean(errors['total_position_errors']**2)):.4f}")
    print(f"  Standard deviation: {np.std(errors['total_position_errors']):.4f}")

# Example usage
if __name__ == "__main__":
    # Replace 'paste.txt' with your actual log file path
    visualize_robot_errors('../logs/default.txt')