import numpy as np
import time
from robot_env.RTDE_RW_test_collect import RobotAction
from robot_env.time_env import time_ms

class RobotArmController:
    def __init__(self, control_hz=5):
        # Initialize the environment for controlling the robot
        self.env_urt = RobotAction(control_hz=control_hz)
        self.env_urt.reset()
        
        # Initial position of the robot (home position)
        self.current_position = np.array([0, 0, 0])
    
    def move_to_position(self, target_position):
        """
        Move the robot to a specific target position in real-world coordinates.
        :param target_position: Target position [x, y, z] in meters.
        """
        # Calculate delta from the current position to the target position
        delta = np.subtract(target_position, self.current_position)
        
        # Action list: [tcp_x, tcp_y, tcp_z, rx, ry, rz, gripper_state]
        # We assume zero rotation and fixed gripper state for simplicity
        action_single = delta.tolist() + [0, 0, 0, 0]
        
        tcp_coor = action_single[:3]  # TCP coordinates (x, y, z)
        tcp_euler = action_single[3:6]  # TCP Euler angles (rotation)
        gripper_state = action_single[-1]  # Gripper state (fixed at 1)
        
        # Combine all actions into a robot action array
        robot_action = np.concatenate([tcp_coor, tcp_euler, [1 - gripper_state]])
        
        # Send action to the robot
        self.env_urt.send_action(action=robot_action, non_blocking=True)
        
        # Update the current position to the target
        self.current_position = np.array(target_position)
        print(f"Moved to position: {self.current_position}")
    
    def move_to_cube_points(self, cube_points):
        """
        Move the robot to the 8 points of a cube in midair.
        :param cube_points: List of 8 points representing the corners of a cube.
        """
        for point in cube_points:
            control_timestamps = time_ms()
            
            # Move to the current point
            self.move_to_position(point)
            
            # Wait for the next control cycle
            comp_time = time_ms() - control_timestamps
            sleep_left = (1000 / 500) - comp_time  # 5 Hz control rate
            if sleep_left > 0:
                time.sleep(sleep_left / 1000)
    
    def close(self):
        # Close the environment after use
        self.env_urt.close()


def generate_sphere_points(radius, num_points=100):
    """
    Generate points on the surface of a sphere using spherical coordinates.
    :param radius: The radius of the sphere.
    :param num_points: The number of points to generate on the sphere's surface.
    :return: List of points in Cartesian coordinates.
    """
    points = []
    theta_values = np.linspace(0, np.pi, num_points)  # Polar angle
    phi_values = np.linspace(0, 2 * np.pi, num_points)  # Azimuthal angle
    
    for theta in theta_values:
        for phi in phi_values:
            # Convert spherical to Cartesian coordinates
            x = radius * np.sin(theta) * np.cos(phi)
            y = radius * np.sin(theta) * np.sin(phi)
            z = radius * np.cos(theta)
            points.append([x, y, z])
    
    return points

# Create an instance of the RobotArmController
robot_controller = RobotArmController(control_hz=100)

# Define the radius of the sphere
radius = 0.3  # 10 cm radius

# Generate points on the surface of the sphere
sphere_points = generate_sphere_points(radius, num_points=100)

# Move the robot to each of the points on the sphere
for point in sphere_points:
    robot_controller.move_to_position(point)
    # Optionally, wait to ensure movements are spaced out
    time.sleep(1 / robot_controller.env_urt.control_hz)

# Close the environment after use
robot_controller.close()
