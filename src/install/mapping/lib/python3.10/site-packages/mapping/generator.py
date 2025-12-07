#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import TrajectorySetpoint ,  OffboardControlMode, VehicleControlMode, VehicleStatus, VehicleCommand, VehicleOdometry
from nav_msgs.msg import Odometry

import csv
import numpy as np
import signal

import math
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt
import math
from matplotlib.colors import ListedColormap # Needed for the 3-color map
import ast
from std_msgs.msg import Float32

def quaternion_to_euler(x, y, z, w):
    r = R.from_quat([x, y, z, w])
    roll, pitch, yaw = r.as_euler('xyz', degrees=False)
    return roll, pitch, yaw


class Offboard(Node):

    def __init__(self):
        super().__init__("offboard")

        self.takeoff_mode=False

        self.counter=0

        self.t=0.1                  # time
        self.dt=0.1

        self.scan_data=[]

        #Mapping Variables

        # --- 1. Grid and Sensor Parameters ---

        # We use a small grid so you can see individual cells
        self.GRID_WIDTH_M = 10.0  # Meters
        self.GRID_HEIGHT_M = 10.0 # Meters
        self.RESOLUTION = 0.1    # Meters per cell (10cm)
        self.GRID_WIDTH_CELLS = int(self.GRID_WIDTH_M / self.RESOLUTION)   # 40 cells
        self.GRID_HEIGHT_CELLS = int(self.GRID_HEIGHT_M / self.RESOLUTION) # 40 cells

        # Place the world origin (0,0) at the center of the grid
        self.GRID_ORIGIN_X_CELL = self.GRID_WIDTH_CELLS // 2  # Cell (20, 20)
        self.GRID_ORIGIN_Y_CELL = self.GRID_HEIGHT_CELLS // 2

        # Sensor model parameters (in log-odds)
        self.LOG_ODDS_OCC = 2.2  # P(occ) = 0.9 (Strong belief)
        self.LOG_ODDS_FREE = -0.7 # P(occ) = 0.3 (Weak belief)
        self.LOG_ODDS_MAX = 5.0   # Max confidence
        self.LOG_ODDS_MIN = -5.0  # Min confidence
        self.MAX_RANGE =  20
        # --- 2. Coordinate Conversion Functions ---

        self.occupancy_grid = np.zeros((self.GRID_HEIGHT_CELLS, self.GRID_WIDTH_CELLS))

         # QOS Profiles
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_profile_gt = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)

        qos_profile_helipad = QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
        # Publishers

        self.offboard_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
        self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)

         # Subscribers
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)
        self.status_subscriber = self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode", self.state_callback, qos_profile)
        
        self.vehicle_odometry_subscriber = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.vehicle_odometry_callback, qos_profile)
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Topic name where LiDAR data is published
            self.laser_callback,
            10)                 # QoS profile depth
        
        self.get_logger().info('LidarSubscriber node has been started.')
        
        
        self.vehicle_status = None
        self.state = None
       
        self.vehicle_odometry = None

        self.takeoff_height=-5.0

        self.create_timer(0.1, self.command_loop)  # call every 2 sec
        self.create_timer(0.33,self.mapping)

        self.rmse_x=0.0
        self.rmse_y=0.0
        self.rmse_z=0.0


        self.rmse_vx=0.0
        self.rmse_vy=0.0
        self.rmse_vz=0.0

        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.integral_error_z = 0.0

        self.acc_mode=True



        #self.state=             # exploration state ,  Go to state , Trace State , Land state 

        with open("state_data_task_4.csv", mode="w+", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["time","pos_x", "pos_y", "pos_z","ideal_x","ideal_y","ideal_z","error_x","error_y","error_z","vel_x", "vel_y", "vel_z" ,"ideal_vx","ideal_vy","ideal_vz","error_vx","error_vy","error_vz","roll","pitch","yaw","roll_rate","pitch_rate","yaw_rate"])



    # Subscriber Callback Functions
    
    def state_callback(self, status_msg):
        self.status = status_msg
    
    def vehicle_status_callback(self, vehicle_status_msg):
        self.vehicle_status = vehicle_status_msg
   
    def vehicle_odometry_callback(self, vehicle_odometry_msg):
        self.vehicle_odometry = vehicle_odometry_msg

    def laser_callback(self, msg):

        # Process the LaserScan message here
        self.get_logger().info(f'Received LaserScan data:')
        self.get_logger().info(f'  Header: {msg.header}')
        self.get_logger().info(f'  Angle Min: {msg.angle_min}, Angle Max: {msg.angle_max}, Angle Increment: {msg.angle_increment}')
        self.get_logger().info(f'  Time Increment: {msg.time_increment}, Scan Time: {msg.scan_time}')
        self.get_logger().info(f'  Range Min: {msg.range_min}, Range Max: {msg.range_max}')
        
        # Accessing the range measurements (distances)
        # msg.ranges is a list of float values representing distances in meters
        # Infinite values indicate no reflection within the maximum range

        self.scan_data=msg.ranges

        self.get_logger().info(f'  Number of ranges: {len(msg.ranges)}')
        # You can iterate through msg.ranges to access individual measurements
        # For example, to print the first 10 range values:
        # for i in range(min(10, len(msg.ranges))):
        #     self.get_logger().info(f'    Range[{i}]: {msg.ranges[i]}')

        # Accessing intensity measurements (if available)
        # msg.intensities is a list of float values representing intensity
        # if msg.intensities:
        #     self.get_logger().info(f'  Number of intensities: {len(msg.intensities)}')



    def world_to_grid(self,wx, wy):
        """Converts world coordinates (meters) to grid cell coordinates (pixels)."""
        gx = int((wx / self.RESOLUTION) + self.GRID_ORIGIN_X_CELL)
        gy = int((wy / self.RESOLUTION) + self.GRID_ORIGIN_Y_CELL)
        return gx, gy

    def is_in_grid_bounds(self,gx, gy):
        """Checks if a grid cell is within the map boundaries."""
        return 0 <= gx < self.GRID_WIDTH_CELLS and 0 <= gy < self.GRID_HEIGHT_CELLS

    # --- 3. Bresenham's Line Algorithm ---

    def bresenham_line(self,x0, y0, x1, y1):
        """
        Generates all grid cells on the line from (x0, y0) to (x1, y1).
        Yields: (x, y) grid cell coordinates
        """
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy  # error value e_xy

        while True:
            yield (x0, y0)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:  # e_xy+e_x > 0
                err += dy
                x0 += sx
            if e2 <= dx:  # e_xy+e_y < 0
                err += dx
                y0 += sy

    # --- 4. Main Occupancy Grid Update Function ---

    def update_occupancy_grid(self,grid, robot_pose, scan_data):
        """
        Updates the occupancy grid with one new LiDAR scan.
        
        :param grid: The log-odds occupancy grid (numpy array)
        :param robot_pose: (x, y, theta) of the robot in world coordinates (meters, radians)
        :param scan_data: List of (angle, range) tuples from the LiDAR
        """
        
        robot_x, robot_y, robot_theta = robot_pose
        
        # Get the robot's position in grid coordinates
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        print(scan_data)
        # Loop through each laser beam in the scan
        for angle, range_m in scan_data:
            # Calculate the beam's global angle
            global_angle = robot_theta + angle

            """if range_m == Float32('inf'):"""
            if math.isinf(range_m):
                range_m = self.MAX_RANGE
            

            # Calculate the hit point in world coordinates
            hit_x = robot_x - (range_m * math.sin(global_angle))
            hit_y = robot_y - (range_m * math.cos(global_angle))
            
            # Convert hit point to grid coordinates
            hit_gx, hit_gy = self.world_to_grid(hit_x, hit_y)
            
            # --- Raycasting with Bresenham's ---
            # Get all cells from robot to hit point
            line_cells = list(self.bresenham_line(robot_gx, robot_gy, hit_gx, hit_gy))
            
            # Mark all cells *before* the hit as FREE
            # We stop one short (hence line_cells[:-1])
            for (gx, gy) in line_cells[:-1]:
                if self.is_in_grid_bounds(gx, gy):
                    # Use [gy, gx] for numpy array (row, col)
                    grid[gy, gx] += self.LOG_ODDS_FREE
                    # Clamp the value
                    grid[gy, gx] = max(self.LOG_ODDS_MIN, grid[gy, gx])
                    
            # Mark the hit cell as OCCUPIED
            if self.is_in_grid_bounds(hit_gx, hit_gy):
                # Use [gy, gx] for numpy array (row, col)
                grid[hit_gy, hit_gx] += self.LOG_ODDS_OCC
                # Clamp the value
                grid[hit_gy, hit_gx] = min(self.LOG_ODDS_MAX, grid[hit_gy, hit_gx])

    def convert_to_three_state_matrix(self,grid):
        """
        Converts log-odds occupancy grid to three-state matrix:
        1 = Occupied (black)
        0 = Free (white)
        -1 = Unexplored (grey)
        
        :param grid: Log-odds occupancy grid
        :return: Three-state matrix (100x100)
        """
        # Convert log-odds to probabilities
        prob_grid = 1.0 - 1.0 / (1.0 + np.exp(grid))
        
        # Define thresholds
        FREE_THRESH = 0.2
        OCC_THRESH = 0.8
        
        # Initialize matrix with -1 (unexplored)
        three_state_matrix = np.full(prob_grid.shape, -1, dtype=int)
        
        # Mark free cells as 0
        three_state_matrix[prob_grid < FREE_THRESH] = 0
        
        # Mark occupied cells as 1
        three_state_matrix[prob_grid > OCC_THRESH] = 1
        
        return three_state_matrix


    # --- 5. Visualization Function (3-Color Thresholded) ---

    def visualize_grid_with_lines(self,grid, robot_pose=None):
        """
        Visualizes the grid with explicit cell lines and 3-color thresholding.
        """
        
        # --- 1. Convert log-odds to probabilities (0.0 to 1.0) ---
        # p = 1 - 1 / (1 + exp(L))
        prob_grid = 1.0 - 1.0 / (1.0 + np.exp(grid))
        
        # --- 2. Define Thresholds and Create 3-Color Grid ---
        
        # Define our probability thresholds
        FREE_THRESH = 0.2  # Below this is "Free" (White)
        OCC_THRESH = 0.8   # Above this is "Occupied" (Black)
        # Anything in between is "Unknown" (Gray)
        
        # Start with a new grid full of 0.5 (Gray)
        # 0.5 is the value for "Unknown"
        display_grid = np.full(prob_grid.shape, 0.5)
        
        # Mark "Free" cells (white) - we use 0.0 for white
        display_grid[prob_grid < FREE_THRESH] = 0.0
        
        # Mark "Occupied" cells (black) - we use 1.0 for black
        display_grid[prob_grid > OCC_THRESH] = 1.0

        # --- 3. Create the 3-color colormap ---
        # We create a colormap with 3 distinct colors:
        # 0.0 -> White
        # 0.5 -> Gray
        # 1.0 -> Black
        cmap = ListedColormap(['white', 'gray', 'black'])

        # --- 4. Plot the map ---
        """fig, ax = plt.subplots(figsize=(12, 12))
        
        # Plot this new 3-color grid
        # vmin=0, vmax=1 maps our 0.0, 0.5, 1.0 values to the colormap
        # 'nearest' interpolation ensures sharp, non-blurry cells
        ax.imshow(display_grid, cmap=cmap, vmin=0.0, vmax=1.0, interpolation='nearest')
        ax.set_title("Thresholded Map (3 Distinct Colors)")

        # --- 5. Draw grid lines and robot ---
        height, width = prob_grid.shape
        ax.set_xticks(np.arange(width) - 0.5, minor=True)
        ax.set_yticks(np.arange(height) - 0.5, minor=True)
        ax.grid(which='minor', color='black', linestyle='-', linewidth=0.5)
        """
        # --- Rotate grid 90 degrees CCW ---
        rotated_grid = np.rot90(display_grid, k=1)

        # --- Plot ---
        fig, ax = plt.subplots(figsize=(6,6))
        ax.imshow(rotated_grid, cmap=cmap ,  vmin=0.0, vmax=1.0, interpolation='nearest')

        # --- Invert y to match y → -y ---
        #ax.invert_yaxis()

        # --- Grid lines ---
        height, width = rotated_grid.shape
        ax.set_xticks(np.arange(width) - 0.5, minor=True)
        ax.set_yticks(np.arange(height) - 0.5, minor=True)
        ax.grid(which='minor', color='black', linestyle='-', linewidth=0.5)
    

        # --- Axis labels after rotation ---
        ax.set_xlabel("Y (old X)")
        ax.set_ylabel("-X (old Y)")

        if robot_pose is not None:
            robot_x, robot_y, _ = robot_pose
            robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
            # Plot a blue circle for the robot
            ax.plot(robot_gx, robot_gy, 'bo', markersize=12, label='Robot')
            ax.legend()
        
        ax.set_xlabel("X (cells)")
        ax.set_ylabel("Y (cells)")
        
        # Set tick labels to show cell indices
        ax.set_xticks(np.arange(0, width, 10))
        ax.set_yticks(np.arange(0, height, 10))
        
        plt.show()

    def mapping(self):
        robot_pose_1 = (0.0, 0.0, 0.0) # (x, y, theta)
        scan_data_1 = []
        #scan_data_2 = [np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 6.039574146270752, 6.0108537673950195, 6.002606391906738, 6.004460334777832, 6.023561954498291, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 5.01814079284668, 4.991277694702148, 4.975718021392822, 4.974824905395508, 4.98310661315918, 4.997000694274902, 5.027985572814941, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 3.952235460281372, 3.904370069503784, 3.8852181434631348, 3.8760104179382324, 3.8731400966644287, 3.8750698566436768, 3.879990816116333, 3.8923091888427734, 3.9161465167999268, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 5.920563220977783, 5.870619773864746, 5.845969200134277, 5.8421311378479, 5.847636699676514, 5.876046657562256, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 4.567161560058594, 4.540376663208008, 4.534478664398193, 4.529103755950928, 4.536867618560791, 4.5442795753479, 4.575791358947754, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 5.1843671798706055, 5.157827377319336, 5.140520095825195, 5.139183044433594, 5.144273281097412, 5.166998863220215, 5.203649520874023, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 4.4417877197265625, 4.4296555519104, 4.418459892272949, 4.415445804595947, 4.422060012817383, 4.433501720428467, 4.469255447387695, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 5.759964942932129, 5.720346927642822, 5.703962326049805, 5.696531772613525, 5.703655242919922, 5.718306541442871, np.inf, np.inf, np.inf, np.inf, np.inf, 1.8608652353286743, 1.8441987037658691, 1.8320039510726929, 1.823478102684021, 1.8151930570602417, 1.8092752695083618, 1.8071973323822021, 1.8051753044128418, 1.8048317432403564, 1.8046373128890991, 1.8058580160140991, 1.8090033531188965, 1.811928629875183, 1.819077968597412, 1.8237330913543701, 1.8345746994018555, 1.8502806425094604, 1.8797507286071777, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]
        
        scan_data_2=self.scan_data
        count = 0
        for angle in np.linspace(-math.pi, math.pi,len(scan_data_2)):
            # Create a simple flat wall at x = 1.5 meters
            # range = 1.5 / cos(angle)
            
            if (scan_data_2[count] == float('inf')):
                range_m = float('inf')
            else:
                range_m = scan_data_2[count]

            scan_data_1.append((angle, range_m))
            count+=1
        # --- Simulate a second scan of a side wall ---
        # This will create a corner, like in your obstacle
        robot_pose_2 = (0.0, 0.0, 0.0) # Robot hasn't moved
        scan_data_2 = []
        # Scan from 30 to 120 degrees
        for angle in np.linspace(math.pi/6, 2*math.pi/3, 40):
            # Create a wall at y = 1.0 meters
            # range = 1.0 / sin(angle)
            range_m = 100
            scan_data_2.append((angle, range_m))

        
        # --- Step 4: Process the scans ---
        print("Updating grid with scan 1 (front wall)...")
        self.update_occupancy_grid(self.occupancy_grid, robot_pose_1, scan_data_1)
        
        print("Updating grid with scan 2 (side wall)...")
        self.update_occupancy_grid(self.occupancy_grid, robot_pose_2, scan_data_2)
        
        # Convert to three-state matrix
        print("\nConverting to three-state matrix...")
        three_state_matrix = self.convert_to_three_state_matrix(self.occupancy_grid)
        
        # Print the three-state matrix
        print("\n=== THREE-STATE MATRIX (100x100) ===")
        print("Legend: 1 = Occupied (Black), 0 = Free (White), -1 = Unexplored (Grey)\n")
        
        for row in range(100):
            for column in range(100):
                print(f"{three_state_matrix[row, column]:2d}", end=" ")
            print()

        # --- Step 5: Visualize the final map ---
        print("Displaying 3-Color Thresholded map...")
        # This call will now produce the 3-color map
        self.visualize_grid_with_lines(self.occupancy_grid, robot_pose_1)
        print(self.occupancy_grid)
        print(self.occupancy_grid.shape)
        print("/n")

        """for row in range(100):
            for column in range(100):
                print(occupancy_grid[row, column], end=", ")
            print()   # Moves to next line after each row'''"""

    def offboard_control_heartbeat_signal_publisher(self):
        msg = OffboardControlMode()
        msg.position = self.takeoff_mode
        msg.velocity = False
        msg.acceleration = self.acc_mode
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False 
        msg.direct_actuator = False 
        
        """match what_control:
            case 'position':
                msg.position = True
            case 'velocity':
                msg.velocity = True
            case 'acceleration':
                msg.acceleration = True
            case 'attitude':
                msg.attitude = True
            case 'body_rate':
                msg.body_rate = True
            case 'thrust_and_torque':
                msg.thrust_and_torque = True
            case 'direct_actuator':
                msg.direct_actuator = True"""
        
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.offboard_mode_publisher.publish(msg)


    def engage_offboard_mode(self):
        instance_num = 1
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = instance_num
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        instance_num = 1
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = instance_num
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.vehicle_command_publisher.publish(msg)

    def disarm(self):
        instance_num = 1
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = instance_num
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.vehicle_command_publisher.publish(msg)

    


    def takeoff(self):
        instance_num = 1
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        msg.param1 = 0.0
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = instance_num
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.vehicle_command_publisher.publish(msg)

    def publish_trajectory_wpt(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position[0] = x
        msg.position[1] = y
        msg.position[2] = z
        msg.velocity[0] = float('nan')
        msg.velocity[1] = float('nan')
        msg.velocity[2] = float('nan')
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        msg.jerk[0] = float('nan')
        msg.jerk[1] = float('nan')
        msg.jerk[2] = float('nan')
        msg.yaw = 1.57079
        msg.yawspeed = float('nan')
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.trajectory_publisher.publish(msg)

    def publish_trajectory_v(self, vx, vy, vz):
        msg = TrajectorySetpoint()
        msg.position[0] = float('nan')
        msg.position[1] = float('nan')
        msg.position[2] = float('nan')
        msg.velocity[0] = vx
        msg.velocity[1] = vy
        msg.velocity[2] = vz
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        msg.jerk[0] = float('nan')
        msg.jerk[1] = float('nan')
        msg.jerk[2] = float('nan')
        msg.yaw = 1.57079
        msg.yawspeed = float('nan')
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.trajectory_publisher.publish(msg)

    def publish_trajectory_a(self, ax,ay,az):
        msg = TrajectorySetpoint()
        msg.position[0] = float('nan')
        msg.position[1] = float('nan')
        msg.position[2] = float('nan')
        msg.velocity[0] = float('nan')
        msg.velocity[1] = float('nan')
        msg.velocity[2] = float('nan')
        msg.acceleration[0] = ax
        msg.acceleration[1] = ay
        msg.acceleration[2] = az
        msg.jerk[0] = float('nan')
        msg.jerk[1] = float('nan')
        msg.jerk[2] = float('nan')
        msg.yaw = 1.57079
        msg.yawspeed = float('nan')
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        self.trajectory_publisher.publish(msg)

    def error(self):

        error_x=self.helipad_odometry.pose.pose.position.y-self.vehicle_odometry.position[0]
        error_y=self.helipad_odometry.pose.pose.position.x-self.vehicle_odometry.position[1]
        error_z=self.z_ref-self.vehicle_odometry.position[2]

        error_vx=self.helipad_odometry.twist.twist.linear.y-self.vehicle_odometry.velocity[0]
        error_vy=self.helipad_odometry.twist.twist.linear.x-self.vehicle_odometry.velocity[1]
        error_vz=0.0-self.vehicle_odometry.velocity[2]

        self.rmse_x+=error_x**2
        self.rmse_y+=error_y**2
        self.rmse_z+=error_z**2

        self.rmse_vx+=error_vx**2
        self.rmse_vy+=error_vy**2
        self.rmse_vz+=error_vz**2

        return(error_x,error_y,error_z,error_vx,error_vy,error_vz)
    
    def tracking_pp_controller(self):


        
        l=list(self.error())


        del_ax=(3*l[0]+5*l[3])
        del_ay=(3*l[1]+5*l[4])
        del_az=(3*l[2]+5*l[5])

        return (del_ax,del_ay,del_az)
    
    def tracking_ppid_controller(self):

        l=list(self.error())

        """
        r_sp : desired position (np.array, shape (3,))
        r_hat: measured/estimated position (np.array, shape (3,))
        v_hat: measured/estimated velocity (np.array, shape (3,))
        return: acceleration setpoint (np.array, shape (3,))
        """

        K_r_xy=0.95
        K_p_xy=1.8                                         #1.8
        K_i_xy=0.4
        K_d_xy=0.2

        K_r_z=1.0
        K_p_z=4.0
        K_i_z=2.0
        K_d_z=0.0


        # --- Position Error ---
        delta_r_x,delta_r_y,delta_r_z=l[0],l[1],l[2]

        


        # --- P-controller for position → velocity setpoint ---
        v_sp_x = K_r_xy * delta_r_x
        v_sp_y = K_r_xy * delta_r_y
        v_sp_z = K_r_z * delta_r_z

        #v_sp = self.saturate(v_sp, self.v_max)
        
        #estimated velocity

        v_hat_x=self.vehicle_odometry.velocity[0]
        v_hat_y=self.vehicle_odometry.velocity[1]
        v_hat_z=self.vehicle_odometry.velocity[2]

        # --- Velocity Error ---
        delta_v_x = v_sp_x - v_hat_x
        delta_v_y = v_sp_y - v_hat_y
        delta_v_z = v_sp_z - v_hat_z

        # --- Integral Term ---
        self.integral_error_x += delta_v_x
        self.integral_error_y += delta_v_y
        self.integral_error_z += delta_v_z

        self.v_hat_prev_x=0.0
        self.v_hat_prev_y=0.0
        self.v_hat_prev_z=0.0


        # --- Derivative Term (backward difference on velocity) ---
        dv_dt_x = (v_hat_x - self.v_hat_prev_x) / self.dt
        dv_dt_y = (v_hat_y - self.v_hat_prev_y) / self.dt
        dv_dt_z = (v_hat_z - self.v_hat_prev_z) / self.dt

    

        """dv_dt_x=0.0
        dv_dt_y=0.0
        dv_dt_z=0.0"""

        # --- Final Acceleration Setpoint ---
        a_sp_x = (K_p_xy * delta_v_x + K_i_xy * self.integral_error_x - K_d_xy * dv_dt_x)
        
        a_sp_y = (K_p_xy * delta_v_y + K_i_xy * self.integral_error_y - K_d_xy * dv_dt_y)
        a_sp_z = (K_p_z * delta_v_z + K_i_z * self.integral_error_z - K_d_z * dv_dt_z)

        # Store velocity setpoint for next cycle (if needed elsewhere)
        self.v_hat_prev_x=v_hat_x
        self.v_hat_prev_y=v_hat_y
        self.v_hat_prev_z=v_hat_z

        a_sp_z_2=3*(self.z_ref-self.vehicle_odometry.position[2])+5*(0.0-self.vehicle_odometry.velocity[2])
        


        return a_sp_x,a_sp_y,a_sp_z

        
 

    def command_loop(self):

        
        self.offboard_control_heartbeat_signal_publisher()

        print(self.takeoff_mode)
        print(self.counter)
        self.counter+=1

        if self.counter<100:
            return
        
        elif self.counter == 100:
            self.engage_offboard_mode()
            self.arm()
            self.takeoff_mode=True

       

        

        if self.takeoff_mode:

            print("yes_okay")
            self.publish_trajectory_wpt(5,5,-5.0)
            
            if (-5.0-0.1 <= self.vehicle_odometry.position[2] <= -5.0+0.1) :
                self.z_ref=self.vehicle_odometry.position[2]
                self.takeoff_mode=False

        
        """if not self.takeoff_mode and self.counter>100 and self.state=="go_to":

            #self.publish_trajectory_v(vx + l[0], vy + l[1], vz + l[2])
            #print(vx,vy,vz)
            #self.publish_trajectory_a(-300.0,0.0,0.0)

            ax,ay,az=self.tracking()
            self.publish_trajectory_a(ax,ay,az)
            #print(self.vehicle_odometry.position)
            
            if self.vehicle_odometry.position[2]>-0.1:
                self.disarm()

        elif not self.takeoff_mode and self.counter>100 and self.state=="track":

            #self.publish_trajectory_v(vx + l[0], vy + l[1], vz + l[2])
            #print(vx,vy,vz)
            #self.publish_trajectory_a(-300.0,0.0,0.0)

            ax,ay,az=self.tracking()
            self.publish_trajectory_a(ax,ay,az)
            #print(self.vehicle_odometry.position)
            
            if self.vehicle_odometry.position[2]>-0.1:
                self.disarm()

        elif not self.takeoff_mode and self.counter>100 and self.state=="track_and_land":

            #self.publish_trajectory_v(vx + l[0], vy + l[1], vz + l[2])
            #print(vx,vy,vz)
            #self.publish_trajectory_a(-300.0,0.0,0.0)

            ax,ay,az=self.tracking()
            self.publish_trajectory_a(ax,ay,az)
            #print(self.vehicle_odometry.position)
            
            if self.vehicle_odometry.position[2]>-0.1:
                self.disarm()"""

            

        


                


        

        if not self.takeoff_mode and self.counter>100:

            #self.publish_trajectory_v(vx + l[0], vy + l[1], vz + l[2])
            #print(vx,vy,vz)
            #self.publish_trajectory_a(-300.0,0.0,0.0)
            
            self.publish_trajectory_wpt(3,0,self.z_ref)

            #print(self.vehicle_odometry.position)
            
            

        position = [self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], self.vehicle_odometry.position[2]]
        orientation = [self.vehicle_odometry.q[0], self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3]]
        
        roll,pitch,yaw=quaternion_to_euler(self.vehicle_odometry.q[0], self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3])

        velocity = [self.vehicle_odometry.velocity[0], self.vehicle_odometry.velocity[1], self.vehicle_odometry.velocity[2]]
        angular_velocity = [self.vehicle_odometry.angular_velocity[0], self.vehicle_odometry.angular_velocity[1], self.vehicle_odometry.angular_velocity[2]]
            

        """if self.vehicle_odometry != None and self.takeoff_mode==False and self.counter>100:
            print(self.t)
            

            e_x,e_y,e_z,e_vx,e_vy,e_vz=self.error()

            e_z=0.0-self.vehicle_odometry.position[2]

            

            if (e_x**2 + e_y**2)<=0.02:

                self.z_ref=0.0

            else :
                self.z_ref=self.vehicle_odometry.position[2]

            

            if self.vehicle_odometry.position[2]>-0.1 and (e_x**2 + e_y**2)<=0.02:
                print("disarm")
                
                
                self.acc_mode=False
                self.disarm()

            current = np.concatenate([[self.t],position,[self.helipad_odometry.pose.pose.position.y,self.helipad_odometry.pose.pose.position.x,0],[e_x,e_y,e_z],velocity,[self.helipad_odometry.twist.twist.linear.y,self.helipad_odometry.twist.twist.linear.x,-self.helipad_odometry.twist.twist.linear.z],[e_vx,e_vy,e_vz],[roll,pitch,yaw],angular_velocity])
            with open("state_data_task_4.csv", mode="a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(current)

            self.t+=0.1


        """
           


            
            

            


        #print("current drone_height= ",self.vehicle_odometry.position[2])
        print("x = ",self.helipad_odometry.pose.pose.position.x)
        print("y = ",self.helipad_odometry.pose.pose.position.y)

        print("\n")

        print("vx = ",self.helipad_odometry.twist.twist.linear.x)
        print("vy = ",self.helipad_odometry.twist.twist.linear.y)

        print("\n")

        


        
        #rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)
    offboard = Offboard()

    try:
        rclpy.spin(offboard)   # Runs until Ctrl+C
    except KeyboardInterrupt:
        print("Stopping with Ctrl+C...")
    finally:
        print("rmse_x = ",math.sqrt(offboard.rmse_x/offboard.t))
        print("rmse_y = ",math.sqrt(offboard.rmse_y/offboard.t))
        print("rmse_z = ",math.sqrt(offboard.rmse_z/offboard.t))

        print("rmse_vx = ",math.sqrt(offboard.rmse_vx/offboard.t))
        print("rmse_vy = ",math.sqrt(offboard.rmse_vy/offboard.t))
        print("rmse_vz = ",math.sqrt(offboard.rmse_vz/offboard.t))


        offboard.destroy_node()

if __name__ == "__main__":
    main()