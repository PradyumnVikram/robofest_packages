#!/usr/bin/env python3
"""
Integrated PX4 Drone Flight and Occupancy Grid Mapping Node
Combines: Flight Control + LiDAR Subscription + Real-time Occupancy Grid Mapping
Updates and visualizes the occupancy grid at 3 Hz
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleCommand,
    VehicleOdometry
)
from sensor_msgs.msg import LaserScan 

import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import time
import os


class IntegratedDroneMappingNode(Node):
    """
    Integrated node for drone flight control and occupancy grid mapping
    """

    def __init__(self):
        super().__init__("integrated_drone_mapping_node")

        # ========== FLIGHT CONTROL PARAMETERS ==========
        self.flight_state = "INIT"
        self.counter = 0
        
        self.mapping_begin=False
        
        self.TAKEOFF_HEIGHT = -1.0
        self.FORWARD_DISTANCE = -5.0
        self.POSITION_THRESHOLD = 0.15
        self.HOVER_TIME = 3.0
        
        self.current_waypoint = None
        self.takeoff_position = None
        self.hover_start_time = None
        self.vehicle_odometry = None

        # ========== OCCUPANCY GRID PARAMETERS ==========
        self.GRID_WIDTH_M = 10.0
        self.GRID_HEIGHT_M = 10.0
        self.RESOLUTION = 0.1
        self.GRID_WIDTH_CELLS = int(self.GRID_WIDTH_M / self.RESOLUTION)
        self.GRID_HEIGHT_CELLS = int(self.GRID_HEIGHT_M / self.RESOLUTION)
        
        self.GRID_ORIGIN_X_CELL = self.GRID_WIDTH_CELLS // 2
        self.GRID_ORIGIN_Y_CELL = self.GRID_HEIGHT_CELLS // 2
        
        self.LOG_ODDS_OCC = 2.2
        self.LOG_ODDS_FREE = -0.7
        self.LOG_ODDS_MAX = 5.0
        self.LOG_ODDS_MIN = -5.0
        self.MAX_RANGE = 20
        
        # Initialize occupancy grid
        self.occupancy_grid = np.zeros((self.GRID_HEIGHT_CELLS, self.GRID_WIDTH_CELLS))
        
        # LiDAR scan data storage
        self.latest_scan_data = None
        self.scan_count = 0
        self.update_count = 0
        self.output_dir = os.path.expanduser("~/occupancy_maps")
        os.makedirs(self.output_dir, exist_ok=True)

        # ========== QOS PROFILES ==========
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ========== PUBLISHERS (Flight Control) ==========
        self.offboard_mode_publisher = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            qos_profile
        )
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos_profile
        )
        
        self.trajectory_publisher = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            qos_profile
        )

        # ========== SUBSCRIBERS ==========
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.vehicle_odometry_callback,
            qos_profile
        )
        
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            "/scan",
            self.laser_callback,
            10
        )

        # ========== TIMERS ==========
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.mapping_iterations = 0
        self.mapping_timer = self.create_timer(0.1, self.mapping_loop)  # 3 Hz
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Integrated Drone Mapping Node Initialized")
        self.get_logger().info(f"Grid: {self.GRID_WIDTH_CELLS}x{self.GRID_HEIGHT_CELLS} cells")
        self.get_logger().info(f"Mapping update rate: 3 Hz")
        self.get_logger().info("=" * 60)

    # ============= CALLBACK FUNCTIONS =============
    
    def vehicle_odometry_callback(self, msg):
        """Store latest odometry data"""
        self.vehicle_odometry = msg

    def laser_callback(self, msg):
        """Process incoming LiDAR scan data"""
        self.latest_scan_data = msg
        
        
        # Log scan info periodically
        if self.scan_count % 10 == 0:
            self.get_logger().info(
                f"LiDAR: {len(msg.ranges)} beams, "
                f"Range: [{msg.range_min:.2f}, {msg.range_max:.2f}]m"
            )
        self.scan_count += 1

    # ============= FLIGHT CONTROL FUNCTIONS =============
    
    def publish_offboard_control_mode(self):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle command"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_publisher.publish(msg)

    def engage_offboard_mode(self):
        """Switch to offboard mode"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Engaging Offboard Mode")

    def arm(self):
        """Arm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arming Vehicle")

    def disarm(self):
        """Disarm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarming Vehicle")

    def publish_position_setpoint(self, x, y, z, yaw=0.0):
        """Publish position setpoint"""
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
        msg.yaw = yaw
        msg.yawspeed = float('nan')
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_publisher.publish(msg)

    def get_position_error(self, target_x, target_y, target_z):
        """Calculate 3D distance from current position to target"""
        if self.vehicle_odometry is None:
            return float('inf')
        
        dx = target_x - self.vehicle_odometry.position[0]
        dy = target_y - self.vehicle_odometry.position[1]
        dz = target_z - self.vehicle_odometry.position[2]
        
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def at_position(self, target_x, target_y, target_z, threshold=None):
        """Check if drone is at target position"""
        if threshold is None:
            threshold = self.POSITION_THRESHOLD
        
        error = self.get_position_error(target_x, target_y, target_z)
        return error < threshold

    # ============= OCCUPANCY GRID MAPPING FUNCTIONS =============
    
    def world_to_grid(self, wx, wy):
        """Convert world coordinates to grid cell coordinates"""
        gx = int((wx / self.RESOLUTION) + self.GRID_ORIGIN_X_CELL)
        gy = int((wy / self.RESOLUTION) + self.GRID_ORIGIN_Y_CELL)
        return gx, gy

    def is_in_grid_bounds(self, gx, gy):
        """Check if grid cell is within boundaries"""
        return 0 <= gx < self.GRID_WIDTH_CELLS and 0 <= gy < self.GRID_HEIGHT_CELLS

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for raycasting"""
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            yield (x0, y0)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def update_occupancy_grid(self, robot_pose, scan_data):
        """Update occupancy grid with LiDAR scan data"""
        robot_x, robot_y, robot_theta = robot_pose
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        valid_updates = 0
        for angle, range_m in scan_data:
            global_angle = robot_theta + angle

            print(range_m)

            if range_m == float('inf') or np.isnan(range_m):
            
                range_m = self.MAX_RANGE

            # Calculate hit point in world coordinates

            print(range_m)

            hit_x = robot_x - (range_m * math.sin(global_angle))
            hit_y = robot_y - (range_m * math.cos(global_angle))
            
            hit_gx, hit_gy = self.world_to_grid(hit_x, hit_y)
            
            """if not self.is_in_grid_bounds(hit_gx, hit_gy):
                continue"""
            
            
            # Raycasting with Bresenham's
            line_cells = list(self.bresenham_line(robot_gx, robot_gy, hit_gx, hit_gy))
            
            # Mark cells before hit as FREE
            for (gx, gy) in line_cells[:-1]:
                if self.is_in_grid_bounds(gx, gy):
                    self.occupancy_grid[gy, gx] += self.LOG_ODDS_FREE
                    self.occupancy_grid[gy, gx] = max(self.LOG_ODDS_MIN, self.occupancy_grid[gy, gx])
            
            # Mark hit cell as OCCUPIED
            if self.is_in_grid_bounds(hit_gx, hit_gy):
                self.occupancy_grid[hit_gy, hit_gx] += self.LOG_ODDS_OCC
                self.occupancy_grid[hit_gy, hit_gx] = min(self.LOG_ODDS_MAX, self.occupancy_grid[hit_gy, hit_gx])
            valid_updates += 1
        
        return valid_updates

    def convert_to_three_state_matrix(self):
        """Convert log-odds grid to three-state matrix"""
        prob_grid = 1.0 - 1.0 / (1.0 + np.exp(self.occupancy_grid))
        
        FREE_THRESH = 0.2
        OCC_THRESH = 0.8
        
        three_state_matrix = np.full(prob_grid.shape, -1, dtype=int)
        three_state_matrix[prob_grid < FREE_THRESH] = 0
        three_state_matrix[prob_grid > OCC_THRESH] = 1
        
        return three_state_matrix

    def visualize_grid(self, robot_pose=None, iteration=0):
        """Visualize occupancy grid with 3-color map"""
        prob_grid = 1.0 - 1.0 / (1.0 + np.exp(self.occupancy_grid))
        
        FREE_THRESH = 0.2
        OCC_THRESH = 0.8
        
        display_grid = np.full(prob_grid.shape, 0.5)
        display_grid[prob_grid < FREE_THRESH] = 0.0
        display_grid[prob_grid > OCC_THRESH] = 1.0

        cmap = ListedColormap(['white', 'gray', 'black'])
        plt.ion()

        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(display_grid, cmap=cmap, vmin=0.0, vmax=1.0, interpolation='nearest')
        ax.set_title(f"Occupancy Grid Map - Update #{iteration}")

        height, width = prob_grid.shape
        ax.set_xticks(np.arange(width) - 0.5, minor=True)
        ax.set_yticks(np.arange(height) - 0.5, minor=True)
        ax.grid(which='minor', color='black', linestyle='-', linewidth=0.5)

        if robot_pose is not None:
            robot_x, robot_y, _ = robot_pose
            robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
            ax.plot(robot_gx, robot_gy, 'ro', markersize=12, label='Robot')
            ax.legend()

        ax.set_xlabel("X (cells)")
        ax.set_ylabel("Y (cells)")
        ax.set_xticks(np.arange(0, width, 10))
        ax.set_yticks(np.arange(0, height, 10))
        
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.01)
        
       
    # ============= MAIN CONTROL LOOPS =============
    
    def control_loop(self):
        """Flight control loop - 10 Hz"""
        self.publish_offboard_control_mode()
        self.counter += 1
        
        if self.flight_state == "INIT":
            if self.counter < 100:
                return
            
            self.engage_offboard_mode()
            self.arm()
            self.flight_state = "ARMING"
            self.get_logger().info("State: ARMING")
            
        elif self.flight_state == "ARMING":
            if self.counter < 110:
                return
            
            if self.vehicle_odometry is not None:
                self.takeoff_position = {
                    'x': self.vehicle_odometry.position[0],
                    'y': self.vehicle_odometry.position[1],
                    'z': self.vehicle_odometry.position[2]
                }
                self.get_logger().info(f"Takeoff position: {self.takeoff_position}")
            
            self.current_waypoint = {
                'x': self.takeoff_position['x'],
                'y': self.takeoff_position['y'],
                'z': self.TAKEOFF_HEIGHT
            }
            
            self.flight_state = "TAKEOFF"
            self.get_logger().info(f"State: TAKEOFF to {-self.TAKEOFF_HEIGHT}m")
            
        elif self.flight_state == "TAKEOFF":
            self.publish_position_setpoint(
                self.current_waypoint['x'],
                self.current_waypoint['y'],
                self.current_waypoint['z'],
                yaw=0.0
            )
            
            if self.at_position(
                self.current_waypoint['x'],
                self.current_waypoint['y'],
                self.current_waypoint['z']
            ):
                self.current_waypoint = {
                    'x': self.takeoff_position['x'] + self.FORWARD_DISTANCE,
                    'y': self.takeoff_position['y'],
                    'z': self.TAKEOFF_HEIGHT
                }
                
                self.flight_state = "FORWARD"
                self.get_logger().info(f"State: FORWARD - Moving {self.FORWARD_DISTANCE}m")
                
        elif self.flight_state == "FORWARD":
            self.publish_position_setpoint(
                self.current_waypoint['x'],
                self.current_waypoint['y'],
                self.current_waypoint['z'],
                yaw=0.0
            )
            
            if self.at_position(
                self.current_waypoint['x'],
                self.current_waypoint['y'],
                self.current_waypoint['z']
            ):
                self.hover_start_time = self.get_clock().now()
                self.flight_state = "HOVER"
                self.get_logger().info(f"State: HOVER for {self.HOVER_TIME}s")
                
        elif self.flight_state == "HOVER":
            self.publish_position_setpoint(
                self.current_waypoint['x'],
                self.current_waypoint['y'],
                self.current_waypoint['z'],
                yaw=0.0
            )
            
            elapsed_time = (self.get_clock().now() - self.hover_start_time).nanoseconds / 1e9
            if elapsed_time >= self.HOVER_TIME:
                self.current_waypoint = {
                    'x': self.current_waypoint['x'],
                    'y': self.current_waypoint['y'],
                    'z': 0.0
                }
                
                self.flight_state = "LANDING"
                self.get_logger().info("State: LANDING")
                
        elif self.flight_state == "LANDING":
            self.publish_position_setpoint(
                self.current_waypoint['x'],
                self.current_waypoint['y'],
                self.current_waypoint['z'],
                yaw=0.0
            )
            
            if self.vehicle_odometry is not None:
                if self.vehicle_odometry.position[2] > -0.2:
                    self.disarm()
                    self.flight_state = "LANDED"
                    self.get_logger().info("State: LANDED - Mission Complete!")
                    
        elif self.flight_state == "LANDED":
            pass

    def mapping_loop(self):
        """Mapping loop - 3 Hz (updates every 0.33 seconds)"""
        if self.latest_scan_data is None or self.vehicle_odometry is None:
            self.get_logger().warn("Waiting for LiDAR and odometry data...")
            return
        
        

        # Get current robot pose
        robot_x = self.vehicle_odometry.position[0]
        robot_y = self.vehicle_odometry.position[1]
        # For theta, we could extract from quaternion, but for simplicity using 0
        robot_theta = 0.0
        
        robot_pose = (robot_x, robot_y, robot_theta)

        # Convert LaserScan to scan_data formatbui
        scan_data = []
        angle = self.latest_scan_data.angle_min
        for range_m in self.latest_scan_data.ranges:
            scan_data.append((angle, range_m))
            angle += self.latest_scan_data.angle_increment

        # Update occupancy grid

        
        if  self.flight_state == "FORWARD" and not self.mapping_begin:
            self.mapping_begin=True

        print(self.mapping_begin)

        if self.mapping_begin:

            self.mapping_iterations += 1
            iteration = self.mapping_iterations

            

            self.get_logger().info(f"Updating occupancy grid with {len(scan_data)} beams...")
            self.update_occupancy_grid(robot_pose, scan_data)

            # Convert to three-state matrix
            three_state_matrix = self.convert_to_three_state_matrix()
            
            # Log statistics
            occupied = np.sum(three_state_matrix == 1)
            free = np.sum(three_state_matrix == 0)
            unexplored = np.sum(three_state_matrix == -1)
            
            # self.get_logger().info("=" * 60)
            # self.get_logger().info(f"Grid Update #{iteration}")
            # self.get_logger().info(f"  Occupied cells: {occupied}")
            # self.get_logger().info(f"  Free cells: {free}")
            # self.get_logger().info(f"  Unexplored cells: {unexplored}")
            # self.get_logger().info("=" * 60)

            # Visualize
            self.visualize_grid(robot_pose, iteration)
        
            # Print a portion of the three-state matrix (first 10x10)
            # self.get_logger().info("Three-state matrix sample :")
            for row in range(100):
                row_str = "".join(f"{three_state_matrix[row, col]:2d}" for col in range(100))
                #print(row_str)
            if iteration >= 10:
                # self.get_logger().info("Done 3 mapping iterations, stopping mapping timer.")
                self.mapping_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    
    node = IntegratedDroneMappingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down with Ctrl+C...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()