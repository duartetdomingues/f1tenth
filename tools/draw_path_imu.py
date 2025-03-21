import numpy as np
import matplotlib.pyplot as plt
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUPathPlotter(Node):
    def __init__(self):
        super().__init__('imu_path_plotter')
        self.subscription1 = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback1,
            10
        )
        self.subscription2 = self.create_subscription(
            Imu,
            'vesc/imu/raw',
            self.imu_callback2,
            10
        )
        self.x, self.y = 0, 0  # Position
        self.vx, self.vy = 0, 0  # Velocity
        self.theta = 0  # Orientation
        self.path = [(self.x, self.y)]
        self.last_time = self.get_clock().now()
        
        # Complementary filter parameters
        self.alpha = 0.98  # Filter coefficient
        self.filtered_ax = 0
        self.filtered_ay = 0
        self.filtered_wz = 0
    
    def imu_callback1(self, msg):
        """Update position based on IMU1 readings"""
        self.process_imu_data(msg)
    
    def imu_callback2(self, msg):
        """Update position based on IMU2 readings"""
        self.process_imu_data(msg)
    
    def process_imu_data(self, msg):
        """Process IMU data and update position"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        wz = msg.angular_velocity.z
        
        # Apply complementary filter
        self.filtered_ax = self.alpha * self.filtered_ax + (1 - self.alpha) * ax
        self.filtered_ay = self.alpha * self.filtered_ay + (1 - self.alpha) * ay
        self.filtered_wz = self.alpha * self.filtered_wz + (1 - self.alpha) * wz
        
        self.theta += self.filtered_wz * dt  # Integrate angular velocity
        
        # Rotate acceleration to global frame
        ax_global = self.filtered_ax * np.cos(self.theta) - self.filtered_ay * np.sin(self.theta)
        ay_global = self.filtered_ax * np.sin(self.theta) + self.filtered_ay * np.cos(self.theta)
        
        # Integrate acceleration to get velocity
        self.vx += ax_global * dt
        self.vy += ay_global * dt
        
        # Integrate velocity to get position
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        self.path.append((self.x, self.y))
        self.plot()
    
    def plot(self):
        """Plot the estimated path"""
        plt.clf()
        path = np.array(self.path)
        plt.plot(path[:, 0], path[:, 1], '-b')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Estimated Path from IMU')
        plt.pause(0.01)

if __name__ == "__main__":
    rclpy.init()
    imu_plotter = IMUPathPlotter()
    rclpy.spin(imu_plotter)
    imu_plotter.destroy_node()
    rclpy.shutdown()
    plt.show()