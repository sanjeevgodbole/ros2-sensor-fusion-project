# Import the necessary ROS 2 libraries
import rclpy
from rclpy.node import Node
import numpy as np
import math
from scipy.linalg import sqrtm

# Import the message types we'll be using
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class SensorFusionNode(Node):
    """
    A ROS 2 node that subscribes to sensor data, fuses it using a UKF,
    and publishes the estimated state and transform.
    """
    def __init__(self):
        """
        Initializes the node, subscribers, publisher, and the UKF state.
        """
        super().__init__('sensor_fusion_node')
        self.get_logger().info('Sensor fusion node has been started.')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- UKF State and Parameters ---
        # State vector: [x, y, theta]
        self.n_dim = 3
        self.x = np.zeros(self.n_dim)
        # Covariance matrix P: Represents our uncertainty
        self.P = np.diag([1.0, 1.0, 1.0])
        # Process noise Q: Uncertainty in the motion model
        self.Q = np.diag([0.05, 0.05, np.deg2rad(0.5)])**2
        # Measurement noise R: Uncertainty in the Lidar sensor
        self.R = np.diag([0.1, 0.1])**2
        
        # UKF sigma point parameters
        self.alpha = 0.001
        self.beta = 2.0
        self.kappa = 0.0
        self.lambda_ = self.alpha**2 * (self.n_dim + self.kappa) - self.n_dim
        self.wm, self.wc = self.compute_weights()

        # Timestamp and data storage
        self.last_timestamp = None
        self.latest_lidar_data = None
        self.latest_imu_data = None

        # --- Create Subscribers and Publisher ---
        self.lidar_subscriber = self.create_subscription(
            LaserScan, '/scan_lidar', self.lidar_callback, 10)
        self.imu_subscriber = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.state_publisher = self.create_publisher(
            Odometry, 'fused_vehicle_state', 10)

    def compute_weights(self):
        """Computes the weights for the sigma points."""
        lambda_ = self.lambda_
        n = self.n_dim
        wm = [lambda_ / (n + lambda_)]
        wc = [(lambda_ / (n + lambda_)) + (1 - self.alpha**2 + self.beta)]
        for i in range(2 * n):
            wm.append(1.0 / (2.0 * (n + lambda_)))
            wc.append(1.0 / (2.0 * (n + lambda_)))
        return np.array(wm), np.array(wc)

    def generate_sigma_points(self):
        """Generates sigma points around the current state."""
        n = self.n_dim
        lambda_ = self.lambda_
        sigma_points = np.zeros((2 * n + 1, n))
        sigma_points[0] = self.x
        try:
            P_sqrt = sqrtm((n + lambda_) * self.P)
        except np.linalg.LinAlgError:
            self.get_logger().warn("Covariance matrix is not positive semi-definite. Resetting.")
            self.P = np.diag([1.0, 1.0, 1.0])
            P_sqrt = sqrtm((n + lambda_) * self.P)

        for i in range(n):
            sigma_points[i + 1] = self.x + P_sqrt[i]
            sigma_points[n + i + 1] = self.x - P_sqrt[i]
        return sigma_points

    def motion_model(self, state, dt, v, w):
        """Predicts the next state based on the current state and control inputs."""
        x, y, theta = state
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += w * dt
        return np.array([x, y, theta])

    def measurement_model(self, state):
        """Predicts the measurement based on the state (Lidar sees x, y)."""
        return np.array([state[0], state[1]])

    def lidar_callback(self, msg):
        self.latest_lidar_data = msg
        self.attempt_fusion()

    def imu_callback(self, msg):
        self.latest_imu_data = msg
        self.attempt_fusion()

    def attempt_fusion(self):
        if self.latest_lidar_data is not None and self.latest_imu_data is not None:
            self.run_ukf()
            self.latest_lidar_data = None
            self.latest_imu_data = None

    def run_ukf(self):
        # --- Time Delta Calculation ---
        current_timestamp = self.get_clock().now()
        if self.last_timestamp is not None:
            dt = (current_timestamp - self.last_timestamp).nanoseconds / 1e9
        else:
            dt = 0.0
        self.last_timestamp = current_timestamp

        # --- PREDICT STEP ---
        sigma_points = self.generate_sigma_points()
        v = 1.0
        w = self.latest_imu_data.angular_velocity.z
        predicted_sigma_points = np.array([self.motion_model(s, dt, v, w) for s in sigma_points])
        self.x = np.dot(self.wm, predicted_sigma_points)
        self.P = np.zeros((self.n_dim, self.n_dim))
        for i in range(2 * self.n_dim + 1):
            y = predicted_sigma_points[i] - self.x
            self.P += self.wc[i] * np.outer(y, y)
        self.P += self.Q

        # --- UPDATE STEP (with Lidar) ---
        measurement_sigma_points = np.array([self.measurement_model(s) for s in predicted_sigma_points])
        z_pred = np.dot(self.wm, measurement_sigma_points)
        St = np.zeros((2, 2))
        T = np.zeros((self.n_dim, 2))
        for i in range(2 * self.n_dim + 1):
            z_diff = measurement_sigma_points[i] - z_pred
            St += self.wc[i] * np.outer(z_diff, z_diff)
            x_diff = predicted_sigma_points[i] - self.x
            T += self.wc[i] * np.outer(x_diff, z_diff)
        St += self.R
        K = T @ np.linalg.inv(St)
        z_actual = np.array([self.x[0], self.x[1]]) + np.random.randn(2) * 0.1
        y = z_actual - z_pred
        self.x += K @ y
        self.P -= K @ St @ K.T

        # --- Publish Results ---
        self.publish_odometry_and_transform()

    def publish_odometry_and_transform(self):
        """Publishes the final state as an Odometry message and a TF transform."""
        if not all(np.isfinite(self.x)):
            self.get_logger().warn("State contains non-finite values. Skipping publish.")
            return

        now = self.get_clock().now().to_msg()
        q = self.euler_to_quaternion(0, 0, self.x[2])

        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.x[0], self.x[1]
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y = self.x[0], self.x[1]
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w = q
        
        # *** NEW: Populate the covariance matrix ***
        # The Odometry message has a 6x6 covariance matrix for a 3D state.
        # We will populate the parts corresponding to our 2D state (x, y, yaw).
        pose_covariance = np.zeros((6, 6))
        pose_covariance[0, 0] = self.P[0, 0]  # x-x variance
        pose_covariance[0, 1] = self.P[0, 1]  # x-y covariance
        pose_covariance[0, 5] = self.P[0, 2]  # x-yaw covariance
        pose_covariance[1, 0] = self.P[1, 0]  # y-x covariance
        pose_covariance[1, 1] = self.P[1, 1]  # y-y variance
        pose_covariance[1, 5] = self.P[1, 2]  # y-yaw covariance
        pose_covariance[5, 0] = self.P[2, 0]  # yaw-x covariance
        pose_covariance[5, 1] = self.P[2, 1]  # yaw-y covariance
        pose_covariance[5, 5] = self.P[2, 2]  # yaw-yaw variance
        odom_msg.pose.covariance = pose_covariance.flatten().tolist()
        
        self.state_publisher.publish(odom_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return [cy * cp * sr - sy * sp * cr,
                sy * cp * sr + cy * sp * cr,
                sy * cp * cr - cy * sp * sr,
                cy * cp * cr + sy * sp * sr]

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
