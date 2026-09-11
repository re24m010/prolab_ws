#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
import matplotlib.pyplot as plt
import time
import os

class EvaluationNode(Node):
    def __init__(self):
        super().__init__('evaluation_node')
        
        # 1. Ground Truth aus dem Simulator
        self.create_subscription(Odometry, '/odom', self.gt_callback, 10)
        
        # 2. Verrauschte Messung (zum Vergleich)
        self.create_subscription(Odometry, '/odom_noisy', self.noisy_callback, 10)
        
        # 3. Filter-Schätzungen (PoseWithCovarianceStamped)
        self.create_subscription(PoseWithCovarianceStamped, '/estimated_pose_kf', self.kf_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/estimated_pose_ekf', self.ekf_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/estimated_pose_pf', self.pf_callback, 10)
        
        self.gt_pose = None
        self.noisy_pose = None
        self.kf_pose = None
        self.ekf_pose = None
        self.pf_pose = None
        
        self.timestamps = []
        self.errors_noisy = []
        self.errors_kf = []
        self.errors_ekf = []
        self.errors_pf = []

        # Posenverläufe für 2D-Trajektorienplot
        self.traj_gt_x, self.traj_gt_y = [], []
        self.traj_kf_x, self.traj_kf_y = [], []
        self.traj_ekf_x, self.traj_ekf_y = [], []
        self.traj_pf_x, self.traj_pf_y = [], []
        
        self.start_time = time.time()
        self.create_timer(0.1, self.log_data)
        self.get_logger().info('Evaluierungs-Node aktiv. Zeichne Fehlerdaten gegen /odom auf...')

    def gt_callback(self, msg):
        self.gt_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def noisy_callback(self, msg):
        self.noisy_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def kf_callback(self, msg):
        self.kf_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def ekf_callback(self, msg):
        self.ekf_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def pf_callback(self, msg):
        self.pf_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def compute_distance(self, p1, p2):
        if p1 is None or p2 is None:
            return None
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def log_data(self):
        if self.gt_pose is None:
            return
            
        t = time.time() - self.start_time
        err_noisy = self.compute_distance(self.gt_pose, self.noisy_pose)
        err_kf = self.compute_distance(self.gt_pose, self.kf_pose)
        err_ekf = self.compute_distance(self.gt_pose, self.ekf_pose)
        err_pf = self.compute_distance(self.gt_pose, self.pf_pose)
        
        if err_kf is not None and err_ekf is not None and err_pf is not None:
            self.timestamps.append(t)
            self.errors_noisy.append(err_noisy if err_noisy is not None else 0.0)
            self.errors_kf.append(err_kf)
            self.errors_ekf.append(err_ekf)
            self.errors_pf.append(err_pf)

            # Trajektorienpunkte sichern
            self.traj_gt_x.append(self.gt_pose[0])
            self.traj_gt_y.append(self.gt_pose[1])
            self.traj_kf_x.append(self.kf_pose[0])
            self.traj_kf_y.append(self.kf_pose[1])
            self.traj_ekf_x.append(self.ekf_pose[0])
            self.traj_ekf_y.append(self.ekf_pose[1])
            self.traj_pf_x.append(self.pf_pose[0])
            self.traj_pf_y.append(self.pf_pose[1])

    def save_and_plot(self):
        self.get_logger().info('Berechne RMSE und erstelle Plots...')
        
        def rmse(err_list):
            if not err_list: return 0.0
            return math.sqrt(sum(e**2 for e in err_list) / len(err_list))
            
        rmse_noisy = rmse(self.errors_noisy)
        rmse_kf = rmse(self.errors_kf)
        rmse_ekf = rmse(self.errors_ekf)
        rmse_pf = rmse(self.errors_pf)
        
        print("\n" + "="*45)
        print("         EXPERIMENT ERGEBNISSE (RMSE)        ")
        print("="*45)
        print(f"Sensor-Rohdaten (/odom_noisy) : {rmse_noisy:.4f} m")
        print(f"Kalman Filter (KF)            : {rmse_kf:.4f} m")
        print(f"Extended Kalman Filter (EKF)  : {rmse_ekf:.4f} m")
        print(f"Partikelfilter (PF)           : {rmse_pf:.4f} m")
        print("="*45 + "\n")
        
        home_dir = os.path.expanduser('~')
        
        # 1. Fehler-über-Zeit Plot (Englisch beschriftet)
        plt.figure(figsize=(10, 5))
        plt.plot(self.timestamps, self.errors_noisy, 'gray', linestyle='--', alpha=0.6, label=f'Noisy Input (RMSE: {rmse_noisy:.3f}m)')
        plt.plot(self.timestamps, self.errors_kf, 'g-', label=f'KF (RMSE: {rmse_kf:.3f}m)')
        plt.plot(self.timestamps, self.errors_ekf, 'b-', label=f'EKF (RMSE: {rmse_ekf:.3f}m)')
        plt.plot(self.timestamps, self.errors_pf, 'y-', label=f'PF (RMSE: {rmse_pf:.3f}m)')
        plt.title('Localization Error Relative to Ground Truth')
        plt.xlabel('Time [s]')
        plt.ylabel('Absolute Position Error [m]')
        plt.legend()
        plt.grid(True)
        plot_error_path = os.path.join(home_dir, 'filter_error_plot.png')
        plt.savefig(plot_error_path)
        plt.close()

        # 2. 2D Trajektorien-Vergleich (Englisch beschriftet)
        plt.figure(figsize=(8, 8))
        plt.plot(self.traj_gt_x, self.traj_gt_y, 'k-', linewidth=2, label='Ground Truth (/odom)')
        plt.plot(self.traj_kf_x, self.traj_kf_y, 'g--', label='KF')
        plt.plot(self.traj_ekf_x, self.traj_ekf_y, 'b:', label='EKF')
        plt.plot(self.traj_pf_x, self.traj_pf_y, 'y-.', label='PF')
        plt.title('2D Trajectory Comparison')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.axis('equal')
        plt.legend()
        plt.grid(True)
        plot_traj_path = os.path.join(home_dir, 'filter_trajectory_plot.png')
        plt.savefig(plot_traj_path)
        plt.close()

        self.get_logger().info(f'Diagramme gespeichert in {home_dir}:')
        self.get_logger().info(f' - filter_error_plot.png')
        self.get_logger().info(f' - filter_trajectory_plot.png')

def main(argc=None):
    rclpy.init(args=argc)
    node = EvaluationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_and_plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()