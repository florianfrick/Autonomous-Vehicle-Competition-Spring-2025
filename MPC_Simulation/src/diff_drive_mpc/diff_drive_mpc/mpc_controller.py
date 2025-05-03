import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import cvxpy as cp
from diff_drive_mpc.utils import discretize_linear_model, generate_square_ref, compute_linearization

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.path_pub = self.create_publisher(Path, '/ref_path', 10)

        self.x = np.array([0.0, 0.0, 0.0])
        waypoints = np.array([[0, 0], [5, 0], [5, 5], [0, 5], [0, 0]])
        self.traj = generate_square_ref(300, 0.1, waypoints)  

        self.N = 15
        self.dt = 0.1
        self.Q = np.diag([20, 20, 1])
        self.R = np.diag([0.1, 0.05])
        self.Qf = self.Q

        self.step = 0
        self.last_ref_idx = 0

        self.create_timer(0.5, self.publish_reference_path_once) 
        self.timer = self.create_timer(0.1, self.control_loop)

    def publish_reference_path_once(self):
        self.publish_reference_path()
        self.get_logger().info("Published reference path")
        self.publish_reference_path = lambda: None  # disable future calls
    
    def publish_reference_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        for pose in self.traj:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'odom'
            pose_stamped.pose.position.x = pose[0]
            pose_stamped.pose.position.y = pose[1]
            pose_stamped.pose.orientation.z = np.sin(pose[2] / 2.0)
            pose_stamped.pose.orientation.w = np.cos(pose[2] / 2.0)
            path_msg.poses.append(pose_stamped)
        self.path_pub.publish(path_msg)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z)
        cosy_cosp = 1.0 - 2.0 * (q.z * q.z)
        theta = np.atan2(siny_cosp, cosy_cosp)
        self.x = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            theta
        ])

    # def control_loop(self):
    #     # t = min(len(self.traj) - self.N - 1, int(self.get_clock().now().nanoseconds * 1e-9 / self.dt))
    #     # x_ref = self.traj[t:t+self.N+1]

    #     # t = min(len(self.traj) - self.N - 1, self.step)
    #     # x_ref = self.traj[t:t+self.N+1]
    #     # self.step += 1

    #     # # Find nearest point in trajectory to current pose
    #     # dists = np.linalg.norm(self.traj[:, :2] - self.x[:2], axis=1)
    #     # nearest_idx = np.argmin(dists)
    #     # t = min(nearest_idx, len(self.traj) - self.N - 1)
    #     # x_ref = self.traj[t:t+self.N+1]

    #     # Search ahead for the nearest point
    #     search_range = self.traj[self.last_ref_idx : self.last_ref_idx + 50]
    #     search_indices = np.arange(self.last_ref_idx, self.last_ref_idx + 50)

    #     if len(search_range) < self.N + 1:
    #         return  # End of path

    #     # Compute distance to all future points
    #     dists = np.linalg.norm(search_range[:, :2] - self.x[:2], axis=1)
    #     min_idx_local = np.argmin(dists)
    #     nearest_idx = search_indices[min_idx_local]

    #     # Update tracker
    #     self.last_ref_idx = nearest_idx

    #     # Set reference horizon
    #     t = min(nearest_idx, len(self.traj) - self.N - 1)
    #     x_ref = self.traj[t:t + self.N + 1]


    #     # Use relinearization at current reference theta and v
    #     # theta_r = x_ref[0][2]
    #     # v_r = 0.5  # assumed constant nominal velocity
    #     # A, B = compute_linearization(v_r, theta_r)
    #     # A_d, B_d = discretize_linear_model(A, B, self.dt)

    #     x_var = cp.Variable((3, self.N + 1))
    #     u_var = cp.Variable((2, self.N))

    #     cost = 0
    #     constraints = [x_var[:, 0] == self.x]
    #     for k in range(self.N):
    #         theta_r = x_ref[k][2]
    #         v_r = np.linalg.norm(x_ref[k+1][:2] - x_ref[k][:2]) / self.dt
    #         A, B = compute_linearization(v_r, theta_r)
    #         A_d, B_d = discretize_linear_model(A, B, self.dt)
    #         cost += cp.quad_form(x_var[:, k] - x_ref[k], self.Q)
    #         cost += cp.quad_form(u_var[:, k], self.R)
    #         constraints += [u_var[0, k] >= 0.0]
    #         constraints += [x_var[:, k+1] == A_d @ x_var[:, k] + B_d @ u_var[:, k]]
    #         constraints += [cp.norm(u_var[:,k], 'inf') <= 2.0]

    #     cost += cp.quad_form(x_var[:, self.N] - x_ref[self.N], self.Qf)
    #     cp.Problem(cp.Minimize(cost), constraints).solve(solver=cp.OSQP)

    #     # print("Current state:", self.x)
    #     # print("First ref:", x_ref[0])
    #     # print("Initial control guess:", u_var.value[:, 0] if u_var.value is not None else None)

    #     cmd = Twist()
    #     cmd.linear.x = u_var.value[0, 0]
    #     cmd.angular.z = u_var.value[1, 0]
    #     self.publisher_.publish(cmd)

    def control_loop(self):
        # ---- Find nearest point ahead on trajectory ----
        search_window = 50
        lookahead = 2
        max_idx = len(self.traj) - self.N - 1

        # Clip to available future points
        search_end = min(self.last_ref_idx + search_window, len(self.traj))
        search_range = self.traj[self.last_ref_idx:search_end]
        search_indices = np.arange(self.last_ref_idx, search_end)

        if len(search_range) < self.N + 1:
            self.get_logger().info("End of trajectory reached.")
            return

        # Find closest future point to current position
        dists = np.linalg.norm(search_range[:, :2] - self.x[:2], axis=1)
        min_idx_local = np.argmin(dists)
        nearest_idx = search_indices[min_idx_local]
        self.last_ref_idx = nearest_idx  # Advance forward

        # Apply lookahead to avoid tracking a point just behind
        t = min(nearest_idx + lookahead, max_idx)
        x_ref = self.traj[t:t + self.N + 1]

        # ---- Setup Optimization Problem ----
        x_var = cp.Variable((3, self.N + 1))
        u_var = cp.Variable((2, self.N))

        cost = 0
        constraints = [x_var[:, 0] == self.x]

        for k in range(self.N):
            # Local reference heading and velocity
            theta_r = x_ref[k][2]
            v_r = np.linalg.norm(x_ref[k + 1][:2] - x_ref[k][:2]) / self.dt

            # Time-varying linearization
            A_k, B_k = compute_linearization(v_r, theta_r)
            A_dk, B_dk = discretize_linear_model(A_k, B_k, self.dt)

            cost += cp.quad_form(x_var[:, k] - x_ref[k], self.Q)
            cost += cp.quad_form(u_var[:, k], self.R)
            # constraints += [u_var[0, k] >= 0.0]
            constraints += [x_var[:, k + 1] == A_dk @ x_var[:, k] + B_dk @ u_var[:, k]]
            constraints += [cp.norm(u_var[:, k], 'inf') <= 2.0]

        cost += cp.quad_form(x_var[:, self.N] - x_ref[self.N], self.Qf)

        # Solve
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP)

        # ---- Send Control ----
        cmd = Twist()
        if u_var.value is not None:
            cmd.linear.x = u_var.value[0, 0]
            cmd.angular.z = u_var.value[1, 0]
        else:
            self.get_logger().warn("MPC solver failed. Sending zero command.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.publisher_.publish(cmd)

    # def control_loop(self):
    #     if self.step + self.N + 1 >= len(self.traj):
    #         self.get_logger().info("Reached end of trajectory.")
    #         return
        
    #     dists = np.linalg.norm(self.traj[:, :2] - self.x[:2], axis=1)
    #     nearest_idx = np.argmin(dists)
    #     t = min(nearest_idx, len(self.traj) - self.N - 1)
    #     x_ref = self.traj[t : t + self.N + 1]
    #     # Reference input: nominal forward velocity
    #     u_ref = np.tile(np.array([0.5, 0.0]), (self.N, 1))  # shape (N, 2)


    #     # Linearize at the heading of the current reference point
    #     theta_r = x_ref[0][2]
    #     # theta_r = self.x[2]
    #     A, B = compute_linearization(0.5, theta_r)
    #     A_d, B_d = discretize_linear_model(A, B, self.dt)

    #     # Setup optimization problem
    #     x_var = cp.Variable((3, self.N + 1))
    #     u_var = cp.Variable((2, self.N))
    #     cost = 0
    #     constraints = [x_var[:, 0] == self.x]

    #     for k in range(self.N):
    #         cost += cp.quad_form(x_var[:, k] - x_ref[k], self.Q)
    #         cost += cp.quad_form(u_var[:, k] - u_ref[k], self.R)

    #         # Lateral error penalty
    #         dx = x_var[0, k] - x_ref[k][0]
    #         dy = x_var[1, k] - x_ref[k][1]
    #         theta_ref = x_ref[k][2]

    #         e_lat = -np.sin(theta_ref) * dx + np.cos(theta_ref) * dy
    #         cost += 5.0 * cp.square(e_lat)

    #         constraints += [x_var[:, k + 1] == A_d @ x_var[:, k] + B_d @ u_var[:, k]]
    #         constraints += [u_var[0, k] >= -0.2, u_var[0, k] <= 1.0]   # linear v
    #         constraints += [u_var[1, k] >= -1.0, u_var[1, k] <= 1.0]  # angular ω

    #     cost += cp.quad_form(x_var[:, self.N] - x_ref[self.N], self.Qf)

    #     prob = cp.Problem(cp.Minimize(cost), constraints)
    #     prob.solve(solver=cp.OSQP)

    #     # Apply first control
    #     cmd = Twist()
    #     if u_var.value is not None:
    #         cmd.linear.x = u_var.value[0, 0]
    #         cmd.angular.z = u_var.value[1, 0]
    #     else:
    #         self.get_logger().warn("Solver failed. Sending zero command.")
    #         cmd.linear.x = 0.0
    #         cmd.angular.z = 0.0

    #     self.publisher_.publish(cmd)
    #     self.step += 1


def main():
    rclpy.init()
    node = MPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
