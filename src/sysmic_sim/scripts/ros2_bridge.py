#!/usr/bin/python3
"""ROS 2 Humble bridge for the SYSMIC V2 RoboCasa simulation.

Started by the launch file (ros2 launch sysmic_sim sim.launch.py), or by hand:

    ros2 run sysmic_sim ros2_bridge.py

It uses the system Python that ROS 2 is built for, never the conda one, which is
why the first line of this file is /usr/bin/python3.

Publishes
    /clock          rosgraph_msgs/Clock         simulation time
    /joint_states   sensor_msgs/JointState      all 25 movable URDF joints
    /odom           nav_msgs/Odometry           pose since start, twist in base_link
    /tf             odom -> base_link

Subscribes
    /cmd_vel                 geometry_msgs/Twist       linear.x (m/s), angular.z (rad/s)
    /sysmic/joint_command    sensor_msgs/JointState    name[] + position[]:
                             lift_joint (m), {left,right}_{shoulder_pan,shoulder_lift,
                             elbow_flex,wrist_flex,wrist_roll,gripper} (rad)

Nodes that consume these topics should set use_sim_time:=true.
"""
import json
import math
import socket

STATE_ADDR = ("127.0.0.1", 9870)  # must match sim/sysmic_sim_server.py
COMMAND_ADDR = ("127.0.0.1", 9871)
ODOM_FRAME, BASE_FRAME = "odom", "base_link"


def stamp_parts(t):
    """Simulation time in seconds -> (sec, nanosec)."""
    sec = int(math.floor(t))
    nanosec = int(round((t - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec, nanosec = sec + 1, nanosec - 1_000_000_000
    return sec, nanosec


def yaw_to_quat(yaw):
    """(x, y, z, w) for a rotation about Z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def twist_command(linear_x, angular_z):
    return json.dumps({"cmd_vel": [float(linear_x), float(angular_z)]}).encode()


def joint_command(names, positions):
    """JointState-style lists -> command packet, or None if there is nothing usable."""
    joints = {n: float(p) for n, p in zip(names, positions) if math.isfinite(p)}
    return json.dumps({"joints": joints}).encode() if joints else None


def main():
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from tf2_ros import TransformBroadcaster

    class SysmicBridge(Node):
        def __init__(self):
            super().__init__("sysmic_bridge")
            self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.rx.bind(STATE_ADDR)
            self.rx.setblocking(False)
            self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            self.clock_pub = self.create_publisher(Clock, "/clock", 10)
            self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
            self.tf_pub = TransformBroadcaster(self)
            self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
            self.create_subscription(JointState, "/sysmic/joint_command", self.on_joints, 10)

            self.packets = 0
            self.last_sim_time = None
            self.create_timer(0.005, self.poll)  # 200 Hz; the sim sends at 20 Hz
            self.create_timer(5.0, self.report)
            self.get_logger().info(
                f"Waiting for the simulation on udp {STATE_ADDR[1]} "
                "(it is started by sim.launch.py).")

        # ---- ROS -> simulation ----
        def on_cmd_vel(self, msg):
            self.tx.sendto(twist_command(msg.linear.x, msg.angular.z), COMMAND_ADDR)

        def on_joints(self, msg):
            if len(msg.name) != len(msg.position):
                self.get_logger().warn("joint_command ignored: name and position differ in length")
                return
            packet = joint_command(list(msg.name), list(msg.position))
            if packet:
                self.tx.sendto(packet, COMMAND_ADDR)

        # ---- simulation -> ROS ----
        def poll(self):
            while True:
                try:
                    packet, _ = self.rx.recvfrom(65535)
                except BlockingIOError:
                    return
                try:
                    self.publish(json.loads(packet.decode()))
                except (ValueError, KeyError, TypeError) as exc:
                    self.get_logger().warn(f"bad state packet ignored: {exc}")

        def publish(self, state):
            t = float(state["t"])
            if self.last_sim_time is not None and t < self.last_sim_time:
                self.get_logger().info("simulation time went backwards (scene was reset)")
            self.last_sim_time = t
            self.packets += 1
            sec, nanosec = stamp_parts(t)

            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = sec, nanosec
            self.clock_pub.publish(clock)

            js = JointState()
            js.header.stamp.sec, js.header.stamp.nanosec = sec, nanosec
            js.name = list(state["names"])
            js.position = [float(v) for v in state["pos"]]
            js.velocity = [float(v) for v in state["vel"]]
            self.joint_pub.publish(js)

            o = state["odom"]
            qx, qy, qz, qw = yaw_to_quat(float(o["yaw"]))

            odom = Odometry()
            odom.header.stamp.sec, odom.header.stamp.nanosec = sec, nanosec
            odom.header.frame_id = ODOM_FRAME
            odom.child_frame_id = BASE_FRAME
            odom.pose.pose.position.x = float(o["x"])
            odom.pose.pose.position.y = float(o["y"])
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            odom.twist.twist.linear.x = float(o["vx"])
            odom.twist.twist.linear.y = float(o["vy"])
            odom.twist.twist.angular.z = float(o["wz"])
            self.odom_pub.publish(odom)

            tf = TransformStamped()
            tf.header.stamp.sec, tf.header.stamp.nanosec = sec, nanosec
            tf.header.frame_id = ODOM_FRAME
            tf.child_frame_id = BASE_FRAME
            tf.transform.translation.x = float(o["x"])
            tf.transform.translation.y = float(o["y"])
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_pub.sendTransform(tf)

        def report(self):
            if self.packets == 0:
                self.get_logger().warn("No data from the simulation yet. Is sysmic_sim_server.py running?")
            else:
                self.get_logger().info(
                    f"{self.packets / 5.0:.0f} state msgs/s, sim time {self.last_sim_time:.1f} s")
            self.packets = 0

    rclpy.init()
    node = SysmicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
