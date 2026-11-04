import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import sys

class TCPPoseSubscriber(Node):
    def __init__(self):
        super().__init__('tcp_pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.listener_callback,
            10)
        
        self.latest_msg = None
        # Refresh the screen exactly 5 times per second
        self.timer = self.create_timer(0.2, self.timer_callback)
        
        # Clear the terminal once on startup
        sys.stdout.write('\033[2J')
        sys.stdout.flush()

    def listener_callback(self, msg):
        # Store the latest data but do not print it yet
        self.latest_msg = msg

    def timer_callback(self):
        if self.latest_msg is None:
            return
            
        msg = self.latest_msg
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Convert Quaternion to Euler Angles
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2 * (qw * qy - qx * qz)
        pitch = math.degrees(math.asin(sinp)) if abs(sinp) <= 1 else math.degrees(math.copysign(math.pi / 2, sinp))

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        # Convert Quaternion to Rotation Vector (Axis-Angle)
        angle = 2 * math.acos(qw)
        s = math.sqrt(1 - qw * qw)
        
        if s < 0.001:  
            rx, ry, rz = qx, qy, qz
        else:
            rx = (qx / s) * angle
            ry = (qy / s) * angle
            rz = (qz / s) * angle
        
        # \033[H moves cursor home. \033[K clears the rest of the line to prevent trailing artifact characters.
        output = (
            f'\033[H'
            f'--- Current TCP Position & Orientation ---\033[K\n'
            f'Translation (m):    X: {x: 7.5f}, Y: {y: 7.5f}, Z: {z: 7.5f}\033[K\n'
            f'Euler (Degrees):    Roll: {roll: 7.2f}°, Pitch: {pitch: 7.2f}°, Yaw: {yaw: 7.2f}°\033[K\n'
            f'ROS Quaternion:     x: {qx: 7.5f}, y: {qy: 7.5f}, z: {qz: 7.5f}, w: {qw: 7.5f}\033[K\n'
            f'URScript array:     p[{x:.5f}, {y:.5f}, {z:.5f}, {rx:.5f}, {ry:.5f}, {rz:.5f}]\033[K\n\n'
            f'(Press Ctrl+C to exit)\033[K'
        )
        
        sys.stdout.write(output)
        sys.stdout.flush()

def main():
    rclpy.init()
    node = TCPPoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Clear screen upon exit
        sys.stdout.write('\033[2J\033[H')
        sys.stdout.flush()
    rclpy.shutdown()

if __name__ == '__main__':
    main()