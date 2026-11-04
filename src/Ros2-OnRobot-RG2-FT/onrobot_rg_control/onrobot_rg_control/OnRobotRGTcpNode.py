#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import onrobot_rg_control.comModbusTcp
import onrobot_rg_control.baseOnRobotRG
from onrobot_rg_msgs.msg import OnRobotRGInput
from onrobot_rg_msgs.msg import OnRobotRGOutput


class OnRobotRGTcpNode(Node):
    def __init__(self):
        super().__init__('OnRobotRGTcpNode')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.declare_parameter('gripper', 'rg2')
        self.declare_parameter('dummy', False)
        self.declare_parameter('ip', 'setgripperip')
        self.declare_parameter('port', 502)

        self.gripper = onrobot_rg_control.baseOnRobotRG.onrobotbaseRG(
            self.get_parameter('gripper').value)
        self.gripper.client = onrobot_rg_control.comModbusTcp.Communication(
            self.get_parameter('dummy').value)
        self.gripper.client.connectToDevice(
            self.get_parameter('ip').value,
            self.get_parameter('port').value)

        self.pub = self.create_publisher(OnRobotRGInput, 'OnRobotRGInput', 1)
        self.create_subscription(
            OnRobotRGOutput,
            'OnRobotRGOutput',
            self.gripper.refreshCommand,
            10)

        self.gripper.setProximityOffset([230, 170])
        self.prev_msg = []
        self.timer = self.create_timer(0.0005, self.mainLoop)

    def mainLoop(self):
        status = self.gripper.getStatus()
        self.pub.publish(status)

        if not int(format(status.g_sta, '016b')[-1]):
            self.gripper.sendCommand()
        elif self.prev_msg != self.gripper.message:
            self.get_logger().info(': Sending message.')
            self.gripper.sendCommand()
        self.prev_msg = self.gripper.message


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotRGTcpNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
