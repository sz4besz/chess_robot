#!/usr/init/env python3
import rclpy
from rclpy.node import Node
from gripper import SetGripper
from onrobot_rg_control.remote_gripper import RemoteGripper, init, shutdown

class GripperServer(Node):
    def __init__(self):
        super().__init__('gripper_server')
        init()
        self.gripper = RemoteGripper(gripper='rg2ft')
        if not self.gripper.wait_for_connection():
            self.get_logger().error('The gripper TCP node is not running!')
        
        self.srv = self.create_service(SetGripper, 'control_gripper', self.handle_gripper_command)

    def handle_gripper_command(self, request, response):
        self.gripper.move(width=request.width, force=request.force)
        status = self.gripper.get_status()
        
        if status is not None:
            response.success = True
            response.current_width = float(status.grip_width)
        else:
            response.success = False
            response.current_width = 0.0
        return response

    def destroy_node(self):
        self.gripper.destroy_node()
        shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GripperServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()