"""Programmatic ROS 2 client for an OnRobot RG gripper."""

import time
from typing import Optional

import rclpy
from rclpy.node import Node

from onrobot_rg_msgs.msg import OnRobotRGInput, OnRobotRGOutput


class RemoteGripper(Node):
    """Publish gripper commands and receive gripper status."""

    def __init__(
        self,
        node_name: str = 'remote_gripper',
        gripper: str = 'rg2ft',
    ) -> None:
        super().__init__(node_name)
        if gripper == 'rg2ft':
            self._max_force = 400
            self._max_width = 1000
        elif gripper == 'rg6':
            self._max_force = 1200
            self._max_width = 1600
        else:
            raise ValueError("gripper must be 'rg2ft' or 'rg6'")

        self._publisher = self.create_publisher(
            OnRobotRGOutput,
            'OnRobotRGOutput',
            10,
        )
        self._status: Optional[OnRobotRGInput] = None
        self._subscription = self.create_subscription(
            OnRobotRGInput,
            'OnRobotRGInput',
            self._status_callback,
            10,
        )

    def wait_for_connection(self, timeout: float = 5.0) -> bool:
        """Wait until the TCP control node subscribes to commands."""
        deadline = time.monotonic() + timeout
        while self._publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def set(self, width: int, force: int = 200) -> None:
        """Move to ``width`` (0.1 mm units) using ``force`` (0.1 N units)."""
        self._validate_range('width', width, self._max_width)
        self._validate_range('force', force, self._max_force)

        command = OnRobotRGOutput()
        command.r_gwd = width
        command.r_gfr = force
        command.r_ctr = 1
        self._publisher.publish(command)

    def set_force(self, force: int) -> None:
        """Set the force for subsequent commands."""
        self._validate_range('force', force, self._max_force)
        command = OnRobotRGOutput()
        command.r_gfr = force
        command.r_ctr = 0
        self._publisher.publish(command)

    def get_status(self, timeout: float = 1.0) -> Optional[OnRobotRGInput]:
        """Return the latest status, waiting up to ``timeout`` seconds."""
        deadline = time.monotonic() + timeout
        while self._status is None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            rclpy.spin_once(self, timeout_sec=min(remaining, 0.1))
        return self._status

    def _status_callback(self, status: OnRobotRGInput) -> None:
        self._status = status

    @staticmethod
    def _validate_range(name: str, value: int, maximum: int) -> None:
        if not isinstance(value, int) or isinstance(value, bool):
            raise TypeError(f'{name} must be an integer')
        if not 0 <= value <= maximum:
            raise ValueError(f'{name} must be between 0 and {maximum}')


def init() -> None:
    """Initialize ROS 2 before creating a :class:`RemoteGripper`."""
    rclpy.init()


def shutdown() -> None:
    """Shut down ROS 2 after closing the client."""
    rclpy.shutdown()
