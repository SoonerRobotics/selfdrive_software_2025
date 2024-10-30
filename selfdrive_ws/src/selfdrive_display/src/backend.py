#!/usr/bin/env python3

from selfdrive_shared.node import Node
from selfdrive_shared.types import LogLevel, DeviceState, SystemState
import rclpy


class DisplayBackendConfig:
    def __init__(self):
        self.host = "127.0.0.1"
        self.port = 4029


class DisplayBackend(Node):
    def __init__(self):
        super().__init__("selfdrive_display")
        self.write_config(DisplayBackendConfig())

    def init(self):
        self.log("Initialized")
        self.set_device_state(DeviceState.READY)
        
def main():
    rclpy.init()
    example = DisplayBackend()
    rclpy.spin(example)
    rclpy.shutdown()

if __name__ == "__main__":
    main()