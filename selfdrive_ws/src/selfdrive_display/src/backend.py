#!/usr/bin/env python3

from selfdrive_shared.node import Node
from selfdrive_shared.types import LogLevel, DeviceState, SystemState
import rclpy

from flask import Flask, Response, jsonify
from flask_socketio import SocketIO, emit
import threading
import time
import cv2
import numpy as np


class DisplayBackendConfig:
    def __init__(self):
        self.host = "127.0.0.1"
        self.port = 4029


class DisplayBackend(Node):
    def __init__(self):
        super().__init__("selfdrive_display")
        self.write_config(DisplayBackendConfig())
        self.init_flask_server()

    def init(self):
        self.log("Initialized")
        self.set_device_state(DeviceState.READY)

    def init_flask_server(self):
        """Initialize Flask server in a separate thread."""
        app = Flask(__name__)
        socketio = SocketIO(app, cors_allowed_origins="*")
        config = DisplayBackendConfig()

        @app.route("/")
        def get_time():
            """Returns the current time in milliseconds."""
            millis = int(time.time() * 1000)
            return jsonify({"current_time_ms": millis})
        
        @socketio.on("connect")
        def handle_connect():
            """Handle a new client connection."""
            self.log("Client connected")

        @socketio.on("disconnect")
        def handle_disconnect():
            """Handle a client disconnect."""
            self.log("Client disconnected")

        @socketio.on("test")
        def handle_test(data):
            """Handle a test event."""
            self.log("Received test event: {}".format(data))
            emit("test_response", {"data": "Test response"})

        # Run Flask server in a separate thread to avoid blocking the ROS node.
        thread = threading.Thread(
            target=app.run,
            kwargs={
                "host": config.host,
                "port": config.port,
                "debug": False,
                "use_reloader": False,
                "threaded": True,
            },
        )
        thread.daemon = True  # Ensure the thread exits with the main program.
        thread.start()


def main():
    rclpy.init()
    example = DisplayBackend()
    rclpy.spin(example)  # Keep the ROS node running.
    rclpy.shutdown()


if __name__ == "__main__":
    main()
