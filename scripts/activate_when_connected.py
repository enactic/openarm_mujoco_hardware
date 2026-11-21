#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Activate configured Web hardware; its plugin waits for actual feedback."""

import sys

import rclpy
from controller_manager_msgs.srv import SetHardwareComponentState
from lifecycle_msgs.msg import State


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)
    rclpy.init()
    node = rclpy.create_node("activate_web_hardware")
    client = node.create_client(
        SetHardwareComponentState, args[1].rstrip(
            "/") + "/set_hardware_component_state"
    )
    try:
        if not client.wait_for_service(timeout_sec=60):
            raise RuntimeError("controller_manager did not become available")
        for name in args[2:]:
            request = SetHardwareComponentState.Request()
            request.name = name
            request.target_state.id = State.PRIMARY_STATE_ACTIVE
            node.get_logger().info(
                f"Waiting for Web feedback to activate {name}")
            result = client.call_async(request)
            rclpy.spin_until_future_complete(node, result, timeout_sec=70)
            if (
                not result.done()
                or not result.result().ok
                or result.result().state.id != State.PRIMARY_STATE_ACTIVE
            ):
                raise RuntimeError(
                    f"{name} did not activate with fresh Web feedback")
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
