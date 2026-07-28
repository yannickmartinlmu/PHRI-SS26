#!/usr/bin/env python3

import asyncio
import threading

from bleak import BleakClient

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

ADDRESS = "38:F9:F5:2C:74:C9"
HR_CHAR = "00002a37-0000-1000-8000-00805f9b34fb"
RECONNECT_DELAY = 5.0  # s; BLE supervision-timeout drops are routine, not errors


class HeartrateNode(Node):

    def __init__(self):
        super().__init__("sensor_hr")

        self._pub = self.create_publisher(Int32, "/heartrate", 10)

        self._ble_thread = threading.Thread(
            target=self._run_ble_loop, daemon=True
        )
        self._ble_thread.start()

        self.get_logger().info("Heartrate node started, connecting to sensor...")

    def _hr_callback(self, sender, data):
        hr = data[1]
        msg = Int32()
        msg.data = hr
        self._pub.publish(msg)
        self.get_logger().info(f"Heart rate: {hr} BPM")

    def _run_ble_loop(self):
        asyncio.run(self._ble_task())

    async def _ble_task(self):
        while rclpy.ok():
            try:
                async with BleakClient(ADDRESS) as client:
                    await client.start_notify(HR_CHAR, self._hr_callback)
                    self.get_logger().info("Heartrate sensor connected")
                    while rclpy.ok() and client.is_connected:
                        await asyncio.sleep(1)
                self.get_logger().warn("Heartrate link lost")
            except Exception as e:
                self.get_logger().warn(f"Heartrate connection failed: {e}")
            if rclpy.ok():
                await asyncio.sleep(RECONNECT_DELAY)


def main():
    rclpy.init()
    node = HeartrateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
