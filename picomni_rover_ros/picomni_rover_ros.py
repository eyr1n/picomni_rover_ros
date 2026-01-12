import asyncio
import struct
import uuid

import rclpy
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from tf_transformations import quaternion_from_euler

_BLE_SERVICE_UUID = uuid.UUID("69321c59-8017-488e-b5e2-b6d30c834bc5")
_BLE_CHAR_UUID = uuid.UUID("87bc2dc5-2207-408d-99f6-3d35573c4472")
_BLE_WRITE_INTERVAL_MS = 50


class PicomniRoverROS(Node):
    def __init__(self) -> None:
        super().__init__("picomni_rover_ros")

        self._pose_pub = self.create_publisher(PoseStamped, "pose", 10)
        self._twist_sub = self.create_subscription(
            Twist, "cmd_vel", self._twist_sub_cb, 10
        )
        self._twist_timer = self.create_timer(
            _BLE_WRITE_INTERVAL_MS / 1000.0, self._twist_timer_cb
        )

        self._last_twist = Twist()
        self._twist_queue: asyncio.Queue[Twist] = asyncio.Queue()

    def pose_notify_cb(self, _: BleakGATTCharacteristic, data: bytearray) -> None:
        try:
            x, y, yaw = struct.unpack("<fff", data)
        except Exception as e:
            self.get_logger().error(str(e))
            return

        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        # Frame conversion: BLE (x=right,y=forward) -> ROS (x=forward,y=left).
        msg.pose.position.x = y
        msg.pose.position.y = -x
        msg.pose.position.z = 0.0

        x, y, z, w = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation.x = x
        msg.pose.orientation.y = y
        msg.pose.orientation.z = z
        msg.pose.orientation.w = w

        self._pose_pub.publish(msg)

    def _twist_sub_cb(self, msg: Twist) -> None:
        self._last_twist = msg

    def _twist_timer_cb(self) -> None:
        try:
            self._twist_queue.put_nowait(self._last_twist)
        except Exception as e:
            self.get_logger().error(str(e))


async def rclpy_spin(node: Node) -> None:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        await asyncio.sleep(0.001)


async def bleak_loop(node: PicomniRoverROS) -> None:
    device = await BleakScanner.find_device_by_filter(
        lambda _, adv: str(_BLE_SERVICE_UUID) in adv.service_uuids
    )  # type: ignore
    if device is None:
        raise RuntimeError("failed to find device")

    async with BleakClient(device) as client:
        node.get_logger().info("connected to device")

        await client.start_notify(_BLE_CHAR_UUID, node.pose_notify_cb)

        while True:
            msg = await node._twist_queue.get()
            # Frame conversion: ROS (x=forward,y=left) -> BLE (x=right,y=forward).
            data = struct.pack("<fff", -msg.linear.y, msg.linear.x, msg.angular.z)

            try:
                await client.write_gatt_char(_BLE_CHAR_UUID, data, response=False)
            except Exception as e:
                node.get_logger().error(str(e))


async def main_async(args) -> None:
    rclpy.init(args=args)
    node = PicomniRoverROS()

    try:
        await asyncio.gather(
            rclpy_spin(node),
            bleak_loop(node),
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
