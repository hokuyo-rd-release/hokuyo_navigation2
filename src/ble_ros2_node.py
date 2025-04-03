#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import asyncio
import threading
from ble_communicator import BLECommunicator

class BLEROSNode(Node):
    def __init__(self, device_address, service_uuid, tx_uuid, rx_uuid, send_data):
        super().__init__('ble_ros_node')
        self.charge_percent_pub = self.create_publisher(Float32, 'charge_percent', 10)
        self.voltage_pub = self.create_publisher(Float32, 'voltage', 10)
        self.ble_communicator = BLECommunicator(device_address, service_uuid, tx_uuid, rx_uuid, send_data)
        self.ble_communicator.notification_handler = self.notification_handler
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.start_loop).start()

    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connect_and_loop())

    async def connect_and_loop(self):
        await self.ble_communicator.connect()
        if self.ble_communicator.client:
            await self.ble_communicator.client.start_notify(self.ble_communicator.rx_uuid, self.notification_handler)
            while rclpy.ok():
                await self.ble_communicator.send_and_receive()
                await asyncio.sleep(1)
            self.loop.call_soon_threadsafe(self.loop.stop)

    def notification_handler(self, sender, data):
        voltage = int.from_bytes(data[12:16], byteorder='little') / 1000.0
        remaining_ah = int.from_bytes(data[62:64], byteorder='little') / 100.0
        capacity_ah = int.from_bytes(data[64:66], byteorder='little') / 100.0
        charge_percent = (remaining_ah / capacity_ah) * 100

        voltage_msg = Float32()
        voltage_msg.data = voltage
        self.voltage_pub.publish(voltage_msg)

        charge_percent_msg = Float32()
        charge_percent_msg.data = charge_percent
        self.charge_percent_pub.publish(charge_percent_msg)

def main(args=None):
    rclpy.init(args=args)
    device_address = "C8:47:80:18:B3:78"
    service_uuid = "0000ffe0-0000-1000-8000-00805f9b34fb"
    tx_uuid = "0000ffe1-0000-1000-8000-00805f9b34fb"
    rx_uuid = "0000ffe1-0000-1000-8000-00805f9b34fb"
    send_data = bytes.fromhex("000004011355AA17")

    node = BLEROSNode(device_address, service_uuid, tx_uuid, rx_uuid, send_data)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()