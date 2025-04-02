#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import asyncio
import threading
from ble_communicator import BLECommunicator

class BLEROSNode:
    def __init__(self, device_address, service_uuid, tx_uuid, rx_uuid, send_data):
        rospy.init_node('ble_ros_node', anonymous=True)
        self.charge_percent_pub = rospy.Publisher('charge_percent', Float32, queue_size=10)
        self.voltage_pub = rospy.Publisher('voltage', Float32, queue_size=10)
        self.ble_communicator = BLECommunicator(device_address, service_uuid, tx_uuid, rx_uuid, send_data)
        self.ble_communicator.notification_handler = self.notification_handler # notification_handlerをオーバーライド
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.start_loop).start()

    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connect_and_loop())

    async def connect_and_loop(self):
        await self.ble_communicator.connect()
        if self.ble_communicator.client:
            await self.ble_communicator.client.start_notify(self.ble_communicator.rx_uuid, self.notification_handler)
            while not rospy.is_shutdown():  # rospy.is_shutdown() をチェック
                await self.ble_communicator.send_and_receive()
                await asyncio.sleep(1) # 1秒間隔でループ
            self.loop.call_soon_threadsafe(self.loop.stop) # 非同期処理を止める

    # notification_handler を同期処理に変更
    def notification_handler(self, sender, data):
        voltage = int.from_bytes(data[12:16], byteorder='little') / 1000.0
        remianingAh = int.from_bytes(data[62:64], byteorder='little') / 100.0
        capacityAh = int.from_bytes(data[64:66], byteorder='little') / 100.0
        chargePercent = (remianingAh / capacityAh) * 100

        voltage_msg = Float32()
        voltage_msg.data = voltage
        self.voltage_pub.publish(voltage_msg)

        charge_percent_msg = Float32()
        charge_percent_msg.data = chargePercent
        self.charge_percent_pub.publish(charge_percent_msg)

if __name__ == '__main__':
    try:
        DEVICE_ADDRESS = "C8:47:80:18:B3:78"
        SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
        TX_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
        RX_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
        SEND_DATA = bytes.fromhex("000004011355AA17")

        publisher = BLEROSNode(DEVICE_ADDRESS, SERVICE_UUID, TX_UUID, RX_UUID, SEND_DATA)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass