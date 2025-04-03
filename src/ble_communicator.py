#!/usr/bin/env python3

import asyncio
from bleak import BleakClient

class BLECommunicator:
    def __init__(self, device_address, service_uuid, tx_uuid, rx_uuid, send_data):
        self.device_address = device_address
        self.service_uuid = service_uuid
        self.tx_uuid = tx_uuid
        self.rx_uuid = rx_uuid
        self.send_data = send_data
        self.client = None

    async def connect(self):
        self.client = BleakClient(self.device_address)
        try:
            await self.client.connect()
            print(f"{self.device_address} に接続成功！")
        except Exception as e:
            print(f"{self.device_address} への接続に失敗: {e}")
            self.client = None

    async def disconnect(self):
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            print(f"{self.device_address} から切断しました。")
            self.client = None

    async def send_and_receive(self):
        if self.client and self.client.is_connected:
            await self.client.start_notify(self.rx_uuid, self.notification_handler)
            print(f"送信データ: {self.send_data.hex().upper()}")
            await self.client.write_gatt_char(self.tx_uuid, self.send_data, response=True)
            await asyncio.sleep(1)  # 1秒間待機
            await self.client.stop_notify(self.rx_uuid)
        else:
            print("デバイスが接続されていません。")

    async def notification_handler(self, sender, data):
        """BLEデバイスからのデータを受信して表示"""
        print(f"受信データ: {data.hex().upper()}")
        voltage = int.from_bytes(data[12:16], byteorder='little') / 1000.0
        print(f"電圧値: {voltage:.3f}")
        remianingAh = int.from_bytes(data[62:64], byteorder='little') / 100.0
        capacityAh  = int.from_bytes(data[64:66], byteorder='little') / 100.0
        chargePercent = (remianingAh / capacityAh) * 100
        print(f"電池残量: {chargePercent:.3f}%")

async def main():
    DEVICE_ADDRESS = "C8:47:80:18:B3:78"  # BLEデバイスのMACアドレス
    SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"  # サービスUUID
    TX_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"  # 書き込み（送信）UUID
    RX_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"  # 読み取り（受信）UUID
    SEND_DATA = bytes.fromhex("000004011355AA17")

    ble_communicator = BLECommunicator(DEVICE_ADDRESS, SERVICE_UUID, TX_UUID, RX_UUID, SEND_DATA)
    await ble_communicator.connect()
    if ble_communicator.client:
        while True:  # 無限ループ
            await ble_communicator.send_and_receive()
            await asyncio.sleep(0)  # 他のタスクに制御を譲る

if __name__ == "__main__":
    asyncio.run(main())