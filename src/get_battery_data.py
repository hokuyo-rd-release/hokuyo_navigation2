import asyncio
from bleak import BleakClient

class charge_check:    
    voltage = 0.0
    chargePercent = 0.0

    # BLEデバイス設定
    BLE_DEVICE_ADDRESS = "C8:47:80:18:B3:78"  # BLEデバイスのMACアドレス
    BLE_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"  # サービスUUID
    BLE_TX_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"  # 書き込み（送信）UUID
    BLE_RX_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"  # 読み取り（受信）UUID

    # 送信データ（16進数） 0x000004011355AA17
    SEND_DATA = bytes.fromhex("000004011355AA17")

    def get_chargePercent(self):
        return self.chargePercent

    def get_voltage(self):
        return self.voltage

    async def notification_handler(self, data):
        """BLEデバイスからのデータを受信して表示"""
        print(f"受信データ: {data.hex().upper()}")
        
        # バイナリデータに変換
        #payload = bytes.fromhex(data)
        
        # インデックス12から4バイトのデータをリトルエンディアンで取得し、
        # 1000で割る (電圧値)
        self.voltage = int.from_bytes(data[12:16], byteorder='little') / 1000.0
        #print(f"電圧値: {voltage:.3f}")

        # 電池残量
        remianingAh = int.from_bytes(data[62:64], byteorder='little') / 100.0
        capacityAh  = int.from_bytes(data[64:66], byteorder='little') / 100.0
        self.chargePercent = (remianingAh / capacityAh) * 100
        #print(f"電池残量: {chargePercent:.3f}%")

    async def main(self):
        async with BleakClient(self.BLE_DEVICE_ADDRESS) as client:
            #print(f"{BLE_DEVICE_ADDRESS} に接続成功！")

            # 受信データの通知を開始
            # 受信が呼ばれるごとに関数が登録される。
            await client.start_notify(self.BLE_RX_UUID, self.notification_handler)

            while True:
                # データを送信
                print(f"送信データ: {self.SEND_DATA.hex().upper()}")
                await client.write_gatt_char(self.BLE_TX_UUID, self.SEND_DATA, response=True)
                Per = self.get_chargePercent()
                vol = self.get_voltage()
                print("グローバルper: ")
                print(Per)
                print("\nグローバルvol: ")
                print(vol)
                # 5秒間データ受信を待つ
                await asyncio.sleep(1)

                # 受信データの通知を停止
                # await client.stop_notify(BLE_RX_UUID)
    def __init__(self):
        asyncio.run(self.main())


if __name__ == "__main__":
    charge_check()
