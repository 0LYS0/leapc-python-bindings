import asyncio
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "MAR8"  # The name your ESP32 advertises
WRITE_CHAR_UUID = "9aeec8fc-66e4-4234-90ca-df9661cdce0d"  # Replace with your actual write characteristic UUID

class ESP32:
    def __init__(self, device_name=DEVICE_NAME, write_char_uuid=WRITE_CHAR_UUID):
        self.device_name = device_name
        self.write_char_uuid = WRITE_CHAR_UUID
        self.client = None

    def connect(self):
        asyncio.run(self.async_connect())

    def send_command(self, command=str):
        asyncio.run(self.async_send_command(command))

    def disconnect(self):
        asyncio.run(self.async_disconnect())

    async def async_connect(self):
        self.device = None
        timeout = 5
        while self.device is None:
            devices = await BleakScanner.discover(timeout=timeout)
            for d in devices:
                if d.name == self.device_name:
                    self.device = d
                    print("Found device:", d)

            timeout *= 2
            if timeout >= 500:
                print("Faild to find device")
                raise ConnectionError

        self.client = BleakClient(self.device.address)
        await self.client.connect()
        if self.client.is_connected:
            print(f"Connected to: {self.device.name}")
        else:
            print("Faild to connect")
            raise ConnectionError

    async def async_disconnect(self):
        await self.client.disconnect()
        print(f"Disconnected to {self.device.name}")

    async def async_send_command(self, command=str):
        """
        command = "200,200,200,200,200"
        0~255 value
        """
        await self.client.write_gatt_char(self.write_char_uuid, command.encode("utf-8"))