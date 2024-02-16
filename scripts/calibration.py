import tkinter as tk
from bleak import BleakClient
import asyncio
import struct
from threading import Thread

# Example characteristics: names to UUIDs mapping
CHARACTERISTICS = {
    "ul-C": "00001234-0000-1000-8000-00805f9b34fb",
    "ul-O": "00001235-0000-1000-8000-00805f9b34fb",
    "S_FREQ": "00001236-0000-1000-8000-00805f9b34fb",
    "S_DAMP": "00001237-0000-1000-8000-00805f9b34fb",
    "ll-C": "00001238-0000-1000-8000-00805f9b34fb",
    "ll-O": "00001239-0000-1000-8000-00805f9b34fb",
    "ur-C": "0000123a-0000-1000-8000-00805f9b34fb",
    "ur-O": "0000123b-0000-1000-8000-00805f9b34fb",
    "lr-C": "0000123c-0000-1000-8000-00805f9b34fb",
    "lr-O": "0000123d-0000-1000-8000-00805f9b34fb"
}

DEVICE_NAME = "Mehrak"
DEVICE_ADDRESS = "DC:BE:45:55:AA:18"
SERVICE_UUID = "00001234-0000-1000-8000-00805f9b34fb"  # Optional: Use if you want to filter by service UUID

class BLEDeviceManager:
    def __init__(self, loop, address):
        self.address = address
        self.loop = loop
        self.client = BleakClient(address, loop=self.loop)

    async def connect(self):
        await self.client.connect()
        return self.client.is_connected

    async def read_from_characteristic(self, char_uuid):
        value = await self.client.read_gatt_char(char_uuid)
        float_value, = struct.unpack('<f', value)
        return float_value

    async def write_to_characteristic(self, char_uuid, value):
        float_value = float(value)
        write_value = bytearray(struct.pack('<f', float_value))
        await self.client.write_gatt_char(char_uuid, write_value)
        print(f"Wrote {value} to {char_uuid}")

def start_asyncio_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()

asyncio_loop = asyncio.new_event_loop()
Thread(target=start_asyncio_loop, args=(asyncio_loop,), daemon=True).start()

ble_manager = BLEDeviceManager(asyncio_loop, DEVICE_ADDRESS)

def on_connect():
    future = asyncio.run_coroutine_threadsafe(ble_manager.connect(), asyncio_loop)
    connected = future.result()
    if connected:
        print("Connected to device")
    else:
        print("Failed to connect to device")
        return

def create_characteristic_row(name, uuid, parent):
    row = tk.Frame(parent)
    row.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

    label = tk.Label(row, text=name, width=20)
    label.pack(side=tk.LEFT)

    entry = tk.Entry(row)
    entry.pack(side=tk.LEFT, expand=tk.YES, fill=tk.X)

    def on_send():
        value = entry.get()
        future = asyncio.run_coroutine_threadsafe(ble_manager.write_to_characteristic(uuid, value), asyncio_loop)
        future.result()

    button = tk.Button(row, text="Send", command=on_send)
    button.pack(side=tk.RIGHT)

def main():
    root = tk.Tk()
    root.title("BLE Characteristic Writer")

    connect_button = tk.Button(root, text="Connect", command=on_connect)
    connect_button.pack(side=tk.TOP, pady=(10, 0))

    for name, uuid in CHARACTERISTICS.items():
        create_characteristic_row(name, uuid, root)

    root.mainloop()

if __name__ == "__main__":
    main()