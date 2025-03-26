import asyncio
from bleak import BleakClient, BleakScanner

# BLE Device Name or Address (change if necessary)
DEVICE_NAME = "IMUSensor"
CHAR_UUID_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # TX characteristic UUID

async def notification_handler(sender, data):
    imu_data = data.decode("utf-8")  # Convert byte data to string
    print(f"IMU Data: {imu_data}")  # Example: "0.12,-0.98,9.81,0.01,0.02,-0.03"

async def main():
    print("🔍 Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    
    imu_device = None
    for d in devices:
        if d.name is not None and DEVICE_NAME in d.name:  # ✅ Check if name exists
            imu_device = d.address
            print(f"✅ Found {DEVICE_NAME} at {imu_device}")
            break

    if not imu_device:
        print("❌ IMU Sensor not found! Make sure it's powered on and advertising.")
        return

    print(f"🔗 Connecting to {imu_device}...")
    async with BleakClient(imu_device) as client:
        print("✅ Connected!")
        await client.start_notify(CHAR_UUID_TX, notification_handler)

        try:
            while True:
                await asyncio.sleep(1)  # Keep script running
        except KeyboardInterrupt:
            print("⏹ Stopping...")
        finally:
            await client.stop_notify(CHAR_UUID_TX)

asyncio.run(main())

