#!/usr/bin/env python3
"""
BLE IMU Data Reader with Battery Status
For ESP32-C3 with LSM6DSO IMU, BQ27427 Fuel Gauge, and BQ21080 Charger
Firmware: TBK_Firmware with battery management
"""

import asyncio
import struct
import sys
from datetime import datetime
from bleak import BleakClient, BleakScanner

# BLE UUIDs - Match the firmware's GATT service definition
SERVICE_UUID = "0000fff0-0000-1000-8000-00805f9b34fb"  # Primary service
CHAR_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"     # 20-byte notify characteristic

# Device name as defined in firmware (common.h)
DEVICE_NAME = "GloveRaw"

class IMUReader:
    def __init__(self):
        self.client = None
        self.start_time = datetime.now()
        self.packet_count = 0
        self.last_timestamp = 0
        self.battery_soc = 0
        self.is_charging = False
        self.battery_voltage_mv = 0
        # Smoothing
        self.battery_soc_history = []
        self.sample_rate_history = []

    async def find_device(self):
        """Scan for the GloveRaw device"""
        print(f"Scanning for {DEVICE_NAME}...")
        print("Make sure the ESP32-C3 is powered on and advertising")

        devices = await BleakScanner.discover(timeout=20.0)
        device_list = list(devices)

        print(f"\nFound {len(device_list)} BLE devices:")
        for device in device_list:
            print(f"  {device.name}: {device.address}")
            if device.name == DEVICE_NAME:
                print(f"\n[OK] Found target device: {DEVICE_NAME} at {device.address}")
                return device

        # Fallback: try a known MAC if available in logs
        mac_address = "7C:2C:67:0C:40:3A"
        for device in device_list:
            if device.address and device.address.upper() == mac_address:
                print(f"\n[OK] Found device by MAC: {device.address}")
                return device

        return None

    def parse_packet(self, data):
        """
        Parse the 20-byte packet structure from firmware:
        - Bytes 0-11: IMU data (6x int16): gx, gy, gz, ax, ay, az
        - Bytes 12-15: Timestamp (uint32) in milliseconds
        - Byte 16: Battery SOC (uint8) 0-100%
        - Byte 17: Charging status (uint8) 0=discharging, 1=charging
        - Bytes 18-19: Battery voltage (uint16) in mV from BQ27427
        """
        if len(data) != 20:
            print(f"Warning: Invalid packet size: {len(data)} bytes (expected 20)")
            return None

        try:
            # Little-endian: 6 shorts, 1 uint, 2 bytes, 1 ushort
            gx, gy, gz, ax, ay, az, timestamp, battery_soc, charging, voltage_mv = \
                struct.unpack('<hhhhhh I BB H', data)

            # Use immediate values from firmware (no smoothing) to match log responsiveness
            self.battery_soc = battery_soc
            self.is_charging = (charging == 1)
            self.battery_voltage_mv = voltage_mv if voltage_mv > 0 else 0

            # Timing and sample-rate smoothing
            time_delta = 0
            if self.last_timestamp > 0 and timestamp > self.last_timestamp:
                time_delta = timestamp - self.last_timestamp
                if time_delta > 0:
                    sample_rate = 1000.0 / time_delta
                    self.sample_rate_history.append(sample_rate)
                    if len(self.sample_rate_history) > 20:
                        self.sample_rate_history.pop(0)
            self.last_timestamp = timestamp

            return {
                'gyro': {'x': gx, 'y': gy, 'z': gz},
                'accel': {'x': ax, 'y': ay, 'z': az},
                'timestamp': timestamp,
                'delta_ms': time_delta,
                'battery_soc': battery_soc,
                'is_charging': (charging == 1),  # Real-time charging status from packet
                'voltage_mv': voltage_mv,
            }
        except struct.error as e:
            print(f"Error unpacking data: {e}")
            print(f"Raw data hex: {data.hex()}")
            return None

    def get_battery_symbol(self, soc, is_charging):
        """Return simple ASCII battery status without emoji."""
        if is_charging:
            return "[CHARGING]"
        if soc >= 75:
            return "[FULL]"
        elif soc >= 50:
            return "[GOOD]"
        elif soc >= 25:
            return "[HALF]"
        elif soc >= 10:
            return "[LOW]"
        else:
            return "[CRITICAL]"

    def display_data(self, imu_data):
        """Display IMU data with battery status"""
        if not imu_data:
            return

        self.packet_count += 1

        # LSM6DSO scaling factors
        accel_scale = 0.122 / 1000.0   # g
        gyro_scale  = 17.50 / 1000.0   # deg/s

        ax = imu_data['accel']['x'] * accel_scale
        ay = imu_data['accel']['y'] * accel_scale
        az = imu_data['accel']['z'] * accel_scale
        gx = imu_data['gyro']['x'] * gyro_scale
        gy = imu_data['gyro']['y'] * gyro_scale
        gz = imu_data['gyro']['z'] * gyro_scale

        accel_mag = (ax**2 + ay**2 + az**2) ** 0.5
        gyro_mag  = (gx**2 + gy**2 + gz**2) ** 0.5

        # Clear screen and header
        print("\033[H\033[J", end='')
        print("=" * 70)
        print(f"ESP32-C3 + LSM6DSO IMU Data - {DEVICE_NAME}")
        print("=" * 70)

        # Battery symbol and charge status - use real-time charging flag from packet
        real_time_charging = imu_data['is_charging']
        display_soc = imu_data['battery_soc']
        display_voltage = imu_data['voltage_mv']
        battery_symbol = self.get_battery_symbol(display_soc, real_time_charging)

        # Charge status: mirror firmware semantics (BLE byte 17)
        charge_status = "Charging" if real_time_charging else (
            "Discharging" if self.battery_voltage_mv > 0 else "Unknown"
        )

        # Battery bar
        bar_w = 20
        filled = max(0, min(bar_w, int(bar_w * display_soc / 100)))
        bar = "#" * filled + "-" * (bar_w - filled)

        # Color by SOC
        if display_soc >= 50:
            color = "\033[92m"  # Green
        elif display_soc >= 25:
            color = "\033[93m"  # Yellow
        else:
            color = "\033[91m"  # Red

        voltage_str = f" {display_voltage/1000.0:.2f}V" if display_voltage > 0 else ""
        print(f"Battery {battery_symbol}: {color}[{bar}]\033[0m {display_soc:3d}%{voltage_str} - {charge_status}")
        print("-" * 70)

        print(f"Packets received: {self.packet_count:5d}")
        print(f"Timestamp: {imu_data['timestamp']:8d} ms")

        if self.sample_rate_history:
            avg_rate = sum(self.sample_rate_history) / len(self.sample_rate_history)
            print(f"Sample rate: {avg_rate:6.1f} Hz")
        elif imu_data['delta_ms'] > 0:
            print(f"Sample rate: {1000/imu_data['delta_ms']:6.1f} Hz")

        if display_soc == 0 and not real_time_charging:
            print("Note: BQ27427 needs calibration - perform 2 charge cycles")
        else:
            print("Note: Firmware updates battery status every 1 second (100 packets)")
        print()

        print("ACCELEROMETER")
        print(f"  X: {ax:8.4f} g  (raw: {imu_data['accel']['x']:6d})")
        print(f"  Y: {ay:8.4f} g  (raw: {imu_data['accel']['y']:6d})")
        print(f"  Z: {az:8.4f} g  (raw: {imu_data['accel']['z']:6d})")
        print(f"  Magnitude: {accel_mag:8.4f} g")
        print()

        print("GYROSCOPE")
        print(f"  X: {gx:8.2f} deg/s  (raw: {imu_data['gyro']['x']:6d})")
        print(f"  Y: {gy:8.2f} deg/s  (raw: {imu_data['gyro']['y']:6d})")
        print(f"  Z: {gz:8.2f} deg/s  (raw: {imu_data['gyro']['z']:6d})")
        print(f"  Magnitude: {gyro_mag:8.2f} deg/s")
        print()

        # Orientation (simple heuristic)
        if abs(az) > abs(ax) and abs(az) > abs(ay):
            orientation = "Flat (Z-up)" if az > 0 else "Upside down (Z-down)"
        elif abs(ax) > abs(ay):
            orientation = "Standing on X+ edge" if ax > 0 else "Standing on X- edge"
        else:
            orientation = "Standing on Y+ edge" if ay > 0 else "Standing on Y- edge"

        if gyro_mag > 50:
            motion = "Fast rotation"
        elif gyro_mag > 10:
            motion = "Moving"
        else:
            motion = "Stationary"

        print(f"Orientation: {orientation}")
        print(f"Motion: {motion}")
        print()

        if self.battery_soc < 20 and self.battery_soc > 0 and not real_time_charging:
            print("\033[91m*** LOW BATTERY WARNING! ***\033[0m")
            print()

        print("Press Ctrl+C to stop")

    async def notification_handler(self, sender, data):
        """Handle incoming BLE notifications"""
        imu_data = self.parse_packet(data)
        if imu_data:
            self.display_data(imu_data)

    async def run(self):
        """Main async function"""
        device = await self.find_device()
        if not device:
            print(f"\n[X] Device {DEVICE_NAME} not found!")
            print("\nTroubleshooting:")
            print("1. Make sure ESP32-C3 is powered on")
            print("2. Check that it's advertising (LED should indicate status)")
            print("3. Make sure Bluetooth is enabled")
            print("4. Try moving closer to the device")
            return

        print(f"\nConnecting to {device.address}...")
        try:
            async with BleakClient(device.address) as client:
                self.client = client
                print("[OK] Connected successfully!")

                services = client.services
                service_list = list(services)
                print(f"\nFound {len(service_list)} services:")

                service_found = False
                char_found = False

                for service in services:
                    print(f"  Service: {service.uuid}")
                    if service.uuid.lower() == SERVICE_UUID.lower():
                        service_found = True
                        print(f"    [OK] Target service found!")
                        for char in service.characteristics:
                            print(f"      Char: {char.uuid} [{char.properties}]")
                            if char.uuid.lower() == CHAR_UUID.lower():
                                char_found = True
                                print(f"        [OK] Target characteristic found!")

                if not service_found:
                    print(f"\n[X] Service {SERVICE_UUID} not found!")
                    return
                if not char_found:
                    print(f"\n[X] Characteristic {CHAR_UUID} not found!")
                    return

                print(f"\nStarting notifications...")
                await client.start_notify(CHAR_UUID, self.notification_handler)
                print("[OK] Receiving IMU data with battery status!\n")

                while client.is_connected:
                    await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\n\nStopping...")
        except Exception as e:
            print(f"\n[X] Error: {e}")
        finally:
            print("Disconnected")

async def main():
    """Main entry point"""
    print("TBK Firmware - IMU Data Reader with Battery Management")
    print("Hardware: ESP32-C3 + LSM6DSO + BQ27427 + BQ21080")
    print("=" * 60)
    print(f"Looking for device: {DEVICE_NAME}")
    print()

    reader = IMUReader()
    try:
        await reader.run()
    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    try:
        import bleak
    except ImportError:
        print("Error: bleak module not found!")
        print("Install it with: pip3 install bleak")
        sys.exit(1)

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
        sys.exit(0)
