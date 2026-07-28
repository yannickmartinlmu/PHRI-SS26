# examples/basic_usage.py

import asyncio
from py_e4lib import E4Client


def on_bvp(values):
    """Handle BVP data."""
    avg = sum(values) / len(values)
    print(f"BVP avg: {avg:.2f}")


def on_gsr(values):
    """Handle GSR/EDA data."""
    avg = sum(values) / len(values)
    print(f"EDA avg: {avg:.3f} µS")


def on_temp(values):
    """Handle temperature data."""
    avg = sum(values) / len(values)
    print(f"Temp avg: {avg:.2f}°C")


def on_acc(values):
    """Handle accelerometer data."""
    # values are (x, y, z) tuples, divide by 64.0 for g-force
    avg_x = sum(x for x, _, _ in values) / len(values) / 64.0
    avg_y = sum(y for _, y, _ in values) / len(values) / 64.0
    avg_z = sum(z for _, _, z in values) / len(values) / 64.0
    print(f"ACC: X={avg_x:.2f}g, Y={avg_y:.2f}g, Z={avg_z:.2f}g")


async def main():
    # Option 1: Auto-discover device
    client = await E4Client.find()
    async with client:
        # Enable sensors you want
        client.enable_bvp(on_bvp)
        client.enable_gsr(on_gsr)
        client.enable_temp(on_temp)
        client.enable_acc(on_acc)

        # Start streaming
        await client.start()

        # Run for 30 seconds
        print("Streaming for 30 seconds...")
        await asyncio.sleep(30)

        print("Done!")

    # Option 2: Connect to specific address
    # async with E4Client("AA:BB:CC:DD:EE:FF") as client:
    #     client.enable_bvp(on_bvp)
    #     await client.start()
    #     await asyncio.sleep(30)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
