import asyncio
from bleak import BleakScanner

# Apple / Microsoft / HTC / Meta / Nintendo / Bambu — all consumer noise, never the E4
NOISE = {76, 6, 1373, 1422, 40972, 42367}


async def main():
    devices = await BleakScanner.discover()
    for d in devices:
        print(d)


async def main_advanced():
    found = await BleakScanner.discover(return_adv=True)
    for dev, adv in sorted(found.values(), key=lambda x: x[1].rssi, reverse=True):
        if set(adv.manufacturer_data) & NOISE:
            continue  # skip phones/watches/laptops
        addr_type = "random" if dev.address[0].upper() in "CDEF" else "public"
        print(f"{adv.rssi:>4} dBm  {dev.address}  {addr_type:6}  "
              f"name={adv.local_name!r}  mfr={list(adv.manufacturer_data)}  "
              f"svcs={adv.service_uuids}")


asyncio.run(main())
