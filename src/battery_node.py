import get_battery_data
import asyncio
import time

charge = get_battery_data.charge_check()

#time.sleep(10)

a = charge.get_chargePercent()
b = charge.get_voltage()

print(a, b)