# main.py -- MicroPython for ESP32 (Wokwi)
import time
import machine
import ujson
from machine import Pin, ADC, I2C

# --- DHT ---
try:
    import dht
    dht_pin = Pin(4, Pin.IN, Pin.PULL_UP)   # DHT data to GPIO4
    dht_sensor = dht.DHT22(dht_pin)         # use DHT22; if DHT11, use dht.DHT11
except Exception as e:
    dht_sensor = None
    print("WARN: DHT lib not available:", e)

# --- BMP280 (I2C) ---
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
# lightweight BMP280 driver (attempt to import / small local class)
class BMP280:
    def __init__(self, i2c, addr=0x76):
        self.i2c = i2c
        self.addr = addr
        # If chip missing, will raise OSError on reads later

    def read_raw(self):
        # try read some registers to ensure device present
        return True

    def read(self):
        # We will try to use standard micropython-bmp280 if present; otherwise fallback
        try:
            import bmp280
            sensor = bmp280.BMP280(i2c)
            temp = sensor.temperature
            pres = sensor.pressure
            return temp, pres
        except Exception:
            # fallback: return None, None
            return None, None

bmp = BMP280(i2c)

# --- Analog sensors ---
adc_soil = ADC(Pin(34))  # soil moisture
adc_ldr  = ADC(Pin(35))  # LDR
# ADC config for ESP32
adc_soil.atten(ADC.ATTN_11DB)  # full range
adc_ldr.atten(ADC.ATTN_11DB)

def read_adc_pct(adc):
    # read raw 0-4095 on esp32; convert to 0-100%
    raw = adc.read()
    pct = (raw / 4095.0) * 100.0
    return raw, round(pct,1)

# --- helper to read sensors ---
def read_sensors():
    data = {}
    # timestamp
    data['ts'] = time.time()
    # DHT
    if dht_sensor:
        try:
            dht_sensor.measure()
            data['temp_air_c'] = round(dht_sensor.temperature(), 2)
            data['humid_pct'] = round(dht_sensor.humidity(), 2)
        except Exception as e:
            data['temp_air_c'] = None
            data['humid_pct'] = None
    else:
        data['temp_air_c'] = None
        data['humid_pct'] = None

    # BMP280
    try:
        t, p = bmp.read()
        data['bmp_temp_c'] = round(t,2) if t is not None else None
        data['pressure_pa'] = round(p,2) if p is not None else None
    except Exception:
        data['bmp_temp_c'] = None
        data['pressure_pa'] = None

    # ADC sensors
    raw_soil, soil_pct = read_adc_pct(adc_soil)
    raw_ldr, ldr_pct = read_adc_pct(adc_ldr)
    data['soil_raw'] = raw_soil
    data['soil_pct'] = soil_pct     # higher value -> wetter if your divider maps that way (you may need invert)
    data['ldr_raw'] = raw_ldr
    data['ldr_pct'] = ldr_pct       # light level %
    return data

# --- main loop ---
INTERVAL = 5  # seconds
print("Starting sensor loop. Outputting JSON lines every", INTERVAL, "s")
while True:
    try:
        s = read_sensors()
        # Print JSON single line for host to parse
        print(ujson.dumps(s))
    except Exception as e:
        print("ERROR reading sensors:", e)
    time.sleep(INTERVAL)
