import random

import ubluetooth
from machine import Timer, Pin, I2C
import ssd1306
from MPU6050 import MPU6050
from bh1750 import BH1750
import dht
'''
TELE-ESP-BOARD LEDS:
    R: D25
    W: D4?
    G: ?? 
'''

# Set this to enable/disable ssd1306 display functionality
DISPLAY = 0

# Set this to enable/disable DHT11 temperature/humidity sensor.
# If setting this to 0, data will be simulated.
TEMP_SENSOR = 1


class Display:
    def __init__(self, i2c_obj):
        if DISPLAY:
            self.last_status = []
            self.oled = ssd1306.SSD1306_I2C(128, 64, i2c_obj)  # Display OLED 128x64
        else:
            print("Adafruit SSD1306 is disabled by current configuration.")

    def write_text(self, text, x=0, y=0, clear=True):
        if DISPLAY:
            if [text, x, y, clear] != self.last_status:
                if clear:
                    self.oled.fill(0)  # Pulisce lo schermo

                lines = self._wrap_text(text) if len(text) > 14 else [text]
                for i, line in enumerate(lines):
                    self.oled.text(line, x, y + i * 10)  # Adjust y-offset for each line
                self.oled.show()
                self.last_status = [text, x, y, clear]


    def _wrap_text(self, text):
        """Wraps text to fit within 14 characters per line (more efficiently)."""
        lines = []
        start = 0
        while start < len(text):
            end = start + 14
            if end >= len(text):
                lines.append(text[start:].strip())
                break

            # Check for space to avoid cutting words
            if text[end] == ' ':  # ideal case
                lines.append(text[start:end].strip())
                start = end + 1
                continue

            # Check for space before the limit
            rfind_index = text.rfind(' ', start, end)
            if rfind_index != -1:
                lines.append(text[start:rfind_index].strip())
                start = rfind_index + 1
                continue

            #force split
            lines.append(text[start:end].strip())
            start = end
        return lines


class SafeHelmet:
    def __init__(self, data_interval=5):

        # I2C init
        self.i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=100000)

        self.standby_led = Pin(2, Pin.OUT)
        self.standby_led.value(0)

        self.collect_led = Pin(4, Pin.OUT)
        self.collect_led.value(0)

        # BLE init and service/characteristics setup
        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self._irq)

        self._connections = set()
        self._service_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d479")
        self._temp_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d480")
        self._hum_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d481")
        self._lux_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d482")
        self._state_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d483")

        self._temp_char = (self._temp_uuid, ubluetooth.FLAG_INDICATE | ubluetooth.FLAG_READ)
        self._hum_char = (self._hum_uuid, ubluetooth.FLAG_INDICATE | ubluetooth.FLAG_READ)
        self._lux_char = (self._lux_uuid, ubluetooth.FLAG_INDICATE | ubluetooth.FLAG_READ)
        self._state_char = (self._state_uuid, ubluetooth.FLAG_INDICATE | ubluetooth.FLAG_READ)
        self._service = (self._service_uuid, (self._temp_char, self._hum_char, self._lux_char, self._state_char))

        ((self._temp_handle, self._hum_handle, self._lux_handle, self._state_handle),) = self.ble.gatts_register_services(
            [self._service])

        self.standby = False

        self.adv_led = Pin(25, Pin.OUT)
        self.adv_led.value(0)

        if TEMP_SENSOR:
            self.dht_sensor = dht.DHT11(Pin(18))
        else:
            print("DHT11 sensor is disabled by current configuration. Data will be simulated")

        self.display = Display(self.i2c)
        self.light_sensor = BH1750(self.i2c)
        self.mpu = MPU6050(self.i2c)

        self.data_interval = data_interval
        self.base_interval = 10  # 10ms precision on virtual timers

        self.virtual_timers = []
        self.adv_led_timer_id = None  # store the id of the adv led timer
        self.standby_led_timer_id = None  # store the id of the standby led timer
        self.data_timer_id = None  # store the id of the data timer

        self.data_timer_id = self.create_virtual_timer(self.data_interval * 1000, self._collect_and_send_data)
        self.adv_led_timer_id = self.create_virtual_timer(500, self._toggle_adv_led)
        self.standby_led_timer_id = self.create_virtual_timer(1000, self._standby_mode)

        self.base_timer = Timer(-1)
        self.base_timer.init(period=self.base_interval, mode=Timer.PERIODIC, callback=self._virtual_timer_callback)

        self._start_advertising()

    def _virtual_timer_callback(self, timer):
        for vt in self.virtual_timers:
            vt["counter"] += self.base_interval
            if vt["counter"] >= vt["period"]:
                vt["callback"]()
                vt["counter"] = 0

    def create_virtual_timer(self, period, callback):
        timer_id = len(self.virtual_timers)  # assign an id equal to the index in the list
        self.virtual_timers.append({"id": timer_id, "period": period, "callback": callback, "counter": 0})
        return timer_id  # return the id

    def stop_virtual_timer(self, timer_id):
        for i, vt in enumerate(self.virtual_timers):
            if vt["id"] == timer_id:
                self.virtual_timers.pop(i)  # remove the timer from the list
                return  # exit from the function
        print("Timer id not found")  # if the id is not found print a message

    def _irq(self, event, data):
        if event == 1:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("Device connected: {}".format(conn_handle))
            self._stop_advertising()
            self.stop_virtual_timer(self.adv_led_timer_id)  # stop adv LED
            self.adv_led.value(0)  # turn off the LED
        elif event == 2:
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            print("Device disconnected")
            self._start_advertising()
            self.adv_led_timer_id = self.create_virtual_timer(500, self._toggle_adv_led)  #restart the timer
        elif event == 3:
            print("Writing is not supported on this characteristic")

    def _start_advertising(self):
        name = b'SafeHelmet-01'
        self.ble.gap_advertise(100,
                               b'\x02\x01\x06' +
                               b'\x03\x03\xf4\x7a' +
                               bytes([len(name) + 1]) + b'\x09' + name)
        print("SafeHelmet is advertising...")
        self.display.write_text('Waiting for connection...')

    def _stop_advertising(self):
        self.ble.gap_advertise(interval_us=None)

    def _toggle_adv_led(self):
        self.adv_led.value(not self.adv_led.value())

    def _collect_and_send_data(self):
        if self.standby:
            self._check_exit_standby()
            return

        if not self._connections:
            return

        self.collect_led.value(1)

        lux = self.light_sensor.luminance()
        z_accel = self.mpu.read_accel_data()['z']

        if TEMP_SENSOR:
            self.dht_sensor.measure()
            temperature = self.dht_sensor.temperature()
            humidity = self.dht_sensor.humidity()
        else:
            humidity = random.uniform(10, 98)
            temperature = random.uniform(20.0, 22.0)

        print("Temp: {:.1f} / Hum: {:.1f} / Lux: {:.1f} / Accel (Z): {:.1f}".format(temperature, humidity, lux, z_accel))

        for conn_handle in self._connections:
            self.ble.gatts_indicate(conn_handle, self._temp_handle, "{:.1f}".format(temperature).encode())
        for conn_handle in self._connections:
            self.ble.gatts_indicate(conn_handle, self._hum_handle, "{:.1f}".format(humidity).encode())
        for conn_handle in self._connections:
            self.ble.gatts_indicate(conn_handle, self._lux_handle, "{:.1f}".format(lux).encode())

        self.display.write_text("Temp.: {:.1f}C".format(temperature))
        self.display.write_text("Hum.: {:.1f} hPa".format(humidity), clear=False, y=16)
        self.display.write_text("Lux.: {:.1f} lum".format(lux), clear=False, y=32)

        self.collect_led.value(0)

        if z_accel < -2:
            self._enter_standby()

    def _enter_standby(self):
        print("Entering standby mode...")
        for conn_handle in self._connections:
            self.ble.gatts_indicate(conn_handle, self._state_handle, b"Entering standby")

            self.standby = True

            self.display.write_text("Standby")

    def _standby_mode(self):
        if self.standby:
            self.standby_led.value(not self.standby_led.value())

    def _check_exit_standby(self):
        z_accel = self.mpu.read_accel_data()['z']

        if z_accel >= 5:
            print("Exiting standby mode...")
            for conn_handle in self._connections:
                self.ble.gatts_indicate(conn_handle, self._state_handle, b"Exiting standby")

            self.standby = False
            self.standby_led.value(0)


# Esegui il server BLE per sensori
ble_sensor = SafeHelmet(data_interval=5)

try:
    while True:
        pass
except KeyboardInterrupt:
    ble_sensor.base_timer.deinit()
    print("Manually stopped")
