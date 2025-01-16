import struct

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

        # Sample LED init
        self.standby_led = Pin(2, Pin.OUT)
        self.standby_led.value(0)

        self.send_led = Pin(4, Pin.OUT)
        self.send_led.value(0)

        self.adv_led = Pin(25, Pin.OUT)
        self.adv_led.value(0)

        # BLE init and service/characteristics setup
        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self._irq)

        self._connections = set()
        self._service_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d479")
        self._data_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d480")
        self._state_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d481")

        self._data_char = (self._data_uuid, ubluetooth.FLAG_NOTIFY)
        self._state_char = (self._state_uuid, ubluetooth.FLAG_NOTIFY)
        self._service = (self._service_uuid, (self._data_char, self._state_char))

        ((self._data_handle, self._state_handle),) = self.ble.gatts_register_services(
            [self._service])

        # Sensors setup and init
        self.standby = False

        self.dht_sensor = dht.DHT11(Pin(18))
        self.display = Display(self.i2c)
        self.light_sensor = BH1750(self.i2c)
        self.mpu = MPU6050(self.i2c)

        # First value is total sum of readings, second value is number of readings
        self._temperature = [0, 0]
        self._humidity = [0, 0]
        self._lux = [0, 0]

        # Physical and virtual timers init
        self.data_interval = data_interval
        self.base_interval = 60  # 100ms precision on virtual timers

        self.virtual_timers = []
        self.adv_led_timer_id = None  # store the id of the adv led timer
        self.data_timer_id = None  # store the id of the data timer

        self.data_timer_id = self.create_virtual_timer(self.data_interval * 1000, self._send_data)
        self.adv_led_timer_id = self.create_virtual_timer(500, self._toggle_adv_led)

        self.base_timer = Timer(-1)
        self.base_timer.init(period=self.base_interval, mode=Timer.PERIODIC, callback=self._virtual_timer_callback)

        # Start advertising process after startup
        self._start_advertising()

    def _virtual_timer_callback(self, timer):
        for vt in self.virtual_timers:
            vt["counter"] += self.base_interval
            if vt["counter"] >= vt["period"]:
                vt["callback"]()
                #print("period:{}, counter is: {}".format(vt["period"], vt["counter"]))
                vt["counter"] = 0

    def create_virtual_timer(self, period, callback):
        timer_id = len(self.virtual_timers)  # assign an id equal to the index in the list
        self.virtual_timers.append({"id": timer_id, "period": period, "callback": callback, "counter": 0})
        return timer_id  # return the id

    def stop_virtual_timer(self, timer_id):
        to_remove = None
        for i, vt in enumerate(self.virtual_timers):
            if vt["id"] == timer_id:
                to_remove = i
                break
        if to_remove is None:
            print("Timer id not found")  # if the id is not found print a message
            return
        else:
            self.virtual_timers.pop(to_remove)  # remove the timer from the list

    # Manage BLE events (connection, disconnection, ...)
    def _irq(self, event, data):
        if event == 1:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("Device connected: {}".format(conn_handle))
            self._stop_advertising()
            self.stop_virtual_timer(self.adv_led_timer_id)  # stop adv LED
            self.adv_led.value(0)  # turn off the LED

            self._start_data_collection()

        elif event == 2:
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            print("Device disconnected")
            self._start_advertising()
            self.adv_led_timer_id = self.create_virtual_timer(500, self._toggle_adv_led)  #restart the timer
            self._stop_data_collection()
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

    #### DATA COLLECTING METHODS ####
    def _read_temperature(self):
        self.dht_sensor.measure()
        self._temperature[0] += self.dht_sensor.temperature()
        self._temperature[1] += 1

    def _read_humidity(self):
        self.dht_sensor.measure()
        self._humidity[0] += self.dht_sensor.humidity()
        self._humidity[1] += 1

    def _read_lux(self):
        self._lux[0] += self.light_sensor.luminance()
        self._lux[1] += 1

    def _get_orientation(self) -> float:
        return self.mpu.read_accel_data()['z']

    def _detect_crash(self):
        pass

    '''Start collecting data from all sensors and calculating mean averages. Each sensor has a different priority in
    worker's safety, thus the need for such a method that allows to set different retrieval intervals. 
    At the time of data packing and sending through BLE, mean averages are calculated on all the data obtained
    between one send and another.'''

    def _start_data_collection(self):
        self._temp_timer = self.create_virtual_timer(1000, self._read_temperature)
        self._hum_timer = self.create_virtual_timer(3000, self._read_humidity)
        self._lux_timer = self.create_virtual_timer(2000, self._read_lux)
        self._accel_timer = self.create_virtual_timer(10000, self._get_orientation)

    '''Stop data collection. By default, accelerometer data continues to be retrieved in order to provide
    for entering/exiting standby mode. To stop that (for example when advertising), just set accel = True.'''

    def _stop_data_collection(self, accel=False):
        print("data collection stopped")
        if accel:
            self.stop_virtual_timer(self._accel_timer)
        self.stop_virtual_timer(self._temp_timer)
        self.stop_virtual_timer(self._hum_timer)
        self.stop_virtual_timer(self._lux_timer)
        self._clean_collected_data()

    def _clean_collected_data(self):
        self._temperature[0] = 0
        self._temperature[1] = 0
        self._humidity[0] = 0
        self._humidity[1] = 0
        self._lux[0] = 0
        self._lux[1] = 0

    def _send_data(self):
        if not self._connections:
            return

        self.send_led.value(1)

        calculate_mean = lambda data: data[0] / data[1] if data[1] != 0 else 0

        temperature = calculate_mean(self._temperature)
        humidity = calculate_mean(self._humidity)
        lux = calculate_mean(self._lux)
        crash_detection = 0
        sensor_states = 0b101
        anomaly_mask = 0b00000000

        payload = struct.pack("ffffBB", temperature, humidity, lux, crash_detection, sensor_states, anomaly_mask)
        print(payload)

        print("Temp: {:.1f} / Hum: {:.1f} / Lux: {:.1f}".format(temperature, humidity, lux))

        for conn_handle in self._connections:
            self.ble.gatts_notify(conn_handle, self._data_handle, payload)
        '''for conn_handle in self._connections:
            self.ble.gatts_notify(conn_handle, self._state_handle, self.standby)'''

        self.display.write_text("Temp.: {:.1f}C".format(temperature))
        self.display.write_text("Hum.: {:.1f} hPa".format(humidity), clear=False, y=16)
        self.display.write_text("Lux.: {:.1f} lum".format(lux), clear=False, y=32)

        self.send_led.value(0)

        self._clean_collected_data()


# Esegui il server BLE per sensori
ble_sensor = SafeHelmet(data_interval=10)

try:
    while True:
        pass
except KeyboardInterrupt:
    ble_sensor.base_timer.deinit()
    print("Manually stopped")
