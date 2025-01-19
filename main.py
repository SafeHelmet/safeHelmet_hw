import random
import struct

import ubluetooth
import ubinascii
from machine import Timer, Pin, I2C
import machine
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


TEMP_THRESHOLD = 28.0  # Temperatura > 28 °C
HUMIDITY_THRESHOLD = 65.0  # Umidità > 65 %
LUX_THRESHOLD = 800.0  # Luminosità > 800 lux
CRASH_THRESHOLD = 3.0  # Crash detection > 3 g


class SafeHelmet:
    def __init__(self, data_interval=5):
        print("\nInitializing SafeHelmet...")

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

        # Generate unique UUIDs for this device
        uuids = self.generate_uuids()
        print("\nThis device has UUIDs:")
        for key, uuid in uuids.items():
            print("{}: {}".format(key, uuid))
        print("\n")

        # Convert string UUIDs to ubluetooth.UUID objects
        self._service_uuid = ubluetooth.UUID(uuids['service'])
        self._data_uuid = ubluetooth.UUID(uuids['data'])
        self._state_uuid = ubluetooth.UUID(uuids['state'])

        self._data_char = (self._data_uuid, ubluetooth.FLAG_NOTIFY, [(ubluetooth.UUID(0x0044), ubluetooth.FLAG_READ), ])
        self._state_char = (
            self._state_uuid, ubluetooth.FLAG_NOTIFY, [(ubluetooth.UUID(0x0053), ubluetooth.FLAG_READ), ])
        self._service = (self._service_uuid, [self._data_char, self._state_char])

        # Fix: Correct unpacking of service handles
        [handles] = self.ble.gatts_register_services([self._service])
        self._data_handle = handles[0]
        self._state_handle = handles[1]

        # Sensors setup and init
        self.standby = False

        # First value is total sum of readings, second value is number of readings
        self._temperature = [0, 0]
        self._humidity = [0, 0]
        self._lux = [0, 0]

        # Physical and virtual timers init
        self.data_interval = data_interval
        self.base_interval = 60  # 100ms precision on virtual timers

        self.virtual_timers = []
        self.oneshot_timer_removal_list = []
        self.adv_led_timer_id = None  # store the id of the adv led timer
        self.data_timer_id = None  # store the id of the data timer

        self.data_timer_id = self.create_virtual_timer(self.data_interval * 1000, self._send_data)
        self.adv_led_timer_id = self.create_virtual_timer(500, self._toggle_adv_led)

        self.base_timer = Timer(-1)
        self.base_timer.init(period=self.base_interval, mode=Timer.PERIODIC, callback=self._virtual_timer_callback)


        print("Device is ready.")

        # Start advertising process after startup
        self._start_advertising()

    def generate_uuids(self):
        # Get unique ID from the microcontroller
        unique_id = ubinascii.hexlify(machine.unique_id()).decode('utf-8')

        # Pad unique_id if necessary to ensure we have enough characters
        while len(unique_id) < 32:  # We need at least 32 chars
            unique_id = unique_id + unique_id

        def create_uuid(prefix):
            # Format: xxxxxxxx-58cc-4372-a567-xxxxxxxxxxxx
            # We use different parts of unique_id for each UUID, prefixed to ensure uniqueness
            uuid_base = prefix + unique_id[:24]  # Use prefix and part of unique_id
            return (f"{uuid_base[:8]}-58cc-4372-a567-{uuid_base[8:20]}")

        return {
            'service': create_uuid('1'),  # Service UUID
            'data': create_uuid('2'),  # Data characteristic UUID
            'state': create_uuid('3')  # State characteristic UUID
        }

    def _virtual_timer_callback(self, timer):
        for vt in self.virtual_timers:
            vt["counter"] += self.base_interval
            if vt["counter"] >= vt["period"]:
                vt["callback"]()
                # print("period:{}, counter is: {}".format(vt["period"], vt["counter"]))
                if vt["one_shot"]:
                    self.oneshot_timer_removal_list.append(vt)
                else:
                    vt["counter"] = 0

        for vt in self.oneshot_timer_removal_list:
            self.virtual_timers.remove(vt)
        self.oneshot_timer_removal_list.clear()

    def create_virtual_timer(self, period, callback, one_shot=False):
        timer_id = len(self.virtual_timers)  # assign an id equal to the index in the list
        self.virtual_timers.append(
            {"id": timer_id, "period": period, "callback": callback, "counter": 0, "one_shot": one_shot})
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


        elif event == 2:
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            print("Device disconnected")
            self._start_advertising()
            self.adv_led_timer_id = self.create_virtual_timer(500, self._toggle_adv_led)  # restart the timer

        elif event == 3:
            print("Writing is not supported on this characteristic")

    def _start_advertising(self):
        name = b'SafeHelmet-01'
        self.ble.gap_advertise(100,
                               b'\x02\x01\x06' +
                               b'\x03\x03\xf4\x7a' +
                               bytes([len(name) + 1]) + b'\x09' + name)
        print("SafeHelmet is advertising...")

    def _stop_advertising(self):
        self.ble.gap_advertise(interval_us=None)

    def _toggle_adv_led(self):
        self.adv_led.value(not self.adv_led.value())

    def _send_data(self):
        if not self._connections:
            return

        self.send_led.value(1)

        # Simula i dati casuali
        temperature = round(random.uniform(20.0, 30.0), 1)  # Temperatura simulata (20-30 °C)
        humidity = round(random.uniform(30.0, 70.0), 1)  # Umidità simulata (30-70 %)
        lux = round(random.uniform(100.0, 1000.0), 1)  # Luminosità simulata (100-1000 lux)
        crash_detection = round(random.uniform(0.0, 5.0), 2)  # Crash simulato (0-5 g)

        # Simula lo stato dei sensori di gas con probabilità del 10% di valore "1" per ciascun bit
        sensor_states = 0
        for i in range(3):  # Tre sensori
            if random.random() < 0.1:  # 10% di probabilità
                sensor_states |= (1 << i)  # Imposta il bit i-esimo a 1

        # Maschera di anomalie (per ora impostata a zero)
        anomaly_mask = 0b00000000

        if temperature > TEMP_THRESHOLD:
            anomaly_mask |= 0b00010000  # Anomalia temperatura
        if humidity > HUMIDITY_THRESHOLD:
            anomaly_mask |= 0b00001000  # Anomalia umidità
        if lux > LUX_THRESHOLD:
            anomaly_mask |= 0b00000100  # Anomalia luminosità
        if crash_detection > CRASH_THRESHOLD:
            anomaly_mask |= 0b00000010  # Anomalia crash detection
        if sensor_states != 0:
            anomaly_mask |= 0b00000001  # Anomalia sensori di gas (almeno uno attivo)

        print(anomaly_mask)

        # Crea il payload
        payload = struct.pack("ffffBB", temperature, humidity, lux, crash_detection, sensor_states, anomaly_mask)
        print(payload)

        print("Temp: {:.1f} / Hum: {:.1f} / Lux: {:.1f} / Crash: {:.2f} / Sensor States: {:03b}".format(
            temperature, humidity, lux, crash_detection, sensor_states))

        # Invia il payload ai dispositivi connessi
        for conn_handle in self._connections:
            self.ble.gatts_notify(conn_handle, self._data_handle, payload)

        self.send_led.value(0)



# Esegui il server BLE per sensori
ble_sensor = SafeHelmet(data_interval=10)

try:
    while True:
        pass
except KeyboardInterrupt:
    ble_sensor.base_timer.deinit()
    print("Manually stopped")

