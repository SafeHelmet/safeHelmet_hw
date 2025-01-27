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
from math import sqrt

'''
TELE-ESP-BOARD LEDS:
    R: D25
    W: D4?
    G: ?? 
'''

# Set this to enable/disable ssd1306 display functionality
DISPLAY = 0

TEMP_THRESHOLD = 28.0  # Temperatura > 28 °C
HUMIDITY_THRESHOLD = 65.0  # Umidità > 65 %
LUX_THRESHOLD = 800.0  # Luminosità > 800 lux
CRASH_THRESHOLD = 5.0 * 9.81  # Crash detection > 5 g
#  MAGNITUDO_WINDOW = 20 #  16
CRASH_CHECK_INTERVAL = 200  # in ms

STANDBY_TRESHOLD = -2.0
WAKEUP_TRESHOLD = 3.0

POSTURE_Z_MIN = 4.0  # Z deve essere almeno 7.0 m/s^2
POSTURE_XY_MAX = 3.0  # X e Y devono essere compresi entro ±3.0 m/s^2
POSTURE_CHECK_INTERVAL = 1000  # in ms


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

            # force split
            lines.append(text[start:end].strip())
            start = end
        return lines


class SafeHelmet:
    def __init__(self, data_interval=5):

        print("\nInitializing SafeHelmet...")
        # I2C init
        self.i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=100000)

        # LED init
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

        self._data_char = (
            self._data_uuid,
            ubluetooth.FLAG_NOTIFY,
            [(ubluetooth.UUID(0x0044), ubluetooth.FLAG_READ), ]
        )
        self._state_char = (
            self._state_uuid,
            ubluetooth.FLAG_NOTIFY,
            [(ubluetooth.UUID(0x0053), ubluetooth.FLAG_READ), ]
        )
        self._service = (
            self._service_uuid,
            (self._data_char, self._state_char)
        )

        [handles] = self.ble.gatts_register_services([self._service])
        self._data_handle = handles[0]
        self._state_handle = handles[2]

        # Sensors setup and init
        self.standby = False
        self.posture_incorrect_time = 0  # Tempo accumulato in postura scorretta (in ms)
        self.posture_last_checked = 0  # Timestamp dell'ultima verifica

        self.dht_sensor = dht.DHT11(Pin(18))
        self.display = Display(self.i2c)
        self.light_sensor = BH1750(self.i2c)
        self.mpu = MPU6050(self.i2c)

        self.welding_mask_gpio = Pin(13, Pin.IN, Pin.PULL_UP)
        self.gas_mask_gpio = Pin(12, Pin.IN, Pin.PULL_UP)

        self.vibration_gpio = Pin(14, Pin.OUT, Pin.PULL_UP)
        self.vibration_gpio.value(1)

        # First value is total sum of readings, second value is number of readings
        self._temperature = [0, 0]
        self._humidity = [0, 0]
        self._lux = [0, 0]
        self._posture = {"x": [0, 0], "y": [0, 0], "z": [0, 0]}
        #  self._magnitudo = []
        #  self._magnitudo_mean = 0
        #  self._magnitudo_dev_std = 0
        #  self._magnitudo_max = 0
        #  self._is_crash = 0
        self._module_stats = {
            "sum_acc": 0,  # Somma delle accelerazioni
            "sum_acc_squared": 0,  # Somma dei quadrati delle accelerazioni
            "count": 0,  # Numero di campioni
            "crash_flag": False,  # Flag per il crash
            "current_max": float('-inf')  # Valore massimo nell'intervallo
        }

        # Physical and virtual timers init
        self.data_interval = data_interval
        self.base_interval = 60  # 100ms precision on virtual timers

        self.virtual_timers = []
        self.oneshot_timer_removal_list = []
        self.adv_led_timer_id = None  # store the id of the adv led timer
        self.data_timer_id = None  # store the id of the data timer
        self.standby_check_timer_id = None
        self.standby_led_timer_id = None
        self._crash_detection_timer = None

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
        # print(timer_id)
        self.virtual_timers.append(
            {"id": timer_id, "period": period, "callback": callback, "counter": 0, "one_shot": one_shot})
        return timer_id  # return the id

    def stop_virtual_timer(self, timer_id):
        # print("deleting timer {}".format(timer_id))
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
            if self.standby:
                self.stop_virtual_timer(self.standby_led_timer_id)
                self.stop_virtual_timer(self.standby_check_timer_id)
                self.standby = False
            if self.data_timer_id is None:
                self.data_timer_id = self.create_virtual_timer(self.data_interval * 1000, self._send_data)
            self._start_data_collection()

        elif event == 2:
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            print("Device disconnected")
            self._start_advertising()
            self.adv_led_timer_id = self.create_virtual_timer(500, self._toggle_adv_led)  # restart the timer
            if not self.standby:
                self._stop_data_collection()
            else:
                self.stop_virtual_timer(self.standby_led_timer_id)
                self.stop_virtual_timer(self.standby_check_timer_id)
                self.standby = False

        elif event == 3:
            print("Writing is not supported on this characteristic")

    def _start_advertising(self):
        name = b'SafeHelmet-01'
        self.ble.gap_advertise(100,
                               b'\x02\x01\x06' +
                               b'\x03\x03\xf4\x7a' +
                               bytes([len(name) + 1]) + b'\x09' + name)
        print("SafeHelmet is advertising...")
        self.display.write_text('Waiting for connection...', y=20)

    def _stop_advertising(self):
        self.ble.gap_advertise(interval_us=None)

    def _toggle_adv_led(self):
        self.adv_led.value(not self.adv_led.value())

    def _toggle_standby_led(self):
        self.standby_led.value(not self.standby_led.value())

    #### DATA COLLECTING METHODS ####

    def _check_posture(self):
        # Leggi i dati dell'accelerometro
        accel_data = self.mpu.read_accel_data()
        x, y, z = accel_data["x"], accel_data["y"], accel_data["z"]

        self._posture["x"][0] += x
        self._posture["y"][0] += y
        self._posture["z"][0] += z
        self._posture["x"][1] += 1
        self._posture["y"][1] += 1
        self._posture["z"][1] += 1

        print(accel_data)

        # Verifica se i valori sono fuori dai limiti
        if abs(x) > POSTURE_XY_MAX or abs(y) > POSTURE_XY_MAX or z < POSTURE_Z_MIN:
            self.posture_incorrect_time += POSTURE_CHECK_INTERVAL  # Incrementa il tempo scorretto

    def _read_temperature(self):
        try:
            self.dht_sensor.measure()
            self._temperature[0] += self.dht_sensor.temperature()
            self._temperature[1] += 1
            #  print("TEMPERATURA LETTA")
        except OSError as e:
            print(f"Errore nella lettura della temperatura: {e}")

    def _read_humidity(self):
        try:
            self.dht_sensor.measure()
            self._humidity[0] += self.dht_sensor.humidity()
            self._humidity[1] += 1
            #  print("UMIDITA LETTA")
        except OSError as e:
            print(f"Errore nella lettura dell'umidità: {e}")

    def _read_dht(self):
        try:
            self.dht_sensor.measure()
            self._temperature[0] += self.dht_sensor.temperature()
            self._temperature[1] += 1
            self._humidity[0] += self.dht_sensor.humidity()
            self._humidity[1] += 1
            #  print("DATI LETTI")
        except OSError as e:
            print(f"Errore nella lettura dei dati: {e}")

    def _read_lux(self):
        self._lux[0] += self.light_sensor.luminance()
        self._lux[1] += 1

    def _get_orientation(self) -> float:
        return self.mpu.read_accel_data()['z']

    def vibrate(self, duration_ms):
        """
        Activates the vibration motor for the specified duration in milliseconds.
        """
        print("vibrating motor")
        self.vibration_gpio.value(0)  # Activate vibration
        self.create_virtual_timer(duration_ms, self._vibration_stop, one_shot=True)

    def _vibration_stop(self):
        """
        Callback function to stop the vibration motor after its duration.
        """
        print("stop vibrating motor")
        self.vibration_gpio.value(1)  # Deactivate vibration

    def vibration_notify(self):
        self.vibrate(500)
        self.create_virtual_timer(1000, lambda: self.vibrate(500), one_shot=True)
        self.create_virtual_timer(2000, lambda: self.vibrate(500), one_shot=True)

    def vibration_anomaly(self):
        self.vibrate(1000)
        self.create_virtual_timer(2000, lambda: self.vibrate(1000), one_shot=True)
        self.create_virtual_timer(4000, lambda: self.vibrate(1000), one_shot=True)
        self.create_virtual_timer(6000, lambda: self.vibrate(1000), one_shot=True)

    def _detect_crash(self):
        module = self.mpu.read_accel_abs()
        self._module_stats["count"] += 1
        self._module_stats["sum_acc"] += module
        self._module_stats["sum_acc_squared"] += module ** 2
        if module > CRASH_THRESHOLD:
            print(f"Urto rilevato! Modulo: {module:.1f} m/s2")
            self._module_stats["crash_flag"] = True
        self._module_stats["current_max"] = max(self._module_stats["current_max"], module)

    def _start_data_collection(self):
        """
        Start collecting data from all sensors and calculating mean averages. Each sensor has a different priority in
        worker's safety, thus the need for such a method, that allows to set different retrieval intervals.
        At the time of data packing and sending through BLE, mean averages are calculated on all the data obtained
        between one send and another.
        """
        #  self._temp_timer = self.create_virtual_timer(1000, self._read_temperature)
        #  self._hum_timer = self.create_virtual_timer(3000, self._read_humidity)
        self._dht_timer = self.create_virtual_timer(2000, self._read_dht)
        self._lux_timer = self.create_virtual_timer(2000, self._read_lux)
        self._posture_timer = self.create_virtual_timer(POSTURE_CHECK_INTERVAL, self._check_posture)
        self._crash_detection_timer = self.create_virtual_timer(CRASH_CHECK_INTERVAL, self._detect_crash)

    def _stop_data_collection(self):
        """
        Stop data collection. By default, accelerometer data continues to be retrieved in order to provide
        for entering/exiting standby mode. To stop that (for example when advertising), just set accel = True.
        """
        print("data collection stopped")
        #  self.stop_virtual_timer(self._temp_timer)
        #  self.stop_virtual_timer(self._hum_timer)
        self.stop_virtual_timer(self._dht_timer)
        self.stop_virtual_timer(self._lux_timer)
        self.stop_virtual_timer(self._posture_timer)
        self.stop_virtual_timer(self._crash_detection_timer)
        self._clean_collected_data()

    def _clean_collected_data(self):
        self._temperature[0] = 0
        self._temperature[1] = 0
        self._humidity[0] = 0
        self._humidity[1] = 0
        self._lux[0] = 0
        self._lux[1] = 0
        self.posture_incorrect_time = 0  # reset posture incorrect time
        #  self._magnitudo = []
        #  self._is_crash = 0
        self._module_stats["sum_acc"] = 0
        self._module_stats["sum_acc_squared"] = 0
        self._module_stats["count"] = 0
        self._module_stats["crash_flag"] = False
        self._module_stats["current_max"] = float('-inf')

    def _send_data(self):
        if not self._connections:
            return

        if self._get_orientation() > STANDBY_TRESHOLD:

            self.send_led.value(1)

            wearables_bitmask = 0

            if self.welding_mask_gpio.value() == 0:  # device connected
                wearables_bitmask = 1
            if self.gas_mask_gpio.value() == 0:  # device connected
                wearables_bitmask |= (1 << 1)

            calculate_mean = lambda data: data[0] / data[1] if data[1] != 0 else 0
            temperature = calculate_mean(self._temperature)
            humidity = calculate_mean(self._humidity)
            lux = calculate_mean(self._lux)

            posture_dict = {}
            for k, v in self._posture.items():
                posture_dict[k] = v[0] / v[1] if v[1] != 0 else 0  # evitare ZeroDivisionError

            print(posture_dict)

            incorrect_posture_percent_raw = (self.posture_incorrect_time / (self.data_interval * 1000))
            print("Totale: {} - Incorretto: {}".format(self.data_interval * 1000, self.posture_incorrect_time))
            print("perc. tempo passato in postura scorretta: {}%".format(incorrect_posture_percent_raw * 100))
            self.posture_incorrect_time = 0

            #  crash_detection = 0
            #  if len(self._magnitudo) == MAGNITUDO_WINDOW:
            #      self.accel_stats()
            #      print("MEDIA MAGNITUDO: {:.1f} / DEV_STD MAGNITUDO: {:.1f} / MAX_MAGNITUDO: {:.1f}".format(self._magnitudo_mean, self._magnitudo_dev_std, self._magnitudo_max))

            if self._module_stats["count"] == 0:
                print("Nessun dato da inviare.")
            else:
                # Calcola media e deviazione standard
                mean_acc = self._module_stats["sum_acc"] / self._module_stats["count"]
                variance = (self._module_stats["sum_acc_squared"] / self._module_stats["count"]) - (mean_acc ** 2)
                std_dev = sqrt(variance)
                # Invia i dati
                print(f"Media: {mean_acc:.2f}, Deviazione Standard: {std_dev:.2f}, "
                      f"Max: {self._module_stats['current_max']:.2f}, Crash: {self._module_stats['crash_flag']}")

            # Simula lo stato dei sensori di gas con probabilità del 10% di valore "1" per ciascun bit
            sensor_states = 0
            for i in range(3):  # Tre sensori
                if random.random() < 0.1:  # 10% di probabilità
                    sensor_states |= (1 << i)  # Imposta il bit i-esimo a 1

            if sensor_states:  # if mask has some bits active, notify the worker for some anomaly through vibration motor
                pass
                # self.vibration_notify()

            # Crea il payload
            payload = struct.pack("ffffBB",
                                  temperature,
                                  humidity,
                                  lux,
                                  self._module_stats["crash_flag"],  # crash_detection,
                                  sensor_states,
                                  wearables_bitmask
                                  )

            print(
                "Temp: {:.1f} / Hum: {:.1f} / Lux: {:.1f} / Crash: {:.2f} / Sensor States: {:03b} / Wearables {:02b}".format(
                    temperature, humidity, lux, self._module_stats["crash_flag"], sensor_states, wearables_bitmask))

            # Invia il payload ai dispositivi connessi
            for conn_handle in self._connections:
                self.ble.gatts_notify(conn_handle, self._data_handle, payload)

            # Aggiorna il display
            self.display.write_text("Temp.: {:.1f}C".format(temperature))
            self.display.write_text("Hum.: {:.1f} hPa".format(humidity), clear=False, y=10)
            self.display.write_text("Lux.: {:.1f}".format(lux), clear=False, y=20)
            self.display.write_text("Crash: {:.2f} g".format(self._module_stats["crash_flag"]), clear=False, y=30)
            self.display.write_text("Sensors: {:03b}".format(sensor_states), clear=False, y=40)

            self.send_led.value(0)

            # Pulisce i dati raccolti
            self._clean_collected_data()
        else:
            self._stop_data_collection()
            self._standby_mode()

    def _standby_mode(self):
        print("Entering standby mode")
        self.standby = True
        for conn_handle in self._connections:
            self.ble.gatts_notify(conn_handle, self._state_handle, "Sleep: ON")
        self.stop_virtual_timer(self.data_timer_id)
        self.data_timer_id = None
        self.standby_led_timer_id = self.create_virtual_timer(500, self._toggle_standby_led)
        self.standby_check_timer_id = self.create_virtual_timer(5000, self._check_standby_exit)
        self.display.write_text("-Standby mode-")

    def _check_standby_exit(self):
        if self._get_orientation() > WAKEUP_TRESHOLD:
            self.stop_virtual_timer(self.standby_led_timer_id)
            self.stop_virtual_timer(self.standby_check_timer_id)
            self.standby_led.value(0)
            self.standby = False
            print("Exiting standby mode")
            for conn_handle in self._connections:
                self.ble.gatts_notify(conn_handle, self._state_handle, "Sleep: OFF")

            self._clean_collected_data()
            self._start_data_collection()
            if self.data_timer_id is not None:
                self.stop_virtual_timer(self.data_timer_id)
            self.data_timer_id = self.create_virtual_timer(self.data_interval * 1000, self._send_data)


# Esegui il server BLE per sensori
ble_sensor = SafeHelmet(data_interval=5)

try:
    while True:
        pass
except KeyboardInterrupt:
    ble_sensor.base_timer.deinit()
    print("Manually stopped")
