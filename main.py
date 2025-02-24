import struct
import ubluetooth
import ubinascii
from machine import Timer, Pin, I2C
import machine
from MPU6050 import MPU6050
from bh1750 import BH1750
import dht

'''
TELE-ESP-BOARD LEDS:
    R: D25
    W: D4?
    G: ?? 
'''

#  TEMP_THRESHOLD = 28.0  # Temperature > 28 °C
#  HUMIDITY_THRESHOLD = 65.0  # Humidity > 65 %
#  LUX_THRESHOLD = 800.0  # Brightness > 800 lux

BASE_INTERVAL = 100  # in ms
ADV_LED_TIMER = 500
DHT_INTERVAL = 2000
LUX_INTERVAL = 2000
CRASH_AND_POSTURE_INTERVAL = 100
GAS_INTERVAL = 1000

GAS_WARMUP_TIME = 120000  # 2 minutes of heating

STANDBY_TRESHOLD = -2.0
WAKEUP_TRESHOLD = 3.0

POSTURE_Z_MIN = 4.0  # Z must be at least 7.0 m/s^2
POSTURE_XY_MAX = 3.0  # X e Y must be included within ±3.0 m/s^2

PIN_DHT = 5
PIN_MQ2 = 18
PIN_MQ4 = 19
PIN_MQ7 = 23
PIN_WELDING_MASK = 25  # not usable on Andrea’s board
PIN_GAS_MASK = 15
PIN_STANDBY_LED = 2
PIN_SEND_LED = 4
PIN_ADV_LED = 17
PIN_VIBRATION_MOTOR = 16
PIN_SCL = 22
PIN_SDA = 21


class SafeHelmet:
    def __init__(self, data_interval=5):

        print("\nInitializing SafeHelmet...")
        # I2C init
        self.i2c = I2C(1, scl=Pin(PIN_SCL), sda=Pin(PIN_SDA), freq=100000)

        # LED init
        self.standby_led = Pin(PIN_STANDBY_LED, Pin.OUT)
        self.standby_led.value(0)

        self.send_led = Pin(PIN_SEND_LED, Pin.OUT)
        self.send_led.value(0)

        self.adv_led = Pin(PIN_ADV_LED, Pin.OUT)
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
        self._crash1_uuid = ubluetooth.UUID(uuids['crash1'])
        self._crash2_uuid = ubluetooth.UUID(uuids['crash2'])
        self._data_uuid = ubluetooth.UUID(uuids['data'])
        self._standby_uuid = ubluetooth.UUID(uuids['standby'])
        self._feedback_uuid = ubluetooth.UUID(uuids['feedback'])

        self._data_char = (
            self._data_uuid,
            ubluetooth.FLAG_NOTIFY,
            [(ubluetooth.UUID(0x0044), ubluetooth.FLAG_READ), ]
        )
        self._accel_char_1 = (
            self._crash1_uuid,
            ubluetooth.FLAG_NOTIFY,
            [(ubluetooth.UUID(0x4331), ubluetooth.FLAG_READ), ]
        )
        self._accel_char_2 = (
            self._crash2_uuid,
            ubluetooth.FLAG_NOTIFY,
            [(ubluetooth.UUID(0x4332), ubluetooth.FLAG_READ), ]
        )
        self._feedback_char = (
            self._feedback_uuid,
            ubluetooth.FLAG_WRITE,  # Important: WRITE flag
            [(ubluetooth.UUID(0x0046), ubluetooth.FLAG_READ), ]
        )
        self._state_char = (
            self._standby_uuid,
            ubluetooth.FLAG_NOTIFY,
            [(ubluetooth.UUID(0x0053), ubluetooth.FLAG_READ), ]
        )
        self._service = (
            self._service_uuid,
            (self._data_char, self._accel_char_1, self._accel_char_2, self._feedback_char, self._state_char)
        )

        [handles] = self.ble.gatts_register_services([self._service])
        self._data_handle = handles[0]
        self._accel_handle_1 = handles[2]
        self._accel_handle_2 = handles[4]
        self._feedback_handle = handles[6]
        self._state_handle = handles[8]

        self._feedback = None

        # Sensors setup and init
        self.standby = False
        self.posture_incorrect_time = 0  # Accumulated time in incorrect posture (in ms)
        self.posture_last_checked = 0  # Timestamp of last verification
        self.gas_anomaly = 0b000

        self.dht_sensor = dht.DHT11(Pin(PIN_DHT))
        self.mq2 = Pin(PIN_MQ2, Pin.IN)
        self.mq4 = Pin(PIN_MQ4, Pin.IN)
        self.mq7 = Pin(PIN_MQ7, Pin.IN)
        self.are_preheated = False

        self.light_sensor = BH1750(self.i2c)
        self.mpu = MPU6050(self.i2c)

        self.welding_mask_gpio = Pin(PIN_WELDING_MASK, Pin.IN, Pin.PULL_UP)
        self.gas_mask_gpio = Pin(PIN_GAS_MASK, Pin.IN, Pin.PULL_UP)

        self.vibration_gpio = Pin(PIN_VIBRATION_MOTOR, Pin.OUT, Pin.PULL_UP)
        self.vibration_gpio.value(0)

        # First value is total sum of readings, second value is number of readings
        self._temperature = [0, 0]
        self._humidity = [0, 0]
        self._lux = [0, 0]

        self._accel_stats = {"x": [0, 0, 0],  # sum, square sum, readings
                             "y": [0, 0, 0],
                             "z": [0, 0, 0],
                             "m": [0, 0, 0],  # acceleration module
                             "current_max": 0}

        self.movement_cumulative = 0  # Cumulative movement between two calls to send_data()
        self.last_accel = {"x": 0, "y": 0, "z": 0}  # Last reading of the accelerometer

        # Physical and virtual timers init
        self.data_interval = data_interval
        self.base_interval = BASE_INTERVAL  # ms precision on virtual timers

        self.virtual_timers = []
        self.timer_id_incremental = 0
        self.oneshot_timer_removal_list = []
        self.adv_led_timer_id = None  # store the id of the adv led timer
        self.data_timer_id = None  # store the id of the data timer
        self.standby_check_timer_id = None
        self.standby_led_timer_id = None
        self._crash_detection_timer = None
        self._dht_timer = None
        self._gas_timer = None

        self._sensor_preheating_start()
        self.adv_led_timer_id = self.create_virtual_timer(ADV_LED_TIMER, self._toggle_adv_led)

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
            'data': create_uuid('1'),  # Data characteristic UUID
            'crash1': create_uuid('2'),  # Crash characteristic UUID
            'crash2': create_uuid('3'),  # Crash characteristic UUID
            'feedback': create_uuid('4'),  # Feedback characteristic UUID
            'standby': create_uuid('5'),  # Standby characteristic UUID
            'service': create_uuid('6')  # Service UUID
        }

    def create_virtual_timer(self, period, callback, one_shot=False):
        self.timer_id_incremental += 1
        timer_id = self.timer_id_incremental  # assign an id equal to the index in the list
        self.virtual_timers.append(
            {"id": timer_id, "period": period, "callback": callback, "counter": 0, "one_shot": one_shot})
        return timer_id  # return the id

    def stop_virtual_timer(self, timer_id):
        # print("Deleting timer {}".format(timer_id))
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

    def _virtual_timer_callback(self, timer):
        for vt in self.virtual_timers:
            vt["counter"] += self.base_interval
            if vt["counter"] >= vt["period"]:
                vt["callback"]()
                # print("Period:{}, counter is: {}".format(vt["period"], vt["counter"]))
                if vt["one_shot"]:
                    self.oneshot_timer_removal_list.append(vt)
                else:
                    vt["counter"] = 0

        for vt in self.oneshot_timer_removal_list:
            self.virtual_timers.remove(vt)
        self.oneshot_timer_removal_list.clear()

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
            self.adv_led_timer_id = self.create_virtual_timer(ADV_LED_TIMER, self._toggle_adv_led)  # restart the timer
            if not self.standby:
                self._stop_data_collection()
            else:
                self.stop_virtual_timer(self.standby_led_timer_id)
                self.stop_virtual_timer(self.standby_check_timer_id)
                self.standby = False

        elif event == 3:
            print(data)
            conn_handle, value = data
            value = bytes(value).decode('utf-8')
            print("Received command: {}".format(value))

    def _start_advertising(self):
        name = b'SafeHelmet-02'
        self.ble.gap_advertise(100,
                               b'\x02\x01\x06' +
                               b'\x03\x03\xf4\x7a' +
                               bytes([len(name) + 1]) + b'\x09' + name)
        print("SafeHelmet is advertising...")

    def _stop_advertising(self):
        self.ble.gap_advertise(interval_us=None)

    def _toggle_adv_led(self):
        self.adv_led.value(not self.adv_led.value())

    def _toggle_standby_led(self):
        self.standby_led.value(not self.standby_led.value())

    #### DATA COLLECTING METHODS ####

    def _read_dht(self):
        try:
            self.dht_sensor.measure()
            self._temperature[0] += self.dht_sensor.temperature()
            self._temperature[1] += 1
            self._humidity[0] += self.dht_sensor.humidity()
            self._humidity[1] += 1
        except Exception as e:
            print(f"Sensor reading error dht11: {e}")

    def _read_lux(self):
        try:
            self._lux[0] += self.light_sensor.luminance()
            self._lux[1] += 1
        except Exception as e:
            print(f"Sensor reading error bh1750: {e}")

    def _set_preheated(self):
        print("Preheating sequence terminated, MQ sensors are operative.")
        self.are_preheated = True

    def _sensor_preheating_start(self):
        self.create_virtual_timer(GAS_WARMUP_TIME, self._set_preheated, one_shot=True)

    def _read_gas(self):
        if self.are_preheated:
            try:
                if not self.mq2.value():  # MQ2 detects gas
                    self.gas_anomaly |= (1 << 2)  # Set the most significant bit (bit 2)

                if not self.mq4.value():  # MQ4 detects gas
                    self.gas_anomaly |= (1 << 1)  # Set the intermediate bit (bit 1)

                if not self.mq7.value():  # MQ7 detects gas
                    self.gas_anomaly |= (1 << 0)  # Set the least significant bit (bit 0)
            except Exception as e:
                self.gas_anomaly = 0b000
                print(f"Error in reading MQ sensors: {e}")

    def _get_orientation(self) -> float:
        return self.mpu.read_accel_data()['z']

    def vibrate(self, duration_ms):
        """
        Activates the vibration motor for the specified duration in milliseconds.
        """
        print("vibrating motor")
        self.vibration_gpio.value(1)  # Activate vibration
        self.create_virtual_timer(duration_ms, self._vibration_stop, one_shot=True)

    def _vibration_stop(self):
        """
        Callback function to stop the vibration motor after its duration.
        """
        print("stop vibrating motor")
        self.vibration_gpio.value(0)  # Deactivate vibration

    def vibration_notify(self):
        self.vibrate(500)
        self.create_virtual_timer(1000, lambda: self.vibrate(500), one_shot=True)
        self.create_virtual_timer(2000, lambda: self.vibrate(500), one_shot=True)

    def _check_crash_and_posture(self):
        try:
            # Read the accelerometer data
            accel_data = self.mpu.read_accel_data()
            x, y, z = accel_data["x"], accel_data["y"], accel_data["z"]
            module = ((x ** 2 + y ** 2 + z ** 2) ** 0.5) / 9.81  # in g

            # Crash detection
            self._accel_stats["m"] = [
                self._accel_stats["m"][0] + module,
                self._accel_stats["m"][1] + module ** 2,
                self._accel_stats["m"][2] + 1
            ]

            self._accel_stats["current_max"] = max(self._accel_stats["current_max"], module)

            for axis, value in zip(["x", "y", "z"], [x, y, z]):
                self._accel_stats[axis][0] += value
                self._accel_stats[axis][1] += value ** 2
                self._accel_stats[axis][2] += 1

            # Check if values are outside the limits
            if abs(x) > POSTURE_XY_MAX or abs(y) > POSTURE_XY_MAX or z < POSTURE_Z_MIN:
                self.posture_incorrect_time += CRASH_AND_POSTURE_INTERVAL  # Increases the incorrect time

            # Calculate the change from previous reading (movement)
            delta_x = x - self.last_accel["x"]
            delta_y = y - self.last_accel["y"]
            delta_z = z - self.last_accel["z"]

            # Add cumulative variation to total movement
            self.movement_cumulative += (delta_x ** 2 + delta_y ** 2 + delta_z ** 2) ** 0.5

            # Update the last reading
            self.last_accel = {"x": x, "y": y, "z": z}
        except Exception as e:
            print(f"Error in reading sensor mpu6050: {e}")

    def _start_data_collection(self):
        """
        Start collecting data from all sensors and calculating mean averages. Each sensor has a different priority in
        worker's safety, thus the need for such a method, that allows to set different retrieval intervals.
        At the time of data packing and sending through BLE, mean averages are calculated on all the data obtained
        between one send and another.
        """
        print("Starting data collection")
        self._dht_timer = self.create_virtual_timer(DHT_INTERVAL, self._read_dht)
        self._lux_timer = self.create_virtual_timer(LUX_INTERVAL, self._read_lux)
        self._gas_timer = self.create_virtual_timer(GAS_INTERVAL, self._read_gas)
        self._crash_detection_and_posture_timer = self.create_virtual_timer(CRASH_AND_POSTURE_INTERVAL,
                                                                            self._check_crash_and_posture)

    def _stop_data_collection(self):
        """
        Stop data collection.
        """
        print("Data collection stopped")
        self.stop_virtual_timer(self._dht_timer)
        self.stop_virtual_timer(self._lux_timer)
        self.stop_virtual_timer(self._crash_detection_and_posture_timer)
        self.stop_virtual_timer(self._gas_timer)
        self._clean_collected_data()

    def _clean_collected_data(self):
        for key in ["_temperature", "_humidity", "_lux"]:
            setattr(self, key, [0, 0])

        self.posture_incorrect_time = 0
        self.movement_cumulative = 0

        self._accel_stats = {"current_max": 0}
        for key in ["x", "y", "z", "m"]:
            self._accel_stats[key] = [0, 0, 0]

        self.gas_anomaly = 0b000

    def _send_data(self):
        if not self._connections:
            return

        if self.last_accel["z"] < STANDBY_TRESHOLD and self.movement_cumulative < 15.0:
            self.movement_cumulative = 0
            self._standby_mode()

        else:

            self.send_led.value(1)

            # WEARABLES DETECTION
            wearables_bitmask = 0
            if self.welding_mask_gpio.value() == 0:  # device connected
                wearables_bitmask = 1
            if self.gas_mask_gpio.value() == 0:  # device connected
                wearables_bitmask |= (1 << 1)

            # DATA MEAN AVERAGE CALCULATION
            calculate_mean = lambda data: data[0] / data[1] if data[1] != 0 else 0
            temperature = calculate_mean(self._temperature)
            humidity = calculate_mean(self._humidity)
            lux = calculate_mean(self._lux)

            if temperature == humidity == lux == 0:
                print("NO DATA")
                return

            if self.gas_anomaly:  # if mask has some bits active, notify the worker for some anomaly through vibration motor
                self.vibration_notify()
            else:  # if not you, maybe someone else in the worksite had some anomaly going on. Check for that and vibrate
                self._feedback = self.ble.gatts_read(self._feedback_handle)
                if self._feedback:
                    self.vibration_notify()
                self.ble.gatts_write(self._feedback_handle, '', True)

            # POSTURE MEAN AVERAGE AND CRASH DETECTION
            accel_dict = {}
            for k, v in self._accel_stats.items():
                if k in ["x", "y", "z", "m"]:
                    mean = v[0] / v[2] if v[2] else 0
                    std_dev = ((v[1] / v[2]) - mean ** 2) ** 0.5 if v[2] else 0
                    accel_dict[k] = {"mean": mean, "std_dev": std_dev}

            print()
            for k, v in accel_dict.items():
                if k != "m":
                    print(f"Axis: {k}, Mean: {v['mean']:.2f} m/s2, StdDev: {v['std_dev']:.2f} m/s2")
                else:
                    print(f"Magnitude, Mean: {v['mean']:.2f} g, StdDev: {v['std_dev']:.2f} g")

            incorrect_posture_percent_raw = (self.posture_incorrect_time / (self.data_interval * 1000))
            print("Percentage of time spent in incorrect posture: {}%".format(incorrect_posture_percent_raw * 100))

            # Create the payload
            data_payload = struct.pack("fffBB",
                                       temperature,
                                       humidity,
                                       lux,
                                       self.gas_anomaly,
                                       wearables_bitmask
                                       )

            print(
                "Temp: {:.1f} / Hum: {:.1f} / Lux: {:.1f} / Crash: {:.2f} / Sensor States: {:03b} / Wearables {:02b}".format(
                    temperature, humidity, lux, self._accel_stats["current_max"], self.gas_anomaly, wearables_bitmask))

            accel_payload_1 = struct.pack("fffff",
                                          accel_dict["x"]["mean"],
                                          accel_dict["y"]["mean"],
                                          accel_dict["z"]["mean"],
                                          accel_dict["m"]["mean"],
                                          accel_dict["m"]["std_dev"]
                                          )

            accel_payload_2 = struct.pack("fffff",
                                          accel_dict["x"]["std_dev"],
                                          accel_dict["y"]["std_dev"],
                                          accel_dict["z"]["std_dev"],
                                          self._accel_stats["current_max"],
                                          incorrect_posture_percent_raw
                                          )

            # Send payload to connected devices
            for conn_handle in self._connections:
                self.ble.gatts_notify(conn_handle, self._data_handle, data_payload)
                self.ble.gatts_notify(conn_handle, self._accel_handle_1, accel_payload_1)
                self.ble.gatts_notify(conn_handle, self._accel_handle_2, accel_payload_2)

            self.send_led.value(0)

            # Clean up the data collected
            self._clean_collected_data()

    def _standby_mode(self):
        print("Entering standby mode")
        self._stop_data_collection()
        self.standby = True
        for conn_handle in self._connections:
            self.ble.gatts_notify(conn_handle, self._state_handle, (0x1).to_bytes(1, 'little'))

        self.stop_virtual_timer(self.data_timer_id)
        self.data_timer_id = None
        self.standby_led_timer_id = self.create_virtual_timer(500, self._toggle_standby_led)
        self.standby_check_timer_id = self.create_virtual_timer(5000, self._check_standby_exit)

    def _check_standby_exit(self):
        if self._get_orientation() > STANDBY_TRESHOLD:
            self.stop_virtual_timer(self.standby_led_timer_id)
            self.stop_virtual_timer(self.standby_check_timer_id)
            self.standby_led.value(0)
            self.standby = False
            print("Exiting standby mode")
            for conn_handle in self._connections:
                self.ble.gatts_notify(conn_handle, self._state_handle, (0x0).to_bytes(1, 'little'))

            self._clean_collected_data()
            self._start_data_collection()
            if self.data_timer_id is not None:
                self.stop_virtual_timer(self.data_timer_id)
            self.data_timer_id = self.create_virtual_timer(self.data_interval * 1000, self._send_data)


# Run the BLE server for sensors
ble_sensor = SafeHelmet(data_interval=5)

try:
    while True:
        pass
except KeyboardInterrupt:
    ble_sensor.base_timer.deinit()
    print("Manually stopped")
