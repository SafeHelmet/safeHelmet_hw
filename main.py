import ubluetooth
from machine import Timer, Pin, I2C
import time
import bme280
import ssd1306  # Libreria per il display OLED

'''
TELE-ESP-BOARD LEDS:
    R: D25
    W: D4?
    G: ?? 
'''

class Display:

    def __init__(self, i2c_obj):
        self.last_status = []
        self._priority_flag = False
        self.text_timer = Timer(-1)
        self.oled = ssd1306.SSD1306_I2C(128, 64, i2c_obj)  # Display OLED 128x64

    def write_text(self, text, x=0, y=0, clear=True):
        if [text, x, y, clear] != self.last_status and not self._priority_flag:
            if clear:
                self.oled.fill(0)  # Pulisce lo schermo

            lines = self._wrap_text(text) if len(text) > 14 else [text]
            for i, line in enumerate(lines):
                self.oled.text(line, x, y + i * 10)  # Adjust y-offset for each line
            self.oled.show()
            self.last_status = [text, x, y, clear]

        if self._priority_flag:  # queue text, it will be displayed after important message
            self.last_status = [text, x, y, clear]

    def clear_permanent_text(self, timer):
        self.oled.fill(0)
        self._priority_flag = False
        print(self._priority_flag)
        self.text_timer.deinit()
        print(self.last_status)
        self.write_text(*self.last_status)

    def write_permanent_text(self, text, x=0, y=0, duration=1):
        self.oled.fill(0)
        lines = self._wrap_text(text) if len(text) > 14 else [text]
        for i, line in enumerate(lines):
            self.oled.text(line, x, y + i * 10)  # Adjust y-offset for each line
        self.oled.show()
        self._priority_flag = True
        self.text_timer.init(period=duration * 1000, mode=Timer.ONE_SHOT, callback=self.clear_permanent_text)

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
    def __init__(self, interval=5):
        self.i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=100000)  # Imposta I2C (SCL, SDA)

        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self._irq)

        self.standby_led = Pin(2, Pin.OUT)  # GPIO2 su ESP32 per indicazione (opzionale)
        self.standby_led.value(0)

        self.collect_led = Pin(4, Pin.OUT)
        self.collect_led.value(0)

        self._connections = set()
        self._service_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d479")
        self._temp_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d480")  # UUID per temperatura
        self._press_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d481")  # UUID per pressione
        self._state_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d482")  # UUID per stato (standby)

        # Configurazione delle caratteristiche
        self._temp_char = (self._temp_uuid, ubluetooth.FLAG_NOTIFY | ubluetooth.FLAG_READ)
        self._press_char = (self._press_uuid, ubluetooth.FLAG_NOTIFY | ubluetooth.FLAG_READ)
        self._state_char = (self._state_uuid, ubluetooth.FLAG_NOTIFY | ubluetooth.FLAG_READ)
        self._service = (self._service_uuid, (self._temp_char, self._press_char, self._state_char))

        # Registra il servizio BLE
        ((self._temp_handle, self._press_handle, self._state_handle),) = self.ble.gatts_register_services(
            [self._service])

        # Stato del dispositivo
        self.standby = False
        self.standby_timer = Timer(-1)

        # Imposta intervallo per raccolta dati
        self.interval = interval
        self.timer = Timer(-1)

        self.adv_led = Pin(25, Pin.OUT)  # D5 corrisponde a GPIO14
        self.adv_led.value(0)
        self.adv_timer = Timer(-1)  # Timer per il lampeggio del LED di advertising

        self.temp_sensor = bme280.BME280(i2c=self.i2c)

        # Impostazioni per il display OLED
        self.display = Display(self.i2c)

        self._start_advertising()

    def _irq(self, event, data):
        if event == 1:  # Connessione avvenuta
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("Device connected: {}".format(conn_handle))
            self.adv_timer.deinit()
            self.adv_led.value(0)

            self.timer.init(period=self.interval * 1000, mode=Timer.PERIODIC, callback=self._collect_and_send_data)
            self.display.write_permanent_text("permanent",duration=10)

        elif event == 2:  # Disconnessione
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            print("Device disconnected")
            self._start_advertising()
            self.display.write_text('Waiting for connection...', y=16)
        elif event == 3:  # Scrittura su caratteristica
            print("Writing is not supported on this characteristic")

    def _start_advertising(self):
        name = b'SafeHelmet-01'
        self.ble.gap_advertise(100,
                               b'\x02\x01\x06' +  # Flag generale BLE
                               b'\x03\x03\xf4\x7a' +  # UUID servizio (parziale)
                               bytes([len(name) + 1]) + b'\x09' + name)  # Nome dispositivo

        # Avvia lampeggio del LED durante l'advertising
        self.adv_timer.init(period=500, mode=Timer.PERIODIC, callback=self._toggle_adv_led)

        print("SafeHelmet is advertising...")

    def _toggle_adv_led(self, timer):
        self.adv_led.value(not self.adv_led.value())  # Inverti lo stato del LED

    def _collect_and_send_data(self, timer):
        if self.standby:
            self._check_exit_standby()
            return

        if not self._connections:
            return  # Nessun dispositivo connesso, non inviare dati

        self.collect_led.value(1)
        # Legge i dati dal sensore BME280
        temperature, pressure, _ = self.temp_sensor.read_compensated_data()
        temperature = temperature  # Temperatura in °C
        pressure = pressure / 100  # Pressione in hPa

        print("Temperature: {:.1f} / Pressure: {:.1f}".format(temperature, pressure))

        # Invia i dati di temperatura
        for conn_handle in self._connections:
            self.ble.gatts_notify(conn_handle, self._temp_handle, "{:.1f}".format(temperature).encode())

        # Invia i dati di pressione
        for conn_handle in self._connections:
            self.ble.gatts_notify(conn_handle, self._press_handle, "{:.1f}".format(pressure).encode())

        # Visualizza i dati sul display OLED
        self.display.write_text("Temp: {:.1f}C".format(temperature))
        self.display.write_text("Press: {:.1f} hPa".format(pressure), clear=False, y=32)

        self.collect_led.value(0)

        # Controlla se deve entrare in standby
        if temperature > 23:
            self._enter_standby()

    def _enter_standby(self):
        print("Entering standby mode...")
        for conn_handle in self._connections:
            self.ble.gatts_notify(conn_handle, self._state_handle, b"Entering standby")

        self.standby = True
        self.timer.deinit()  # Ferma la raccolta periodica di dati

        self.display.write_text('--Standby mode--')

        self.standby_timer.init(period=1000, mode=Timer.PERIODIC, callback=self._standby_mode)

    def _standby_mode(self, timer):
        self.standby_led.value(not self.standby_led.value())  # Lampeggia il LED
        self._check_exit_standby()

    def _check_exit_standby(self):
        temperature, _, _ = self.temp_sensor.read_compensated_data()
        if temperature <= 23:
            print("Exiting standby mode...")
            for conn_handle in self._connections:
                self.ble.gatts_notify(conn_handle, self._state_handle, b"Exiting standby")

            self.standby = False
            self.standby_led.value(0)
            self.standby_timer.deinit()
            self.timer.init(period=self.interval * 1000, mode=Timer.PERIODIC, callback=self._collect_and_send_data)


# Esegui il server BLE per sensori
ble_sensor = SafeHelmet(interval=5)

try:
    while True:
        pass
except KeyboardInterrupt:
    print("Manually stopped")
