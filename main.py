import ubluetooth
from machine import Timer, Pin, I2C
import time
import bme280
import ssd1306  # Libreria per il display OLED
import struct

class BLESensor:
    def __init__(self, interval=5):
        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self._irq)

        self.led = Pin(2, Pin.OUT)  # GPIO2 su ESP32 per indicazione (opzionale)
        self.led.value(0)

        self._connections = set()
        self._service_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d479")
        self._data_uuid = ubluetooth.UUID("f47ac10b-58cc-4372-a567-0e02b2c3d482")

        # Configurazione della caratteristica NOTIFY
        self._data_char = (self._data_uuid, ubluetooth.FLAG_NOTIFY, )
        self._service = (self._service_uuid, (self._data_char,))

        # Registra il servizio BLE
        ((self._data_handle,),) = self.ble.gatts_register_services([self._service])

        # Imposta intervallo per raccolta dati
        self.interval = interval
        self.timer = Timer(-1)
        self.timer.init(period=self.interval * 1000, mode=Timer.PERIODIC, callback=self._collect_and_send_data)

        self._start_advertising()

        # Impostazioni per il sensore BME280
        self.i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=100000)  # Imposta I2C (SCL, SDA)
        self.sensor = bme280.BME280(i2c=self.i2c)  # Crea l'oggetto sensore

        # Impostazioni per il display OLED
        self.oled = ssd1306.SSD1306_I2C(128, 64, self.i2c)  # Display OLED 128x64
        self.oled.fill(0)  # Pulisce lo schermo
        self.oled.text('waiting for device connection...',0,0)
        self.oled.show()

        # Stato del dispositivo
        self.standby = False
        self.standby_timer = Timer(-1)

    def _irq(self, event, data):
        if event == 1:  # Connessione avvenuta
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("Dispositivo connesso")
        elif event == 2:  # Disconnessione
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            print("Dispositivo disconnesso")
        elif event == 3:  # Scrittura su caratteristica
            print("Scrittura non supportata per questa caratteristica")

    def _start_advertising(self):
        name = b'ESP32-Sensor'
        self.ble.gap_advertise(100,
                               b'\x02\x01\x06' +  # Flag generale BLE
                               b'\x03\x03\xf4\x7a' +  # UUID servizio (parziale)
                               bytes([len(name) + 1]) + b'\x09' + name)  # Nome dispositivo
        print("Advertising in corso...")

    def _collect_and_send_data(self, timer):
        if self.standby:
            self._check_exit_standby()
            return

        if not self._connections:
            return  # Nessun dispositivo connesso, non inviare dati

        # Legge i dati dal sensore BME280
        temperature, pressure, _ = self.sensor.read_compensated_data()
        temperature = temperature  # Temperatura in °C
        pressure = pressure / 100  # Pressione in hPa


        # Prepara i dati da inviare
        binary_data = struct.pack("ff", temperature, pressure)
        print("Temperature: {:.1f} / Pressure: {:.1f}".format(temperature, pressure))

        # Invia i dati a tutti i dispositivi connessi
        for conn_handle in self._connections:
            self.ble.gatts_notify(conn_handle, self._data_handle, binary_data)

        # Visualizza i dati sul display OLED
        self.oled.fill(0)  # Pulisce lo schermo
        self.oled.text("Temp: {:.1f}C".format(temperature), 0, 0)
        self.oled.text("Press: {:.1f} hPa".format(pressure), 0, 16)
        self.oled.show()

        # Controlla se deve entrare in standby
        if temperature > 25:
            self._enter_standby()

    def _enter_standby(self):
        print("Entrando in standby...")
        for conn_handle in self._connections:
            self.ble.gatts_notify(conn_handle, self._data_handle, b"Entering standby")

        self.standby = True
        self.timer.deinit()  # Ferma la raccolta periodica di dati

        self.standby_timer.init(period=1000, mode=Timer.PERIODIC, callback=self._standby_mode)

    def _standby_mode(self, timer):
        self.led.value(not self.led.value())  # Lampeggia il LED
        self.oled.fill(0)
        self.oled.text("Standby", 0, 0)
        self.oled.show()
        self._check_exit_standby()

    def _check_exit_standby(self):
        temperature, _, _ = self.sensor.read_compensated_data()
        if temperature <= 25:
            print("Uscita dallo standby")
            for conn_handle in self._connections:
                self.ble.gatts_notify(conn_handle, self._data_handle, b"Exiting standby")

            self.standby = False
            self.led.value(0)
            self.standby_timer.deinit()
            self.timer.init(period=self.interval * 1000, mode=Timer.PERIODIC, callback=self._collect_and_send_data)

# Esegui il server BLE per sensori
ble_sensor = BLESensor(interval=5)

try:
    while True:
        pass
except KeyboardInterrupt:
    print("Interrotto manualmente")
