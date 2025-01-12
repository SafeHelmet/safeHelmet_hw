from machine import Pin, I2C
import time


class BH1750:
    PWR_DOWN = 0x00
    PWR_ON = 0x01
    RESET = 0x07

    # Modalità di lettura
    CONT_HIRES_1 = 0x10  # Alta risoluzione, 1lx
    CONT_HIRES_2 = 0x11  # Alta risoluzione, 0.5lx
    CONT_LORES = 0x13  # Bassa risoluzione, 4lx

    def __init__(self, i2c, address=0x23):
        self.i2c = i2c
        self.address = address
        self.on()

    def on(self):
        """Accende il sensore."""
        self.i2c.writeto(self.address, bytearray([self.PWR_ON]))

    def off(self):
        """Spegne il sensore."""
        self.i2c.writeto(self.address, bytearray([self.PWR_DOWN]))

    def luminance(self, mode=CONT_HIRES_1):
        """Legge la luminosità in lux."""
        self.i2c.writeto(self.address, bytearray([mode]))
        time.sleep(0.18)  # Attendi il tempo richiesto per la misurazione
        data = self.i2c.readfrom(self.address, 2)
        return (data[0] << 8 | data[1]) / 1.2


# Configura I2C
i2c = I2C(1, scl=Pin(22), sda=Pin(21))  # Modifica scl/sda se necessario

# Inizializza il sensore
sensor = BH1750(i2c)

print("Inizializzazione BH1750 completata. Misurazioni in corso...")

while True:
    try:
        # Legge la luminosità
        lux = sensor.luminance()
        print("Luminosità: {:.2f} lux".format(lux))
    except OSError as e:
        print("Errore nella lettura del sensore:", e)

    # Attendi un secondo prima della prossima lettura
    time.sleep(1)
