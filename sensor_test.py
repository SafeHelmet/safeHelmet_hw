from machine import SoftI2C, Pin
import time
from bh1750 import BH1750
from MPU6050 import MPU6050
import dht

# Configurazione I2C
i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=400000)

mpu = MPU6050(i2c)

light_sensor = BH1750(i2c)


# Sensore DHT11
dht_sensor = dht.DHT11(Pin(18))

print("Avvio lettura sensori...")

while True:
    try:
        # Lettura dal BH1750 (luminosità in lux)
        lux = light_sensor.luminance()

        # Lettura dal MPU6050 (giroscopio: X, Y, Z)
        gyro = mpu.read_angle()

        gyro_x, gyro_y, gyro_z = gyro['x'], gyro['y'], gyro['z']

        # Lettura dal DHT11 (temperatura e umidità)
        dht_sensor.measure()
        temp = dht_sensor.temperature()
        humidity = dht_sensor.humidity()

        # Stampa i dati alla seriale
        print(f"Luminosità (BH1750): {lux:.2f} lux")
        print(f"Giroscopio (MPU6050): X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f}")
        print(f"Temperatura (DHT11): {temp}°C")
        print(f"Umidità (DHT11): {humidity}%")
        print("-" * 40)

    except Exception as e:
        print(f"Errore nella lettura: {e}")

    time.sleep(1)  # Attendere 1 secondo prima di ripetere
