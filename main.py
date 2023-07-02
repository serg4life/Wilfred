from adafruit_esp32spi import adafruit_esp32spi_wifimanager
import time
from MQTT_Manager import MQTT_Manager
from robot import Robot
from myFunctions import esp
try:
    from mysecrets import secrets
except ImportError:
    print("WiFi secrets are kept in mysecrets.py, please add them there!")
    raise


#---------------------------MAIN----------------------------
print("Connecting to %s" % secrets["ssid"])
wifi = adafruit_esp32spi_wifimanager.ESPSPI_WiFiManager(esp, secrets)

# Verificar si la red está disponible
network_available = False
networks = esp.scan_networks()
for network in networks:
    ssid = network["ssid"].decode("utf-8")
    if ssid == secrets['ssid']:
            network_available = True
            break
    break

# Si la red esta disponible se conecta.
network_connection = False
if(network_available):
    wifi.connect()
    print("Connected to %s" % secrets['ssid'])
    network_connection = True
else:
    print("No connections available")

Wilfred = Robot()

# Si no esta conectado a Wifi no intenta conectarse al broker MQTT.
if(network_connection):
    mqtt_manager = MQTT_Manager(Wilfred)
    Wilfred.connect(mqtt_manager)
    
Wilfred.enable_motors()
Wilfred.calibrate()
if(Wilfred.mqtt_connection):
    while True:
        Wilfred.listen()
else:
    while True:
        Wilfred.led.value = True
        time.sleep(0.5)
        Wilfred.led.value = False
        time.sleep(0.5)
        Wilfred.cl_move_forward(50,5.5,0.1,0.01)
        time.sleep(5)
    
