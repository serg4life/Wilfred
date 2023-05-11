from adafruit_minimqtt.adafruit_minimqtt import MQTT, set_socket
import adafruit_esp32spi.adafruit_esp32spi_socket as socket
from command_handler import Command_Handler
from myFunctions import esp
try:
    from mysecrets import secrets
except ImportError:
    print("WiFi secrets are kept in mysecrets.py, please add them there!")
    raise

class MQTT_Manager():
    def __init__(self, robot):
        set_socket(socket, esp)
        mqtt_client = MQTT(
            broker=secrets['broker'],
            port=secrets['port'],
            username=secrets['username'],
            password=secrets['userpass'],
        )
        self.mqtt_client = mqtt_client
        self.robot = robot

    def connect(self):
        self.configure_client()
        try:
            self.mqtt_client.connect()
        except Exception:
            raise Exception("----ERROR CONECTANDO CON EL BROKER MQTT----",Exception)
        else:
            self.subscribe()
            return True

    def subscribe(self, mqtt_topic="commands"):
        self.mqtt_client.subscribe(mqtt_topic)
        return True

    def publish(self, data="", mqtt_topic="Wilfred"):
        self.mqtt_client.publish(mqtt_topic, data, qos=0)
        return True

    # --------Estas son las que se enlazan con las acciones del cliente MQTT---------
    def connected(mqtt_client, userdata, flags, rc):
        # This function will be called when the mqtt_client is connected
        # successfully to the broker.
        print("Connected to MQTT Broker!")
        print("Flags: {0}\n RC: {1}".format(flags, rc))

    def disconnected(mqtt_client, userdata, rc):
        # This method is called when the mqtt_client disconnects
        # from the broker.
        print("Disconnected from MQTT Broker!")

    def subscribed(mqtt_client, userdata, topic, granted_qos):
        # This method is called when the mqtt_client subscribes to a new feed.
        print("Subscribed to {0} with QOS level {1}".format(topic, granted_qos))

    def unsubscribed(mqtt_client, userdata, topic, pid):
        # This method is called when the mqtt_client unsubscribes from a feed.
        print("Unsubscribed from {0} with PID {1}".format(topic, pid))

    def published(mqtt_client, userdata, topic, pid):
        # This method is called when the mqtt_client publishes data to a feed.
        print("Published to {0} with PID {1}".format(topic, pid))

    def message(self, client, topic, message):
        # Method called when a client's subscribed feed has a new value.
        print("{0}: {1}".format(topic, message))
        Handler = Command_Handler(self.robot)
        Handler.handler(message)


    def configure_client(self):
        # Connect callback handlers to mqtt_client
        #self.mqtt_client.on_connect = MQTT_Manager.connected
        #self.mqtt_client.on_disconnect = MQTT_Manager.disconnected
        #self.mqtt_client.on_subscribe = MQTT_Manager.subscribed
        #self.mqtt_client.on_unsubscribe = MQTT_Manager.unsubscribed
        #self.mqtt_client.on_publish = MQTT_Manager.published
        self.mqtt_client.on_message = self.message