import board
import busio
import time
import adafruit_bno055
from digitalio import DigitalInOut
from adafruit_esp32spi import adafruit_esp32spi
from adafruit_esp32spi.adafruit_esp32spi import ESP_SPIcontrol

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#-----------------------------------------ESP---------------------------------------------------------
#  ESP32 pins
esp32_cs = DigitalInOut(board.CS1)
esp32_ready = DigitalInOut(board.ESP_BUSY)
esp32_reset = DigitalInOut(board.ESP_RESET)
#  uses the secondary SPI connected through the ESP32
spi = busio.SPI(board.SCK1, board.MOSI1, board.MISO1)
esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)

#------------------------------BNO055------------------------------------------
class MyIMU():
    def __init__(self):
        self.i2c = board.I2C()
        self.bno = adafruit_bno055.BNO055_I2C(self.i2c)
        self.bno.mode = adafruit_bno055.CONFIG_MODE
        self.bno.accel_mode = adafruit_bno055.ACCEL_2G #USANDO EL MODO 2G no funciona el codigo de DataProcessing
        self.bno.mode = adafruit_bno055.NDOF_MODE

    def set_offsets(self, offsets_acc:tuple, offsets_mag:tuple, offsets_gyro:tuple):
        #bno.mode = adafruit_bno055.CONFIG_MODE No es necesario porque lo hace solo.
        #acc
        self.bno.offsets_accelerometer = offsets_acc
        #mag
        self.bno.offsets_magnetometer = offsets_mag
        #gyro
        self.bno.offsets_gyroscope = offsets_gyro
        return True

    def set_radius(self, acc_radius:int, mag_radius:int):
        self.bno.radius_accelerometer = acc_radius
        self.bno.radius_magnetometer = mag_radius
        return True

    def calibration_routine(self):
        while True:
            status = self.bno.calibration_status
            print(status)
            if (status[0] == 3 and status[1] == 3 and status[2] == 3 and status[3] == 3): #System is calibrated when AC, MAG and GYRO ara calibrated.
                print("\n------------------------------------------------------------------------------------")
                print("BNO055 CALIBRATED")
                print("------------------------------------------------------------------------------------")
                self.print_offsets()
                self.print_radius()
                break
        return ((self.bno.offsets_accelerometer, self.bno.offsets_magnetometer, self.bno.offsets_gyroscope),
                (self.bno.radius_accelerometer, self.bno.radius_magnetometer))

    def print_offsets(self):
        print("\naccel offsets: {}".format(self.bno.offsets_accelerometer))    #(5, 8, -26)
        print("mag offsets: {}".format(self.bno.offsets_magnetometer))   #(41, -82, -308)
        print("gyro offsets: {}".format(self.bno.offsets_gyroscope))     #(-1, -2, 0)

    def print_radius(self):
        print("\naccel radius: {}".format(self.bno.radius_accelerometer))
        print("mag radius: {}\n".format(self.bno.radius_magnetometer))
    
    @property
    def JSON(self):
        data = {
            'Acceleration': self.bno.acceleration,
            'Magnetic_Field' : self.bno.magnetic,
            'Euler_angles' : self.bno.euler,
            'Gyro_values' : self.bno.gyro 
            }
        return data

    @property
    def formated_acceleration(self):
        return "acceleration(BNO055) {} {} {}".format(self.bno.acceleration[0], self.bno.acceleration[1], self.bno.acceleration[2])
    
    @property
    def formated_magnetometer(self):
        return "magnetometer(BNO055) {} {} {}".format(self.bno.magnetic[0], self.bno.magnetic[1], self.bno.magnetic[2])
    
    @property
    def formated_gyro(self):
        return "gyro(BNO055) {} {} {}".format(self.bno.gyro[0], self.bno.gyro[1], self.bno.gyro[2])
    
    @property
    def formated_euler(self):
        return "Euler angles(BNO055) {} {} {}".format(self.bno.euler[0], self.bno.euler[1], self.bno.euler[2])

    @property
    def acceleration(self):
        return self.bno.acceleration
    
    @property
    def magnetometer(self):
        return self.bno.magnetic
    
    @property
    def gyro(self):
        return self.bno.gyro
    
    @property
    def euler(self):
        return self.bno.euler

class RGB():
    def __init__(self, esp:ESP_SPIcontrol):
        self.esp = esp
        #Pines del RGB(27, 25, 26), se encienden en LOW.
        self.esp.set_pin_mode(25, 1)
        self.esp.set_pin_mode(26, 1)
        self.esp.set_pin_mode(27, 1)
        self.esp.set_analog_write(27, 1)
        self.esp.set_analog_write(25, 1)
        self.esp.set_analog_write(26, 1)
        self.current_color = (1, 1, 1)
        self._current_state = False
    
    def color(self, R:int, G:int ,B:int):
        maped_r = map_range(R, 0, 255, 1, 0)
        maped_g = map_range(G, 0, 255, 1, 0)
        maped_b = map_range(B, 0, 255, 1, 0)
        self.current_color = (maped_r, maped_g, maped_b)
        self.state = True

    def apagar(self):
        self.esp.set_analog_write(27, 1)
        self.esp.set_analog_write(25, 1)
        self.esp.set_analog_write(26, 1)

    def encender(self):
        self.esp.set_analog_write(27, self.current_color[0])
        self.esp.set_analog_write(25, self.current_color[1])
        self.esp.set_analog_write(26, self.current_color[2])

    @property
    def state(self):
        return self._current_state
    
    @state.setter
    def state(self, state):
        if state == True:
            self.encender()
        elif state == False:
            self.apagar()
        self._current_state = state

    def blink(self, k=4, speed=0.2):
        initial_state = self._current_state
        n = 0
        while n < k:
            self.state = (not self.state)
            t = t0 = time.monotonic()
            while(t-t0 < speed):
                t = time.monotonic()
            n += 1
        self.state = initial_state
        