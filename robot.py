import board
from digitalio import DigitalInOut, Direction
from pid_controller import PID_Controller
import time
import pwmio
from MQTT_Manager import MQTT_Manager
from myFunctions import MyIMU, RGB, esp, map_range

CLOCKWISE, COUNTER_CLOCKWISE, BRAKE, STANDBY = 0, 1, 2, 3
ROBOT_NAME = "Wilfred"
IMU_OFFSETS = ((3, -8, -33), (746, 218, -241), (0, -1, -1))
IMU_RADIUS = ((1000), (651))
MOTOR_FREQUENCY = 50000
MOTOR_DEATH_ZONE_L = 66  # Medido a 50KHz. Para mejor control una valor independiente para cada motor.
MOTOR_DEATH_ZONE_R = 66 # Parece que no es la misma para las dos direcciones.
MOTOR_DIFF = 0.05

def check_angle(angle):
    if angle > 180:
        new_angle = -(360-angle)
    else:
        new_angle = angle
    return new_angle

class Robot():
    def __init__(self, name=ROBOT_NAME):
        self.motor_L = Motor(board.A2, board.A3, MOTOR_DEATH_ZONE_L) # Obviamente para que funcionen bien los movimientos las entradas del motor tienen que estar bien definidas.
        self.motor_R = Motor(board.A1, board.A0, MOTOR_DEATH_ZONE_R)
        self.enable_signal = DigitalInOut(board.D2)     #Entrada de enable del driver de motores.
        self.enable_signal.direction = Direction.OUTPUT
        self.enable_signal.value = False
        self.name = name
        self.led = DigitalInOut(board.LED)
        self.led.direction=Direction.OUTPUT
        self.IMU = MyIMU()
        self.offsets = IMU_OFFSETS
        self.radius = IMU_RADIUS
        self.status_led = RGB(esp)
        self.status_led.color(255, 0, 0)
        self.mqtt_connection = False
        self.mqtt = None
        self.calibrated = False
        self.status = DigitalInOut(board.D3)    #Entrada de status del driver de motores.
        self.status.direction = Direction.INPUT

    # Las funciones relacionadas con MQTT estan pendientes para moverlas al MQTT_Manager
    def connect(self, mqtt_manager:MQTT_Manager):
        try:
            self.mqtt_connection = mqtt_manager.connect()
        except Exception:
            esp.reset()
            self.status_led.color(255, 255, 0)
            self.status_led.blink(6)
            self.status_led.color(255, 0, 0)
            print("----ERRROR CONECTANDO CON EL BROKER MQTT----",Exception)
            return False
        else:
            self.mqtt = mqtt_manager
            self.status_led.color(0, 255, 255)
            self.status_led.blink(6)
            self.status_led.color(255, 0, 0)
            self.mqtt.publish(data="{} succesfully connected :)\n".format(self.name))
            return True
        

    def listen(self):
        self.mqtt.mqtt_client.loop()

    def set_status_led(self, led:RGB):
        self.status_led = led
        return True
    
    def calibrate(self, reset_calibration=False):
        if(reset_calibration) or (self.offsets or self.radius) is None:
            offsets, radius = self.IMU.calibration_routine()
            self.offsets = offsets
            self.radius = radius
        else:
            self.set_calibration()

        self.status_led.color(0, 255, 0)
        self.calibrated = True
        return True

    def set_calibration(self):
        self.IMU.set_offsets(self.offsets[0], self.offsets[1], self.offsets[2])
        self.IMU.set_radius(self.radius[0], self.radius[1])

    def checkmotors(self):
        if self.motors_enabled:
            return True
        else:
            raise Exception("Enable motors before using them.")

    def standby_control(self, end_t=None):
        if end_t != None:
            t1 = t0 = time.monotonic()
            while (t1 - t0) < end_t:
                t1 = time.monotonic()
            self.motor_R.movement = STANDBY
            self.motor_L.movement = STANDBY
            return True
        else:
            return True

    def move_backward(self, power:float, end_t=None):
        self.checkmotors()
        self.motor_R.movement = COUNTER_CLOCKWISE
        self.motor_L.movement = CLOCKWISE
        self.motor_R.power = power
        self.motor_L.power = power
        self.standby_control(end_t)
        return True

    def move_forward(self, power:float, end_t=None):
        self.checkmotors()
        self.motor_R.movement = CLOCKWISE
        self.motor_L.movement = COUNTER_CLOCKWISE
        self.motor_R.power = power
        self.motor_L.power = power
        self.standby_control(end_t)
        return True

    def stabilize(self, Kp=0, Kd=0, Ki=0, a=0.9, ref_angle=0, end_t=20):
        data = ""
        ii = 0
        angle_measure = self.IMU.euler[2]
        controller = PID_Controller(Kp, Ki, Kd, ref_angle, angle_measure, a)
        t0 = t = time.monotonic()
        while(t0-t) < end_t:
            dt = t-t0
            pid = controller.update(angle_measure, dt)
            
            if pid >= 0:
                u = pid
                self.move_forward(u)
            elif pid < 0:
                u = abs(pid)
                self.move_backward(u)
                
            data += ":" + str(pid) + ":" + str(angle_measure) + "/"
            if ii % 50 == 0 and ii != 0:
                data = data[1:len(data)-2]  # Antes de enviar la informacion quitamos el primer : y el ultimo /
                self.mqtt.publish(data,"data")
                data = []   # Reseteamos el array.
            angle_measure = self.IMU.euler[2]
            t = time.monotonic()
        
        # Al acabar mandamos las muestras que faltaban.
        data = data[1:len(data)-2]
        self.mqtt.publish(data,"data")
        self.standby()
        return True

    def move_forward_PID(self, power:float, end_t=5):
        # Ha sido calibrado para funcionar bien al 50% de potencia al 100% se acaba desviabdo
        self.checkmotors()
        self.motor_R.movement = CLOCKWISE
        self.motor_L.movement = COUNTER_CLOCKWISE
        self.motor_R.power = power  #Aqui se puede añadir correcciones.
        self.motor_L.power = power
        modified_power_R = self.motor_R.power   #Esto es porque si el power es negativo dentro del setter se invierte.
        modified_power_L = self.motor_L.power   #Asi guardamos el valor inicial corregido
        Ki = 0.1   # Puestos a ojo, revisar.
        Kd = 0.0008
        Kp = 5
        pid_i = 0
        prev_error = 0
        t_start = t = time.monotonic()
        reference_angle = self.IMU.euler[0]
        while t - t_start < end_t:
            #----------------------PID-----------------------------
            current_angle = self.IMU.euler[0]
            prev_time = t
            t = time.monotonic()
            elapsed_time = prev_time - t

            error = current_angle - reference_angle
            error = check_angle(error)
            pid_p = Kp*error

            """ if abs(error) < 3:  #Solo para ajustar angulos pequeños
                pid_i = pid_i + Ki*error """
            if pid_i < 20:
                pid_i = pid_i + Ki*error

            pid_d = Kd*(error - prev_error)/elapsed_time
            prev_error = error
            pid = pid_p + pid_d + pid_i
            if pid < -abs(power):
                pid = -abs(power)
            elif pid > abs(power):
                pid = abs(power)                             
            self.motor_R.power = modified_power_R + pid
            self.motor_L.power = modified_power_L - pid
            if self.mqtt_connection:
                self.mqtt.publish(
                    data="{}\n{}\n{}\n{}\n".format(
                    pid, reference_angle, current_angle, error), mqtt_topic="data"
                    )
        self.motor_R.movement = STANDBY
        self.motor_L.movement = STANDBY 

    def brake(self, power:float):   # No se si funciona o hace algo.
        self.checkmotors()
        self.motor_R.movement = self.motor_L.movement = BRAKE
        self.motor_R.power = power
        self.motor_L.power = power

    def standby(self):
        self.checkmotors()
        self.motor_R.power = 0
        self.motor_L.power = 0
        self.motor_R.movement = self.motor_L.movement = STANDBY
    
    def enable_motors(self):
        self.enable_signal.value = True

    def disable_motors(self):
        self.enable_signal.value = False

    def clockwise_rotate(self, power, end_t=None):
        self.checkmotors()
        self.motor_R.movement = COUNTER_CLOCKWISE
        self.motor_L.movement = COUNTER_CLOCKWISE
        self.motor_R.power = power
        self.motor_L.power = power
        self.standby_control(end_t)
        return True

    def counter_clockwise_rotate(self, power, end_t=None):
        self.checkmotors()
        self.motor_R.movement = CLOCKWISE
        self.motor_L.movement = CLOCKWISE
        self.motor_R.power = power
        self.motor_L.power = power   # Es el motor mas lento.
        self.standby_control(end_t)
        return True

    @property
    def motors_enabled(self):
        return self.enable_signal.value

class Motor():
    def __init__(self, pinA:int, pinB:int, death_zone=0):
        self._A = pinA
        self._B = pinB
        self._power = 0
        self._normalized_power = 0
        self._movement = STANDBY
        self.death_zone = death_zone
        self.pwm_signal_A = pwmio.PWMOut(self._A, frequency=MOTOR_FREQUENCY, duty_cycle=0, variable_frequency=False)
        self.pwm_signal_B = pwmio.PWMOut(self._B, frequency=MOTOR_FREQUENCY, duty_cycle=0, variable_frequency=False)

    def calibrate(self, value):
        return map_range(value, 0, 100, self.death_zone, 100)
    
    def change_rotation(self):
        """NO SE USA."""
        rotation = self.movement
        if rotation == CLOCKWISE:
            self.movement = COUNTER_CLOCKWISE
            return True
        elif rotation == COUNTER_CLOCKWISE:
            self.movement = CLOCKWISE
            return True
        else:
            return True

    @property
    def normalized_power(self):
        aux = round(((self.power / 100) * 65535)) # valores entre 0 y 65535
        """ if aux > 65535:
            aux = 65535
        elif aux < 0:
            aux = 0 """
        self._normalized_power = aux
        return self._normalized_power # Must be an integer

    @property
    def power(self):
        return self._power

    @property
    def movement(self):
        return self._movement
    
    @movement.setter
    def movement(self, value):
        self._movement = value
        if self.movement == STANDBY:
            self.pwm_signal_A.duty_cycle = self.pwm_signal_B.duty_cycle = 0
        return True

    @power.setter
    def power(self, value):
        #Habria que calibrar los motores ya que el 0% no equivale al 0 sino que por debajo de cierto valor ya se parann.
        #raise Exception("La potencia del motor debe estar entre el 0 % y el 100 %")
        """ if value == 0:
            self._power = 0
        else:
            if value < 0:
                value = -value
                self.change_rotation()
            if value > 100:
                value  = 100
            self._power = self.calibrate(value) """
            
        if value > 100:
            value = 100
        self._power = self.calibrate(value)
        if self.movement == BRAKE:
            self.pwm_signal_A.duty_cycle = self.pwm_signal_B.duty_cycle = self.normalized_power #Frena con la potencia que se le asigne.
        elif self.movement == CLOCKWISE:
            self.pwm_signal_A.duty_cycle = self.normalized_power
            self.pwm_signal_B.duty_cycle = 0
        elif self.movement == COUNTER_CLOCKWISE:
            self.pwm_signal_A.duty_cycle = 0
            self.pwm_signal_B.duty_cycle = self.normalized_power
        elif self.movement == STANDBY:
            self.pwm_signal_A.duty_cycle = self.pwm_signal_B.duty_cycle = 0
        else:
            print("{} no es un movimiento/estado de motor valido.".format(self.movement))
            return False
        return True
