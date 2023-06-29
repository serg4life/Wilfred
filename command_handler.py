"""Modulo que se encarga de recivir los comandos via MQTT y gestionar las acciones asociadas a cada comando.add()

Contiene la clase `Command_Handler`.

* Author(s): Sergio Fernández
"""

if __name__ == "__main__":
    from robot import Robot

COMANDS = ("LED","*IDN","CALIB","MOVE","STB","CLM")

class Command_Handler():
    """Clase que crea una instancia `Command_Handler`, hace de etapa intermedia entre el MQTT_Manager y el Robot."""
    def __init__(self, robot:Robot):
        self.robot = robot

    def not_valid(self, command):
        self.robot.mqtt.publish("{} no es un comando valido".format(command), self.robot.name)

    def handler(self, command:str):
        if command.split(':')[0] not in COMANDS:
            self.not_valid(command)
            return False
        if command[0] == '*':
            command = command[1:]
            if command == "IDN":
                self.robot.mqtt.publish("I'm {}".format(self.robot.name), self.robot.name)
            elif command == "RST":
                self.robot.reset()
        else:
            # Dividimos el str en parametros.
            c = command.split(':')
            if c[-1] == "":
                c.pop(-1)
            if c[0] == "LED":
                if len(c) > 1:
                    c = c[1:]
                else:
                    return False
                if c[0] == "ON":
                    self.robot.led.value = True
                elif c[0] == "OFF":
                    self.robot.led.value = False
                else:
                    self.not_valid(command)
            elif c[0] == "MOVE":

                if len(c) < 2:
                    self.robot.mqtt.publish("Faltan argumentos", self.robot.name)
                    return False
                c = c[1:]   #lista de argumentos
                c = [float(i) for i in c]
                self.robot.move_forward_PID(*c)
            elif c[0] == "STOP":
                self.robot.standby()
            elif c[0] == "CALIB":
                self.robot.calibrate(True)
            elif c[0] == "STB":
                args = c[1:]
                if len(c) == 2:
                    self.robot.stabilize(Kp=float(args[0]))
                elif len(c) == 3:
                    self.robot.stabilize(Kp=float(args[0]),a=float(args[1]))
                elif len(c) == 4:
                    self.robot.stabilize(Kp=float(args[0]),a=float(args[1]),Ki=float(args[2]))
                else:
                    self.not_valid(command)
                    return False
            elif c[0] == "CLM":
                args = c[1:]
                if len(c) == 2:
                    self.robot.cl_move_forward(power=float(args[0]))
                elif len(c) == 3:
                    self.robot.cl_move_forward(power=float(args[0]),Kp=float(args[1]))
                elif len(c) == 4:
                    self.robot.cl_move_forward(power=float(args[0]),Kp=float(args[1]),a=float(args[2]))
                elif len(c) == 5:
                    self.robot.cl_move_forward(power=float(args[0]),Kp=float(args[1]),a=float(args[2]),Ki=float(args[3]))
                else:
                    self.not_valid(command)
                    return False
            else:
                self.not_valid(command)
                return False



