if __name__ == "__main__":
    from robot import Robot

COMANDS = ("LED","*IDN","*RST","*TST","*OPC","*OPC?","*WAI","*CLS","*ESE","*ESE?","*ESR?","*SRE","*SRE?","*STB?")

class Comands():
    def __init__(self, command):
        self.command = command

class Command_Handler():
    def __init__(self, robot:Robot):
        self.robot = robot

    def not_valid(self, command):
        self.robot.mqtt.publish("{} no es un comando valido".format(command), self.robot.name)

    def handler(self, command:str):
        if len(command) == 0:
            self.not_valid(command)
            return False
        if command[0] == '*':
            command = command[1:]
            if command == "IDN":
                self.robot.mqtt.publish("I'm {}".format(self.robot.name), self.robot.name)
            elif command == "RST":
                self.robot.reset()
        else:
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
                print(c)
                if len(c) < 2:
                    self.robot.mqtt.publish("Faltan argumentos", self.robot.name)
                    return False
                c = c[1:]   #lista de argumentos
                c = [float(i) for i in c]
                self.robot.move_forward_PID(*c)
            elif c[0] == "STOP":
                self.robot.standby()
            else:
                self.not_valid(command)
                return False



