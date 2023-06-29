"""Modulo que contiene la clase `PID_Controller` para crear objetos PID.

* Author(s): Sergio Fernández
"""

import math as mt

class PID_Controller():
    """Clase que crea un objeto `PID_Controller`"""
    def __init__(self, Kp, Ki, Kd, ref, start_measure, a=0.9,  sat=100, Kr=None, b=1, c=1, filter=True):
        self.b = b   #Parametro de ponderacion
        self.c = c   #Parametro de ponderacion
        self.filter = filter
        self.filtro_d = PID_Filter(5, 0.005)

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.a = a
        Kr = 0.01
        if Kr is None:
            #Como no estamos usando Kd, siempre es 0 asique le daremos un valor  a ojo.add()
            self.Kr = mt.sqrt(self.Ki * self.Kd)    #Tipicamente
        else:
            #Ganancia Anti-Windup, Kd > Kr > Ki
            self.Kr = Kr

        self.ref = ref
        self.integral = 0
        self.derivative = 0
        self.error_prev = 0
        self.ref_prev = 0
        self.measure_prev = start_measure
        self.u_prev = 0
        self.es = 0
        self.sat = sat
    
    def update(self, measure, dt):
        """Actualiza los valores de un objeto PID_Controller y devuelve la salida."""
        error = self.ref - measure
        error_pondered = self.b * self.ref - measure
        #self.derivative = (error - self.prev_error) / dt
        self.derivative_pondered = (1-self.a)*(self.c * (self.ref - self.a * self.ref_prev) - (measure - self.a * self.measure_prev)) / dt

        # Anti-Windup
        if self.u_prev > self.sat:
            self.es = self.sat - self.u_prev
            self.integral += (self.Ki * error + self.Kr * self.es) * dt
            i_control = self.integral   #Si el sistema esta saturado añade un termino negativo.
        else:
            self.integral += error * dt
            i_control = self.Ki * self.integral

        # Calcular la señal de control
        #u = self.Kp*error_pondered + i_control + self.filtro_d.get_value(self.Kd * self.derivative_pondered)
        if self.filter:
            self.derivative_pondered = self.filtro_d.get_value(self.derivative_pondered)
        u = self.Kp * self.derivative_pondered + i_control
        
        # Para no dar mas tension de la que los motores aceptan
        if u > self.sat:
            u = self.sat
        elif u < -self.sat:
            u = -self.sat

        # Actualizar los valores previos
        self.error_prev = error
        self.measure_prev = measure
        self.ref_prev = self.ref
        self.u_prev = u

        return u
    
class PID_Filter():
    def __init__(self, fc=500, T=0.001):
        self.vo_prev = 0
        self.vi_prev = 0
        self.ao = mt.exp(-T*2*mt.pi*fc)
        self.ai = mt.exp(-T*2*mt.pi*fc)

    def get_value(self, data):
        """Filtro paso bajo digital"""
        vo = self.vo_prev*self.ao + self.vi_prev*(1 - self.ai)
        self.vo_prev = vo
        self.vi_prev = data
        return vo
