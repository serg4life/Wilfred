"""Modulo que contiene la clase `PID_Controller` para crear objetos PID.

* Author(s): Sergio Fernández
"""

import math as mt

class PID_Controller():
    """Clase que crea un objeto `PID_Controller`"""
    def __init__(self, Kp, Ki, Kd, ref, start_measure, a=0.9,  sat=15, Kr=None, b=1, c=1):
        self.b = b   #Parametro de ponderacion
        self.c = c   #Parametro de ponderacion

        self.filtro_d = PID_Filter(50, 0.001)

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.a = a

        if Kr is None:
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

        self.sat = sat
    
    def update(self, measure, dt):
        error = self.ref - measure
        error_pondered = self.b * self.ref - measure
        self.integral += error * dt
        #self.derivative = (error - self.prev_error) / dt
        self.derivative_pondered = (1-self.a)*(self.c * (self.ref - self.a * self.ref_prev) - (measure - self.a * self.measure_prev)) / dt

        # Anti-Windup
        if self.u_prev > self.sat:
            i_control = self.Ki * self.integral + self.Kr * (self.sat - self.u_prev)  #Si el sistema esta saturado añade un termino negativo.
        else:
            i_control = self.Ki * self.integral

        # Calcular la señal de control
        #u = self.Kp*error_pondered + i_control + self.filtro_d.get_filtered_value(self.Kd * self.derivative_pondered)   
        u = self.Kp * self.filtro_d.get_filtered_value(self.derivative_pondered)
        # Para no dar mas tension de la que los motores aceptan
        if u > self.sat:
            u = self.sat
        elif u < -self.sat:
            u = -abs(self.sat)

        # Actualizar los valores previos
        self.error_prev = error
        self.measure_prev = measure
        self.ref_prev = self.ref
        self.u_prev = u

        return u

class PID_Filter():
    def __init__(self, fc=500, T=0.001):
        self.fc = fc
        self.T = T
        self.vo_prev = 0
        self.vi_prev = 0

    def get_filtered_value(self, data):
        """Filtro paso bajo digital"""
        vo = self.vo_prev*mt.exp(-self.T*2*mt.pi*self.fc) + self.vi_prev*(1 - mt.exp(-self.T*2*mt.pi*self.fc))
        self.vo_prev = vo
        self.vi_prev = data
        return vo
