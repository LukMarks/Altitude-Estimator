# Auhor: Lucas Marques Moreno
# Desriptiton: Aircfrat flight model


import numpy as np
from scipy.integrate import odeint

class FlightModel:

    def __init__(self, aircraft: dict, environment: dict) -> None:
        self.aircraft = aircraft
        self.environment = environment
        self.r = 0
    def altitude_model(self) -> float:
        pass

    def _differential_model(self, state) -> np.array(float):
        """
            Differentical equations for flight dynamics
            with the following state vector
            [u, w, q, phi, theta, h]
        """

        u_dot = state[2]*state[1] - self.g*np.sin(state[4]) + self.pd(state[0], state[1])*self.S*self.cx/self.m

        w_dot = state[2]*state[0] + self.g*np.cos(state[4])*np.sin(state[3]) + self.pd(state[0], state[1])*self.S*self.cz/self.m

        q_dot = (M + self.Izx*self.r**2)/self.Iy

        phi_dot = state[2]*np.sin(state[3])*np.tan(state[4])+ self.r*np.sin(state[3])*np.tan(state[4])

        theta_dot = state[2]*np.cos(state[3]) - self.r*np.sin(state[3])

        h_dot = state[0]*np.sin(state[4]) - state[1]*np.cos(state[3])*np.sin(state[4])

        return u_dot, w_dot, q_dot, phi_dot, theta_dot, h_dot

    
    def _pd(self, u, w):
        """
        Dynamic pressure
        """
        return 0.5 * np.sqrt(u**2 + w**2) * self.p

    def set_aerodynamics_coefficient(self):
        
        self.Ct = 0
        self.cx = self.Cl*np.sin(self.alpha) + self.Ct
        self.cz = -self.Cl*np.cos(self.alpha)

    def set_aircfrat_coefficient(self):
        self.S = self.aircraft.get("wing area", 0)
        self.c = self.aircraft.get("chord", 0)
        self.m = self.aircraft.get("mass", 0)
        self.Cl = self.aircraft.get("CL", 0)
        self.Ix = self.aircraft.get("Inertia", [[0]])[0][0]
        self.Iy = self.aircraft.get("Inertia", [[0]])[1][1]
        self.Izx = self.aircraft.get("Inertia", [[0]])[2][0]
    
    def set_enviroment_properties(self):
        self.p = self.environment.get("air desnity", 1.225)
        self.g = self.environment.get("gravity", 9.81)