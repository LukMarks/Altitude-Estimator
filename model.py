# Auhor: Lucas Marques Moreno
# Desriptiton: Aircfrat flight model


import numpy as np
from scipy.integrate import odeint

class FlightModel:

    def __init__(self, aircraft: dict, environment: dict) -> None:
        self.aircraft = aircraft
        self.environment = environment

    def altitude_model(self) -> float:
        pass

    def _differential_model(self, state, t) -> np.array(float):
        """
            Differentical equations for flight dynamics
            with the following state vector
            [u, w, q, h]
        """

        V = np.sqrt(state[1]**2 + state[0]**2)

        pd = 0.5 * V**2 * self.p

        M = 0.5 * self.p * self.S * self.c * self.Cm * V**2

        u_dot = state[2]*state[1] - self.g*np.sin(state[2]) + pd*self.S*self.cx/self.m

        w_dot = state[2]*state[0] + pd*self.S*self.cz/self.m

        q_dot = M /self.Iy

        h_dot = state[0]*np.sin(state[2]) - state[1]*np.sin(state[2])

        return u_dot, w_dot, q_dot, h_dot

    
    def _pd(self, u, w):
        """
        Dynamic pressure
        """
        return 0.5 * np.sqrt(u**2 + w**2) * self.p

    def _momentum(self, u, w):
        v = np.sqrt(u**2 + w**2)
        return 0.5 * self.p * self.S * self.c * self.Cm * v**2


    def set_aerodynamics_coefficient(self):
        
        self.Ct = 0
        self.cx = self.Cl*np.sin(self.alpha) - self.Cd*np.cos(self.alpha) + self.Ct
        self.cz = -self.Cl*np.cos(self.alpha) - self.Cd*np.sin(self.alpha)

    def set_aircfrat_coefficient(self):
        self.S = self.aircraft.get("wing area", 0)
        self.c = self.aircraft.get("chord", 0)
        self.m = self.aircraft.get("mass", 0)
        self.Cl = self.aircraft.get("CL", 0)
        self.Cd = 0.001625*self.Cl**3 + 0.30061*self.Cl**2 + 0.007446*self.Cl #self.aircraft.get("CD", 0)
        self.Cm = self.aircraft.get("CM", 0)
        self.alpha = self.aircraft.get("AoA", 0)
        self.Ix = self.aircraft.get("Inertia", [[0]])[0][0]
        self.Iy = self.aircraft.get("Inertia", [[0]])[1][1]
        self.Izx = self.aircraft.get("Inertia", [[0]])[2][0]
    
    def set_enviroment_properties(self):
        self.p = self.environment.get("air desnity", 1.225)
        self.g = self.environment.get("gravity", 9.81)