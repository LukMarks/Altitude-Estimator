# Auhor: Lucas Marques Moreno
# Desriptiton: Kalman Filter

from flight_simulator_interface import FlightSimulator
from model import FlightModel
from scipy.integrate import odeint
import numpy as np


class KalmanFilter:
    
    def __init__(self, flight_model: FlightModel, simalution: FlightModel) -> None:

        self.model = flight_model
        self.simulation = simalution
    
    def set_filter_gain(self, gain) -> None:
        self.K = gain

    def set_initial_state(self, state):
        self.initial_state = state

    def altitude_a_priore(self) -> np.array:
        solved = odeint(self.model._differential_model, self.initial_state, self.ode_time)
        u = solved[:, 0]
        w = solved[:, 1]
        q = solved[:, 2]
        h = solved[:, 3]
        
        return np.array([u, w, q, h])
    
    def set_ode_config(self, iterations: float, time_step: float) -> None:
        self.ode_time = np.linspace(0,time_step,iterations)

    def altitude_a_posteriore(self) -> None:
        state = self.altitude_a_priore() 
        x_hat = state + self._calculate_filter_gain()*(zk - state * state)
    
    def _calculate_filter_gain(self) -> float:
        K = P_prediction*state_transpose/(state*P_prediction*state_transpose + R)
        return K
    
        

    # @property
    # def altitude(self) -> float:
    #     return self.altitude