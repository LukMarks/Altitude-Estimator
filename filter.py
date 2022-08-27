# Auhor: Lucas Marques Moreno
# Desriptiton: Kalman Filter

from flight_simulator_interface import FlightSimulator
from model import FlightModel

class KalmanFilter:
    
    def __init__(self, flight_model: FlightModel, simalution: FlightModel) -> None:
        self.model = flight_model
        self.simulation = simalution
        self.altitude = 0
    
    def altitude_estimation(self) -> None:
        pass
    
    @property
    def altitude(self) -> float:
        return self.altitude