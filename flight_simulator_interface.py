# Auhor: Lucas Marques Moreno
# Desriptiton: Flight Gear interface

import pandas as pd
import numpy as np


class FlightSimulator:
    def __init__(self, flight_data: str) -> None:
        self.data = pd.read_csv(flight_data)

    @property
    def simulation_time(self):
        return self.data["_totl_time"]
    
    @property
    def abs_speed(self):
        return self.data["_Vind__mph"] * 1.6/3.6

    @property
    def x_speed(self):
        return self.data["___vX__m/s"]
    
    @property
    def z_speed(self):
        return self.data["___vZ__m/s"]

    @property
    def pitch_angle(self):
        return self.data["pitch__deg"]
    
    @property
    def altitude(self):
        return self.data["__alt__ind"]
    
    def _mph_to_kmh(self, mph_speed):
        return mph_speed*1.6