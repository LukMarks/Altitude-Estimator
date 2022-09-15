# Auhor: Lucas Marques Moreno
# Desriptiton: Xplane 11 Data Handler

import pandas as pd
import numpy as np


class FlightSimulator:
    def __init__(self, flight_data: str) -> None:
        self.data = pd.read_csv(flight_data)
        self.state = []

    @property
    def simulation_time(self):
        return self.data["_totl_time"] 
   
    @property
    def abs_speed(self):
        return self.data["_Vind__mph"] * 1.6/3.6

    @property
    def x_speed(self):
        return self.data["___vY__m/s"] + np.random.normal(0, (0.08*9.81)**2,len(self.data["_totl_time"]))
    
    @property
    def z_speed(self):
        return -1*self.data["___vZ__m/s"] + np.random.normal(0,(0.08*9.81)**2,len(self.data["_totl_time"]))

    @property
    def pitch_angle(self):
        return (self.data["pitch__deg"] + np.random.normal(0,np.sqrt(0.02),len(self.data["_totl_time"])))*np.pi/180
    
    @property
    def altitude(self):
        
        #return self.data["____Z____m"] + np.random.normal(0,10,len(self.data["_totl_time"]))
        return -1*self.data["__alt__ind"]*0.3048 + np.random.normal(0,np.sqrt(1),len(self.data["_totl_time"]))
       

    def format_state(self):
        for row in range(0, len(self.data["_totl_time"])):
            current_state = np.array([self.x_speed[row], self.z_speed[row], self.pitch_angle[row], self.altitude[row]])
            self.state.append(current_state)
 
    def _mph_to_kmh(self, mph_speed):
        return mph_speed*1.6