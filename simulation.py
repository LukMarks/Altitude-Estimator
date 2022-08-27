# Auhor: Lucas Marques Moreno
# Desriptiton: Filter Application

from model import FlightModel
from filter import KalmanFilter
from flight_simulator_interface import FlightSimulator

#inertia
inertia = [
          [1, 0, 0], #Ixx, Ixy, Ixz 
          [0, 1, 0], #Iyx, Iyy, Iyz
          [0, 0 ,0]  #Izx, Izy, Izz

]

allegro = {
            "wing area": 1, #[m]
            "chord": 0.5, #[m]
            "mass": 0.1, #[kg]
            "CL": 1.1, #[]
            "Inertia":inertia, #[kgm²],
            "AoA": 0 #[°]
}

envrioment = {
            "air density": 1.225,
            "gravity": 9.81
}

flight = FlightModel(aircraft=allegro, environment=envrioment)

flight.set_enviroment_properties()
flight.set_aircfrat_coefficient()
flight.set_aerodynamics_coefficient()
