# Auhor: Lucas Marques Moreno
# Desriptiton: Filter Application

from cProfile import label
from model import FlightModel
from filter import KalmanFilter
from flight_simulator_interface import FlightSimulator
import numpy as np
import matplotlib.pyplot as plt


inertia = [
          [1285.32, 0, 0], #Ixx, Ixy, Ixz 
          [0, 1824.93, 0], #Iyx, Iyy, Iyz
          [0, 0, 2666.89]  #Izx, Izy, Izz

]
cesna_172 = {
            "wing area": 16.17, #[m²]
            "chord": 1.494, #[m]
            "mass": 771.1, #[kg]
            "CL": 1.6, #[]
            "CM": -0.015, #[]
            "Inertia":inertia, #[kgm²],
            "AoA": 2 #[°]
}

envrioment = {
            "air density": 1.225,
            "gravity": 9.81
}

flight = FlightModel(aircraft=cesna_172, environment=envrioment)

flight.set_enviroment_properties()
flight.set_aircfrat_coefficient()
flight.set_aerodynamics_coefficient()

flight_data = FlightSimulator(flight_data="/media/lucas/Data Disk/Mestrado/Aula/PMR5014/Artigo/system/Altitude-Estimator/flight data/Data.csv")

flight_data.data = flight_data.data.head(7)
flight_data.data["___vY__m/s"] = 0
flight_data.data["___vZ__m/s"] = 0


flight_filter = KalmanFilter(flight_model=flight, simalution= flight_data)
flight_filter.set_initial_state()

# print(flight_filter.initial_state)
flight_filter.calc_altitude()

# flight_filter.state_a_priore()

# test = []
# for i in range(0, len(flight_filter.h_calc)):
#     test.append(-1*flight_filter.h_calc[i])
#     print("Alt",i, flight_filter.h_calc[i],"\n")



plt.figure()
plt.plot(flight_filter.simulation.simulation_time, -1*flight_filter.simulation.altitude, label = "Sensor (Failing)")
plt.plot(flight_filter.simulation.simulation_time, flight_filter.altitude, label = "Estimated")
plt.plot(flight_filter.simulation.simulation_time, flight_data.data['__alt__ind']*0.3048, label = "Real")
plt.xlabel("Time [s]")
plt.ylabel("Altitude [m]")
plt.grid()
plt.legend()


error = [100*abs(flight_data.data['__alt__ind'][i]*0.3048 + flight_filter.simulation.altitude[i])/(flight_data.data['__alt__ind'][i]*0.3048) for i in range(0, len(flight_filter.simulation.altitude))]

plt.figure()
plt.plot(flight_filter.simulation.simulation_time,  error, label = "Error")
plt.xlabel("Time [s]")
plt.ylabel("Error [%]")
plt.grid()
plt.legend()
plt.show()
