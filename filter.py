# Auhor: Lucas Marques Moreno
# Desriptiton: Kalman Filter

from flight_simulator_interface import FlightSimulator
from model import FlightModel
from scipy.integrate import odeint, solve_ivp
import numpy as np


class KalmanFilter:
    
    def __init__(self, flight_model: FlightModel, simalution: FlightSimulator) -> None:

        self.model = flight_model
        self.simulation = simalution

        self.altitude = []

        self.u_calc = []
        self.w_calc = []
        self.q_calc = []
        self.h_calc = []

        self.state = []

        self.A = np.identity(4)

        sensor = {"u": np.sqrt(0.08*self.model.g),
                  "w": np.sqrt(0.08*self.model.g),
                  "q": np.sqrt(0.02),
                  "h": np.sqrt(1)} # altimetro BMP180
        
        self.H = np.array([[0, 1, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

        self.R = np.array([[np.random.normal(0,sensor["u"]), np.random.normal(0,sensor["w"]), np.random.normal(0,sensor["q"]), 0],
                           [0, 0, 0, np.random.normal(0,sensor["h"])]])

        self.Q =np.identity(4)*0 #np.array([[1, 1, 1, 0],
                           #[0, 1, 0, 1]])

        self.P =np.identity(4)# np.array([[1, 1, 1, 0],
                           #[0, 0, 0, 1]])

        self.zk = self.simulation.altitude    

    def set_filter_gain(self, gain) -> None:
        self.K = gain

    def set_initial_state(self):
        self.initial_state = [self.simulation.x_speed[0], self.simulation.z_speed[0], 
                              self.simulation.pitch_angle[0], self.simulation.altitude[0]]

    def state_a_priore(self) -> None:

        current_state = self.initial_state[0:3]
    
        self.u_calc.append(self.initial_state[0])
        self.w_calc.append(self.initial_state[1])
        self.q_calc.append(self.initial_state[2])
        self.h_calc.append(self.initial_state[3]) 
        self.state.append(np.array([self.initial_state[0], self.initial_state[1], self.initial_state[2], self.initial_state[3]]))

        for i in range(1,len(self.simulation.simulation_time)):
            time_span = [self.simulation.simulation_time[i-1],self.simulation.simulation_time[i]]
            solved = odeint(self.model._differential_model, current_state, time_span)
            #solved = solve_ivp(self.model._differential_model, state, time_span)
            self.u_calc.append(solved[1][0])
            self.w_calc.append(solved[1][1])
            self.q_calc.append(solved[1][2])

            current_altitude = self.model.altitude_model(solved[1], self.h_calc[-1], time_span[0] - time_span[1])
            self.h_calc.append(current_altitude)
            

            self.state.append( np.append(solved[1], current_altitude )   )
            current_state = solved[1]

    def state_a_priore_linearized(self) -> None:
        
        state = self.initial_state
        self.u_calc.append(self.initial_state[0])
        self.w_calc.append(self.initial_state[1])
        self.q_calc.append(self.initial_state[2])
        self.h_calc.append(self.initial_state[3]) 
        self.state.append(np.array([self.initial_state[0], self.initial_state[1], self.initial_state[2], self.initial_state[3]]))

        for i in range(1,len(self.simulation.simulation_time)):
            time_span = [self.simulation.simulation_time[i-1],self.simulation.simulation_time[i]]
            solved = odeint(self.model._linearized_model, state, time_span)
            self.u_calc.append(solved[1][0])
            self.w_calc.append(solved[1][1])
            self.q_calc.append(solved[1][2])
            self.h_calc.append(solved[1][3]) 

            self.state.append(np.array(solved[1]))
            state = solved[1]

    def set_ode_config(self, iterations: float, time_step: float) -> None:
        self.ode_time = np.linspace(0,time_step,iterations)

    def covariance_a_priore(self, state, P_old):
        # print(np.matmul(np.matmul(self.A,P_old), self.A.transpose()))
        # print(self.Q)
        P = np.matmul(np.matmul(self.A,P_old), self.A.transpose()) + self.Q
        return P
    
    def _calculate_filter_gain(self, P_minus) -> None:
        aux = np.matmul(np.matmul(self.H, P_minus), self.H.transpose())# + self.R
        #2x4 * 4x2 = 2x2 *  2x4 + 2x4 =2x4 
        K = np.matmul(np.matmul(P_minus, self.H.transpose()), aux)
        #2x4 . 4x2 = 2x2 * 2x4 = 2x4
        return K
    
    def generate_sensor_noise(self) -> None:
        pass

    def calc_altitude(self):

        self.state_a_priore()#_linearized()
        self.simulation.format_state()
        P = self.P.transpose()
        for iteration in range(0, len(self.simulation.simulation_time)):
            state = [self.u_calc[iteration], self.w_calc[iteration], self.q_calc[iteration], self.h_calc[iteration]]
            P = self.covariance_a_priore(state, P)
            k = self._calculate_filter_gain(P)
            
            tmp =  self.simulation.state[iteration] - self.state[iteration] #np.matmul(self.H.transpose(),self.state[iteration].transpose())
            temp2 = np.matmul(k.transpose(), tmp)
            state_tmp = self.state[iteration] + temp2
            self.altitude.append(-1*state_tmp[3])

    @property
    def get_altitude(self) -> float:
        return self.altitude