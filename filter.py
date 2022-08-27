# Auhor: Lucas Marques Moreno
# Desriptiton: Kalman Filter

class KalmanFilter:
    
    def __init__(self) -> None:
        self.altitude = 0
    
    def altitude_estimation(self) -> None:
        pass
    
    @property
    def altitude(self) -> float:
        return self.altitude