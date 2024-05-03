import numpy as np
import random
import math


#control settings for PID: kp = -3, ki = 0.0001, kd = 0.001

#control settings for P: kp = -3, ki = 0, kd = 0

#control settings for PI: kp = -3, ki = 0.0001, kd = 0

#control settings for PD: kp = -3, ki = 0, kd = 0.0001

#control settings for DI: kp = 0, ki = -0.2 , kd = -0.2

class PID_controller:
    def __init__(self):
        self.prev_action = 0 #action is in torq
        self.prev_error = 0
        self.integral = 0
        self.kp = -3
        self.ki = 0.0001
        self.kd = 0.0001
        self.derivative = 0
        self.proportional = 0

    def reset_state(self):
        pass #TODO if needed
    
    def get_action(self, state, image_state, random_controller=False):
        
        terminal, timestep, x, x_dot, theta, theta_dot, reward = state
        

        if random_controller:
            return np.random.uniform(-1, 1)
        
        else:
            target_angle = 0
            error = target_angle - theta
            

            self.integral += error
            self.derivative = error - self.prev_error

            action = self.kp * error + self.ki * self.integral + self.kd * self.derivative

            self.prev_error = error

            #uncomment the following only for part D: controller disturbances
            if np.random.rand() > 0.99:
                action = 10
            
            return action
