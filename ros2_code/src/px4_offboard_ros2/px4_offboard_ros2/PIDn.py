#!/usr/bin/env python3

"""
An implementation of n-dimensional PID controller in Python.
Features:
 - MIMO implementation 
 - Integral anti-wind up
 - Choice of calculating derivative on measurements or errors or providing the derivative directly
 - Designed with use in ROS2 in mind and with the ability to use online tuning using ros2 parameters and dynamic reconfigure (see examples)
"""

from typing import Tuple
import numpy as np


class PIDn:
    """
    n-dimensional PID control
    """
    def __init__(self,n:int, kp:np.array,ki:np.array, kd:np.array,
                 i_max:np.array, i_min:np.array, anti_windup: bool, 
                 state_derivative:bool=False, user_provide_state_dot:bool=False) -> None:
        """n-dimensional PID control

        Args:
            n (int): number of dimensions. e.g. 3 for cartesian x,y,z control
            kp (np.array): proportional gains
            ki (np.array): integral gains
            kd (np.array): derivative gains
            i_max (np.array): integral max limits (when anti-wind up is enabled)
            i_min (np.array): integral min limits (when anti-wind up is enabled)
            anti_windup (bool): enable integral anti wind-up
            state_derivative (bool, optional): use state/measurement derivatives instead of error derivatives. Defaults to False.
            user_provide_state_dot (bool, optional): Use provided derivative of state, state_derivative should be True to be in effect. Defaults to False.
        """
        self.set_gains(kp,ki,kd)
        # activate integral anti wind up and set the integral limits
        self.set_anti_windup(i_max,i_min,anti_windup)
        # calculate the derivative from state or from errors? refer to https://controlguru.com/pid-control-and-derivative-on-measurement/ for explanation
        self.state_derivative = state_derivative
        self.user_provide_state_dot = user_provide_state_dot # user provides the state derivative
        self.n = n # number of dimensions
        # tracking variables
        self.state   = np.zeros(self.n)
        self.cmd   = np.zeros(self.n)
        self.error   = np.zeros(self.n)
        self.error_dot   = np.zeros(self.n)
        self._prev_error  = np.zeros(self.n)
        self._prev_state = np.zeros(self.n)
        self._prev_cmd = np.zeros(self.n)
        self._prev_derv  = np.zeros(self.n)
        self._integral   = np.zeros(self.n)
        self.p_term   = np.zeros(self.n) # track P effort
        self.i_term   = np.zeros(self.n) # track I effort
        self.d_term   = np.zeros(self.n) # track D effort
        self.effort = np.zeros(self.n) # track total effort
        

        #TODO add derivative filter


    def set_gains(self,  kp:np.array,ki:np.array, kd:np.array):
        """Set PID gains

        Args:
            kp (np.array): proportional gains
            ki (np.array): integral gains
            kd (np.array): derivative gains
        """
        self.kp =  kp
        self.ki = ki
        self.kd = kd
        assert np.all(self.kp>0), "kp must be greater than or equal zero"
        assert np.all(self.ki>0), "ki must be greater than or equal zero"
        assert np.all(self.kd>0), "kd must be greater than or equal zero"

    def set_anti_windup(self, i_max:np.array, i_min:np.array, anti_windup: bool):
        """Enable/disable integral ant windup and change the limits

        Args:
            i_max (np.array): max integral limit
            i_min (np.array): min integral limits
            anti_windup (bool): enable/disable anti windup
        """
        self.anti_windup = anti_windup
        self.i_max = i_max
        self.i_min = i_min
        assert np.all(self.i_max>self.i_min), "i_min is greater than i_max!!"

    def set_derivative_method(self,on_state:bool,user_provide_state_dot:bool):
        """Set the method to calculate derivative and derivative term of PID.

        Args:
            on_state (bool): calculate derivative from state/measurement if True
            user_provide_state_dot (bool): use user provided state derivative if True
        """
        self.state_derivative = on_state
        self.user_provide_state_dot = user_provide_state_dot
        self._prev_derv = np.zeros(self.n)
        self._prev_error = np.zeros(self.n)
        self._prev_state = np.zeros(self.n)

    def reset(self):
        """Reset PID variables such as integral term. This should be called when PID parameters or settings are changed.
        """
        self.error   = np.zeros(self.n)
        self.error_dot   = np.zeros(self.n)
        self._prev_error   = np.zeros(self.n)
        self._prev_state = np.zeros(self.n)
        self._prev_derv  = np.zeros(self.n)
        self._integral   = np.zeros(self.n)
        
        
    def match_state(self, state,state_dot=None):
        """Used before transition into the controller. Ensure the previous state and error are the same as the current one so that
        no derivative kick happens and also zero the integral term
        """
        self._prev_state = state
        self._prev_error = np.zeros(self.n)
        self._integral   = np.zeros(self.n)
        if state_dot is None:
            self.error_dot   = np.zeros(self.n)
        else:
            self.error_dot   = state_dot
        self._prev_derv  = np.zeros(self.n)

    def print_state(self):

        return f"""controller state:   
     - prev_state:{self._prev_state} 
     - _integral:{self._integral}
     - error:{self.error}
     - prev_error:{self._prev_error} 
     - error_dot:{self.error_dot} 
     - prev_derv:{self._prev_derv}
              """

    def _compute_errors(self, dt:float, state:np.array, cmd:np.array, state_dot:np.array)->np.array:
        """Compute the error and error derivative and update PID state

        Args:
            dt (float): time since last update
            state (np.array): current state/measurements
            cmd (np.array): current command values
            state_dot (np.array): user provided state derivative, set to None if not provided.
        """
        self.error = cmd - state
        if self.state_derivative:
            if self.user_provide_state_dot and state_dot is not None:
                # user provides the derivative of measurements
                self.error_dot = state_dot
            else:
                # Calculate the derivative of measurements
                self.error_dot = (self._prev_state - state) / dt
        else:
            # Calculate the derivative of error
            self.error_dot = (self.error  - self._prev_error) / dt
            
        self._prev_error = self.error
        self._prev_state = self.state
        self._prev_cmd = self.cmd
        self.cmd = cmd
        self.state = state

    def update(self,dt:float, state:np.array, cmd:np.array, state_dot:np.array=None) -> np.array:
        """Run PID once and return the PID output (efforts) to be sent to actuators.
        This method should be called regularly within a loop.

        Args:
            dt (float): time since last update
            state (np.array): current measurements or state values
            cmd (np.array): current desired values
            state_dot (np.array, optional): state derivatives provided by user. Defaults to None.

        Returns:
            np.array: PID outputs to send to actuators
        """
        self._compute_errors(dt, state, cmd, state_dot)

        if dt == 0.0  or np.any(np.isnan(self.error)) or np.any(np.isinf(self.error)):
            print(f"Warning: computed error is not defined or infinite {self.error}")

            return np.zeros(self.n)

        if  np.any(np.isinf(self.error_dot)) or np.any(np.isnan(self.error_dot)):
            print(f"Warning: derivative of error is not defined or infinite {self.error_dot}")
            return np.zeros(self.n)

        # Calculate proportional term
        self.p_term = self.kp * self.error

        # Calculate the integral of error
        self._integral += dt * self.error
        # Calculate integral term
        self.i_term = self.ki * self._integral
        if self.anti_windup:
            # Limit i_term so that the limit is meaningful in the output
            self.i_term = np.clip(self.i_term, self.i_min, self.i_max)

        # Calculate derivative term
        if self.state_derivative:
            self.d_term = -self.kd* self.error_dot
        else:
            self.d_term = self.kd* self.error_dot

        # Compute the PID output
        self.effort = self.p_term + self.i_term + self.d_term

        return self.effort

    def get_current_errors(self) -> Tuple[np.array,np.array,np.array]:
        """return the current PID errors

        Returns:
            Tuple[np.array,np.array,np.array]: tuple of P-error, I-error, and D-error
        """
        # get current P-error, I-error and D-error
        return self.error, self._integral, self.error_dot

    def get_current_efforts(self) -> Tuple[np.array,np.array,np.array]:
        """return the current PID efforts coming from individual terms P,I,D

        Returns:
            Tuple[np.array,np.array,np.array]: tuple of P-effort, I-effort, and D-effort
        """
        # get current P-effort, I-effort and D-effort
        return self.p_term, self.i_term, self.d_term
    