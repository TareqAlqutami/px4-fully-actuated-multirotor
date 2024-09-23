import numpy as np


class TrapezoidalProfile:
    """
    Trajectory generation with trapezoidal velocity profile
    """
    def __init__(self,n:int, max_acc:float) -> None:
        """Implements a trapezoidal velocity profile and ensure bounded accelerations.

        Args:
            n (int): number of dimensions. e.g 3 for cartesian x,y,z motion
            max_acc (float): maximum acceleration allowed in all dimensions
        """
        self.n = n
        self.max_acc = max_acc
        self.dt = 0.0
        self.vel_sp_trap = np.zeros(self.n)
        self.prev_acc = np.zeros(self.n)
        self.prev_vel = np.zeros(self.n)
        self.vel = np.zeros(self.n)
        self.pos = np.zeros(self.n)
        self.first_call = True

    def reset(self) -> None:
        """Reset internal state for instance due to change in a parameter.
        """
        self.dt = 0.0
        self.vel_sp_trap = np.zeros(self.n)
        self.prev_acc = np.zeros(self.n)
        self.prev_vel = np.zeros(self.n)
        self.vel = np.zeros(self.n)
        self.pos = np.zeros(self.n)
        self.first_call = True

    def set_max_acc(self,max_acc:float):
        """Change max acceleration value 

        Args:
            max_acc (float): desired max acceleration
        """
        self.max_acc = max_acc
        self.reset()

    def update_vel_sp(self, dt:float, vel_sp:np.array, vel:np.array) -> np.array:
        """Update the velocity set point given raw set point and current velocity 
        and time since last update

        Args:
            dt (float): time since last update in seconds
            vel_sp (np.array): raw velocity set point
            vel (np.array): current velocity

        Returns:
            np.array: desired velocity set point to achieve trapezoidal profile
        """
        if self.first_call:
            self.vel_sp_trap = vel # start from current state
            self.first_call = False

        # calculate acc
        acc = (vel_sp - self.vel_sp_trap)/dt
        # limit to max acceleration
        idx = np.abs(acc)>self.max_acc
        acc[idx] = np.sign(acc[idx])*self.max_acc

        # calculate velocity set-point satisfying trapezoidal profile
        self.vel_sp_trap = self.vel_sp_trap + acc*dt

        # estimate position
        self.pos += self.vel_sp_trap*dt

        return self.vel_sp_trap

    def update_pos_sp(self, dt:float, pos_sp:np.array, pos:np.array, vel:np.array) -> np.array:
        """Update the position set point given raw set point and current position and velocity
        and time since last update.

        Args:
            dt (float): time since last update in seconds
            pos_sp (np.array): raw position set point
            pos (np.array): current position
            vel (np.array): current velocity

        Returns:
            np.array: desired position set point to follow a trapezoidal velocity profile
        """
        # TODO
        raise NotImplementedError("To be implemented")
