import numpy as np
import matplotlib.pyplot as plt


class RotateSteering:

    def __init__(self, rotate_velocity, l=0.711, w=0.558):
        """
        Key Variable Units:

        - `self.lx`             : Raw lateral (steering) input from iBus (unitless, 1000 - 2000)
        - `self.ly`             : Raw longitudinal (throttle) input from iBus (unitless, 1000 - 2000)
        - `self.L`              : Vehicle length (meters, **m**)
        - `self.W`              : Vehicle width (meters, **m**)
        - `self.v`              : Vehicle's forward/reverse speed (meters per second, **m/s**)
        - `self.R`              : Turning radius (meters, **m**)
        - `self.str_angle`      : Steering angle of the centerline (radians)
        - `self.omega`          : Angular velocity (radians per second, **rad/s**)
        - `theta_f_left`, etc.  : Steering angles of each wheel (radians)
        - `v_f_left`, etc.      : Individual wheel velocities (meters per second, **m/s**)
        """
        self.rotate_velocity = rotate_velocity
        self.l = l
        self.w = w
        self.compute_steering()

    def compute_steering(self):
        """
        Computes the individual wheel states for the provided velocity and turning angle.
        """

        # Steering angles
        self.theta_f_right = float(np.arctan2(
            (self.l / 2), (self.w / 2))) * -1.0
        self.theta_f_left = float(-self.theta_f_right)
        self.theta_r_right = float(-self.theta_f_right)
        self.theta_r_left = float(-self.theta_f_left)

        self.v_f_right = -self.rotate_velocity
        self.v_f_left = self.rotate_velocity
        self.v_r_right = -self.rotate_velocity
        self.v_r_left = self.rotate_velocity
