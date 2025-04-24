import numpy as np

class FixedHeadingSteering:

    def __init__(self, lx, ly, direction, l=0.711, w=0.558, max_speed=2.0):
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
        
        self.lx = lx
        self.ly = ly
        self.direction = direction
        self.l = l
        self.w = w
        self.max_speed = max_speed  # max forward/reverse speed
        self.v = 0  # Will be set based on ly and max_speed
        self.str_angle = 0
        self.compute_steering()

    def omega(self):
        """Computes angular velocity (0 if moving straight)"""
        return 0 if self.r == float("inf") else self.v / self.r

    def compute_steering(self):
        """
        Maps:
        - lx ∈ [1000, 2000] → steering angle ∈ [-30°, +30°]
        - ly ∈ [1000, 2000] → velocity ∈ [-max_speed, +max_speed]
        """
        max_angle_deg = 90 # Maximum steering angle in degrees
        max_angle = np.radians(max_angle_deg)

        # Map lx to steering angle
        lx_clamped = np.clip(self.lx, 1000, 2000)
        normalized_lx = (lx_clamped - 1500) / 500.0  # [-1, 1]
        self.str_angle = normalized_lx * max_angle

        # Map ly to velocity
        ly_clamped = np.clip(self.ly, 1000, 2000)
        normalized_ly = (ly_clamped - 1500) / 500.0  # [-1, 1]
        if self.direction == 0 and normalized_ly < 0:
            normalized_ly = 0
        elif self.direction == 1 and normalized_ly > 0:
            normalized_ly = 0
        self.v = normalized_ly * self.max_speed


        # Convert to steering angle in radians
        self.str_angle = normalized_lx * max_angle
        
        self.theta_f_right = self.str_angle
        self.theta_f_left = self.str_angle
        self.theta_r_right = self.str_angle
        self.theta_r_left = self.str_angle
        self.v_f_right = self.v
        self.v_f_left = self.v
        self.v_r_right = self.v
        self.v_r_left = self.v

        