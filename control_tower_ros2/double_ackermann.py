import numpy as np
import matplotlib.pyplot as plt


class DoubleAckermannSteering:

    def __init__(self, velocity, turn_radius, l=0.711, w=0.558):

        self.velocity = velocity
        self.turn_radius = turn_radius
        # self.direction = direction
        self.l = l
        self.w = w
        self.compute_steering()

    def compute_steering(self):
        """
        Computes the individual wheel states for the provided velocity and turning radius.
        """

        # Straight-line motion
        if self.turn_radius == float("inf"):
            self.theta_f_right = 0
            self.theta_f_left = 0
            self.theta_r_right = 0
            self.theta_r_left = 0
            self.v_f_right = self.velocity
            self.v_f_left = self.velocity
            self.v_r_right = self.velocity
            self.v_r_left = self.velocity
        else:
            if self.turn_radius < 0:
                turn_direction = -1
                self.turn_radius *= -1
            else:
                turn_direction = 1

            # Steering angles
            self.theta_f_right = float(turn_direction *
                                       np.arctan2(self.l / 2, (self.turn_radius + (turn_direction * -1 * self.w / 2))))
            self.theta_f_left = float(turn_direction *
                                      np.arctan2(self.l / 2, (self.turn_radius + (turn_direction * self.w / 2))))
            self.theta_r_right = float(-self.theta_f_right)
            self.theta_r_left = float(-self.theta_f_left)

            # Angular velocity
            omega = self.velocity / self.turn_radius

            # Wheel velocities
            self.v_f_right = float(omega * (self.turn_radius - self.w / 2))
            self.v_f_left = float(omega * (self.turn_radius + self.w / 2))
            self.v_r_right = float(self.v_f_right)
            self.v_r_left = float(self.v_f_left)

    def display_results(self):
        """Prints computed steering angles and wheel velocities"""
        print("Double Ackermann Steering Visualization:")
        print(f"Turning Angle R: {self.turn_radius}")
        # print(f"Directional Angle in radians: {self.str_angle}")
        print(
            f"Front right Wheel Angle: {self.theta_f_right:.2f}° | Velocity: {self.v_f_right:.2f} m/s")
        print(
            f"Front left Wheel Angle: {self.theta_f_left:.2f}° | Velocity: {self.v_f_left:.2f} m/s")
        print(
            f"Rear right Wheel Angle: {self.theta_r_right:.2f}° | Velocity: {self.v_r_right:.2f} m/s")
        print(
            f"Rear left Wheel Angle: {self.theta_r_left:.2f}° | Velocity: {self.v_r_left:.2f} m/s")

    def visualize(self):
        """Visualizes the vehicle aligned along the Y-axis with steering angles in radians (displayed in degrees)."""

        # Define the vehicle body (facing +Y)
        vehicle_y = [-self.l / 2, self.l / 2,
                     self.l / 2, -self.l / 2, -self.l / 2]
        vehicle_x = [-self.w / 2, -self.w / 2,
                     self.w / 2, self.w / 2, -self.w / 2]

        # Define wheel positions [X, Y] in top-down view
        wheel_positions = np.array([
            [self.w / 2,  self.l / 2],   # Front Left
            [-self.w / 2,  self.l / 2],   # Front Right
            [self.w / 2, -self.l / 2],   # Rear Left
            [-self.w / 2, -self.l / 2],   # Rear Right
        ])

        # Convert wheel angles to radians (already in radians)
        angles_rad = [
            self.theta_f_right, self.theta_f_left,
            self.theta_r_right, self.theta_r_left
        ]

        # Compute direction vectors (X, Y) from angles
        wheel_vectors = np.array([
            [np.sin(ang), np.cos(ang)] for ang in angles_rad
        ])

        # Scale by velocity
        velocities = [
            self.v_f_right, self.v_f_left,
            self.v_r_right, self.v_r_left
        ]

        wheel_vectors *= np.array(velocities)[:, None]

        # Normalize for plotting
        max_v = max(abs(v) for v in velocities)
        if max_v > 0:
            wheel_vectors /= max_v  # normalize to length 1
        arrow_scale = 0.5
        wheel_vectors *= arrow_scale

        # Start plot
        plt.figure(figsize=(6, 8))
        plt.plot(vehicle_x, vehicle_y, 'k', linewidth=2, label="School Bus")

        # Draw axles
        plt.plot([-self.w / 2, self.w / 2],
                 [self.l / 2,  self.l / 2], 'b', linewidth=1)
        plt.plot([-self.w / 2, self.w / 2],
                 [-self.l / 2, -self.l / 2], 'b', linewidth=1)

        # Draw wheel vectors
        plt.quiver(
            wheel_positions[:, 0], wheel_positions[:, 1],
            wheel_vectors[:, 0], wheel_vectors[:, 1],
            color='r', angles='xy', scale_units='xy', scale=1, width=0.01
        )

        # Mark wheel positions
        plt.scatter(wheel_positions[:, 0],
                    wheel_positions[:, 1], color='k', zorder=3)

        # Annotate angles (in degrees for readability)
        plt.text(self.w / 2 + 0.2,  self.l / 2,
                 f"{np.degrees(self.theta_f_right):.1f}°", color='r', fontsize=12)
        plt.text(-self.w / 2 - 0.8,  self.l / 2,
                 f"{np.degrees(self.theta_f_left):.1f}°", color='r', fontsize=12)
        plt.text(self.w / 2 + 0.2, -self.l / 2,
                 f"{np.degrees(self.theta_r_right):.1f}°", color='r', fontsize=12)
        plt.text(-self.w / 2 - 0.8, -self.l / 2,
                 f"{np.degrees(self.theta_r_left):.1f}°", color='r', fontsize=12)

        # Labels and plot settings
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Double Ackermann Steering Visualization (Facing Y+)")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.show()


# Example Usage
if __name__ == "__main__":
    vehicle = DoubleAckermannSteering(2, 2, l=2.5, w=1.5)
    vehicle.display_results()
    vehicle.visualize()
