from typing import Tuple, Dict, Optional


Gains = Tuple[float, float, float]
"""
Kp, Ki, Kd
"""


class CascadedPIDController:
    def __init__(
        self, gains_inner_loop: Gains, gains_outer_loop: Gains, ref_value:float = 0, dt: float = 1 / 60
    ):
        """
        Args:
            gains_inner_loop(gains): Kp, Ki, and Kd values for the inner loop
            gains_outer_loop(gains): Kp, Ki, and Kd values for the outer loop
            ref_value(float): inner loop reference value
            dt (float): time step
        """
        self.PID_inner = PIDController(*gains_inner_loop, dt=dt)
        self.PID_outer = PIDController(*gains_outer_loop, dt=dt)
        self.ref_value = ref_value
        self.dt = dt

    def step(self, state: float) -> float:
        """
        Compute PID output for balance control.

        Args:
            state(Dict[str:float]): 2D robot state (x, theta, x_dot, theta_dot)

        Returns:
            float: Torque command
        """
        theta = state["theta"]
        x = state["x"]
        theta_ref = self.PID_inner.step(x, self.ref_value)
        command = self.PID_outer.step(theta, theta_ref)
        return command


class PIDController:
    def __init__(
        self, Kp: float = 300, Ki: float = 200, Kd: float = 50, dt: float = 1 / 60
    ):
        """
        Args:
            Kp (float): Proportional gain
            Ki (float): Integral gain
            Kd (float): Derivative gain
            dt (float): time step
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

        self.dt = dt

    def step(self, error: float, ref_value: float = 0) -> float:
        """
        Compute PID output for balance control.

        Args:
            error(float): State variable to send to reference value
            ref_value(float): Reference value
        Returns:
            float: command
        """
        error -= ref_value
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        # Compute control output
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output
