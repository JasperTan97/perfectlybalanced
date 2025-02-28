from controllers.pid import PIDController, CascadedPIDController
from grapher import LivePIDGrapher

import pybullet as p
import time
import pybullet_data
import numpy as np

from typing import Dict, Union, Optional


ControllerType = Union[PIDController, CascadedPIDController]
VisualiserType = Union[LivePIDGrapher]


class BalanceSim:
    """
    Simulation using pybullet for 2D balancing robot
    """

    def __init__(
        self,
        controller: ControllerType,
        dt: float = 1 / 240,
        robot_urdf: str = "description/perfbalanced.urdf",
        use_gui: bool = False,
        grapher: Optional[VisualiserType] = None,
    ):
        """
        Args:
            controller(ControllerType): controller used
            dt(float): sim time step, 1/240 simulates realtime in pybullet
            robot_urdf(str): urdf file
            use_gui(bool): Use pybullet graphical version if true
            grapher(Optional[VisualiserType]): For viewing error values
        """
        self.controller = controller
        self.dt = dt
        self.robot_urdf = robot_urdf
        self.grapher = grapher

        # max values
        self.max_torque = 500

        # temp values
        self.temp_dt_controller = 0  # used to track dt to step controller
        self.output = 0

        if use_gui:
            physicsClient = p.connect(p.GUI)
        else:
            physicsClient = p.connect(p.DIRECT)

        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setPhysicsEngineParameter(numSolverIterations=50)  # Increase solver accuracy
        p.setPhysicsEngineParameter(fixedTimeStep=dt)
        p.setGravity(0, 0, -10)
        self.planeId = p.loadURDF("plane.urdf")
        p.changeDynamics(self.planeId, -1, lateralFriction=1.0)
        robotStartPos = [0, 0, 0.01]
        robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robotId = p.loadURDF(robot_urdf, robotStartPos, robotStartOrientation)
        for joint in range(p.getNumJoints(self.robotId)):
            p.changeDynamics(self.robotId, joint, linearDamping=0.1, angularDamping=0.1)
            p.changeDynamics(self.robotId, joint, lateralFriction=1.0, rollingFriction=0.1, spinningFriction=0.1)

    def get_robot_state(self) -> Dict[str, float]:
        """
        Robot state in 2D defined as:
            - position
            - velocity
            - theta (angle with the ground)
            - angular velocity

        Returns:
            List[float]: robot state
        """
        base_pos, base_quat = p.getBasePositionAndOrientation(self.robotId)
        base_euler = p.getEulerFromQuaternion(base_quat)
        linear_vel, angular_vel = p.getBaseVelocity(self.robotId)

        yaw = base_euler[2]  # Extract yaw rotation

        # Rotate velocity into robot frame
        x_world, y_world = base_pos[0], base_pos[1]
        x_robot = np.cos(yaw) * x_world + np.sin(yaw) * y_world
        x_dot_world, y_dot_world = linear_vel[0], linear_vel[1]
        x_dot_robot = np.cos(yaw) * x_dot_world + np.sin(yaw) * y_dot_world

        state = {
            "x": x_robot,  # Ignore absolute world x for 2D
            "theta": base_euler[0],  # Roll angle (tilt)
            "x_dot": x_dot_robot,  # Velocity in robot frame
            "theta_dot": angular_vel[0],  # Angular velocity
        }
        return state

    def set_wheel_velocity(self, target_wheel_velocity: float) -> None:
        """
        For velocity control
        Args:
            target_wheel_velocity(float): Desired wheel velocity
        """
        p.setJointMotorControl2(
            self.robotId,
            0,
            p.VELOCITY_CONTROL,
            targetVelocity=target_wheel_velocity,
            force=10000,
        )
        p.setJointMotorControl2(
            self.robotId,
            1,
            p.VELOCITY_CONTROL,
            targetVelocity=target_wheel_velocity,
            force=10000,
        )

    def set_wheel_torque(self, target_wheel_torque: float) -> None:
        """
        For force control
        Args:
            target_wheel_torque(float): Desired wheel velocity
        """
        target_wheel_torque = min(max(target_wheel_torque, -self.max_torque), self.max_torque)
        print(target_wheel_torque)
        p.setJointMotorControl2(
            self.robotId, 0, p.TORQUE_CONTROL, force=target_wheel_torque
        )
        p.setJointMotorControl2(
            self.robotId, 1, p.TORQUE_CONTROL, force=target_wheel_torque
        )

    def step_controller(self):
        if self.temp_dt_controller <= 0.0:
            self.temp_dt_controller == self.dt
            state = self.get_robot_state()
            output = self.controller.step(state)
            self.output = output
        self.temp_dt_controller -= self.controller.dt

    def run_simulation(self, steps: int = 20000):
        """
        Runs the simulation
        """
        for step in range(steps):
            if step == 30:  # apply force at t = 30
                p.applyExternalForce(
                    self.robotId, -1, [0, 0, 0], [0, 0, 0], p.LINK_FRAME
                )
                print("Applying force")
            self.step_controller()
            if isinstance(self.grapher, LivePIDGrapher):
                self.grapher.update_plot(self.get_robot_state()["theta"])
            self.set_wheel_torque(self.output)
            p.stepSimulation()
            time.sleep(self.dt)
            contact_points = p.getContactPoints(self.robotId, self.planeId)
            print(f"Contact Points: {len(contact_points)}")


if __name__ == "__main__":
    controller = CascadedPIDController([1.1, 0.01, 0], [400, 200, 10], 1, 1/60)
    grapher = LivePIDGrapher()
    sim = BalanceSim(controller=controller, use_gui=True, grapher=None, dt=1 / 240)
    sim.run_simulation()
