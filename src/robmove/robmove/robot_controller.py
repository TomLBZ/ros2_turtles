"""Provided class: RobotController\n
RobotController class contains a state machine for controlling a robot,
and provides methods for moving the robot forward and turning it,
as well as methods for smooth stopping based on the current velocity, etc.
"""

from math import pi
from typing import Callable, Dict, Self
from robmove.state_machine import StateMachine

class RobotController(StateMachine):
    """
    RobotController class contains a state machine for controlling a robot,
    and provides useful methods for robot control:
    - moving the robot forward
    - turning the robot
    - stopping the robot
    - smooth stopping based on the current velocity
    - setting the initial pose of the robot
    - updating the pose of the robot
    - calculate linear and angular distances from target position / orientation
    - reset robot states\n
    The default states are `idle` and `stop`, more can be added via `add_state`.
    """

    def __init__(self, name: str, drive_func: Callable[[float, float], None], 
                 max_lin_vel: float = 1.0, max_ang_vel: float = 1.0, 
                 ang_delta: float = pi / 9000, lin_delta: float = 0.0002,
                 smoothstop_linvel: float = 0.5, smoothstop_angvel: float = 0.5) -> Self:
        """
        Initialize the robot controller with the default `idle` and `stop` states 
        with the given parameters:
        - `name`: name of the robot
        - `drive_func`: drive function that takes linear and angular velocities as arguments
        - `max_lin_vel`: maximum linear velocity
        - `max_ang_vel`: maximum angular velocity
        - `ang_delta`: minimum angular distance to stop turning
        - `lin_delta`: minimum linear distance to stop moving
        - `smoothstop_linvel`: below which linear velocity is smoothed
        - `smoothstop_angvel`: below which angular velocity is smoothed
        """
        super().__init__(name)
        # initialize parameters
        self.add_state('stop', self.stop)
        self.max_lin_vel: float = max_lin_vel # maximum linear velocity
        self.max_ang_vel: float = max_ang_vel # maximum angular velocity
        self.ang_delta: float = ang_delta # minimum angular distance to stop turning
        self.lin_delta: float = lin_delta # minimum linear distance to stop moving
        self.smoothstop_linvel: float = smoothstop_linvel # below which linear velocity is smoothed
        self.smoothstop_angvel: float = smoothstop_angvel # below which angular velocity is smoothed
        self.drive: Callable[[float, float], None] = drive_func # drive function
        # linear and angular velocities
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        # initial position and orientation
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_theta = 0.0
        # current position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # last position and orientation (at the last vertex)
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0
        self.last_vert_index = 0
        # callbacks used while stopping the robot
        self.stop_callbacks: Dict[str, Callable[[None], None]] = {}

    def lin_dist(self, from_x: float, from_y: float, to_x: float, to_y: float) -> float:
        """Calculate the linear distance between (`from_x`, `from_y`) and (`to_x`, `to_y`)"""
        return ((to_x - from_x)**2 + (to_y - from_y)**2)**0.5

    def lin_dist_from_last_vert(self) -> float:
        """Calculate the linear distance from the last vertex"""
        return self.lin_dist(self.last_x, self.last_y, self.x, self.y)
    
    def ang_dist(self, from_ang: float, to_ang: float) -> float:
        """Calculate the angle between `from_ang` and `to_ang` in radians (0 to 2*pi)"""
        return (2 * pi + to_ang - from_ang) % (2 * pi)

    def ang_dist_from_last_vert(self) -> float:
        """Calculate the angle from the last vertex's orientation in radians (0 to 2*pi)"""
        return self.ang_dist(self.last_theta, self.theta)

    def get_smoothstop_vel(self, value: float, threshold: float, maxvel: float) -> float:
        """Smooth velocity for stopping when `value` is below `threshold`"""
        if value > threshold:
            return maxvel
        else:
            scaling = 1.0 / threshold
            return -maxvel * (scaling * value - 1) ** 2 + maxvel

    def get_smoothstop_linvel(self, value: float) -> float:
        """Get the linear velocity for smooth stopping based on the current `value`"""
        return self.get_smoothstop_vel(value, self.smoothstop_linvel, self.max_lin_vel)
    
    def get_smoothstop_angvel(self, value: float) -> float:
        """Get the angular velocity for smooth stopping based on the current `value`"""
        return self.get_smoothstop_vel(value, self.smoothstop_angvel, self.max_ang_vel)

    def reset_states(self) -> None:
        """Reset the states list to only contain the `idle` and `stop` states"""
        super().reset_states()
        self.add_state('stop', self.stop)

    def update_pose(self, x: float, y: float, theta: float) -> None:
        """Update the current pose of the robot to (`x`, `y`, `theta`)"""
        self.x = x
        self.y = y
        self.theta = theta

    def update_last_pose(self) -> None:
        """Update the last pose of the robot to the current pose"""
        self.last_x = self.x
        self.last_y = self.y
        self.last_theta = self.theta

    def set_init_pose(self, x: float, y: float, theta: float) -> None:
        """Set the initial pose of the robot to (`x`, `y`, `theta`)"""
        self.init_x = x
        self.init_y = y
        self.init_theta = theta

    def initialize_pose(self) -> None:
        """Initialize the current and last poses to initial values, and set vertex index to 0"""
        self.update_pose(self.init_x, self.init_y, self.init_theta)
        self.update_last_pose()
        self.update_last_vert_index(reset=True)

    def update_last_vert_index(self, reset: bool = False) -> None:
        """Increment last vertex index, reset to `0` if needed"""
        self.last_vert_index = 0 if reset else self.last_vert_index + 1

    def update_drive_function(self, drive_func: Callable[[float, float], None]) -> None:
        """Update the drive function of the robot to use `drive_func`"""
        self.drive = drive_func

    def forward(self, target_dist: float, next_state: str) -> None:
        """Move the robot forward by `target_dist` and transition to the `next_state`"""
        dist_left: float = target_dist - self.lin_dist_from_last_vert()
        lin_vel: float = self.get_smoothstop_linvel(dist_left) # get smooth linear velocity based on the distance left
        self.drive(lin_vel, 0.0) # set linear velocity = lin_vel, angular velocity = 0
        if dist_left < self.lin_delta: # if target_dist is considered reached
            self.update_last_pose()
            self.update_last_vert_index()
            self.set_state(next_state)

    def turn(self, target_angle: float, next_state: str) -> None:
        """Turn the robot by `target_angle` in radians and transition to `next_state`"""
        ang_left: float = self.ang_dist(self.ang_dist_from_last_vert(), target_angle)
        ang_vel = self.get_smoothstop_angvel(ang_left) # get smooth angular velocity based on the angle left
        self.drive(0.0, ang_vel) # set linear velocity = 0, angular velocity = ang_vel
        if ang_left < self.ang_delta: # if target_angle is considered reached
            self.update_last_pose()
            self.update_last_vert_index()
            self.set_state(next_state)

    def stop(self) -> None:
        """
        Stop the robot by:
        - setting linear and angular velocities to 0
        - transition to the `idle` state
        - call all stop callbacks\n
        To implement custom behaviours, add a callback to the `stop_callbacks` dictionary
        or override this method in a subclass.
        """
        self.drive(0.0, 0.0)
        self.set_state('idle')
        for callback in self.stop_callbacks.values():
            callback()