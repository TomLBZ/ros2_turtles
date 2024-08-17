from math import pi
from typing import Callable
from robmove.state_machine import StateMachine

# RobotController class contains a state machine for controlling a robot
# provides methods for moving the robot forward and turning it
# provides methods for smooth stopping based on the current velocity
# the `stop` and `idle` states are provided by default
class RobotController(StateMachine):

    # name: string, the name of the RobotControl node
    # max_lin_vel: float, the maximum linear velocity of the robot, default is 1.0
    # max_ang_vel: float, the maximum angular velocity of the robot, default is 1.0
    # ang_delta: float, the angular resolution in radians, default is pi / 9000 (0.02 degrees)
    # lin_delta: float, the linear resolution in meters, default is 0.0002 (0.2 mm if the unit is meters)
    # smoothstop_linvel: float, the linear velocity below which smoothstop kicks in, default is 0.5
    # smoothstop_angvel: float, the angular velocity below which smoothstop kicks in, default is 0.5
    # drive_func: f(float, float) -> None, the drive function to be used, default is None
    def __init__(self, name: str, max_lin_vel: float = 1.0, max_ang_vel: float = 1.0, 
                 ang_delta: float = pi / 9000, lin_delta: float = 0.0002,
                 smoothstop_linvel: float = 0.5, smoothstop_angvel: float = 0.5,
                 drive_func: Callable[[float, float], None] = None):
        super().__init__(name)
        self.add_state('stop', self.stop)
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel
        self.ang_delta = ang_delta
        self.lin_delta = lin_delta
        self.smoothstop_linvel = smoothstop_linvel
        self.smoothstop_angvel = smoothstop_angvel
        self.drive = drive_func
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

    # linear distance between two points
    def lin_dist(self, from_x: float, from_y: float, to_x: float, to_y: float):
        return ((to_x - from_x)**2 + (to_y - from_y)**2)**0.5

    # linear distance from the last vertex
    def lin_dist_from_last_vert(self):
        return self.lin_dist(self.last_x, self.last_y, self.x, self.y)
    
    # angular distance between two angles in radians, values from 0 to 2*pi
    def ang_dist(self, from_ang, to_ang):
        return (2 * pi + to_ang - from_ang) % (2 * pi)

    # angular distance from the last vertex in radians
    def ang_dist_from_last_vert(self):
        return self.ang_dist(self.last_theta, self.theta)

    # get the velocity for smooth stopping based on the current value, threshold, and maximum velocity
    def get_smoothstop_vel(self, val, threshold, maxvel):
        if val > threshold:
            return maxvel
        else:
            scaling = 1.0 / threshold
            return -maxvel * (scaling * val - 1) ** 2 + maxvel

    # get the linear velocity for smooth stopping based on the current value
    def get_smoothstop_linvel(self, val):
        return self.get_smoothstop_vel(val, self.smoothstop_linvel, self.max_lin_vel)
    
    # get the angular velocity for smooth stopping based on the current value
    def get_smoothstop_angvel(self, val):
        return self.get_smoothstop_vel(val, self.smoothstop_angvel, self.max_ang_vel)

    # reset the states list, override to add the default stop state
    def reset_states(self):
        super().reset_states()
        self.add_state('stop', self.stop)

    # update the current pose of the robot
    def update_pose(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    # update the last pose of the robot to the current pose
    def update_last_pose(self):
        self.last_x = self.x
        self.last_y = self.y
        self.last_theta = self.theta

    # set the initial pose of the robot
    def set_init_pose(self, x, y, theta):
        self.init_x = x
        self.init_y = y
        self.init_theta = theta

    # initialize robot pose, last pose, and vertex index
    def initialize_pose(self):
        self.update_pose(self.init_x, self.init_y, self.init_theta)
        self.update_last_pose()
        self.update_last_vert_index(reset=True)

    # update the last vertex index, reset to 0 if needed
    def update_last_vert_index(self, reset: bool = False):
        self.last_vert_index = 0 if reset else self.last_vert_index + 1

    # update the drive function
    def update_drive_function(self, drive_func: Callable[[float, float], None]):
        self.drive = drive_func

    # move the robot forward by a certain distance and transition to the next state
    def forward(self, len, nextstate):
        dlen = len - self.lin_dist_from_last_vert()
        vel = self.get_smoothstop_linvel(dlen)
        self.drive(vel, 0.0)
        if dlen < self.lin_delta:
            self.update_last_pose()
            self.update_last_vert_index()
            self.set_state(nextstate)

    # turn the robot by a certain angle in radians and transition to the next state
    def turn(self, rads, nextstate):
        drad = self.ang_dist(self.ang_dist_from_last_vert(), rads)
        angvel = self.get_smoothstop_angvel(drad)
        self.drive(0.0, angvel)
        if drad < self.ang_delta:
            self.update_last_pose()
            self.update_last_vert_index()
            self.set_state(nextstate)

    # stop the robot
    def stop(self):
        self.drive(0.0, 0.0)
        self.initialize_pose()
        self.set_state('idle')