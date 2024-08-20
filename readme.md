# Time-Controlled Dual Robot Simulator

## Required Functionalities
1. Textbox for user to input time in seconds after which the robots should start moving.
1. Checkbox to select simultaneous or sequential movement.
1. Checkbox to allow user to repeat the task every 2 minutes.
1. Button to start the task execution.
1. Robot ignores new movement commands until the current movement is complete.
1. Movements of the 2 robots are different, i.e. one draws a rectangle and the other draws a triangle.
1. Uses Turtlesim as the simulation environment.

## Project Components

### [√] RobUI, written in C#
1. Generalizable to control different robots in TurtleSim.
1. Decoupled from the ROS2 environment.
1. Communicates to a ROS2 node via a WebSocket connection.
1. Provides a cross-platform, modern and user-friendly GUI interface.

### [√] ROS2Socket, written in Python
1. Contains a WebSocket server to communicate with the RobUI.
1. Arguments configurable via a YAML file.
1. Receives messages from the RobUI and publishes them to its own output topic.
1. Subscribes to its own input topic and sends messages to the RobUI, if connected.
1. Periodically publishes the status of connection to its own status topic.

### [√] Turtlebot, written in Python
1. Can be run by the command line with arguments, or by instantiating the class.
1. On creation, spawns its turtle in the Turtlesim environment.
1. Subscribes to its own input node and control the turtle:
    - Movement Commands (ignored if the robot is already moving):
        - `start` starts the turtle's movement task
        - `startin:t` starts the turtle's movement task after `t` seconds.
        - `repeat:t` sets the robot to repeat the task every `t` seconds.
        - `edit:pt1,pt2,...` sets the goals to the points in the list, stops at the last point.
        - `editloop:pt1,pt2,...` sets the goals to the points in the list and loops back to the first point.
        - `home:x,y,theta` homes the turtle to the point and orientation.
    - Control Commands (can be executed at any time):
        - `pause` pauses the turtle's movement.
        - `resume` resumes the turtle's movement.
        - `stop` stops the turtle's movement and homes it.
        - `kill` kills the turtle and quits the node.
1. Publishes to its own output node with the turtle's state.

### Turtlehub, written in Python
1. Takes responsibility to start `turtlesim_node` if it is not running.
1. Takes responsibility to start `ros2socket` if it is not running.
1. Subscribes to ROS2Socket's output node and interpret the messages to control the Turtlebots.
1. Subscribes to the Turtlebots' state nodes and ROS2Socket's state node. Composite them with its own state and send them to ROS2Socket's input node.

## Compatibility
| ROS Version | Ubuntu Version | ROS2 Package Compatible* |
|-------------|-----------------|-------------|
| ROS2 Jazzy Jalisco | 24.04 LTS | Yes |

*Note that other combinations of ROS and Ubuntu versions may work, but have not been tested.

| OS Version | .NET SDK Version | UI Compatible** |
|------------|-------------------|---------------|
| Ubuntu 24.04 LTS | .NET 8.0 | Yes |
| Windows 11 24H2 | .NET 8.0 | Yes |

** Note that other combinations of OS and .NET versions may work, but have not been tested.

## Dependencies
- `ros-jazzy-turtlesim`
    - Can be installed using the following command:
        ```bash
        sudo apt update && sudo apt install ros-jazzy-turtlesim
        ```
- `dotnet-sdk-8.0`
    - Can be installed using the following command:
        ```bash
        sudo apt update && sudo apt install dotnet-sdk-8.0
        ```
- `rosdep` Dependencies
    - Can be installed using the following command from the root of the workspace:
        ```bash
        rosdep update && rosdep install -i --from-path src --rosdistro jazzy -y
        ```

## Usage
1. Depending on your ROS installation, source the appropriate ROS2 setup file in EVERY terminal you open, or add this line to your `.bashrc` file.
    ```bash
    source /opt/ros/jazzy/setup.bash
    ```
1. Build the package from the root of the workspace
    ```bash
    colcon build
    ```
1. Source the workspace overlay setup file.
    ```bash
    source install/setup.bash
    ```
1. Run the `turtlehub` node
    ```bash
    ros2 run robmove turtlehub --ros-args --params-file config/turtlehub.yaml
    ```
1. In another terminal, build and run the UI project in the `src/robui/robui` directory
    ```bash
    cd src/robui/robui
    dotnet run -c Release
    ```
1. The UI should open up. Click the `Manual` button if unsure of how to use the UI.

## UI Documentation
### Interface
| Dark Mode (on Windows 11 24H2) | Light Mode (on Ubuntu 24.04 LTS in WSL2) |
|-----------|------------|
| ![Dark Mode](img/dark_mode.png) | ![Light Mode](img/light_mode.png) |

### Manual
TODO

### Communication Protocol
The UI communicates via WebSocket to the ROS2Socket node. The messages are plain strings with the following format:
```
cmd:data
```
Where `cmd` is the command and `data` is the data associated with the command. The following commands are supported:
```
spwn:name,x,y,theta,loopback,goal1,goal2,...
kill:name
strt:t_after,t_repeat,is_repeat,is_simultaneous
```
Where each command `cmd` is defined as follows:
| Command | Description |
|---------|-------------|
| `spwn` | Spawns a new robot with the given parameters or changes the parameters of an existing robot. |
| `kill` | Kills the robot with the given name. |
| `strt` | Starts the task execution with the given parameters. |

For example, when the UI starts, 2 robots: `rectbot` and `trigbot` are spawned with the following commands:
```
spwn:rectbot,1.0,4.0,0.0,T,4.0,4.0,4.0,6.0,1.0,6.0
spwn:trigbot,6.0,3.0,0.0,T,9.0,7.0,7.0,6.0
```
If the user:
- has input `5` in the `Start After ... seconds` textbox and `2` in the `Repeat in ... minutes` textbox,
- has checked the `Simultaneous` checkbox and the `Repeat in` checkbox,
- clicks on the `Start Task Execution` button,

the following command will be sent to the ROS2Socket node (where `120` is the number of seconds in `2` minutes):
```
strt:5,120,T,T
```
which will then be interpreted by the ROS2Socket node to start the task execution accordingly.