# Time-Controlled Dual Robot Simulator

## Functionalities
1. Textbox for user to input time in seconds after which the robots should start moving.
1. Checkbox to select simultaneous or sequential movement.
1. Checkbox to allow user to repeat the task every 2 minutes.
1. Button to start the simulation.

## Compatibility
| ROS Version | Ubuntu Version | Compatible |
|-------------|-----------------|-------------|
| Jazzy Jalisco | 24.04 LTS | Yes |

## Dependencies
- `ros-jazzy-turtlesim`

## Usage
1. Update rosdep database
    ```bash
    rosdep update
    ```
1. Install dependencies
    ```bash
    rosdep install -i --from-path src --rosdistro jazzy -y
    ```
1. Build the package
    ```bash
    colcon build
    ```
1. Source the workspace
    ```bash
    source install/setup.bash
    ```
1. Install .NET SDK
    ```bash
    sudo apt update && sudo apt install dotnet-sdk-8.0
    ```
1. Build the csharp project
    ```bash
    dotnet build -c Release
    ```