using Avalonia.Controls;
using Avalonia.Markup.Xaml;
using robui.ViewModels;
using robui.Networking;
using System.Threading.Tasks;
using System;

namespace robui.Views
{
    /// <summary>
    /// The MainWindow class represents the main window of the application.
    /// </summary>
    public partial class MainWindow : Window
    {
        // private fields for the states of the application
        private readonly WsClient wsClient;
        private readonly MainViewModel mvm = new();
        private readonly float[] rectbotGoals = [4.0f, 4.0f, 4.0f, 6.0f, 1.0f, 6.0f];
        private readonly float[] trigbotGoals = [9.0f, 7.0f, 7.0f, 6.0f];
        private bool isDefaultBotsAdded = false;

        /// <summary>
        /// The constructor <c>MainWindow</c> creates a new main window.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
            DataContext = mvm;
            wsClient = new WsClient(mvm.ServerAddress, mvm.ServerPort);
            wsClient.MessageReceived += WsClientMessageReceived;
            wsClient.ConnectionChanged += WsClientConnectionChanged;
            wsClient.Connect();
        }

        /// <summary>
        /// The method <c>InitializeComponent</c> initializes the main window.
        /// </summary>
        private void InitializeComponent()
        {
            AvaloniaXamlLoader.Load(this);
        }

        /// <summary>
        /// The method <c>BoolToString</c> converts a boolean value to a string.
        /// </summary>
        /// <param name="value">the boolean value to be converted</param>
        /// <returns></returns>
        private string BoolToString(bool value)
        {
            return value ? "T" : "F";
        }

        /// <summary>
        /// The method <c>RobotSpawnCommand</c> creates a command to spawn a robot.
        /// </summary>
        /// <param name="rvm">the RobotViewModel specified</param>
        /// <param name="goals">the goal points for the spawned robot</param>
        /// <returns></returns>
        private string RobotSpawnCommand(RobotViewModel rvm, float[] goals)
        {
            // spwn:name,x,y,theta,loopback,goal1,goal2,...
            return $"spwn:{rvm.Name},{rvm.X},{rvm.Y},{rvm.Rotation},{BoolToString(mvm.IsLoopPath)}," + string.Join(',', goals);

        }

        /// <summary>
        /// The method <c>TaskStartCommand</c> creates a command to start a task.
        /// </summary>
        /// <returns></returns>
        private string TaskStartCommand()
        {
            // strt:start_delay,repeat_cycle,repeatable,simultaneous
            return $"strt:{Math.Abs(mvm.TaskStartDelaySeconds)},{Math.Abs(mvm.TaskRepeatCycleMinutes) * 60},{BoolToString(mvm.IsTaskRepeatable)},{BoolToString(mvm.IsTaskSimultaneous)}";
        }

        /// <summary>
        /// The method <c>WsClientConnectionChanged</c> handles the connection state change of the WebSocket client.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="state">representing the connection status of the <c>WsClient</c></param>
        private void WsClientConnectionChanged(object? sender, ConnectionState state)
        {
            mvm.ServerStatus = state == ConnectionState.Connected;
            if (!mvm.ServerStatus) mvm.IsTurtlesimOnline = false;
            if (state == ConnectionState.Connected && !isDefaultBotsAdded)
            {
                isDefaultBotsAdded = true;
                _ = Task.Run(async () =>
                {
                    await wsClient.SendMessageAsync(RobotSpawnCommand(mvm.Robots[0], rectbotGoals));
                    await Task.Delay(500);
                    await wsClient.SendMessageAsync(RobotSpawnCommand(mvm.Robots[1], trigbotGoals));
                    await Task.Delay(500);
                });
            }
            else if (state == ConnectionState.Disconnected)
            {
                wsClient.Disconnect(); // be sure to disconnect the client, trigerring the reconnection timer
                isDefaultBotsAdded = false;
            }
        }

        /// <summary>
        /// The method <c>WsClientMessageReceived</c> handles the message received event of the WebSocket client.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="message">the message received</param>
        private void WsClientMessageReceived(object? sender, string message)
        {
            // soc:T|sim:T|tsk:F|rectbot:idle,1.0,4.0,0.0,0.0,0.0|trigbot:idle,6.0,3.0,0.0,0.0,0.0
            string[] tokens = message.Split('|');
            foreach (string token in tokens)
            {
                string[] kv = token.Split(':');
                if (kv.Length != 2)
                {
                    continue;
                }
                string key = kv[0];
                string value = kv[1];
                switch (key)
                {
                    case "soc":
                        // do nothing
                        break;
                    case "sim":
                        mvm.IsTurtlesimOnline = value == "T";
                        break;
                    case "tsk":
                        mvm.IsTaskRunning = value == "T";
                        break;
                    default:
                        string[] robtokens = value.Split(',');
                        if (robtokens.Length != 6)
                        {
                            continue;
                        }
                        // There may be a robot in mvm.Robots with the same name, if so, update its properties
                        bool isRobotFound = false;
                        foreach (RobotViewModel robot in mvm.Robots)
                        {
                            if (robot.Name == key)
                            {
                                robot.State = robtokens[0];
                                robot.X = float.Parse(robtokens[1]);
                                robot.Y = float.Parse(robtokens[2]);
                                robot.Rotation = float.Parse(robtokens[3]);
                                robot.LinearV = float.Parse(robtokens[4]);
                                robot.AngularV = float.Parse(robtokens[5]);
                                isRobotFound = true;
                            }
                        }
                        // if not, add a new robot to mvm.Robots
                        if (!isRobotFound)
                        {
                            mvm.Robots.Add(new RobotViewModel
                            {
                                Name = key,
                                State = robtokens[0],
                                X = float.Parse(robtokens[1]),
                                Y = float.Parse(robtokens[2]),
                                Rotation = float.Parse(robtokens[3]),
                                LinearV = float.Parse(robtokens[4]),
                                AngularV = float.Parse(robtokens[5])
                            });
                        }
                        break;
                }
            }
        }

        /// <summary>
        /// The method <c>BtnStartTask_Click</c> handles the click event of the "Start Task Execution" button.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnStartTask_Click(object? sender, Avalonia.Interactivity.RoutedEventArgs e)
        {
            _ = Task.Run(async () =>
            {
                await wsClient.SendMessageAsync(TaskStartCommand());
            });
        }

        /// <summary>
        /// The method <c>BtnPopupOK_Click</c> handles the click event of the "OK" button in the popup window.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnPopupOK_Click(object? sender, Avalonia.Interactivity.RoutedEventArgs e)
        {
            mvm.IsPopupVisible = false;
        }

        /// <summary>
        /// The method <c>BtnAbout_Click</c> handles the click event of the "About" button.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnAbout_Click(object? sender, Avalonia.Interactivity.RoutedEventArgs e)
        {
            mvm.PopupWidth = 400;
            mvm.PopupHeight = 300;
            mvm.IsPopupVisible = true;
            mvm.PopupMessage = @"RobUI - A Robotics User Interface (ver. 0.0.1)
                        by LBZ
robui is a simple robotics user interface that allows you 
to control robots in a turtlesim environment.
It is built using Avalonia, a cross-platform XAML-based 
UI framework for .NET.
The project is for demonstration purposes only and is 
not intended for production use.
Interview requirements are all satisfied, 
but some advanced functions are left unfinished.";
        }

        /// <summary>
        /// The method <c>BtnManual_Click</c> handles the click event of the "Manual" button.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnManual_Click(object? sender, Avalonia.Interactivity.RoutedEventArgs e)
        {
            mvm.PopupWidth = 700;
            mvm.PopupHeight = 400;
            mvm.IsPopupVisible = true;
            mvm.PopupMessage = @" RobUI - User Manual

################## [ Simple Manual For the Interview Project: ] ##################

0. Start the server by running it in the terminal BEFORE starting the UI.
1. Observe on the bottom of the window, the status bar shows the connection status of Turtlehub.
2. Wait until the text shows ""Turtlehub: Connected  Turtlesim: Online"".
3. Two robots, ""rectbot"" and ""trigbot"", will be spawned automatically. You see them on the list.
4. Right below the list, you can see the task configuration section. Change the values if you want.
5. Click the ""Start Task Execution"" button to start the task, and observe how the robots move.
6. While robots are moving, you can see their states updated in real-time on the list.
7. If the ""Movement Tasks"" label shows ""Running"", subsequent movement instructions will be ignored.
The ignore logic is on the server, not the UI.
8. You can overwrite the robots' movement instructions by changing the task configuration and 
clicking the button again, as long as the task is not running.

################## [ Detailed Manual of Controls: ] ##################

1. Server Section
The program auto-connects to the server at startup using the textbox values:
- Server Address: The IP address of the turtlehub server.
- Server Port: The port number of the turtlehub server.
Unless the server is running on a different IP address or port, there is no need to change these values.

2. Robot Edit Section
This section is grayed-out by default, because the UI is locked in ""Simple Mode"". It is NOT POSSIBLE
to toggle the ""Simple Mode [LOCKED]"" switch to ""Advanced Mode"" in this version because it is a 
UI DEMO program only.
However, the following fields are intended for robot spawn/edit configuration:
- Name: The name of the robot to be spawned.
- Home: The initial x,y-coordinates of the robot.
- Button ""Set Home"" (NOT IMPLEMENTED): Enables the user to click on the Visualization Panel to set
the home coordinates.
- Checkbox ""See Home"" (NOT IMPLEMENTED): Enables displaying the current ""Home"" coordinates on
the Visualization Panel.
- Button ""Set Path"" (NOT IMPLEMENTED): Enables the user to click on the Visulization Panel to set the
path coordinates as a series of goal points.
- Checkbox ""Loop Path"": If checked, the robot will loop back to the first waypoint after reaching the
last goal point. (LOCKED to TRUE in this version.)
- Button ""Apply/Spawn"" (NOT IMPLEMENTED): Spawns the robot with the specified name, home 
coordinates, and path coordinates. If a robot with the same name already exists, it will be updated.
- Button ""Cancel"" (NOT IMPLEMENTED): Resets the fields to their default values.

3. Robot List
This section displays the information of the robot, updated in real-time if the robot is moving and the
server is sending the robot's state information.
The information includes:
- The bold text: The robot's name.
- ?: The robot's current state.
- p: The robot's current x,y-coordinates.
- ¦È: The robot's current orientation.
- v: The robot's current linear velocity.
- ¦Ø: The robot's current angular velocity.
For each robot, there are two buttons:
- Edit (NOT IMPLEMENTED): Edits the robot's name, home coordinates, and path coordinates in the 
Robot Edit Section if in the ""Advanced Mode"". If in the ""Simple Mode"", the button is disabled.
- Delete (NOT IMPLEMENTED): Deletes the robot from the list if in the ""Advanced Mode"". 
If in the ""Simple Mode"", the button is disabled.
At the bottom of the list, there is a Checkbox ""Allow New Robot Spawn"". Only if checked can the 
user spawn a new robot in the Robot Edit Section.

4. Task Configuration (TASK REQUIREMENTS)
- Start after ___ seconds: The delay in seconds before the task starts, default to 3 seconds.
- Repeat in ___ minutes: The cycle time in minutes for the task to repeat, default to 2 minutes.
- Checkbox ""[] Repeat"": If checked, the task will repeat.
- Checkbox ""Simultaneous"": If checked, all robots in the list will run simultaneously. 
If unchecked, they will run one after another.
- Button ""Start Task"": Starts the task with the specified configuration by sending the information 
as a plain string to the server. The server WILL IGNOREthe message if the task is already running.

5. Visualization Panel (NOT IMPLEMENTED)
This section is for visualization purposes only. It displays the robot's current position, home 
position and path coordinates during editing.

6. Status Bar
This section contains the following information:
- Turtlehub Status: The connection status of the server.
- Turtlesim Status: The availability of the turtlesim node.
- Task Status: The status of the task (running or not).
";
        }
    }
}