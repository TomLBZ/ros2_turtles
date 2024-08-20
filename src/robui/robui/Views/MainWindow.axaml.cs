using Avalonia.Controls;
using Avalonia.Markup.Xaml;
using robui.ViewModels;
using robui.Networking;
using System.Threading.Tasks;
using System;

namespace robui.Views
{
    public partial class MainWindow : Window
    {
        private readonly WsClient wsClient;
        private readonly MainViewModel mvm = new();
        private readonly float[] rectbotGoals = [4.0f, 4.0f, 4.0f, 6.0f, 1.0f, 6.0f];
        private readonly float[] trigbotGoals = [9.0f, 7.0f, 7.0f, 6.0f];
        private bool isDefaultBotsAdded = false;
        public MainWindow()
        {
            InitializeComponent();
            DataContext = mvm;
            wsClient = new WsClient(mvm.ServerAddress, mvm.ServerPort);
            wsClient.MessageReceived += WsClientMessageReceived;
            wsClient.ConnectionChanged += WsClientConnectionChanged;
            wsClient.Connect();
        }

        private void InitializeComponent()
        {
            AvaloniaXamlLoader.Load(this);
        }
        private string BoolToString(bool value)
        {
            return value ? "T" : "F";
        }
        private string RobotSpawnCommand(RobotViewModel rvm, float[] goals)
        {
            // spwn:name,x,y,theta,loopback,goal1,goal2,...
            return $"spwn:{rvm.Name},{rvm.X},{rvm.Y},{rvm.Rotation},{BoolToString(mvm.IsLoopPath)}," + string.Join(',', goals);

        }
        private string TaskStartCommand()
        {
            // strt:start_delay,repeat_cycle,repeatable,simultaneous
            return $"strt:{Math.Abs(mvm.TaskStartDelaySeconds)},{Math.Abs(mvm.TaskRepeatCycleMinutes) * 60},{BoolToString(mvm.IsTaskRepeatable)},{BoolToString(mvm.IsTaskSimultaneous)}";
        }
        private void WsClientConnectionChanged(object? sender, ConnectionState state)
        {
            mvm.ServerStatus = state == ConnectionState.Connected;
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
        }
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
        private void BtnStartTask_Click(object? sender, Avalonia.Interactivity.RoutedEventArgs e)
        {
            _ = Task.Run(async () =>
            {
                await wsClient.SendMessageAsync(TaskStartCommand());
            });
        }
    }
}