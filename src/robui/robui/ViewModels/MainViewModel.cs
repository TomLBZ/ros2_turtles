using CommunityToolkit.Mvvm.ComponentModel;
using System.Collections.ObjectModel;

namespace robui.ViewModels;

public partial class MainViewModel : ObservableObject
{
    // strings
    [ObservableProperty]
    private string serverAddress = "localhost";
    [ObservableProperty]
    private string spawnBotName = "robot";
    // numeric values
    [ObservableProperty]
    private int serverPort = 8765;
    [ObservableProperty]
    private float spawnX = 0f;
    [ObservableProperty]
    private float spawnY = 0f;
    [ObservableProperty]
    private float taskStartDelaySeconds = 3f;
    [ObservableProperty]
    private float taskRepeatCycleMinutes = 2f;
    // booleans
    [ObservableProperty]
    private bool isTaskRepeatable = true;
    [ObservableProperty]
    private bool isTaskSimultaneous = true;
    [ObservableProperty]
    private bool serverStatus = false;
    [ObservableProperty]
    private bool isSpawnAllowed = false;
    [ObservableProperty]
    private bool isAdvancedMode = false;
    [ObservableProperty]
    private bool isTurtlesimOnline = false;
    [ObservableProperty]
    private bool isTaskRunning = false;
    [ObservableProperty]
    private bool isLoopPath = true;
    [ObservableProperty]
    private bool isVisualizeHome = false;
    // collections
    private ObservableCollection<RobotViewModel> robots;
    public ObservableCollection<RobotViewModel> Robots
    {
        get => robots;
        set => SetProperty(ref robots, value);
    }
    // dependent properties
    public string HomeCoordinates => $"{SpawnX:F}, {SpawnY:F}";

    public MainViewModel()
    {
        robots = [
            new RobotViewModel
            {
                Name = "rectbot",
                State = "idle",
                X = 1f,
                Y = 4f,
                Rotation = 0f,
                LinearV = 0f,
                AngularV = 0f
            },
            new RobotViewModel
            {
                Name = "trigbot",
                State = "idle",
                X = 6f,
                Y = 3f,
                Rotation = 0f,
                LinearV = 0f,
                AngularV = 0f
            }
        ];
    }
}