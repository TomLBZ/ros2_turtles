using CommunityToolkit.Mvvm.ComponentModel;
using robui.Structs;

namespace robui.ViewModels;

public partial class MainViewModel : ObservableObject
{

    [ObservableProperty]
    private ServerInfo server = new() { Address = "localhost", Port = "8765", Status = false };

    [ObservableProperty]
    private string title = "RobUI";

    [ObservableProperty]
    private bool isSpawnAllowed = false;

    [ObservableProperty]
    private bool isAdvancedMode = false;

    [ObservableProperty]
    private RobotInfo rectbot = new()
    {
        Name = "rectbot",
        State = "idle",
        Position = new(1f, 3f),
        Rotation = 0f,
        LinearV = 0f,
        AngularV = 0f
    };

    [ObservableProperty]
    private RobotInfo trigbot = new()
    {
        Name = "trigbot",
        State = "idle",
        Position = new(6f, 2f),
        Rotation = 0f,
        LinearV = 0f,
        AngularV = 0f
    };

    public MainViewModel()
    {

    }
}