using CommunityToolkit.Mvvm.ComponentModel;

namespace robui.ViewModels;

public partial class RobotViewModel : ObservableObject
{
    [ObservableProperty]
    private string name = "robot";
    [ObservableProperty]
    private string state = "idle";
    [ObservableProperty]
    private float x = 0f;
    [ObservableProperty]
    private float y = 0f;
    [ObservableProperty]
    private float rotation = 0f;
    [ObservableProperty]
    private float linearV = 0f;
    [ObservableProperty]
    private float angularV = 0f;

    [ObservableProperty]
    private bool isEditEnabled = false;
    [ObservableProperty]
    private bool isDeleteEnabled = false;
}
