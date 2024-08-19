using Avalonia;
using Avalonia.Controls.Primitives;
using robui.Structs;
using System.Drawing;

namespace robui.Styles;

public class RobotItem : TemplatedControl
{
    /// <summary>
    /// Robot StyledProperty definition
    /// </summary>
    public static readonly StyledProperty<RobotInfo> RobotProperty =
        AvaloniaProperty.Register<RobotItem, RobotInfo>(nameof(Robot), new RobotInfo()
        {
            Name = "robot",
            State = "idle",
            Position = new PointF(0f, 0f),
            Rotation = 0f,
            LinearV = 0f,
            AngularV = 0f
        });

    /// <summary>
    /// Gets or sets the Robot property. This StyledProperty 
    /// indicates the information of the robot.
    /// </summary>
    public RobotInfo Robot
    {
        get => GetValue(RobotProperty);
        set => SetValue(RobotProperty, value);
    }

    #region Buttons
    /// <summary>
    /// LBtnText StyledProperty definition
    /// </summary>
    public static readonly StyledProperty<string> LBtnTextProperty =
        AvaloniaProperty.Register<RobotItem, string>(nameof(LBtnText), "Edit");

    /// <summary>
    /// Gets or sets the LBtnText property. This StyledProperty 
    /// indicates the text on the left button under the large text.
    /// </summary>
    public string LBtnText
    {
        get => GetValue(LBtnTextProperty);
        set => SetValue(LBtnTextProperty, value);
    }

    /// <summary>
    /// IsLBtnEnabled StyledProperty definition
    /// </summary>
    public static readonly StyledProperty<bool> IsLBtnEnabledProperty =
        AvaloniaProperty.Register<RobotItem, bool>(nameof(IsLBtnEnabled), false);

    /// <summary>
    /// Gets or sets the IsLBtnEnabled property. This StyledProperty
    /// indicates whether the left button is enabled.
    /// </summary>
    public bool IsLBtnEnabled
    {
        get => GetValue(IsLBtnEnabledProperty);
        set => SetValue(IsLBtnEnabledProperty, value);
    }

    /// <summary>
    /// RBtnText StyledProperty definition
    /// </summary>
    public static readonly StyledProperty<string> RBtnTextProperty =
        AvaloniaProperty.Register<RobotItem, string>(nameof(RBtnText), "Delete");

    /// <summary>
    /// Gets or sets the RBtnText property. This StyledProperty 
    /// indicates the text on the right button under the large text.
    /// </summary>
    public string RBtnText
    {
        get => GetValue(RBtnTextProperty);
        set => SetValue(RBtnTextProperty, value);
    }

    /// <summary>
    /// IsRBtnEnabled StyledProperty definition
    /// </summary>
    public static readonly StyledProperty<bool> IsRBtnEnabledProperty =
        AvaloniaProperty.Register<RobotItem, bool>(nameof(IsRBtnEnabled), false);

    /// <summary>
    /// Gets or sets the IsRBtnEnabled property. This StyledProperty
    /// indicates whether the right button is enabled.
    /// </summary>
    public bool IsRBtnEnabled
    {
        get => GetValue(IsRBtnEnabledProperty);
        set => SetValue(IsRBtnEnabledProperty, value);
    }
    #endregion

}