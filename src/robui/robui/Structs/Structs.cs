using System.Drawing;

namespace robui.Structs;

public struct ServerInfo
{
    public string Address { get; set; }
    public string Port { get; set; }
    public bool Status { get; set; }
}

public struct RobotInfo
{
    public RobotInfo() { }

    public string Name { get; set; } = "robot";
    public string State { get; set; } = "idle";
    public PointF Position { get; set; } = new(0f, 0f);
    public float Rotation { get; set; } = 0f;
    public float LinearV { get; set; } = 0f;
    public float AngularV { get; set; } = 0f;
    public readonly string PositionString => $"{Position.X:F}, {Position.Y:F}";
    public readonly string RotationString => $"{Rotation:F} rad";
    public readonly string LinearVString => $"{LinearV:F} m/s";
    public readonly string AngularVString => $"{AngularV:F} rad/s";

}