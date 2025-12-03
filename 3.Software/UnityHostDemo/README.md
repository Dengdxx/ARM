# Unity Host Demo for Dummy Robot

This folder contains C# scripts to help you build a custom Unity-based host software (上位机) for the Dummy Robot.

## File Overview

*   **`RobotConnection.cs`**: Handles the low-level Serial Port communication (opening/closing ports, threading for reading, sending commands).
*   **`DummyRobotController.cs`**: Provides a high-level API to control the robot (e.g., `MoveJ`, `MoveL`, `EnableRobot`) and parses incoming messages.

## How to Use in Unity

1.  **Create a Unity Project**: Open Unity Hub and create a new 3D project.
2.  **Import Scripts**: Drag `RobotConnection.cs` and `DummyRobotController.cs` into your project's `Assets` folder (e.g., under `Assets/Scripts`).
3.  **Setup Configuration**:
    *   Go to `Edit -> Project Settings -> Player`.
    *   Under `Other Settings` -> `Configuration`, change **Api Compatibility Level** to **.NET 4.x** (SerialPort requires full .NET framework support).
4.  **Create Controller Object**:
    *   Create an Empty GameObject in your scene (name it "RobotManager").
    *   Add the `RobotConnection` component to it.
    *   Add the `DummyRobotController` component to it.
5.  **Configure Port**:
    *   In the Inspector for `RobotConnection`, set the **Port Name** to your robot's COM port (e.g., `COM3` on Windows, `/dev/ttyUSB0` on Linux).
    *   Set **Baud Rate** to `115200` (or match your firmware setting).
6.  **Create UI**:
    *   Add Buttons to your scene (Canvas).
    *   Use the `OnClick` event of buttons to call methods on the `DummyRobotController` (e.g., `ConnectToRobot`, `EnableRobot`, `MoveJ`).

## Example Usage (Scripting)

You can create your own script to drive the robot:

```csharp
using UnityEngine;
using DummyRobot.Control;

public class MyRobotApp : MonoBehaviour
{
    public DummyRobotController robot;

    void Start()
    {
        // Connect automatically
        robot.ConnectToRobot();

        // Enable motors
        robot.EnableRobot();
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            // Move to a specific joint configuration
            // Joint angles: 0, 0, 90, 0, 0, 0 | Speed: 20%
            robot.MoveJ(0, 0, 90, 0, 0, 0, 20);
        }
    }
}
```

## Protocol Details

The scripts implement the ASCII protocol defined in the firmware (`ascii_protocol.cpp`):

*   **`>j1,j2,j3,j4,j5,j6,speed`**: Move Joints.
*   **`@x,y,z,a,b,c,speed`**: Move Linear (Inverse Kinematics).
*   **`!START` / `!STOP` / `!DISABLE`**: Enable/Stop/Disable motors.
*   **`#GETJPOS`**: Request current joint angles.
