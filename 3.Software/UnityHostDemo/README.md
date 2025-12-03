# Unity Host Demo for Dummy Robot

This folder contains C# scripts to help you build a custom Unity-based host software (上位机) for the Dummy Robot.

## File Overview

*   **`RobotConnection.cs`**: Handles the low-level Serial Port communication (opening/closing ports, threading for reading, sending commands).
*   **`DummyRobotController.cs`**: Provides a high-level API to control the robot (e.g., `MoveJ`, `MoveL`, `EnableRobot`) and parses incoming messages. It also includes support for driving a 3D model visualization.

## Where are the Robot Models?

The original 3D models for the robot are located in the **`4.Model`** directory of this repository. They are provided as **STEP** files (e.g., `Dummy v164.step`).

Unity does **not** natively support importing STEP files.

## How to Import URDF Models into Unity

To visualize the robot in Unity using these scripts, follow these steps:

### 1. Convert STEP to URDF
Since the models are in STEP format, you need to convert them to URDF (Unified Robot Description Format). This typically involves:
1.  Opening the STEP files in a CAD software (SolidWorks, Fusion 360, Onshape).
2.  Ensuring coordinate systems and joint limits are defined.
3.  Using a URDF Exporter plugin for your CAD software to export a folder containing `.urdf` and `.stl` (or `.dae`) meshes.

### 2. Import into Unity
1.  Open your Unity Project.
2.  Install the **URDF Importer** package.
    *   Recommended: [Unity-Technologies/URDF-Importer](https://github.com/Unity-Technologies/URDF-Importer).
    *   Follow their installation guide (usually via Package Manager -> Add package from git URL).
3.  Drag and drop your generated `.urdf` file into the Unity Assets folder.
4.  Right-click the URDF file -> **Import Robot from URDF**.
5.  This will create a GameObject hierarchy in your scene with `ArticulationBody` components.

### 3. Connect to Controller Script
1.  Select your controller object (the one with `DummyRobotController.cs`).
2.  In the Inspector, verify the **Joint Transforms** array size is 6.
3.  Drag the 6 joint GameObjects from your imported robot hierarchy into these slots (from Base to End-Effector).
4.  **Important**:
    *   If you imported using the official URDF Importer, check the **Use Articulation Body** box in the `DummyRobotController` inspector. This will attempt to drive the `xDrive` targets of the joints.
    *   If you manually built the hierarchy using simple GameObjects, leave the box unchecked. You may need to adjust the **Rotation Axes** vectors (e.g., (0, 1, 0) for Y-axis) to match how your model was exported.

## How to Use the Demo

1.  **Create a Unity Project**: Open Unity Hub and create a new 3D project.
2.  **Import Scripts**: Drag `RobotConnection.cs` and `DummyRobotController.cs` into your project's `Assets` folder.
3.  **Setup Configuration**:
    *   Go to `Edit -> Project Settings -> Player`.
    *   Under `Other Settings` -> `Configuration`, change **Api Compatibility Level** to **.NET 4.x** (SerialPort requires full .NET framework support).
4.  **Create Controller Object**:
    *   Create an Empty GameObject in your scene (name it "RobotManager").
    *   Add the `RobotConnection` component to it.
    *   Add the `DummyRobotController` component to it.
5.  **Configure Port**:
    *   In the Inspector for `RobotConnection`, set the **Port Name** to your robot's COM port (e.g., `COM3` on Windows, `/dev/ttyUSB0` on Linux).
    *   Set **Baud Rate** to `115200`.
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
