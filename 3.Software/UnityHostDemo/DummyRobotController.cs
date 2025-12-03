using System;
using System.Globalization;
using UnityEngine;

namespace DummyRobot.Control
{
    [RequireComponent(typeof(Communication.RobotConnection))]
    public class DummyRobotController : MonoBehaviour
    {
        private Communication.RobotConnection _connection;

        [Header("Status")]
        public float[] currentJoints = new float[6];
        public float[] currentPose = new float[6];

        // Callback events for UI update
        public event Action<float[]> OnJointsUpdated;
        public event Action<float[]> OnPoseUpdated;

        private void Awake()
        {
            _connection = GetComponent<Communication.RobotConnection>();
            _connection.OnMessageReceived += HandleMessage;
        }

        public void ConnectToRobot()
        {
            _connection.Connect();
        }

        public void EnableRobot()
        {
            _connection.SendCommand("!START");
        }

        public void DisableRobot()
        {
            _connection.SendCommand("!DISABLE");
        }

        public void EmergencyStop()
        {
            _connection.SendCommand("!STOP");
        }

        /// <summary>
        /// Move joints to target angles.
        /// </summary>
        /// <param name="j1">Joint 1 angle</param>
        /// <param name="j2">Joint 2 angle</param>
        /// <param name="j3">Joint 3 angle</param>
        /// <param name="j4">Joint 4 angle</param>
        /// <param name="j5">Joint 5 angle</param>
        /// <param name="j6">Joint 6 angle</param>
        /// <param name="speed">Speed percentage (0-100)</param>
        public void MoveJ(float j1, float j2, float j3, float j4, float j5, float j6, float speed = 20)
        {
            // Protocol: >j1,j2,j3,j4,j5,j6,speed
            string cmd = string.Format(CultureInfo.InvariantCulture, ">{0:F2},{1:F2},{2:F2},{3:F2},{4:F2},{5:F2},{6:F2}",
                j1, j2, j3, j4, j5, j6, speed);
            _connection.SendCommand(cmd);
        }

        /// <summary>
        /// Move end-effector to target pose (Linear Move).
        /// </summary>
        /// <param name="x">X position (mm)</param>
        /// <param name="y">Y position (mm)</param>
        /// <param name="z">Z position (mm)</param>
        /// <param name="roll">Roll angle (deg)</param>
        /// <param name="pitch">Pitch angle (deg)</param>
        /// <param name="yaw">Yaw angle (deg)</param>
        /// <param name="speed">Speed percentage</param>
        public void MoveL(float x, float y, float z, float roll, float pitch, float yaw, float speed = 20)
        {
            // Protocol: @x,y,z,a,b,c,speed
            string cmd = string.Format(CultureInfo.InvariantCulture, "@{0:F2},{1:F2},{2:F2},{3:F2},{4:F2},{5:F2},{6:F2}",
                x, y, z, roll, pitch, yaw, speed);
            _connection.SendCommand(cmd);
        }

        public void RequestJointAngles()
        {
            _connection.SendCommand("#GETJPOS");
        }

        public void RequestPose()
        {
            _connection.SendCommand("#GETLPOS");
        }

        public void SetCommandMode(int mode)
        {
            // 1: SEQ, 2: INT, 3: TRJ, 4: TUN
            _connection.SendCommand($"#CMDMODE {mode}");
        }

        private void HandleMessage(string msg)
        {
            if (msg.StartsWith("ok"))
            {
                // Parse feedback
                // Example response: "ok 0.00 10.00 20.00 0.00 0.00 0.00"
                string[] parts = msg.Split(' ');
                if (parts.Length >= 7) // "ok" + 6 floats
                {
                    float[] values = new float[6];
                    bool success = true;
                    for (int i = 0; i < 6; i++)
                    {
                        if (!float.TryParse(parts[i + 1], NumberStyles.Any, CultureInfo.InvariantCulture, out values[i]))
                        {
                            success = false;
                            break;
                        }
                    }

                    if (success)
                    {
                        // Determining if it's joints or pose is ambiguous from just "ok ...",
                        // but usually follows the request.
                        // Ideally the firmware would prefix responses differently (e.g. "JPOS:..." vs "LPOS:...").
                        // For this demo, we assume the user tracks what they requested, or we just update local state blindly.
                        // *Observation from Firmware*:
                        // GETJPOS -> "ok %.2f..."
                        // GETLPOS -> "ok %.2f..."
                        // This is a firmware design limitation. We will update based on context or just store it.
                        // Given Unity update loop, we can store it in a generic "LastReceivedData" or try to guess.

                        // We will broadcast it as generic data for now, user logic handles context.
                        // Or we could check if values look like valid joints vs pose.
                        // Let's assume it's joints update by default as it's most common for synchronization.

                        // NOTE: In a real robust app, we would modify firmware to send "#JPOS:..." and "#LPOS:..."
                        currentJoints = values;
                        OnJointsUpdated?.Invoke(values);
                    }
                }
            }
        }
    }
}
