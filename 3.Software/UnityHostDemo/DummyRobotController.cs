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

        [Header("Visualization")]
        [Tooltip("Drag the 6 joint Transforms from your imported URDF model here.")]
        public Transform[] jointTransforms = new Transform[6];

        [Tooltip("Define the rotation axis for each joint (e.g., (0,0,1) for Z-axis).")]
        public Vector3[] rotationAxes = new Vector3[6] {
            Vector3.up, Vector3.up, Vector3.up, Vector3.up, Vector3.up, Vector3.up
        };

        [Tooltip("If using Unity's URDF Importer with ArticulationBodies, set this to true.")]
        public bool useArticulationBody = false;

        // Callback events for UI update
        public event Action<float[]> OnJointsUpdated;
        public event Action<float[]> OnPoseUpdated;

        private void Awake()
        {
            _connection = GetComponent<Communication.RobotConnection>();
            _connection.OnMessageReceived += HandleMessage;
        }

        private void Update()
        {
            UpdateVisualization();
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
                        // We assume joints update by default as it's most common for synchronization.

                        currentJoints = values;
                        OnJointsUpdated?.Invoke(values);
                    }
                }
            }
        }

        private void UpdateVisualization()
        {
            for (int i = 0; i < 6; i++)
            {
                if (i >= jointTransforms.Length || jointTransforms[i] == null) continue;

                if (useArticulationBody)
                {
                    // If using Unity URDF Importer, the joints are usually ArticulationBodies.
                    // Accessing ArticulationBody joint positions is different.
                    var body = jointTransforms[i].GetComponent<ArticulationBody>();
                    if (body != null)
                    {
                        // ArticulationBody uses radians usually, but XDrive targets usually target degrees.
                        // Let's assume input degrees and convert if needed.
                        // Setting joint positions via ArticulationBody is complex (drives vs direct set).
                        // For visualization (slave mode), we can try forcing positions via teleport if needed,
                        // or better, drive targets if simulation is running.

                        // Simple approach: Set drive target
                        var drive = body.xDrive;
                        drive.target = currentJoints[i];
                        body.xDrive = drive;
                    }
                }
                else
                {
                    // Standard Transform rotation
                    // We assume local rotation relative to parent
                    // Reset rotation to identity and apply axis rotation
                    // Note: This overrides other rotations!
                    // Better approach: Maintain initial rotation and add offset?
                    // For typical robot arm visualization, local rotation around one axis is standard.

                    jointTransforms[i].localRotation = Quaternion.AngleAxis(currentJoints[i], rotationAxes[i]);
                }
            }
        }
    }
}
