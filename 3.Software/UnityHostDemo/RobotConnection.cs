using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using UnityEngine;

namespace DummyRobot.Communication
{
    public class RobotConnection : MonoBehaviour
    {
        [Header("Connection Settings")]
        public string portName = "COM3";
        public int baudRate = 115200;
        public bool connectOnStart = false;

        private SerialPort _serialPort;
        private Thread _readThread;
        private bool _keepReading = false;

        // Queue for thread-safe message passing to Main Thread
        private readonly Queue<string> _incomingMessages = new Queue<string>();
        private readonly object _queueLock = new object();

        // Event for received messages
        public event Action<string> OnMessageReceived;

        private void Start()
        {
            if (connectOnStart)
            {
                Connect();
            }
        }

        private void Update()
        {
            ProcessIncomingMessages();
        }

        private void OnDestroy()
        {
            Disconnect();
        }

        public void Connect()
        {
            if (_serialPort != null && _serialPort.IsOpen)
                return;

            try
            {
                _serialPort = new SerialPort(portName, baudRate);
                _serialPort.ReadTimeout = 500;
                _serialPort.WriteTimeout = 500;
                _serialPort.Open();

                _keepReading = true;
                _readThread = new Thread(ReadPort);
                _readThread.Start();

                Debug.Log($"Connected to {portName} at {baudRate}");
            }
            catch (Exception e)
            {
                Debug.LogError($"Connection failed: {e.Message}");
            }
        }

        public void Disconnect()
        {
            _keepReading = false;

            if (_readThread != null && _readThread.IsAlive)
            {
                _readThread.Join();
            }

            if (_serialPort != null && _serialPort.IsOpen)
            {
                _serialPort.Close();
                _serialPort.Dispose();
            }

            _serialPort = null;
            Debug.Log("Disconnected");
        }

        public void SendCommand(string cmd)
        {
            if (_serialPort != null && _serialPort.IsOpen)
            {
                try
                {
                    _serialPort.WriteLine(cmd);
                    // Debug.Log($"Sent: {cmd}");
                }
                catch (Exception e)
                {
                    Debug.LogError($"Send failed: {e.Message}");
                }
            }
            else
            {
                Debug.LogWarning("Serial port not open.");
            }
        }

        private void ReadPort()
        {
            while (_keepReading && _serialPort != null && _serialPort.IsOpen)
            {
                try
                {
                    string message = _serialPort.ReadLine();
                    if (!string.IsNullOrEmpty(message))
                    {
                        lock (_queueLock)
                        {
                            _incomingMessages.Enqueue(message);
                        }
                    }
                }
                catch (TimeoutException) { }
                catch (Exception e)
                {
                    if (_keepReading) // Only log if we didn't intentionally close
                        Debug.LogError($"Read error: {e.Message}");
                }
            }
        }

        private void ProcessIncomingMessages()
        {
            lock (_queueLock)
            {
                while (_incomingMessages.Count > 0)
                {
                    string msg = _incomingMessages.Dequeue();
                    OnMessageReceived?.Invoke(msg);
                }
            }
        }
    }
}
