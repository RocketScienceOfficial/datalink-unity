using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Threading;
using UnityEngine;

public class SerialCommunication : MonoBehaviour
{
    public static SerialCommunication Instance { get; private set; }

    public event EventHandler OnConnected;
    public event EventHandler OnDisconnected;
    public event EventHandler<SerialCommunicationOnReadEventArgs> OnRead;

    private readonly Queue<DataLinkFrame> _serialReadQueue = new();
    private readonly object _serialReadQueueLock = new();
    private bool _openPort;
    private readonly object _openPortLock = new();
    private bool _disconnectPort;
    private readonly object _disconnectPortLock = new();
    private SerialPort _currentSerialPort;
    private readonly object _serialPortLock = new();
    private Thread _serialOpenThread;
    private Thread _serialReadThread;


    private void Awake()
    {
        Instance = this;
    }

    private void Update()
    {
        lock (_openPortLock)
        {
            if (_openPort)
            {
                FinishConnect();

                _openPort = false;
            }
        }

        lock (_disconnectPortLock)
        {
            if (_disconnectPort)
            {
                FinishDisconnect();

                _disconnectPort = false;
            }
        }

        lock (_serialReadQueueLock)
        {
            if (_serialReadQueue.Count > 0)
            {
                OnRead?.Invoke(this, new SerialCommunicationOnReadEventArgs { Frame = _serialReadQueue.Dequeue() });
            }
        }
    }

    private void OnApplicationQuit()
    {
        Disconnect();
    }

    private void OnApplicationPause(bool pause)
    {
        if (pause)
        {
            Disconnect();
        }
    }


    public void Connect(string port)
    {
        _serialOpenThread = new Thread(() =>
        {
            lock (_serialPortLock)
            {
                try
                {
                    if (_currentSerialPort != null)
                    {
                        print("Serial port is already connected to: " + _currentSerialPort.PortName);

                        return;
                    }

                    print("Connecting to port: " + port + "...");

                    _currentSerialPort = new SerialPort()
                    {
                        PortName = port,
                        BaudRate = 115200,
                        Parity = Parity.None,
                        DataBits = 8,
                        StopBits = StopBits.One,
                        RtsEnable = true,
                        DtrEnable = true,
                        ReadTimeout = 1000,
                        WriteTimeout = 1000,
                    };

                    _currentSerialPort.Open();

                    lock (_openPortLock)
                    {
                        _openPort = true;
                    }

                    _serialReadThread = new Thread(SerialReadThread);
                    _serialReadThread.Start();
                }
                catch (Exception ex)
                {
                    print("Could not find serial port: " + ex);

                    Disconnect();
                }
            }
        });

        _serialOpenThread.Start();
    }

    private void FinishConnect()
    {
        OnConnected?.Invoke(this, EventArgs.Empty);

        print("COM Port Connected!");
    }

    public void Disconnect()
    {
        var closeThread = new Thread(() =>
        {
            lock (_serialPortLock)
            {
                if (_currentSerialPort == null)
                {
                    print("Couldn't disconnect since serial port isn't connected!");

                    return;
                }

                print("Begining disconnecting...");

                _currentSerialPort.Close();
                _currentSerialPort = null;

                lock (_disconnectPortLock)
                {
                    _disconnectPort = true;
                }
            }
        });

        closeThread.Start();
    }

    private void FinishDisconnect()
    {
        if (_serialReadThread != null)
        {
            print("Closing read thread...");

            _serialReadThread.Join();
            _serialReadThread = null;
        }

        OnDisconnected?.Invoke(this, EventArgs.Empty);

        print("COM Port Disconnected!");
    }

    public bool IsConnected()
    {
        return _currentSerialPort != null;
    }

    public string CurrentPortName()
    {
        return _currentSerialPort?.PortName;
    }


    private void SerialReadThread()
    {
        const int BUFFER_SIZE = 512;

        var buffer = new byte[BUFFER_SIZE];
        var len = 0;

        while (_currentSerialPort != null)
        {
            try
            {
                var b = (byte)_currentSerialPort.ReadByte();

                if (len >= BUFFER_SIZE)
                {
                    len = 0;
                }

                buffer[len++] = b;

                if (b == 0x00)
                {
                    print("Received " + len + " bytes from serial");

                    var decodedData = COBS.Decode(buffer, len);
                    var frame = DataLink.Deserialize(decodedData);

                    if (frame != null)
                    {
                        lock (_serialReadQueueLock)
                        {
                            _serialReadQueue.Enqueue(frame);
                        }
                    }
                    else
                    {
                        print("Couldn't deserialize frame!");
                    }

                    len = 0;
                }
            }
            catch (TimeoutException) { }
            catch (InvalidOperationException) { }
            catch (IOException)
            {
                print("There was an error during reading serial port. Aborting...");

                Disconnect();

                break;
            }
        }
    }

    public void SerialPortWrite(DataLinkFrame frame)
    {
        if (_currentSerialPort != null)
        {
            var bytes = DataLink.Serialize(frame);

            if (bytes == null)
            {
                print("Error while serialized frame!");

                return;
            }

            var encodedBuffer = COBS.Encode(bytes);

            _currentSerialPort.Write(encodedBuffer, 0, encodedBuffer.Length);

            print("Written to serial port " + encodedBuffer.Length + " bytes!");
        }
    }
}

public class SerialCommunicationOnReadEventArgs : EventArgs
{
    public DataLinkFrame Frame { get; set; }
}