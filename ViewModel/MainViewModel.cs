using System;
using System.ComponentModel;
using System.Net.Sockets;
using System.IO;
using System.Text;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using System.Windows.Input;
using System.Windows.Threading;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Windows;


namespace UniversalRobotWpf
{
    public class MainViewModel : BaseViewModel
    {
        private string _robotIp = "192.168.56.101";
        private string _connectionStatus = "Disconnected";
        private string _poseData = "No data";
        private string _jointData = "No data";
        private string _logMessages = "";
        private bool _isConnected = false;
        private URRClient _client;
        private CancellationTokenSource _cancellationTokenSource;

        public MainViewModel()
        {
            ConnectCommand = new RelayCommand(ConnectToRobot, () => !IsConnected);
            DisconnectCommand = new RelayCommand(DisconnectFromRobot, () => IsConnected);
        }

        public string RobotIp
        {
            get => _robotIp;
            set => SetProperty(ref _robotIp, value);
        }

        public string ConnectionStatus
        {
            get => _connectionStatus;
            set
            {
                if (SetProperty(ref _connectionStatus, value))
                {
                    // Update IsConnected based on status
                    IsConnected = (value == "Connected");
                }
            }
        }

        public bool IsConnected
        {
            get => _isConnected;
            set
            {
                if (SetProperty(ref _isConnected, value))
                {
                    // Notify command state changes
                    ConnectCommand.RaiseCanExecuteChanged();
                    DisconnectCommand.RaiseCanExecuteChanged();
                }
            }
        }

        public string PoseData
        {
            get => _poseData;
            set => SetProperty(ref _poseData, value);
        }

        public string JointData
        {
            get => _jointData;
            set => SetProperty(ref _jointData, value);
        }

        public string LogMessages
        {
            get => _logMessages;
            set => SetProperty(ref _logMessages, value);
        }
        public RelayCommand DisconnectCommand { get; }
        public RelayCommand ConnectCommand { get; }

        private async void ConnectToRobot()
        {
            _cancellationTokenSource = new CancellationTokenSource();
            _client = new URRClient(RobotIp);

            try
            {
                // --- STEP 1: SETUP (on UI thread) ---
                ConnectionStatus = "Connecting...";
                await _client.ConnectAsync();

                if (!await _client.NegotiateProtocolVersionAsync())
                {
                    throw new Exception("Failed to negotiate protocol version.");
                }

                var variables = new[] { "actual_TCP_pose", "actual_q" };
                await _client.SendOutputSetupAsync(variables);

                var setupResponse = await _client.ReceivePackageAsync();
                if (setupResponse == null || !RTDEDataParser.ParseOutputSetupResponse(setupResponse))
                {
                    throw new Exception("Failed to setup RTDE output recipe.");
                }

                await _client.SendStartAsync();
                ConnectionStatus = "Connected";
                AddLogMessage("Connection successful. Starting data stream...");

                // --- STEP 2: START BACKGROUND DATA RECEIVER ---
                Task.Run(() => DataReceiverLoop(variables, _cancellationTokenSource.Token));
            }
            catch (Exception ex)
            {
                ConnectionStatus = "Connection Failed";
                AddLogMessage($"Error: {ex.Message}");
                _client?.Disconnect();
            }
        }
        private void DisconnectFromRobot()
        {
            _cancellationTokenSource?.Cancel();
            _client?.Disconnect();
            ConnectionStatus = "Disconnected";
            AddLogMessage("Disconnected from robot.");
        }

        private async void DataReceiverLoop(string[] variables, CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                try
                {
                    var dataPackage = await _client.ReceivePackageAsync();
                    if (dataPackage == null || token.IsCancellationRequested)
                    {
                        AddLogMessage("Connection lost.");
                        break;
                    }

                    if (dataPackage[0] == (byte)'U')
                    {
                        var parsedData = RTDEDataParser.ParseDataPackage(dataPackage, variables);
                        if (parsedData != null)
                        {
                            // Use Dispatcher to update UI from background thread
                            Application.Current.Dispatcher.Invoke(() =>
                            {
                                if (parsedData.ContainsKey("actual_TCP_pose"))
                                {
                                    var pose = parsedData["actual_TCP_pose"];
                                    PoseData = $"Pose: X={pose[0]:F3}, Y={pose[1]:F3}, Z={pose[2]:F3}, Rx={pose[3]:F3}, Ry={pose[4]:F3}, Rz={pose[5]:F3}";
                                }

                                if (parsedData.ContainsKey("actual_q"))
                                {
                                    var joints = parsedData["actual_q"];
                                    var jointsDeg = joints.Select(rad => rad * 180.0 / Math.PI).ToArray();
                                    JointData = $"Joints (deg): J1={jointsDeg[0]:F2}, J2={jointsDeg[1]:F2}, J3={jointsDeg[2]:F2}, J4={jointsDeg[3]:F2}, J5={jointsDeg[4]:F2}, J6={jointsDeg[5]:F2}";
                                }
                            });
                        }
                    }
                    else if (dataPackage[0] == (byte)'S')
                    {
                        AddLogMessage("Received Start confirmation. Waiting for data...");
                    }
                    else
                    {
                        AddLogMessage($"Received non-data package of type: {(char)dataPackage[0]} and length {dataPackage.Length}");
                    }
                }
                catch (Exception ex)
                {
                    AddLogMessage($"Error in data loop: {ex.Message}");
                    break;
                }
            }
            // Cleanup after loop finishes
            _client.Disconnect();
            Application.Current.Dispatcher.Invoke(() => {
                ConnectionStatus = "Disconnected";
            });
        }

        private void AddLogMessage(string message)
        {
            // Use dispatcher to ensure thread safety when updating logs
            Application.Current.Dispatcher.Invoke(() =>
            {
                LogMessages += $"[{DateTime.Now:HH:mm:ss}] {message}\n";
            });
        }
    }
    #region URRClient
    public class URRClient
    {
        private readonly string _robotIp;
        private readonly int _robotPort = 30004;
        private TcpClient _tcpClient;
        private NetworkStream _stream;

        private enum RtdEPackageType : byte
        {
            RTDE_REQUEST_PROTOCOL_VERSION = 86, // 'V'
            RTDE_GET_URCONTROL_VERSION = 118, // 'v'
            RTDE_TEXT_MESSAGE = 77, // 'M'
            RTDE_DATA_PACKAGE = 85, // 'U'
            RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79, // 'O'
            RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73, // 'I'
            RTDE_CONTROL_PACKAGE_START = 83, // 'S'
            RTDE_CONTROL_PACKAGE_PAUSE = 80  // 'P'
        }

        public URRClient(string ip)
        {
            _robotIp = ip;
        }

        public async Task ConnectAsync()
        {
            if (_tcpClient?.Connected == true)
            {
                Disconnect();
            }
            _tcpClient = new TcpClient();
            await _tcpClient.ConnectAsync(_robotIp, _robotPort);
            _stream = _tcpClient.GetStream();
            Console.WriteLine("Connected to RTDE interface.");
        }

        public void Disconnect()
        {
            _stream?.Close();
            _tcpClient?.Close();
            Console.WriteLine("Disconnected from RTDE interface.");
        }

        public async Task<bool> NegotiateProtocolVersionAsync()
        {
            // Request to use protocol version 2
            ushort protocolVersion = 2;

            byte[] sizeBytes = BitConverter.GetBytes((ushort)5); // size (2) + type (1) + payload (2)
            if (BitConverter.IsLittleEndian) Array.Reverse(sizeBytes);

            byte[] versionBytes = BitConverter.GetBytes(protocolVersion);
            if (BitConverter.IsLittleEndian) Array.Reverse(versionBytes);

            using (var ms = new MemoryStream())
            {
                ms.Write(sizeBytes, 0, sizeBytes.Length);
                ms.WriteByte((byte)RtdEPackageType.RTDE_REQUEST_PROTOCOL_VERSION);
                ms.Write(versionBytes, 0, versionBytes.Length);
                await _stream.WriteAsync(ms.ToArray(), 0, (int)ms.Length);
            }
            Console.WriteLine("Sent protocol version request.");

            // Receive response
            var response = await ReceivePackageAsync();
            // response[0] should be the type 'V'
            // response[1] should be the result (1 for success)
            if (response != null && response[0] == (byte)RtdEPackageType.RTDE_REQUEST_PROTOCOL_VERSION && response[1] == 1)
            {
                Console.WriteLine("Protocol version 2 accepted by controller.");
                return true;
            }

            Console.WriteLine("Failed to negotiate protocol version.");
            return false;
        }

        public async Task SendOutputSetupAsync(string[] variables, double frequency = 125.0)
        {
            string payloadStr = string.Join(",", variables);
            byte[] payload = Encoding.UTF8.GetBytes(payloadStr);

            byte[] frequencyBytes = BitConverter.GetBytes(frequency);
            if (BitConverter.IsLittleEndian) Array.Reverse(frequencyBytes);

            ushort size = (ushort)(3 + 8 + payload.Length);

            byte[] sizeBytes = BitConverter.GetBytes(size);
            if (BitConverter.IsLittleEndian) Array.Reverse(sizeBytes);

            using (var ms = new MemoryStream())
            {
                ms.Write(sizeBytes, 0, sizeBytes.Length);
                ms.WriteByte((byte)RtdEPackageType.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS);
                ms.Write(frequencyBytes, 0, frequencyBytes.Length);
                ms.Write(payload, 0, payload.Length);
                await _stream.WriteAsync(ms.ToArray(), 0, (int)ms.Length);
                Console.WriteLine($"Sent output setup request for: {payloadStr}");
            }
        }

        public async Task SendStartAsync()
        {
            byte[] sizeBytes = BitConverter.GetBytes((ushort)3);
            if (BitConverter.IsLittleEndian) Array.Reverse(sizeBytes);

            using (var ms = new MemoryStream())
            {
                ms.Write(sizeBytes, 0, sizeBytes.Length);
                ms.WriteByte((byte)RtdEPackageType.RTDE_CONTROL_PACKAGE_START);
                await _stream.WriteAsync(ms.ToArray(), 0, (int)ms.Length);
            }
            Console.WriteLine("Sent start command.");
        }

        public async Task<byte[]> ReceivePackageAsync()
        {
            byte[] sizeBuffer = new byte[2];
            int bytesRead = await _stream.ReadAsync(sizeBuffer, 0, 2);
            if (bytesRead < 2) return null;
            if (BitConverter.IsLittleEndian) Array.Reverse(sizeBuffer);
            ushort packageSize = BitConverter.ToUInt16(sizeBuffer, 0);

            // The full package includes the size header, so we read packageSize - 2 bytes for the payload
            byte[] payloadBuffer = new byte[packageSize - 2];
            int totalRead = 0;
            while (totalRead < payloadBuffer.Length)
            {
                bytesRead = await _stream.ReadAsync(payloadBuffer, totalRead, payloadBuffer.Length - totalRead);
                if (bytesRead == 0) return null;
                totalRead += bytesRead;
            }

            // Reconstruct the full package for context (type is the first byte of payload)
            byte[] fullPackage = new byte[packageSize];
            Array.Copy(sizeBuffer, 0, fullPackage, 0, 2);
            Array.Copy(payloadBuffer, 0, fullPackage, 2, payloadBuffer.Length);

            // We return just the payload (type + data) as before, for consistency
            return payloadBuffer;
        }
    }
    #endregion
    #region RTDEDataParser
    public static class RTDEDataParser
    {
        public static bool ParseOutputSetupResponse(byte[] package)
        {
            if (package[0] != (byte)'O')
            {
                Console.WriteLine("Error: Did not receive expected output setup confirmation.");
                return false;
            }
            string dataTypesStr = System.Text.Encoding.UTF8.GetString(package, 2, package.Length - 2);
            Console.WriteLine($"Received output setup confirmation. Recipe ID: {package[1]}. Data types: {dataTypesStr}");

            if (dataTypesStr.Contains("NOT_FOUND"))
            {
                Console.WriteLine("Error: One or more variables were not found by the controller.");
                return false;
            }
            return true;
        }

        public static Dictionary<string, double[]> ParseDataPackage(byte[] package, string[] variableNames)
        {
            if (package[0] != (byte)'U')
            {
                Console.WriteLine("Received unexpected package type while waiting for data.");
                return null;
            }

            var result = new Dictionary<string, double[]>();
            int offset = 2; // Start after type and recipe_id

            foreach (var name in variableNames)
            {
                // This parser is now more generic
                if (name == "actual_TCP_pose" || name == "target_TCP_pose" || name == "actual_q")
                {
                    var values = new double[6];
                    for (int i = 0; i < 6; i++)
                    {
                        if (offset + 8 > package.Length)
                        {
                            Console.WriteLine("Error: Data package is smaller than expected.");
                            return null;
                        }
                        byte[] doubleBytes = new byte[8];
                        Array.Copy(package, offset, doubleBytes, 0, 8);
                        if (BitConverter.IsLittleEndian) Array.Reverse(doubleBytes);
                        values[i] = BitConverter.ToDouble(doubleBytes, 0);
                        offset += 8;
                    }
                    result[name] = values;
                }
            }
            return result;
        }
    }
    #endregion
}