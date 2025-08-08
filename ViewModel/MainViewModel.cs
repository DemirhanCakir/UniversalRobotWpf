using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Runtime.CompilerServices;
using System.Security.RightsManagement;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;

namespace UniversalRobotWpf
{
    public class MainViewModel : BaseViewModel
    {
        // --- Private fields for state ---
        private URRClient _client; // For RTDE data exchange
        private URRClient _client1; // For URScript commands
        private CancellationTokenSource _cancellationTokenSource;
        private bool _isConnected = false;

        // --- Properties for UI Binding (Connection Info) ---
        private string _robotIp = "192.168.56.101";
        private int _robotPort1 = 30004; // RTDE port
        private int _robotPort2 = 29999; // Sending commands
        private string _robotMode = "Unknown";

        public string RobotIp { get => _robotIp; set => SetProperty(ref _robotIp, value); }
        public string RobotMode { get => _robotMode; set => SetProperty(ref _robotMode, value); }

        private string _connectionStatus = "Disconnected";
        public string ConnectionStatus { get => _connectionStatus; set { SetProperty(ref _connectionStatus, value); IsConnected = (value == "Connected"); } }

        public bool IsConnected { get => _isConnected; set { if (SetProperty(ref _isConnected, value)) { ((RelayCommand)ConnectCommand).RaiseCanExecuteChanged(); ((RelayCommand)DisconnectCommand).RaiseCanExecuteChanged(); } } }

        private string _poseData = "No data";
        public string PoseData { get => _poseData; set => SetProperty(ref _poseData, value); }

        private string _jointData = "No data";
        public string JointData { get => _jointData; set => SetProperty(ref _jointData, value); }

        private string _logMessages = "";
        public string LogMessages { get => _logMessages; set => SetProperty(ref _logMessages, value); }

        // --- Commands ---
        public ICommand ConnectCommand { get; }
        public ICommand DisconnectCommand { get; }
        public ICommand PlayCommand { get; }
        public ICommand PauseCommand { get; }
        public ICommand StopCommand { get; }


        public MainViewModel()
        {
            ConnectCommand = new RelayCommand(ConnectToRobot, () => !IsConnected);
            DisconnectCommand = new RelayCommand(DisconnectFromRobot, () => IsConnected);
            PlayCommand = new RelayCommand(() => SendCommand("play"));
            PauseCommand = new RelayCommand(() => SendCommand("pause"));
            StopCommand = new RelayCommand(() => SendCommand("stop"));
        }

        #region Digital Output Properties
        // Note: Setters now only update the property. The loop handles sending the data.
        private bool _digitalOut0; public bool DigitalOut0 { get => _digitalOut0; set => SetProperty(ref _digitalOut0, value); }
        private bool _digitalOut1; public bool DigitalOut1 { get => _digitalOut1; set => SetProperty(ref _digitalOut1, value); }
        private bool _digitalOut2; public bool DigitalOut2 { get => _digitalOut2; set => SetProperty(ref _digitalOut2, value); }
        private bool _digitalOut3; public bool DigitalOut3 { get => _digitalOut3; set => SetProperty(ref _digitalOut3, value); }
        private bool _digitalOut4; public bool DigitalOut4 { get => _digitalOut4; set => SetProperty(ref _digitalOut4, value); }
        private bool _digitalOut5; public bool DigitalOut5 { get => _digitalOut5; set => SetProperty(ref _digitalOut5, value); }
        private bool _digitalOut6; public bool DigitalOut6 { get => _digitalOut6; set => SetProperty(ref _digitalOut6, value); }
        private bool _digitalOut7; public bool DigitalOut7 { get => _digitalOut7; set => SetProperty(ref _digitalOut7, value); }

        private bool _configOut0; public bool ConfigOut0 { get => _configOut0; set => SetProperty(ref _configOut0, value); }
        private bool _configOut1; public bool ConfigOut1 { get => _configOut1; set => SetProperty(ref _configOut1, value); }
        private bool _configOut2; public bool ConfigOut2 { get => _configOut2; set => SetProperty(ref _configOut2, value); }
        private bool _configOut3; public bool ConfigOut3 { get => _configOut3; set => SetProperty(ref _configOut3, value); }
        private bool _configOut4; public bool ConfigOut4 { get => _configOut4; set => SetProperty(ref _configOut4, value); }
        private bool _configOut5; public bool ConfigOut5 { get => _configOut5; set => SetProperty(ref _configOut5, value); }
        private bool _configOut6; public bool ConfigOut6 { get => _configOut6; set => SetProperty(ref _configOut6, value); }
        private bool _configOut7; public bool ConfigOut7 { get => _configOut7; set => SetProperty(ref _configOut7, value); }
        #endregion

        private async Task UpdateAllDigitalOuts()
        {
            if (!_isConnected || _client == null) return;
            var standardStates = new[] { DigitalOut0, DigitalOut1, DigitalOut2, DigitalOut3, DigitalOut4, DigitalOut5, DigitalOut6, DigitalOut7 };
            var configStates = new[] { ConfigOut0, ConfigOut1, ConfigOut2, ConfigOut3, ConfigOut4, ConfigOut5, ConfigOut6, ConfigOut7 };
            await _client.SendAllDigitalOutputStatesAsync(standardStates, configStates);
        }

        private async void ConnectToRobot()
        {
            _cancellationTokenSource = new CancellationTokenSource();
            _client = new URRClient(RobotIp, _robotPort1); // For RTDE data exchange
            _client1 = new URRClient(RobotIp, _robotPort2); // For URScript commands

            try
            {
                ConnectionStatus = "Connecting...";

                await _client.ConnectAsync();
                await _client1.ConnectAsync();


                if (!await _client.NegotiateProtocolVersionAsync()) throw new Exception("Failed to negotiate protocol version.");

                // Setup outputs (for reading data FROM robot)
                var outputVars = new[] { "actual_TCP_pose", "actual_q" };
                await _client.SendOutputSetupAsync(outputVars);
                var setupResponse = await _client.ReceivePackageAsync();
                if (setupResponse == null || !RTDEDataParser.ParseOutputSetupResponse(setupResponse)) throw new Exception("Failed to setup RTDE output recipe.");

                // Setup ONE combined input recipe for all 16 outputs
                var inputVars = new[]
                {
                    "standard_digital_output_mask", "standard_digital_output",
                    "configurable_digital_output_mask", "configurable_digital_output"
                };
                await _client.SetupInputRecipeAsync(inputVars);

                await _client.SendStartAsync();
                ConnectionStatus = "Connected";
                AddLogMessage("Connection successful. Starting data exchange...");

                // Start the loop that handles both receiving data and sending the I/O heartbeat
                Task.Run(() => DataExchangeLoop(outputVars, _cancellationTokenSource.Token));
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
            // The loop will handle calling _client.Disconnect()
        }

        private void SendCommand(string command)
        {
            if (_client1 == null || !_isConnected) return;
            try
            {
                _client1.SendCommandAsync(command);
                AddLogMessage($"Sent command: {command}");
                RobotMode = _client1.SendCommandAsync("robotmode");
            }
            catch (Exception ex)
            {
                AddLogMessage($"Failed to send command '{command}': {ex.Message}");
            }
        }

        private async void DataExchangeLoop(string[] variables, CancellationToken token)
        {
            var lastHeartbeat = DateTime.MinValue;
            var heartbeatInterval = TimeSpan.FromMilliseconds(500); // Send I/O state every 500ms

            while (!token.IsCancellationRequested)
            {
                try
                {
                    // Heartbeat logic to keep the connection alive
                    if (DateTime.UtcNow - lastHeartbeat > heartbeatInterval)
                    {
                        await UpdateAllDigitalOuts();
                        lastHeartbeat = DateTime.UtcNow;
                    }
                    var dataPackage = await _client.ReceivePackageAsync();
                    RobotMode = _client1.SendCommandAsync("robotmode");

                    if (dataPackage == null) { AddLogMessage("Connection lost."); break; }

                    switch (dataPackage[0])
                    {
                        case (byte)'U': // Data Package
                            var parsedData = RTDEDataParser.ParseDataPackage(dataPackage, variables);
                            if (parsedData != null)
                            {
                                Application.Current.Dispatcher.Invoke(() =>
                                {
                                    if (parsedData.TryGetValue("actual_TCP_pose", out var pose))
                                        PoseData = $"X:{pose[0]:F3}, Y:{pose[1]:F3}, Z:{pose[2]:F3}, Rx:{pose[3]:F3}, Ry:{pose[4]:F3}, Rz:{pose[5]:F3}";
                                    if (parsedData.TryGetValue("actual_q", out var joints))
                                    {
                                        var jointsDeg = joints.Select(rad => rad * 180.0 / Math.PI).ToArray();
                                        JointData = $"J1:{jointsDeg[0]:F2}°, J2:{jointsDeg[1]:F2}°, J3:{jointsDeg[2]:F2}°, J4:{jointsDeg[3]:F2}°, J5:{jointsDeg[4]:F2}°, J6:{jointsDeg[5]:F2}°";
                                    }
                                });
                            }
                            break;

                        case (byte)'M': // Text Message from Robot
                            AddLogMessage($"Robot Message: {RTDEDataParser.ParseTextMessage(dataPackage)}");
                            break;

                        case (byte)'S': // Start Confirmation
                            AddLogMessage("Start confirmation received.");
                            break;
                    }
                }
                catch (IOException ex) { AddLogMessage($"Connection closed: {ex.Message}"); break; }
                catch (Exception ex) { AddLogMessage($"Error in data loop: {ex.Message}"); break; }
            }

            _client.Disconnect();
            Application.Current.Dispatcher.Invoke(() => { ConnectionStatus = "Disconnected"; });
        }

        private void AddLogMessage(string message)
        {
            Application.Current.Dispatcher.Invoke(() => { LogMessages += $"[{DateTime.Now:HH:mm:ss}] {message}\n"; });
        }
    }
    #region URRClient 
    public class URRClient
    {
        private readonly string _robotIp;
        private readonly int _robotPort;
        private TcpClient _tcpClient;
        private NetworkStream _stream;
        private byte _digitalOutRecipeId; // A single ID for our combined input recipe

        private enum RtdEPackageType : byte
        {
            RTDE_REQUEST_PROTOCOL_VERSION = 86, RTDE_GET_URCONTROL_VERSION = 118,
            RTDE_TEXT_MESSAGE = 77, RTDE_DATA_PACKAGE = 85,
            RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79, RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73,
            RTDE_CONTROL_PACKAGE_START = 83, RTDE_CONTROL_PACKAGE_PAUSE = 80
        }

        public URRClient(string ip, int port) 
        {
            this._robotIp = ip;
            this._robotPort = port;
        }

        public async Task ConnectAsync()
        {   
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
            ushort protocolVersion = 2;
            byte[] sizeBytes = BitConverter.GetBytes((ushort)5);
            if (BitConverter.IsLittleEndian) Array.Reverse(sizeBytes);
            byte[] versionBytes = BitConverter.GetBytes(protocolVersion);
            if (BitConverter.IsLittleEndian) Array.Reverse(versionBytes);

            var package = new byte[5];
            sizeBytes.CopyTo(package, 0);
            package[2] = (byte)RtdEPackageType.RTDE_REQUEST_PROTOCOL_VERSION;
            versionBytes.CopyTo(package, 3);

            await _stream.WriteAsync(package, 0, package.Length);
            Console.WriteLine("Sent protocol version request.");

            var response = await ReceivePackageAsync();
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
                ms.Write(sizeBytes, 0, 2);
                ms.WriteByte((byte)RtdEPackageType.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS);
                ms.Write(frequencyBytes, 0, 8);
                ms.Write(payload, 0, payload.Length);
                await _stream.WriteAsync(ms.ToArray(), 0, (int)ms.Length);
            }
            Console.WriteLine($"Sent output setup request for: {payloadStr}");
        }

        public async Task SetupInputRecipeAsync(string[] variables)
        {
            string payloadStr = string.Join(",", variables);
            byte[] payload = Encoding.UTF8.GetBytes(payloadStr);
            ushort size = (ushort)(3 + payload.Length);
            byte[] sizeBytes = BitConverter.GetBytes(size);
            if (BitConverter.IsLittleEndian) Array.Reverse(sizeBytes);

            using (var ms = new MemoryStream())
            {
                ms.Write(sizeBytes, 0, 2);
                ms.WriteByte((byte)RtdEPackageType.RTDE_CONTROL_PACKAGE_SETUP_INPUTS);
                ms.Write(payload, 0, payload.Length);
                await _stream.WriteAsync(ms.ToArray(), 0, (int)ms.Length);
            }
            Console.WriteLine($"Sent combined input setup request for: {payloadStr}");

            var response = await ReceivePackageAsync();
            _digitalOutRecipeId = RTDEDataParser.ParseInputSetupResponse(response);
            Console.WriteLine($"Stored Combined Digital Output Recipe ID: {_digitalOutRecipeId}");
        }

        public async Task SendAllDigitalOutputStatesAsync(bool[] standardStates, bool[] configStates)
        {
            if (_digitalOutRecipeId == 0) return; // Recipe not set up yet

            byte standardMask = 0, standardValue = 0, configMask = 0, configValue = 0;
            for (int i = 0; i < 8; i++)
            {
                standardMask |= (byte)(1 << i);
                if (standardStates[i]) standardValue |= (byte)(1 << i);
                configMask |= (byte)(1 << i);
                if (configStates[i]) configValue |= (byte)(1 << i);
            }

            byte[] package = new byte[8];  // The package is 8 bytes: Size(2), Type(1), RecipeID(1), Mask1(1), Val1(1), Mask2(1), Val2(1)
            ushort size = (ushort)package.Length;
            byte[] sizeBytes = BitConverter.GetBytes((ushort)package.Length);
            if (BitConverter.IsLittleEndian) Array.Reverse(sizeBytes);
            sizeBytes.CopyTo(package, 0); // Size
            package[2] = (byte)RtdEPackageType.RTDE_DATA_PACKAGE; // Type 'U'
            package[3] = _digitalOutRecipeId; // Recipe ID
            package[4] = standardMask;
            package[5] = standardValue;
            package[6] = configMask;
            package[7] = configValue;

            await _stream.WriteAsync(package, 0, package.Length);
        }

        public async Task SendStartAsync()
        {
            ushort size = 3; // Size of the package: 2 bytes for size, 1 byte for type
            byte[] sizeBytes = BitConverter.GetBytes(size);
            if (BitConverter.IsLittleEndian) Array.Reverse(sizeBytes);
            using (var ms = new MemoryStream())
            {
                ms.Write(sizeBytes, 0, 2); // Write size
                ms.WriteByte((byte)RtdEPackageType.RTDE_CONTROL_PACKAGE_START); // Write package type
                await _stream.WriteAsync(ms.ToArray(), 0, (int)ms.Length);
            }
        }

        public async Task<byte[]> ReceivePackageAsync()
        {
            byte[] sizeBuffer = new byte[2];
            await _stream.ReadAsync(sizeBuffer, 0, 2);
            if (BitConverter.IsLittleEndian) Array.Reverse(sizeBuffer);
            ushort packageSize = BitConverter.ToUInt16(sizeBuffer, 0);

            byte[] payloadBuffer = new byte[packageSize - 2];
            await _stream.ReadAsync(payloadBuffer, 0, payloadBuffer.Length);
            return payloadBuffer;
        }
        public string SendCommandAsync(string command) // port:29999 function
        {
            byte[] commandBytes = Encoding.UTF8.GetBytes(command);
            _stream.WriteAsync(commandBytes, 0, commandBytes.Length);
            byte[] responseBuffer = new byte[1024]; // Assume max response size
            _stream.ReadAsync(responseBuffer, 0, responseBuffer.Length);
            string response = Encoding.UTF8.GetString(responseBuffer);
            return response;

        }
    }
    #endregion
    #region RTDE Data Parser
    public static class RTDEDataParser
    {
        public static bool ParseOutputSetupResponse(byte[] package)
        {
            if (package[0] != (byte)'O')
            {
                Console.WriteLine("Error: Did not receive expected output setup confirmation.");
                return false;
            }
            string dataTypesStr = Encoding.UTF8.GetString(package, 2, package.Length - 2);
            if (dataTypesStr.Contains("NOT_FOUND"))
            {
                Console.WriteLine("Error: One or more output variables were not found by the controller.");
                return false;
            }
            Console.WriteLine($"Received output setup confirmation. Recipe ID: {package[1]}. Data types: {dataTypesStr}");
            return true;
        }

        public static byte ParseInputSetupResponse(byte[] package)
        {
            if (package[0] != (byte)'I')
                throw new Exception("Error: Did not receive expected input setup confirmation.");

            string result = Encoding.UTF8.GetString(package, 1, package.Length - 1);
            if (result.Contains("NOT_FOUND"))
                throw new Exception("Error: One or more input variables were not found by the controller.");

            // The recipe ID is the second byte of the payload
            return package[1];
        }

        public static string ParseTextMessage(byte[] package)
        {
            if (package.Length < 2) return "Unknown message.";
            return Encoding.UTF8.GetString(package, 1, package.Length - 1);
        }

        public static Dictionary<string, double[]> ParseDataPackage(byte[] package, string[] variableNames)
        {
            var result = new Dictionary<string, double[]>();
            int offset = 2; // Start after type 'U' and recipe_id

            foreach (var name in variableNames)
            {
                if (name == "actual_TCP_pose" || name == "target_TCP_pose" || name == "actual_q")
                {
                    if (offset + 48 > package.Length) { Console.WriteLine("Error: Data package is smaller than expected for vector data."); return null; }
                    var values = new double[6];
                    for (int i = 0; i < 6; i++)
                    {
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