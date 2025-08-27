using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
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
        private DashboardClient _client1; // Foe Dashboard commands
        private SecondaryClient _client2; // Secondary interface
        private CancellationTokenSource _cancellationTokenSource;
        private bool _isConnected = false;

        // --- Properties for UI Binding (Connection Info) ---
        private string _robotIp = "192.168.56.101";
        private int _robotPort1 = 30004; // RTDE port
        private int _robotPort2 = 29999; // Dashboard
        private int _robotPort3 = 30002; // Secondary interface
        private string _robotMode = "Unknown";
        private string _programState = "Unknown";
        private readonly Dictionary<string, byte> _inputRecipeMap = new Dictionary<string, byte>();
        private string _variable;
        private string _regValue;
        private string _regType;
        private string _regId; 

        public string Variable { get => _variable; set => SetProperty(ref _variable, value); }
        public string RegValue { get => _regValue; set => SetProperty(ref _regValue, value); }
        public string RegType { get => _regType; set => SetProperty(ref _regType, value); }
        public string RegId { get => _regId; set => SetProperty(ref _regId, value); }
        public string RobotIp { get => _robotIp; set => SetProperty(ref _robotIp, value); }
        public string RobotMode { get => _robotMode; set => SetProperty(ref _robotMode, value); }
        public string ProgramState { get => _programState; set => SetProperty(ref _programState, value); }

        private string _connectionStatus = "Disconnected";
        public string ConnectionStatus { get => _connectionStatus; set { SetProperty(ref _connectionStatus, value); IsConnected = (value == "Connected"); } }
        private string _poseData = "No data";
        public string PoseData { get => _poseData; set => SetProperty(ref _poseData, value); }

        private string _jointData = "No data";
        public string JointData { get => _jointData; set => SetProperty(ref _jointData, value); }

        private string _logMessages = "";
        public byte _recipeId;
        public byte RecipeId { get => _recipeId; set => SetProperty(ref _recipeId, value); }
        public string LogMessages { get => _logMessages; set => SetProperty(ref _logMessages, value); }
        public string _registerTypeSet = "Unknown";

        public bool IsConnected
        {
            get => _isConnected;
            set
            {
                if (SetProperty(ref _isConnected, value))
                {
                    ((RelayCommand)ConnectCommand).RaiseCanExecuteChanged();
                    ((RelayCommand)DisconnectCommand).RaiseCanExecuteChanged();
                    ((RelayCommand)PlayCommand).RaiseCanExecuteChanged();
                    ((RelayCommand)StopCommand).RaiseCanExecuteChanged();
                    ((RelayCommand)PauseCommand).RaiseCanExecuteChanged();
                    ((RelayCommand)PowerOnCommand).RaiseCanExecuteChanged();
                    ((RelayCommand)PowerOffCommand).RaiseCanExecuteChanged();
                    ((RelayCommand)BreakeReleaseCommand).RaiseCanExecuteChanged();
                    ((RelayCommand)SetRegisterCommand).RaiseCanExecuteChanged();
                    ((RelayCommand)GetRegisterCommand).RaiseCanExecuteChanged();
                    
                }
            }
        }

        // --- Commands ---
        public ICommand ConnectCommand { get; }
        public ICommand DisconnectCommand { get; }
        public ICommand PlayCommand { get; }
        public ICommand PauseCommand { get; }
        public ICommand StopCommand { get; }
        public ICommand PowerOnCommand { get; }

        public ICommand PowerOffCommand { get; }
        public ICommand BreakeReleaseCommand { get; }
        public ICommand SetRegisterCommand { get; }
        public ICommand GetRegisterCommand { get; }



        public MainViewModel()
        {
            ConnectCommand = new RelayCommand(ConnectToRobot, () => !IsConnected);
            DisconnectCommand = new RelayCommand(DisconnectFromRobot, () => IsConnected);
            PlayCommand = new RelayCommand(() => SendCommand("play"), () => IsConnected);
            PauseCommand = new RelayCommand(() => SendCommand("pause"), () => IsConnected);
            StopCommand = new RelayCommand(() => SendCommand("stop"), () => IsConnected);
            PowerOnCommand = new RelayCommand(() => SendCommand("power on"), () => IsConnected);
            PowerOffCommand = new RelayCommand(() => SendCommand("power off"), () => IsConnected);
            BreakeReleaseCommand = new RelayCommand(() => SendCommand("brake release"), () => IsConnected);
            SetRegisterCommand = new RelayCommand(() => { _ = SetRegCommand(_regType, _regId, _regValue); }, () => IsConnected);
            GetRegisterCommand = new RelayCommand(() => { _ = GetRegCommand(_variable); }, () => IsConnected);
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
        private bool _configOut7;
        public bool ConfigOut7 { get => _configOut7; set => SetProperty(ref _configOut7, value); }
        #endregion

        // --- Methods for Commands ---
        public async Task<string> SetRegCommand(string regType, string regId, string regValue)
        {
            try
            {
                this._regType = regType;
                this._regId = regId;
                this._regValue = regValue;
                string command = $"sec setVar():\n write_output_{regType}_register({regId}, {regValue})\n x= read_output_integer_register(24)\n textmsg(\"register:\", x)\nend\n";
                Console.WriteLine(command);

                string response = await _client2.SendCommandAsync(command);
                return response;

            }
            catch (Exception ex)
            {
                AddLogMessage($"Error setting register: {ex.Message}");
                return null;
            }
        }
        public async Task<string> SetRegCommand()
        {
            try
            {
                
                string command = $"sec setVar():\n write_output_{RegType}_register({RegId}, {RegValue})\n x= read_output_integer_register(24)\n textmsg(\"register:\", x)\nend\n";
                Console.WriteLine(command);

                string response = await _client2.SendCommandAsync(command);
                return response;

            }
            catch (Exception ex)
            {
                AddLogMessage($"Error setting register: {ex.Message}");
                return null;
            }
        }
        public async Task<string> GetRegCommand(string variable)
        {
            _client1 = new DashboardClient(RobotIp, _robotPort2);
            await _client1.ConnectAsync();
            string response = await _client1.SendCommandAsync($"getVariable {variable}");
            Console.WriteLine($"Received response: {response}");
            AddLogMessage($"Get Register Response: {response}");
            return response;
        }
        public async Task<string> GetRegCommand()
        {
            _client1 = new DashboardClient(RobotIp, _robotPort2);
            await _client1.ConnectAsync();
            string response = await _client1.SendCommandAsync($"getVariable {Variable}");
            Console.WriteLine($"Received response: {response}");
            AddLogMessage($"Get Register Response: {response}");
            return response;
        }

        /// <summary>
        /// Pushes the current states of all 16 digital outputs to the robot via RTDE.
        /// Packs the 8 standard and 8 configurable outputs into mask/value pairs
        /// matching RTDE inputs:
        /// - standard_digital_output_mask, standard_digital_output
        /// - configurable_digital_output_mask, configurable_digital_output
        /// Note: Requires a valid RTDE connection and an input recipe set up earlier.
        /// </summary>
        public async Task UpdateAllDigitalOuts()
        {
            // Guard: do nothing if we're not connected or the RTDE client isn't ready.
            if (!_isConnected || _client == null) return;

            // Collect current UI-bound states for the 8 standard digital outputs (bits 0..7).
            var standardStates = new[]
            {
                DigitalOut0, DigitalOut1, DigitalOut2, DigitalOut3,
                DigitalOut4, DigitalOut5, DigitalOut6, DigitalOut7
            };

            // Collect current UI-bound states for the 8 configurable digital outputs (bits 0..7).
            var configStates = new[]
            {
                ConfigOut0, ConfigOut1, ConfigOut2, ConfigOut3,
                ConfigOut4, ConfigOut5, ConfigOut6, ConfigOut7
            };

            // Ask the RTDE client to pack these into a single 'U' data package
            // (recipe id + std mask/value + cfg mask/value) and send to the controller.
            await _client.SendAllDigitalOutputStatesAsync(standardStates, configStates);
        }
        public async void ConnectToRobot()
        {
            _cancellationTokenSource = new CancellationTokenSource();
            _client = new URRClient(RobotIp, _robotPort1); // For RTDE data exchange
            _client1 = new DashboardClient(RobotIp, _robotPort2); // For URScript commands
            _client2 = new SecondaryClient(RobotIp, _robotPort3); // For secondary interface

            try
            {
                ConnectionStatus = "Connecting...";

                await _client.ConnectAsync();
                await _client1.ConnectAsync();
                await _client2.ConnectAsync();


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
                _ = Task.Run(() => DataExchangeLoop(outputVars, _cancellationTokenSource.Token));
            }
            catch (Exception ex)
            {
                ConnectionStatus = "Connection Failed";
                AddLogMessage($"Error: {ex.Message}");
                _client?.Disconnect();
            }
        }
        public void DisconnectFromRobot()
        {
            AddLogMessage("Disconnected from the robot");
            _cancellationTokenSource?.Cancel();
            _client.Disconnect();
            _client1.Disconnect();
            _client2.Disconnect();
            // The loop will handle calling _client.Disconnect()
        }
        public async void SendCommand(string command)
        {
            if (_client1 == null || !_isConnected) return;
            try
            {
                await _client1.SendCommandAsync(command);
                AddLogMessage($"Sent command: {command}");
                await Task.Delay(100);
                RobotMode = await _client1.SendCommandAsync("robotmode");
                ProgramState = await _client1.SendCommandAsync("programState"); 
            }
            catch (Exception ex)
            {
                AddLogMessage($"Failed to send command '{command}': {ex.Message}");
            }
        }
        public async void DataExchangeLoop(string[] variables, CancellationToken token)
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
                        RobotMode = await _client1.SendCommandAsync("robotmode");
                        ProgramState = await _client1.SendCommandAsync("programState");
                        lastHeartbeat = DateTime.UtcNow;
                    }

                    byte[] dataPackage;
                    dataPackage = await _client.ReceivePackageAsync(); 

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

        public void AddLogMessage(string message)
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
        private byte _digitalOutRecipeId;

        /// <summary>
        /// RTDE package types (values are ASCII codes as defined in the RTDE guide).
        /// 'V'=86, 'v'=118, 'M'=77, 'U'=85, 'O'=79, 'I'=73, 'S'=83, 'P'=80
        /// </summary>
        private enum RtdEPackageType : byte
        {
            RTDE_REQUEST_PROTOCOL_VERSION = 86, RTDE_GET_URCONTROL_VERSION = 118,
            RTDE_TEXT_MESSAGE = 77, RTDE_DATA_PACKAGE = 85,
            RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79, RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73,
            RTDE_CONTROL_PACKAGE_START = 83, RTDE_CONTROL_PACKAGE_PAUSE = 80
        }

        /// <summary>
        /// Create an RTDE client for the specified IP/port (usually 30004).
        /// </summary>
        public URRClient(string ip, int port)
        {
            this._robotIp = ip;
            this._robotPort = port;
        }

        /// <summary>
        /// Open a TCP connection and get the network stream for RTDE.
        /// </summary>
        public async Task ConnectAsync()
        {
            _tcpClient = new TcpClient();
            await _tcpClient.ConnectAsync(_robotIp, _robotPort);
            _stream = _tcpClient.GetStream();
            Console.WriteLine("Connected to RTDE interface.");
        }

        /// <summary>
        /// Close the RTDE network stream and socket.
        /// </summary>
        public void Disconnect()
        {
            _stream?.Close();
            _tcpClient?.Close();
            Console.WriteLine("Disconnected from RTDE interface.");
        }

        /// <summary>
        /// Ask the controller to use protocol version 2.
        /// Returns true if the controller accepts (payload: ['V', accepted=1]).
        /// </summary>
        public async Task<bool> NegotiateProtocolVersionAsync()
        {
            ushort protocolVersion = 2;

            // Build header: size(2, big-endian) + type(1) + version(2, big-endian)
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
            // Expect: [ 'V', accepted(1|0) ]
            if (response != null && response[0] == (byte)RtdEPackageType.RTDE_REQUEST_PROTOCOL_VERSION && response[1] == 1)
            {
                Console.WriteLine("Protocol version 2 accepted by controller.");
                return true;
            }
            Console.WriteLine("Failed to negotiate protocol version.");
            return false;
        }

        /// <summary>
        /// Configure a single output recipe with desired variables and frequency (1..125 Hz).
        /// Per spec, payload is: type 'O' + frequency(double) + csv variable names.
        /// </summary>
        public async Task SendOutputSetupAsync(string[] variables, double frequency = 125.0)
        {
            string payloadStr = string.Join(",", variables);
            byte[] payload = Encoding.UTF8.GetBytes(payloadStr);

            // RTDE uses big-endian for numbers
            byte[] frequencyBytes = BitConverter.GetBytes(frequency);
            if (BitConverter.IsLittleEndian) Array.Reverse(frequencyBytes);

            // size = 3(header) + 8(freq) + payload length
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

        /// <summary>
        /// Configure a single input recipe for writing controller inputs (e.g., digital outputs mask/value).
        /// Stores the returned recipe id for later use when sending 'U' data packages.
        /// </summary>
        public async Task SetupInputRecipeAsync(string[] variables)
        {
            string payloadStr = string.Join(",", variables);
            byte[] payload = Encoding.UTF8.GetBytes(payloadStr);

            // size = 3(header) + payload length
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

            // Expect: ['I', recipeId, "types..."]
            var response = await ReceivePackageAsync();
            _digitalOutRecipeId = RTDEDataParser.ParseInputSetupResponse(response);
            Console.WriteLine($"Stored Combined Digital Output Recipe ID: {_digitalOutRecipeId}");
        }


        /// <summary>
        /// Receive a single RTDE package from the stream.
        /// By default returns only the payload (without the 2-byte size). Set includeSizeField=true to get both.
        /// Returns null if the stream closes before full read.
        /// </summary>
        public async Task<byte[]> ReceivePackageAsync(bool includeSizeField = false)
        {
            if (_stream == null) return null;

            // 1) Read 2-byte big-endian total size (includes these 2 bytes)
            var sizeBytesBE = new byte[2];
            int read = await ReadExactAsync(_stream, sizeBytesBE, 0, 2);
            if (read != 2) return null; // Stream might be closed

            ushort totalSize = (ushort)((sizeBytesBE[0] << 8) | sizeBytesBE[1]);
            if (totalSize < 2) return null; // Invalid

            int payloadLen = totalSize - 2;
            var payload = new byte[payloadLen];

            // 2) Read the remaining payload (type + body)
            read = await ReadExactAsync(_stream, payload, 0, payloadLen);
            if (read != payloadLen) return null; // Early close

            if (!includeSizeField)
                return payload;

            // Return full frame if requested
            var full = new byte[totalSize];
            Buffer.BlockCopy(sizeBytesBE, 0, full, 0, 2);
            Buffer.BlockCopy(payload, 0, full, 2, payloadLen);
            return full;
        }

        /// <summary>
        /// Read exactly 'count' bytes from the stream (unless the connection closes).
        /// Returns the number of bytes actually read.
        /// </summary>
        private static async Task<int> ReadExactAsync(NetworkStream stream, byte[] buffer, int offset, int count)
        {
            int total = 0;
            while (total < count)
            {
                int n = await stream.ReadAsync(buffer, offset + total, count - total);
                if (n == 0) break; // Connection closed
                total += n;
            }
            return total;
        }

        /// <summary>
        /// Send a single RTDE_DATA_PACKAGE ('U') using the stored input recipe id to update
        /// standard and configurable digital outputs (mask/value for 8 bits each).
        /// </summary>
        public async Task SendAllDigitalOutputStatesAsync(bool[] standardStates, bool[] configStates)
        {
            if (_digitalOutRecipeId == 0) return; // Input recipe not valid

            byte standardMask = 0, standardValue = 0, configMask = 0, configValue = 0;
            for (int i = 0; i < 8; i++)
            {
                standardMask |= (byte)(1 << i);
                if (standardStates[i]) standardValue |= (byte)(1 << i);
                configMask |= (byte)(1 << i);
                if (configStates[i]) configValue |= (byte)(1 << i);
            }

            // Frame: size_be(2) | 'U'(1) | recipe_id(1) | std_mask(1) | std_val(1) | cfg_mask(1) | cfg_val(1)
            byte[] package = new byte[8];
            byte[] sizeBytes = BitConverter.GetBytes((ushort)package.Length);
            if (BitConverter.IsLittleEndian) Array.Reverse(sizeBytes);
            sizeBytes.CopyTo(package, 0);
            package[2] = (byte)RtdEPackageType.RTDE_DATA_PACKAGE;
            package[3] = _digitalOutRecipeId;
            package[4] = standardMask;
            package[5] = standardValue;
            package[6] = configMask;
            package[7] = configValue;

            await _stream.WriteAsync(package, 0, package.Length);
        }

        /// <summary>
        /// Utility: set up an output recipe for a single variable and return its recipe id and type string.
        /// Useful for one-off reads of a specific output.
        /// </summary>
        public async Task<(byte recipeId, string dataTypes)> SetupSingleOutputAsync(string variable, double frequency = 125.0)
        {
            byte[] payload = Encoding.UTF8.GetBytes(variable);
            byte[] freqBytes = BitConverter.GetBytes(frequency);
            if (BitConverter.IsLittleEndian) Array.Reverse(freqBytes);
            ushort size = (ushort)(3 + 8 + payload.Length);
            byte[] sizeBytes = BitConverter.GetBytes(size);
            if (BitConverter.IsLittleEndian) Array.Reverse(sizeBytes);

            using (var ms = new MemoryStream())
            {
                ms.Write(sizeBytes, 0, 2);
                ms.WriteByte((byte)RtdEPackageType.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS);
                ms.Write(freqBytes, 0, 8);
                ms.Write(payload, 0, payload.Length);
                await _stream.WriteAsync(ms.ToArray(), 0, (int)ms.Length);
            }

            var response = await ReceivePackageAsync();
            if (response == null || response[0] != (byte)RtdEPackageType.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS)
                throw new Exception("Output setup response alınamadı.");
            byte recipeId = response[1];
            string dataTypes = Encoding.UTF8.GetString(response, 2, response.Length - 2);
            if (dataTypes.Contains("NOT_FOUND"))
                throw new Exception("Register output değişkeni bulunamadı.");
            Console.WriteLine($"Output setup '{variable}' => RecipeID={recipeId} Types={dataTypes}");
            return (recipeId, dataTypes);
        }

        /// <summary>
        /// Wait for the next RTDE_DATA_PACKAGE for the given recipe id, or time out.
        /// </summary>
        public async Task<byte[]> ReceiveDataForRecipeAsync(byte recipeId, int timeoutMs = 1000)
        {
            var start = DateTime.UtcNow;
            while ((DateTime.UtcNow - start).TotalMilliseconds < timeoutMs)
            {
                var pkg = await ReceivePackageAsync();
                if (pkg == null) break;
                if (pkg[0] == (byte)RtdEPackageType.RTDE_DATA_PACKAGE && pkg[1] == recipeId)
                    return pkg;
            }
            throw new TimeoutException("İstenen recipe için veri paketi gelmedi.");
        }

        /// <summary>
        /// Send RTDE start command ('S') so the controller begins streaming outputs.
        /// </summary>
        public async Task SendStartAsync()
        {
            // Minimal header-only control packet
            ushort size = 3;
            byte[] sizeBytes = BitConverter.GetBytes(size);
            if (BitConverter.IsLittleEndian) Array.Reverse(sizeBytes);
            using (var ms = new MemoryStream())
            {
                ms.Write(sizeBytes, 0, 2);
                ms.WriteByte((byte)RtdEPackageType.RTDE_CONTROL_PACKAGE_START);
                await _stream.WriteAsync(ms.ToArray(), 0, (int)ms.Length);
            }
        }

        /// <summary>
        /// Send RTDE pause command ('P') to stop streaming outputs.
        /// </summary>
        public async Task SendPauseAsync()
        {
            // Minimal header-only control packet
            ushort size = 3;
            var sizeBytes = BitConverter.GetBytes(size);
            if (BitConverter.IsLittleEndian) Array.Reverse(sizeBytes);
            using (var ms = new MemoryStream())
            {
                ms.Write(sizeBytes, 0, 2);
                // NOTE: literal 84 == 'T'. For clarity, prefer (byte)RtdEPackageType.RTDE_CONTROL_PACKAGE_PAUSE ('P'=80).
                ms.WriteByte((byte)84 /*'P' yerine enum kullanıyorsanız (byte)RtdEPackageType.RTDE_CONTROL_PACKAGE_PAUSE*/ );
                await _stream.WriteAsync(ms.ToArray(), 0, (int)ms.Length);
            }
        }
    }
    #endregion
    #region DashboardClient
    public class DashboardClient
    {
        private readonly string _robotIp;
        private readonly int _dashboardPort;
        private TcpClient _tcpClient;
        private NetworkStream _stream;
        private StreamReader _reader;

        public DashboardClient(string ip, int port)
        {
            _robotIp = ip;
            _dashboardPort = port;
        }

        public bool IsConnected => _tcpClient?.Connected ?? false;

        public async Task ConnectAsync()
        {
            if (IsConnected) return;

            _tcpClient = new TcpClient();
            var connectTask = _tcpClient.ConnectAsync(_robotIp, _dashboardPort);
            if (await Task.WhenAny(connectTask, Task.Delay(3000)) != connectTask)
            {
                throw new TimeoutException("Failed to connect to the Dashboard Server.");
            }

            _stream = _tcpClient.GetStream();
            _reader = new StreamReader(_stream, Encoding.UTF8);

            await _reader.ReadLineAsync();
        }
        public void Disconnect()
        {
            _reader?.Dispose();
            _stream?.Close();
            _tcpClient?.Close();
        }
        public async Task<string> SendCommandAsync(string command)
        {
            if (!IsConnected || _reader == null)
            {
                throw new IOException("Not connected to the robot's Dashboard Server.");
            }
            byte[] buffer = Encoding.ASCII.GetBytes(command + "\n");
            await _stream.WriteAsync(buffer, 0, buffer.Length);

            // Yanıtı satır satır okumak için ReadLineAsync kullanın
            var response = await _reader.ReadLineAsync();
            return response?.Trim() ?? string.Empty;
        }
    }
    #endregion
    #region RTDE Data Parser
    /// <summary>
    /// Helpers to parse RTDE packages according to the "Real-Time Data Exchange (RTDE) Guide".
    /// Notes:
    /// - All multi-byte numbers from the controller are big-endian.
    /// - This class contains only minimal parsing for what the app uses (pose/joints, basic setup acks).
    /// </summary>
    public static class RTDEDataParser
    {
        /// <summary>
        /// Parses an Output Setup response ('O').
        /// Expects: ['O', output_recipe_id, variable_types_string...]
        /// Returns true when no variable type is "NOT_FOUND".
        /// </summary>
        public static bool ParseOutputSetupResponse(byte[] package)
        {
            if (package[0] != (byte)'O')
            {
                Console.WriteLine("Error: Did not receive expected output setup confirmation.");
                return false;
            }

            // Byte[1] is the output recipe id, the rest is a CSV of variable types.
            string dataTypesStr = Encoding.UTF8.GetString(package, 2, package.Length - 2);
            if (dataTypesStr.Contains("NOT_FOUND"))
            {
                Console.WriteLine("Error: One or more output variables were not found by the controller.");
                return false;
            }

            Console.WriteLine($"Received output setup confirmation. Recipe ID: {package[1]}. Data types: {dataTypesStr}");
            return true;
        }

        /// <summary>
        /// Parses an Input Setup response ('I').
        /// Expects: ['I', input_recipe_id, variable_types_string...]
        /// Throws if any variable is "NOT_FOUND".
        /// Tip: The RTDE spec also allows "IN_USE" for inputs; handle it if needed.
        /// </summary>
        public static byte ParseInputSetupResponse(byte[] package)
        {
            if (package[0] != (byte)'I')
                throw new Exception("Error: Did not receive expected input setup confirmation.");

            // Byte[1] is the input recipe id, the rest is a CSV of variable types.
            string result = Encoding.UTF8.GetString(package, 1, package.Length - 1);
            if (result.Contains("NOT_FOUND"))
                throw new Exception("Error: One or more input variables were not found by the controller.");

            // The recipe ID is the second byte of the payload
            return package[1];
        }

        /// <summary>
        /// Parses a text message ('M') payload to a readable string.
        /// Note: Protocol v2 has structured fields; here we flatten bytes[1..] for compact logging.
        /// </summary>
        public static string ParseTextMessage(byte[] package)
        {
            if (package.Length < 2) return "Unknown message.";
            return Encoding.UTF8.GetString(package, 1, package.Length - 1);
        }

        /// <summary>
        /// Parses a data package ('U') for selected VECTOR6D variables (pose/joints).
        /// Assumptions:
        /// - The incoming payload order matches variableNames (recipe order).
        /// - Only variables "actual_TCP_pose", "target_TCP_pose", and "actual_q" are parsed.
        /// - Each VECTOR6D = 6 x DOUBLE (big-endian).
        /// Returns a map: variableName -> double[6].
        /// </summary>
        public static Dictionary<string, double[]> ParseDataPackage(byte[] package, string[] variableNames)
        {
            var result = new Dictionary<string, double[]>();

            // Skip package type ('U') and recipe_id.
            int offset = 2;

            foreach (var name in variableNames)
            {
                // Only parse variables we know are VECTOR6D
                if (name == "actual_TCP_pose" || name == "target_TCP_pose" || name == "actual_q")
                {
                    // 6 doubles x 8 bytes = 48 bytes
                    if (offset + 48 > package.Length)
                    {
                        Console.WriteLine("Error: Data package is smaller than expected for vector data.");
                        return null;
                    }

                    var values = new double[6];
                    for (int i = 0; i < 6; i++)
                    {
                        byte[] doubleBytes = new byte[8];
                        Array.Copy(package, offset, doubleBytes, 0, 8);

                        // Convert big-endian -> little-endian if needed
                        if (BitConverter.IsLittleEndian) Array.Reverse(doubleBytes);

                        values[i] = BitConverter.ToDouble(doubleBytes, 0);
                        offset += 8;
                    }

                    result[name] = values;
                }
                else
                {
                    // If your recipe includes other variables, you must advance 'offset'
                    // according to their data type sizes to stay aligned.
                    // This implementation ignores unknown names and leaves offset unchanged,
                    // assuming the recipe contains only the known VECTOR6D variables.
                }
            }

            return result;
        }
    }
    #endregion
    public class SecondaryClient
    {
        private readonly string _robotIp;
        private readonly int _secondaryPort;
        private TcpClient _client;
        private NetworkStream _stream;

        public SecondaryClient(string ip, int port)
        {
            _robotIp = ip;
            _secondaryPort = port;
        }

        public bool IsConnected => _client?.Connected ?? false;

        public async Task ConnectAsync()
        {
            _client = new TcpClient(_robotIp, _secondaryPort); 
            _stream = _client.GetStream();
        }

        public void Disconnect()
        {
            _stream?.Close();
            _client?.Close();
        }


        public async Task<string> SendCommandAsync(string command)
        {

            if (!IsConnected)
            {
                throw new IOException("Not connected to the robot's Dashboard Server.");
            }

            byte[] data = System.Text.Encoding.ASCII.GetBytes(command);
            await _stream.WriteAsync(data, 0, data.Length);
            byte[] responseBytes = new byte[1024]; // Adjust size as needed
            int responseSize = await _stream.ReadAsync(responseBytes, 0, responseBytes.Length);
            string response = System.Text.Encoding.ASCII.GetString(responseBytes, 0, responseSize).Trim();
            Console.WriteLine($"Received response: {response}");
            return response;
        }
        public async Task<string> ReceiveResponseAsync()
        {
           
            // Read response line
            byte[] responseBytes = new byte[1024]; // Adjust size as needed
            int responseSize = await _stream.ReadAsync(responseBytes, 0, responseBytes.Length);
            string response = System.Text.Encoding.ASCII.GetString(responseBytes, 0, responseSize).Trim();
            Console.WriteLine($"Received response: {response}");
            return response;
        }   
    }
}