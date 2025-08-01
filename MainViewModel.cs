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

namespace UniversalRobotWpf
{
    public class MainViewModel : INotifyPropertyChanged
    {
        private string _robotAddress = "192.168.56.101";
        private string _connectionStatus = "Disconnected";
        private string _poseData = "No data";
        private string _jointData = "No data";
        private string _logMessages = "";
        private DispatcherTimer _updateTimer;
        private bool _isConnected = false;
        private URDirectCommunication _primaryInterface;
        private URRtdeCommunication _rtde;
        private URDashboardCommunication _dashboard;

        public MainViewModel()
        {
            ConnectCommand = new RelayCommand(ConnectToRobot, CanConnect);
            _updateTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(100)
            };
            _updateTimer.Tick += UpdateTimer_Tick;
        }

        public string RobotAddress
        {
            get => _robotAddress;
            set => SetProperty(ref _robotAddress, value);
        }

        public string ConnectionStatus
        {
            get => _connectionStatus;
            set => SetProperty(ref _connectionStatus, value);
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

        public ICommand ConnectCommand { get; }

        private bool CanConnect()
        {
            return !_isConnected && !string.IsNullOrWhiteSpace(RobotAddress);
        }

        private async void ConnectToRobot()
        {
            try
            {
                ConnectionStatus = "Connecting...";
                AddLogMessage("Attempting to connect to robot...");

                // 1. Dashboard bağlantısı
                _dashboard = new URDashboardCommunication(RobotAddress);
                if (_dashboard.Connect())
                {
                    AddLogMessage("Dashboard connected");

                    // 2. Primary Interface bağlantısı
                    _primaryInterface = new URDirectCommunication(RobotAddress);
                    if (_primaryInterface.Connect())
                    {
                        AddLogMessage("Primary Interface connected");

                        // 3. RTDE bağlantısı
                        _rtde = new URRtdeCommunication(RobotAddress);
                        if (_rtde.Connect())
                        {
                            AddLogMessage("RTDE connected");

                            _isConnected = true;
                            ConnectionStatus = "Connected";
                            AddLogMessage("Successfully connected to robot!");

                            // Timer başlat
                            _updateTimer.Start();
                            ((RelayCommand)ConnectCommand).RaiseCanExecuteChanged();
                        }
                        else
                        {
                            AddLogMessage("RTDE Connection failed");
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                ConnectionStatus = "Connection Failed";
                AddLogMessage($"Error connecting to robot: {ex.Message}");

                // Bağlantıları temizle
                _primaryInterface?.Disconnect();
                _rtde?.Disconnect();
                _dashboard?.Disconnect();
            }
        }

        private void UpdateTimer_Tick(object sender, EventArgs e)
        {
            if (_isConnected && _primaryInterface != null)
            {
                try
                {
                    byte[] rawData = _primaryInterface.ReceiveData();
                    if (rawData != null && rawData.Length > 0)
                    {
                        Dictionary<string, double> cartesianValues = ParseCartesianData(rawData);
                        Dictionary<string, double> jointValues = ParseJointData(rawData);
                        
                        if (cartesianValues.Count >= 6)
                        {
                            PoseData = $"X: {(cartesianValues["X"] * 1000):F3}\n" +
                                       $"Y: {(cartesianValues["Y"] * 1000):F3}\n" +
                                       $"Z: {(cartesianValues["Z"] * 1000):F3}\n" +
                                       $"Rx: {cartesianValues["Rx"]:F3}\n" +
                                       $"Ry: {cartesianValues["Ry"]:F3}\n" +
                                       $"Rz: {cartesianValues["Rz"]:F3}";
                        }

                        if (jointValues.Count >= 6)
                        {
                            JointData = $"Base: {(jointValues["Base"] * (180 / Math.PI)):F3}\n" +
                                        $"Shoulder: {(jointValues["Shoulder"] * (180 / Math.PI)):F3}\n" +
                                        $"Elbow: {(jointValues["Elbow"] * (180 / Math.PI)):F3}\n" +
                                        $"Wrist1: {(jointValues["Wrist1"] * (180 / Math.PI)):F3}\n" +
                                        $"Wrist2: {(jointValues["Wrist2"] * (180 / Math.PI)):F3}\n" +
                                        $"Wrist3: {(jointValues["Wrist3"] * (180 / Math.PI)):F3}";
                        }
                    }
                }
                catch (Exception ex)
                {
                    AddLogMessage($"Error updating data: {ex.Message}");
                }
            }
        }
        private Dictionary<string, double> ParseCartesianData(byte[] rawData)
        {
            Dictionary<string, double> result = new Dictionary<string, double>();
            
            try
            {
                // UR CB3/e-Series protocol için offset değerleri
                // Not: Bu değerler robot firmware versiyonuna göre değişebilir
                int tcpPoseBaseOffset = 0; // Örnek offset değeri, dokümantasyona göre ayarlanmalı
                
                if (rawData.Length >= tcpPoseBaseOffset + 48) // 6 adet double (her biri 8 byte)
                {
                    result["X"] = BitConverter.ToDouble(rawData, tcpPoseBaseOffset);
                    result["Y"] = BitConverter.ToDouble(rawData, tcpPoseBaseOffset + 8);
                    result["Z"] = BitConverter.ToDouble(rawData, tcpPoseBaseOffset + 16);
                    result["Rx"] = BitConverter.ToDouble(rawData, tcpPoseBaseOffset + 24);
                    result["Ry"] = BitConverter.ToDouble(rawData, tcpPoseBaseOffset + 32);
                    result["Rz"] = BitConverter.ToDouble(rawData, tcpPoseBaseOffset + 40);
                }
            }
            catch (Exception ex)
            {
                AddLogMessage($"Error parsing cartesian data: {ex.Message}");
            }
            
            return result;
        }

        private Dictionary<string, double> ParseJointData(byte[] rawData)
        {
            Dictionary<string, double> result = new Dictionary<string, double>();
            
            try
            {
                // UR CB3/e-Series protocol için offset değerleri
                int jointPosBaseOffset = 363; // Örnek offset değeri, dokümantasyona göre ayarlanmalı
                
                if (rawData.Length >= jointPosBaseOffset + 48) // 6 adet double (her biri 8 byte)
                {
                    result["Base"] = BitConverter.ToDouble(rawData, jointPosBaseOffset);
                    result["Shoulder"] = BitConverter.ToDouble(rawData, jointPosBaseOffset + 8);
                    result["Elbow"] = BitConverter.ToDouble(rawData, jointPosBaseOffset + 16);
                    result["Wrist1"] = BitConverter.ToDouble(rawData, jointPosBaseOffset + 24);
                    result["Wrist2"] = BitConverter.ToDouble(rawData, jointPosBaseOffset + 32);
                    result["Wrist3"] = BitConverter.ToDouble(rawData, jointPosBaseOffset + 40);
                }
            }
            catch (Exception ex)
            {
                AddLogMessage($"Error parsing joint data: {ex.Message}");
            }
            
            return result;
        }

        private void AddLogMessage(string message)
        {
            LogMessages += $"[{DateTime.Now:HH:mm:ss}] {message}\n";
        }

        public event PropertyChangedEventHandler PropertyChanged;

        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        protected bool SetProperty<T>(ref T field, T value, [CallerMemberName] string propertyName = null)
        {
            if (EqualityComparer<T>.Default.Equals(field, value)) return false;
            field = value;
            OnPropertyChanged(propertyName);
            return true;
        }
    }

    public class RelayCommand : ICommand
    {
        private readonly Action _execute;
        private readonly Func<bool> _canExecute; 

        public RelayCommand(Action execute, Func<bool> canExecute = null)
        {
            _execute = execute ?? throw new ArgumentNullException(nameof(execute));
            _canExecute = canExecute;
        }

        public event EventHandler CanExecuteChanged;

        public bool CanExecute(object parameter) => _canExecute == null ? true : _canExecute();

        public void Execute(object parameter) => _execute();

        public void RaiseCanExecuteChanged() => CanExecuteChanged?.Invoke(this, EventArgs.Empty);
    }
    public class URDirectCommunication
    {
        private TcpClient _client;
        private NetworkStream _stream;
        private readonly string _ipAddress;
        private readonly int _port = 30001; // Primary interface portu

        public URDirectCommunication(string ipAddress)
        {
            _ipAddress = ipAddress;
        }

        public bool Connect()
        {
            try
            {
                _client = new TcpClient(_ipAddress, _port);
                _stream = _client.GetStream();
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Bağlantı hatası: {ex.Message}");
                return false;
            }
        }

        public void Disconnect()
        {
            _stream?.Close();
            _client?.Close();
        }

        public void SendCommand(string command)
        {
            if (_client == null || !_client.Connected) return;

            try
            {
                // UR Script komutunu gönder
                byte[] data = Encoding.ASCII.GetBytes(command + "\n");
                _stream.Write(data, 0, data.Length);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Komut gönderme hatası: {ex.Message}");
            }
        }

        public byte[] ReceiveData()
        {
            if (_client == null || !_client.Connected) return null;

            try
            {
                byte[] buffer = new byte[4096];
                int bytesRead = _stream.Read(buffer, 0, buffer.Length);
                byte[] result = new byte[bytesRead];
                Array.Copy(buffer, result, bytesRead);
                return result;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Veri alma hatası: {ex.Message}");
                return null;
            }
        }
    }
    public class URRtdeCommunication
    {
        private TcpClient _client;
        private NetworkStream _stream;
        private readonly string _ipAddress;
        private readonly int _port = 30003; 

        public URRtdeCommunication(string ipAddress)
        {
            _ipAddress = ipAddress;
        }

        public bool Connect()
        {
            try
            {
                _client = new TcpClient(_ipAddress, _port);
                _client.ReceiveTimeout = 2000;
                _client.SendTimeout = 1000;
                _stream = _client.GetStream();

                // Protokol versiyonu anlaşması
                RequestProtocolVersion(2); // V2 protokolünü iste

                // Çıktı yapılandırması
                ConfigureOutputs(new[] { "actual_q", "actual_TCP_pose" });

                // Veri akışını başlat
                StartDataSync();

                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"RTDE Bağlantı hatası: {ex.Message}");
                return false;
            }
        }

        private void RequestProtocolVersion(int version)
        {
            using (MemoryStream ms = new MemoryStream())
            using (BinaryWriter writer = new BinaryWriter(ms))
            {
                writer.Write((ushort)3); // Paket boyutu
                writer.Write((byte)0); // REQUEST_PROTOCOL_VERSION komutu
                writer.Write((byte)version);

                byte[] requestData = ms.ToArray();
                _stream.Write(requestData, 0, requestData.Length);

                // Yanıtı bekle ve işle
                byte[] responseBuffer = new byte[4096];
                int bytesRead = _stream.Read(responseBuffer, 0, responseBuffer.Length);
                // Yanıt işleme kodu burada olacak
            }
        }

        private void ConfigureOutputs(string[] variableNames)
        {
            string variables = string.Join(",", variableNames);

            using (MemoryStream ms = new MemoryStream())
            using (BinaryWriter writer = new BinaryWriter(ms))
            {
                byte[] textBytes = System.Text.Encoding.ASCII.GetBytes(variables);

                writer.Write((ushort)(3 + textBytes.Length)); // Paket boyutu
                writer.Write((byte)2); // OUTPUT_SETUP komutu
                writer.Write(textBytes, 0, textBytes.Length);

                byte[] setupData = ms.ToArray();
                _stream.Write(setupData, 0, setupData.Length);

                // Yanıtı bekle ve işle
                byte[] responseBuffer = new byte[4096];
                int bytesRead = _stream.Read(responseBuffer, 0, responseBuffer.Length);
                // Yanıt işleme kodu burada olacak
            }
        }

        private void StartDataSync()
        {
            using (MemoryStream ms = new MemoryStream())
            using (BinaryWriter writer = new BinaryWriter(ms))
            {
                writer.Write((ushort)3); // Paket boyutu
                writer.Write((byte)7); // START komutu

                byte[] startData = ms.ToArray();
                _stream.Write(startData, 0, startData.Length);

                // Yanıtı bekle ve işle
                byte[] responseBuffer = new byte[4096];
                int bytesRead = _stream.Read(responseBuffer, 0, responseBuffer.Length);
                // Yanıt işleme kodu burada olacak
            }
        }

        public void Disconnect()
        {
            _stream?.Close();
            _client?.Close();
        }
    }
    public class URDashboardCommunication
    {
        private TcpClient _client;
        private NetworkStream _stream;
        private readonly string _ipAddress;
        private readonly int _port = 29999; // Dashboard portu

        public URDashboardCommunication(string ipAddress)
        {
            _ipAddress = ipAddress;
        }

        public bool Connect()
        {
            try
            {
                _client = new TcpClient(_ipAddress, _port);
                _stream = _client.GetStream();

                // Bağlantı sonrası hoşgeldin mesajını oku
                string welcome = ReceiveResponse();
                Console.WriteLine($"Dashboard bağlantısı: {welcome}");

                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Dashboard bağlantı hatası: {ex.Message}");
                return false;
            }
        }

        public void Disconnect()
        {
            _stream?.Close();
            _client?.Close();
        }

        public string SendCommand(string command)
        {
            if (_client == null || !_client.Connected) return null;

            try
            {
                byte[] data = Encoding.ASCII.GetBytes(command + "\n");
                _stream.Write(data, 0, data.Length);
                return ReceiveResponse();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Dashboard komut hatası: {ex.Message}");
                return null;
            }
        }

        private string ReceiveResponse()
        {
            byte[] buffer = new byte[4096];
            int bytesRead = _stream.Read(buffer, 0, buffer.Length);
            return Encoding.ASCII.GetString(buffer, 0, bytesRead);
        }

        // Temel komutlar
        public string GetRobotMode() => SendCommand("robotmode");
        public string PowerOn() => SendCommand("power on");
        public string PowerOff() => SendCommand("power off");
        public string ReleaseBrake() => SendCommand("brake release");
        public string LoadProgram(string programPath) => SendCommand($"load {programPath}");
        public string PlayProgram() => SendCommand("play");
        public string StopProgram() => SendCommand("stop");
        public string PauseProgram() => SendCommand("pause");
    }

}