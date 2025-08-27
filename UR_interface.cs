using System;
using System.Threading.Tasks;
using System.Windows.Input;

namespace UniversalRobotWpf
{
    // Simple facade/wrapper
    public class UR_interface
    {
        private readonly MainViewModel _vm;

        public UR_interface(MainViewModel viewModel)
        {
            _vm = viewModel ?? throw new ArgumentNullException(nameof(viewModel));
        }

        public UR_interface() : this(new MainViewModel())
        {
        }

        public MainViewModel ViewModel => _vm;

        // Connection operations (optional direct usage)
        public void Connect() => _vm.ConnectToRobot();
        public void Disconnect() => _vm.DisconnectFromRobot();

        // Dashboard commands (optional direct usage)
        public void SendDashboardCommand(string command) => _vm.SendCommand(command);

        // Register operations (method forward)
        public Task<string> SetRegCommand() => _vm.SetRegCommand();
        public Task<string> GetRegCommand() => _vm.GetRegCommand();

        // Async aliases (for external usage if any)
        public Task<string> SetRegisterAsync(string regType, string regId, string regValue) => _vm.SetRegCommand(regType, regId, regValue);
        public Task<string> GetRegisterAsync(string variable) => _vm.GetRegCommand(variable);

        // Commands used in XAML (forward)
        public ICommand ConnectCommand => _vm.ConnectCommand;
        public ICommand DisconnectCommand => _vm.DisconnectCommand;
        public ICommand PlayCommand => _vm.PlayCommand;
        public ICommand PauseCommand => _vm.PauseCommand;
        public ICommand StopCommand => _vm.StopCommand;
        public ICommand PowerOnCommand => _vm.PowerOnCommand;
        public ICommand PowerOffCommand => _vm.PowerOffCommand;
        public ICommand BreakeReleaseCommand => _vm.BreakeReleaseCommand;
        public ICommand SetRegisterCommand => _vm.SetRegisterCommand;
        public ICommand GetRegisterCommand => _vm.GetRegisterCommand;

        // Properties used in XAML (forward)
        public bool IsConnected { get => _vm.IsConnected; set => _vm.IsConnected = value; }

        public string RobotIp { get => _vm.RobotIp; set => _vm.RobotIp = value; }
        public string ConnectionStatus { get => _vm.ConnectionStatus; set => _vm.ConnectionStatus = value; }
        public string RobotMode { get => _vm.RobotMode; set => _vm.RobotMode = value; }
        public string ProgramState { get => _vm.ProgramState; set => _vm.ProgramState = value; }

        public string PoseData { get => _vm.PoseData; set => _vm.PoseData = value; }
        public string JointData { get => _vm.JointData; set => _vm.JointData = value; }
        public string LogMessages { get => _vm.LogMessages; set => _vm.LogMessages = value; }

        // Digital outputs (forward)
        public bool DigitalOut0 { get => _vm.DigitalOut0; set => _vm.DigitalOut0 = value; }
        public bool DigitalOut1 { get => _vm.DigitalOut1; set => _vm.DigitalOut1 = value; }
        public bool DigitalOut2 { get => _vm.DigitalOut2; set => _vm.DigitalOut2 = value; }
        public bool DigitalOut3 { get => _vm.DigitalOut3; set => _vm.DigitalOut3 = value; }
        public bool DigitalOut4 { get => _vm.DigitalOut4; set => _vm.DigitalOut4 = value; }
        public bool DigitalOut5 { get => _vm.DigitalOut5; set => _vm.DigitalOut5 = value; }
        public bool DigitalOut6 { get => _vm.DigitalOut6; set => _vm.DigitalOut6 = value; }
        public bool DigitalOut7 { get => _vm.DigitalOut7; set => _vm.DigitalOut7 = value; }

        public bool ConfigOut0 { get => _vm.ConfigOut0; set => _vm.ConfigOut0 = value; }
        public bool ConfigOut1 { get => _vm.ConfigOut1; set => _vm.ConfigOut1 = value; }
        public bool ConfigOut2 { get => _vm.ConfigOut2; set => _vm.ConfigOut2 = value; }
        public bool ConfigOut3 { get => _vm.ConfigOut3; set => _vm.ConfigOut3 = value; }
        public bool ConfigOut4 { get => _vm.ConfigOut4; set => _vm.ConfigOut4 = value; }
        public bool ConfigOut5 { get => _vm.ConfigOut5; set => _vm.ConfigOut5 = value; }
        public bool ConfigOut6 { get => _vm.ConfigOut6; set => _vm.ConfigOut6 = value; }
        public bool ConfigOut7 { get => _vm.ConfigOut7; set => _vm.ConfigOut7 = value; }

        // Register fields in MainViewModel (forward)
        public string RegType { get => _vm.RegType; set => _vm.RegType = value; }
        public string RegId { get => _vm.RegId; set => _vm.RegId = value; }
        public string RegValue { get => _vm.RegValue; set => _vm.RegValue = value; }
        public string Variable { get => _vm.Variable; set => _vm.Variable = value; }

        // XAML aliases (to match existing XAML names exactly)
        public string SetRegisterType { get => _vm.RegType; set => _vm.RegType = value; }
        public string SetRegisterId { get => _vm.RegId; set => _vm.RegId = value; }
        public string SetRegisterValue { get => _vm.RegValue; set => _vm.RegValue = value; }
        public string GetRegisterId { get => _vm.Variable; set => _vm.Variable = value; }
    }
}