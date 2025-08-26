using System;
using System.Threading.Tasks;
using System.Windows.Markup;

namespace UniversalRobotWpf
{
    // Basit facade/sarmalayıcı
    public class UR_interface
    {
        private readonly MainViewModel _vm;

        // İsterseniz dışarıdan VM enjekte edin
        public UR_interface(MainViewModel viewModel)
        {
            _vm = viewModel ?? throw new ArgumentNullException(nameof(viewModel));
        }

        // İsterseniz parametresiz ctor ile kendi VM'inizi yaratın
        public UR_interface() : this(new MainViewModel())
        {
        }

        // Dışarıya VM'i de isterseniz expose edebilirsiniz (UI binding için)
        public MainViewModel ViewModel => _vm;

        // Bağlantı operasyonları
        public void Connect()
        {
            // MainViewModel içinde async void ConnectToRobot() var.
            // Direkt çağırıyoruz.
            _vm.ConnectToRobot();
        }

        public void Disconnect()
        {
            _vm.DisconnectFromRobot();
        }

        // Dashboard komutları
        public void SendDashboardCommand(string command)
        {
            _vm.SendCommand(command);
        }

        // Register işlemleri
        public Task<string> SetRegisterAsync(string regType, string regId, string regValue)
        {
            return _vm.SetRegCommand(regType, regId, regValue);
        }

        public Task<string> GetRegisterAsync(string variable)
        {
            return _vm.GetRegCommand(variable);
        }

        // Bazı durum/ayar alanlarını dışarı açmak isteyebilirsiniz
        public bool IsConnected { get => _vm.IsConnected; set => _vm.IsConnected = value; }
        public string RobotIp { get => _vm.RobotIp; set => _vm.RobotIp = value; }
        public string RobotMode { get => _vm.RobotMode; set => _vm.RobotMode = value; }
    
        public string ProgramState { get => _vm.ProgramState; set => _vm.ProgramState = value; }

    }
}