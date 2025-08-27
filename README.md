# UniversalRobotWpf

A Windows Presentation Foundation (WPF) application written in C# for interacting with Universal Robots (UR) cobots. The app provides a simple UI to connect to a robot over the network, send commands, and lay the groundwork for building richer operator/control panels.

> Safety first: Always test with the robot in a safe environment. Ensure proper safety configurations and clear workspace before commanding motion.

---

## Features

- WPF desktop UI (XAML) for Windows
- Basic connection workflow to a UR robot over TCP/IP
- Separation of UI, logic, and view-models
- Extensible command structure 

---

## Repository Structure

- `UniversalRobotWpf.sln` — Solution file
- `UniversalRobotWpf.csproj` — Project file
- `App.xaml`, `App.xaml.cs` — App bootstrap
- `MainWindow.xaml`, `MainWindow.xaml.cs` — Main UI window
- `UR_interface.cs` — Core interface logic for communicating with the robot
- `ViewModel/` — ViewModel layer for MVVM patterns
- `COMMAND/` 
- `Properties/` — WPF project resources and settings
- `App.config` — Application configuration (e.g., default connection settings)
- `.gitattributes`, `.gitignore`
- `LICENSE.txt` — MIT License

---

## Getting Started

### Prerequisites

- Windows 10/11
- Visual Studio 2019 or 2022 (Community or higher)
- .NET Framework/WPF tooling (installed with Visual Studio)
- Network access to a Universal Robots controller

> Note: This project uses the classic WPF project style. If prompted, let Visual Studio restore any NuGet packages.

### Build and Run

1. Clone the repository:
   ```bash
   git clone https://github.com/DemirhanCakir/UniversalRobotWpf.git
   ```
2. Open `UniversalRobotWpf.sln` in Visual Studio.
3. Set `UniversalRobotWpf` as the startup project (if not already).
4. Build the solution (Ctrl+Shift+B).
5. Run (F5).

---

## Configuration

You can set default connection parameters (e.g., robot IP, ports) in one of these places:
- In the UI at runtime (recommended for quick testing)
- In `App.config` (for persistent defaults)

Common UR network endpoints you might use:
- Dashboard Server: 29999 (system-level commands)
- Primary/Secondary URScript interfaces: 30001/30002/30003
- RTDE: 30004

Which endpoints you use depends on the commands you plan to send and the data you want to read. Ensure the robot’s firewall/network settings allow the connection.

---

## Usage

1. Launch the application.
2. Enter the robot controller’s IP address.
3. Connect to the robot.
4. Use the UI to send commands or load scripts.
5. Observe responses/status in the UI.

## Architecture

- WPF (XAML) front-end (`MainWindow.xaml`) provides a simple operator panel.
- `ViewModel/` contains presentation logic for MVVM separation.
- `UR_interface.cs` encapsulates communication logic to UR controller, making it easier to:
  - Open/close connections
  - Send URScript or dashboard commands
  - Extend with logging, retry, and richer error handling

This separation keeps UI responsive and logic testable.

---


## Contributing

1. Fork the repo and create a feature branch.
2. Follow existing code style and keep UI responsive.
3. Submit a pull request with a clear description and any screenshots of UI changes.

---

## License

This project is licensed under the MIT License — see `LICENSE.txt` for details.

---

## Disclaimer

This project is not affiliated with Universal Robots. Use at your own risk. Ensure compliance with all safety standards and robot manufacturer guidelines.
