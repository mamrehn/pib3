# Installation Guide

This guide provides detailed instructions for installing the `pib-ik` library on Linux and Windows.

## Requirements

- **Python**: 3.8 or higher
- **pip**: Latest version recommended
- **Git**: For cloning the repository (optional)

## Quick Install

If you just want to get started quickly:

```bash
# in your existing python environment (see venv below) type:
pip install -U "pib-ik[all] @ git+https://github.com/mamrehn/pib_ik.git"
```

For a proper isolated installation, follow the detailed instructions below.

## Detailed Installation

### Linux (Ubuntu/Debian)

#### 1. Install Python (if not already installed)

```bash
# Check Python version
python3 --version

# Install Python if needed
sudo apt update
sudo apt install python3 python3-pip python3-venv
```

#### 2. Create a Project Directory

```bash
mkdir ~/pib_project
cd ~/pib_project
```

#### 3. Create a Virtual Environment

A virtual environment isolates your project dependencies from the system Python.

```bash
# Create virtual environment
python3 -m venv venv

# Activate it
source venv/bin/activate
```

Your prompt should now show `(venv)` at the beginning.

#### 4. Install pib-ik

Choose one of the following options:

```bash
# Basic installation
pip install git+https://github.com/mamrehn/pib_ik.git

# With image processing support (recommended)
pip install "pib-ik[image] @ git+https://github.com/mamrehn/pib_ik.git"

# With Swift visualization
pip install "pib-ik[viz] @ git+https://github.com/mamrehn/pib_ik.git"

# With real robot support (rosbridge)
pip install "pib-ik[robot] @ git+https://github.com/mamrehn/pib_ik.git"

# All features
pip install "pib-ik[all] @ git+https://github.com/mamrehn/pib_ik.git"
```

#### 5. Verify Installation

```bash
python -c "import pib_ik; print(f'pib-ik version: {pib_ik.__version__}')"
```

#### 6. Deactivate When Done

```bash
deactivate
```

#### 7. Reactivate Later

```bash
cd ~/pib_project
source venv/bin/activate
```

---

### Windows

#### 1. Install Python

1. Download Python from [python.org](https://www.python.org/downloads/)
2. Run the installer
3. **Important**: Check "Add Python to PATH" during installation
4. Click "Install Now"

Verify installation in Command Prompt or PowerShell:

```powershell
python --version
```

#### 2. Create a Project Directory

**Command Prompt:**
```cmd
mkdir %USERPROFILE%\pib_project
cd %USERPROFILE%\pib_project
```

**PowerShell:**
```powershell
mkdir $env:USERPROFILE\pib_project
cd $env:USERPROFILE\pib_project
```

#### 3. Create a Virtual Environment

```powershell
# Create virtual environment
python -m venv venv
```

#### 4. Activate the Virtual Environment

**Command Prompt:**
```cmd
venv\Scripts\activate.bat
```

**PowerShell:**
```powershell
# You may need to allow script execution first (run as Administrator):
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser

# Then activate:
.\venv\Scripts\Activate.ps1
```

Your prompt should now show `(venv)` at the beginning.

#### 5. Install pib-ik

```powershell
# Basic installation
pip install git+https://github.com/mamrehn/pib_ik.git

# With all features
pip install "pib-ik[all] @ git+https://github.com/mamrehn/pib_ik.git"
```

#### 6. Verify Installation

```powershell
python -c "import pib_ik; print(f'pib-ik version: {pib_ik.__version__}')"
```

#### 7. Deactivate When Done

```powershell
deactivate
```

#### 8. Reactivate Later

**Command Prompt:**
```cmd
cd %USERPROFILE%\pib_project
venv\Scripts\activate.bat
```

**PowerShell:**
```powershell
cd $env:USERPROFILE\pib_project
.\venv\Scripts\Activate.ps1
```

---

## Development Installation

For contributing to pib-ik or modifying the source code:

### Linux

```bash
# Clone the repository
git clone https://github.com/mamrehn/pib_ik.git
cd pib_ik

# Create and activate virtual environment
python3 -m venv venv
source venv/bin/activate

# Install in editable mode with dev dependencies
pip install -e ".[dev]"

# Or with all optional dependencies
pip install -e ".[all,dev]"
```

### Windows

```powershell
# Clone the repository
git clone https://github.com/mamrehn/pib_ik.git
cd pib_ik

# Create and activate virtual environment
python -m venv venv
.\venv\Scripts\Activate.ps1

# Install in editable mode
pip install -e ".[all,dev]"
```

---

## Installation Options

The package provides several optional dependency groups:

| Option | Dependencies | Use Case |
|--------|--------------|----------|
| `image` | scikit-image | Advanced image processing |
| `viz` | swift-sim, spatialgeometry | 3D visualization in browser |
| `robot` | roslibpy | Real robot connection via rosbridge |
| `dev` | pytest, black, ruff, mypy | Development and testing |
| `all` | All of the above (except dev) | Full functionality |

Install multiple options:

```bash
pip install "pib-ik[image,viz] @ git+https://github.com/mamrehn/pib_ik.git"
```

---

## Troubleshooting

### "pip: command not found"

Use `pip3` instead of `pip`, or install pip:

```bash
# Linux
sudo apt install python3-pip

# Or use python module directly
python3 -m pip install ...
```

### "python: command not found" (Windows)

- Reinstall Python and ensure "Add Python to PATH" is checked
- Or use the full path: `C:\Users\YourName\AppData\Local\Programs\Python\Python3x\python.exe`

### Permission errors on Linux

Don't use `sudo pip install`. Instead, use a virtual environment (recommended) or:

```bash
pip install --user git+https://github.com/mamrehn/pib_ik.git
```

### PowerShell script execution error

Run PowerShell as Administrator and execute:

```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

### SSL/Certificate errors

```bash
pip install --trusted-host pypi.org --trusted-host files.pythonhosted.org git+https://github.com/mamrehn/pib_ik.git
```

### "No module named 'pib_ik'"

Make sure your virtual environment is activated:

```bash
# Linux
source venv/bin/activate

# Windows (PowerShell)
.\venv\Scripts\Activate.ps1
```

### Swift visualization not working

Install with visualization support:

```bash
pip install "pib-ik[viz] @ git+https://github.com/mamrehn/pib_ik.git"
```

### Cannot connect to robot

Install with robot support:

```bash
pip install "pib-ik[robot] @ git+https://github.com/mamrehn/pib_ik.git"
```

Then ensure:
1. Robot is powered on and connected to the network
2. Rosbridge is running on the robot
3. You can ping the robot IP

---

## Upgrading

To upgrade to the latest version:

```bash
pip install --upgrade git+https://github.com/mamrehn/pib_ik.git
```

## Uninstalling

```bash
pip uninstall pib-ik
```

---

## Next Steps

- [README](README.md) - Quick start guide and API overview
- [Calibration Guide](CALIBRATION.md) - Calibrate joint limits for your robot
