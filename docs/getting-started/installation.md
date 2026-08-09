# Installation

## Requirements

- **Python**: 3.10–3.13, 64-bit. Not 3.14 — `roboticstoolbox-python` publishes no
  wheels for it yet, so pip would try to compile it from source.
- **pip**: Latest version recommended
- **Git**: Required, since the package is installed from a Git URL

## Quick Install

```bash
pip install -U "pib3 @ git+https://github.com/mamrehn/pib3.git"
```

---

## Linux

```bash
# Create project directory
mkdir ~/pib_project && cd ~/pib_project

# Create and activate virtual environment
python3 -m venv venv
source venv/bin/activate

# Install pib3
pip install "pib3 @ git+https://github.com/mamrehn/pib3.git"

# Verify
python -c "import pib3; print(f'pib3 {pib3.__version__}')"

# Deactivate when done
deactivate
```

## Windows

1. Install the latest **3.13.x** *Windows installer (64-bit)* from the
   [Windows downloads page](https://www.python.org/downloads/windows/) - check
   **"Add python.exe to PATH"**. Do not take the default 3.14 download from the
   python.org front page; see [Windows prerequisites](../../README.md#windows-prerequisites).
2. Install [Git for Windows](https://git-scm.com/download/win)
3. Open a **new** PowerShell window:

```powershell
# Create project directory
mkdir $env:USERPROFILE\pib_project
cd $env:USERPROFILE\pib_project

# Create and activate virtual environment (py -3.13 pins the interpreter)
py -3.13 -m venv venv
.\venv\Scripts\Activate.ps1

# Install pib3
pip install "pib3 @ git+https://github.com/mamrehn/pib3.git"

# Verify
python -c "import pib3; print(f'pib3 {pib3.__version__}')"
```

!!! tip "PowerShell Script Error?"
    Run as Administrator: `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`

---

## Installation Options

Everything needed for **image processing, trajectory generation, the Webots backend and
the real robot** is installed by default — there is no extra to add for those. Only two
optional groups exist:

| Option | Dependencies | Use Case |
|--------|--------------|----------|
| `sim` | ultralytics, mediapipe | Run the OAK-D-equivalent AI models on simulated Webots frames |
| `dev` | pytest, pytest-cov, black, ruff, mypy | Development/testing |

```bash
# Simulated AI perception in Webots
pip install "pib3[sim] @ git+https://github.com/mamrehn/pib3.git"
```

!!! warning "Older docs mention `[robot]`, `[image]`, `[viz]` or `[all]`"
    Those groups no longer exist. `roslibpy` (robot) and `opencv-python-headless` (image)
    are core dependencies now. pip only warns about an unknown extra rather than failing,
    so such a command appears to work while silently doing nothing extra.

---

## Development Installation

```bash
git clone https://github.com/mamrehn/pib3.git
cd pib3
python3 -m venv venv          # Windows: py -3.13 -m venv venv
source venv/bin/activate      # Windows: .\venv\Scripts\Activate.ps1
pip install -e ".[sim,dev]"
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `pip: command not found` | Use `pip3` or `python3 -m pip install` |
| `No module named 'pib3'` | Activate venv: `source venv/bin/activate` |
| Permission errors (Linux) | Use venv or `pip install --user` |
| `Package 'pib3' requires a different Python` | You are outside 3.10–3.13; see [Windows prerequisites](../../README.md#windows-prerequisites) |
| `Cannot find command 'git'` | Install Git and reopen the terminal |
| `Microsoft Visual C++ 14.0 or greater is required` | Wrong Python version, not a missing build tool — see [Windows prerequisites](../../README.md#windows-prerequisites) |
| PowerShell script error | `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser` |
| SSL/Certificate errors | Add `--trusted-host pypi.org --trusted-host files.pythonhosted.org` |
| Robot connection fails | Check that rosbridge is running and reachable (no extra install needed) |

---

## Upgrading / Uninstalling

```bash
pip install --upgrade git+https://github.com/mamrehn/pib3.git
pip uninstall pib3
```

---

**Next:** [Quick Start](quickstart.md) | [Calibration](calibration.md)
