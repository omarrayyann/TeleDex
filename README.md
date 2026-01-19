# <img src="https://github.com/user-attachments/assets/be6974a5-d5a7-4f97-8c6a-5468801dc360" width="75" align="center" alt="Logo">&nbsp;&nbsp;Teledex

[![PyPI version](https://img.shields.io/pypi/v/teledex)](https://pypi.org/project/teledex/) [![Downloads](https://static.pepy.tech/badge/mujoco_ar)](https://pepy.tech/project/mujoco_ar) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) 

teledex lets you control robot frames using your iOS device's AR data.

## Installation

You can install teledex package using pip:

```bash
pip install teledex
```

You can download the app from the [App Store](https://apps.apple.com/ae/app/mujoco-ar/id6612039501).

## Usage

### Basic Setup

```python
import teledex

session = teledex.Session()
session.start()

# Retrieve AR data
data = session.get_latest_data()
# Returns: {"position": (3,), "rotation": (3, 3), "button": bool, "toggle": bool}
```

### MuJoCo Setup

Control MuJoCo frames (body, geom, or site) with Teledex:

```python
import teledex

session = teledex.Session()

mujoco_handler = teledex.MujocoHandler(model=my_model, data=my_data)
session.add_handler(mujoco_handler)

mujoco_handler.link_body(name="eef_target")

session.start()
```

## Additional Functions

```python
session.vibrate(sharpness=0.8, intensity=0.4, duration=0.01)
session.pause_updates()
session.resume_updates()
session.reset_position()
```
