# <img src="https://github.com/user-attachments/assets/be6974a5-d5a7-4f97-8c6a-5468801dc360" width="75" align="center" alt="Logo">&nbsp;&nbsp;Teledex

[![PyPI version](https://img.shields.io/pypi/v/teledex)](https://pypi.org/project/teledex/) [![Downloads](https://static.pepy.tech/badge/teledex)](https://pepy.tech/project/teledex) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) 

teledex lets you control robot frames and hands using your iOS device's AR data.

<p align="left">
  <img src="media/oranges_43.gif" width="24%" alt="Oranges">
  <img src="media/react1_43.gif" width="24%" alt="Reactive">
  <img src="media/coffee_43.gif" width="24%" alt="Coffee">
  <img src="media/yuchen_43.gif" width="24%" alt="Handover">
  <img src="media/batteries_43.gif" width="24%" alt="Batteries">
  <img src="media/wipes_43.gif" width="24%" alt="Wipes">
  <img src="media/notes_43.gif" width="24%" alt="Notes">
  <img src="media/precise_43.gif" width="24%" alt="Precise manipulation">
</p>

## Installation

You can install teledex package using pip:

```bash
pip install teledex
```

You can download the app from the [App Store](https://apps.apple.com/ae/app/mujoco-ar/id6612039501).

## Before You Get Started
You can use Teledex in phone-only mode (streams only the device pose) or with an optional 3D-printed holder to track finger motion while teleoperating. All printable parts are available in [`printables`](printables) folder.

See [**ASSEMBLY_GUIDE**](./printables/README.md) for assembly notes and [**SETUP_GUIDE**](GUIDE.md) to get started with your phone and robot setup.
 

## Basic Usage

### Basic Setup

```python
import teledex

session = teledex.Session()
session.start()

# Retrieve AR data
data = session.get_latest_data()
# Returns: {"position": (3,), "rotation": (3, 3), "button": bool, "button_secondary": bool, # "toggle": bool}
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

### MuJoCo Studio Setup

Drive a mocap body in [MuJoCo Studio](https://mujoco.readthedocs.io/en/latest/studio/index.html) and get a TeleDex panel (QR code, connection status, live pose, pause/recenter) inside the viewer. Requires a mujoco build that ships `mujoco.studio`.

```python
from mujoco.studio import launch_passive, viewer_app, viewer_protocol
from teledex.studio import TeleDexSim, TeleDexPanel

launch_passive.run(
    viewer_protocol.ViewerConfig(gfx="web", open_browser=True),
    model_path="scene.xml",
    plugins=[viewer_app.ViewerApp(), *viewer_app.SIM_PLUGINS,
             TeleDexSim(body="target", scale=1.5), TeleDexPanel()],
)
```

Or from the command line, driving the model's first mocap body:

```bash
python -m mujoco.studio.viewer --model=demos/studio_scene.xml --gfx=web --plugins=demos/studio_plugin.py
```

`TeleDexSim(mode="delta", max_dpos=0.02)` follows pose changes with a per-step cap instead of the absolute pose. See [`demos/studio_demo.py`](demos/studio_demo.py).

## Additional Functions

```python
session.vibrate(sharpness=0.8, intensity=0.4, duration=0.01)
session.pause_updates()
session.resume_updates()
session.reset_position()
```

## Acknowledgment

The CAD is based on the models from [ACE Teleop](https://ace-teleop.github.io).

## Citation

If you use Teledex in your research, please cite:

```bibtex
@article{rayyan2026teledex,
  title={TeleDex: Accessible Dexterous Teleoperation},
  author={Rayyan, Omar and Gilles, Maximilian and Cui, Yuchen},
  journal={arXiv preprint arXiv:2603.17065},
  year={2026}
}
