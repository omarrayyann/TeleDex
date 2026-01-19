# <img src="https://github.com/user-attachments/assets/9b731c7f-7ad1-4607-90aa-f6ff1830a936" width="50" align="center" alt="Logo">&nbsp;&nbsp;Teledex

[![PyPI version](https://img.shields.io/pypi/v/teledex)](https://pypi.org/project/teledex/) [![Downloads](https://static.pepy.tech/badge/mujoco_ar)](https://pepy.tech/project/mujoco_ar) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) 

teledex lets you control robot frames using your iOS device's AR data.

**MuJoCo Demos**

<table>
  <tr>
    <th colspan="4">
         Position Control
      </th>
  </tr>
  <tr>
    <th colspan="2">
          <a href="https://github.com/omarrayyann/mujoco_fruit_picking" target="_blank">MuJoCo Fruits Pick and Place</a>
      </th>
      <th colspan="2">
          <a href="https://github.com/omarrayyann/mujoco_pusht" target="_blank">MuJoCo PushT</a>
      </th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/3d496ce1-0b5d-4a1f-a6d2-dc2e19d1e3d8"  /></td>
    <td><img src="https://github.com/user-attachments/assets/8fd2b0ae-f90a-4df5-b114-3feac7c87e37" /></td>
    <td><img src="https://github.com/user-attachments/assets/c1e927c5-a4af-4c95-a6d0-fe7f8a026c34"  /></td>
    <td><img src="https://github.com/user-attachments/assets/a58ed764-4e05-40a5-b26a-5bd896584f34"/></td>
  </tr>
  <tr>
    <th colspan="4">
         Position and Rotation Control
      </th>
  </tr>
   <tr>
    <th colspan="2">
          <a href="https://github.com/omarrayyann/mujoco_study_desk" target="_blank">MuJoCo Study Desk</a>
      </th>
      <th colspan="2">
          <a href="https://github.com/omarrayyann/mujoco_blocks_stacking" target="_blank">MuJoCo Blocks Stacking</a>
      </th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/e70569ce-5ade-4161-95ab-007b1d612e0a"  /></td>
    <td><img src="https://github.com/user-attachments/assets/88635d5e-63f3-41b3-af83-3af03588c84f"  /></td>
    <td><img src="https://github.com/user-attachments/assets/dbb1dbb7-5dff-4c24-88fb-9f4b8afd7d8b"  /></td>
    <td><img src="https://github.com/user-attachments/assets/df43bb40-6e58-4e94-8d1c-a4fa90359d65"  /></td>
  </tr>
</table>


**Real Demo**

![1127(3)](https://github.com/user-attachments/assets/9b738682-6c7c-4aad-bf5d-de39bd114780)

Examples of Teledex linked to the end-effectors of multiple manipulators can be found in this fork of [Mink](https://github.com/omarrayyann/mink-teledex). 

## Installation

You can install teledex package using pip:

```bash
pip install teledex
```

You can download the app from the [App Store](https://apps.apple.com/ae/app/mujoco-ar/id6612039501) for iOS devices or [here](https://github.com/Lr-2002/arcore-android-sdk/tree/main?tab=readme-ov-file) for Android devices.

## Usage

### Basic Setup

Stream AR pose data from your phone to any application:

```python
from teledex import Session

session = Session()
session.start()

# Retrieve the latest AR data
data = session.get_latest_data()  # {"position": (3,), "rotation": (3, 3), "button": bool, "toggle": bool}
```

### MuJoCo Setup

Control MuJoCo frames (body, geom, or site) with AR data:

```python
from teledex import Session, MujocoHandler

session = Session()

mujoco_handler = MujocoHandler(model=my_model, data=my_data)
session.add_handler(mujoco_handler)

mujoco_handler.link_body(name="eef_target")

session.start()
```

## Additional Functions

```python
session.vibrate(sharpness=0.8, intensity=0.4, duration=0.01) # Trigger a vibration on the connected device
session.pause_updates()  # Temporarily stops receiving updates from the connected device.
session.resume_updates() # Resumes receiving updates from the connected device.
session.reset_position() # Resets the current position as the origin (0,0,0).
```
