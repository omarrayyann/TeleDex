# <img src="https://github.com/user-attachments/assets/9b731c7f-7ad1-4607-90aa-f6ff1830a936" width="50" align="center" alt="Logo">&nbsp;&nbsp;Teledex

[![PyPI version](https://img.shields.io/pypi/v/teledex)](https://pypi.org/project/teledex/) [![Downloads](https://static.pepy.tech/badge/teledex)](https://pepy.tech/project/teledex) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) 

teledex is a plugin for [MuJoCo](https://github.com/google-deepmind/mujoco) that lets you control frames using your iOS/Android device's AR data.

**Recent Updates**

-  Android support thanks to [@Lr-2002](https://github.com/Lr-2002/arcore-android-sdk)'s contribution.
- Trigger vibrations on your iOS device for haptic feedback.

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

### Quick MuJoCo Setup

This setup allows you to directly control a MuJoCo frame (body, geom, or site), with the frame's position and orientation matching the ARKit data received from the connected iOS device.

```python
from teledex import TeledexConnector

# Initialize the connector with your desired parameters
mj_ar = TeledexConnector(mujoco_model=my_model,mujoco_data=my_data)

# Link a MuJoCo frame (link_body(), link_geom() or link_site())
mj_ar.link_body(name="eef_target")

# Start the connector
mj_ar.start()
```
### Full MuJoCo Setup

In addition to what the quick setup allows you to do, this setup allows you to automate the applying of a translation, rotation or scaling of the recieved pose. Additionally, you can pass functions to button_fn and toggle_fn to be triggered when the button or toggle are activated

```python
from teledex import TeledexConnector

# Initialize the connector with your desired parameters
mj_ar = TeledexConnector(
    mujoco_model=my_model, 
    mujoco_data=my_data, 
    port=8888,                                           # Optional, defaults to 8888 if not provided
    debug=False                                          # Optional, defaults to False if not provided
)

# Link a MuJoCo frame (link_body(), link_geom() or link_site())
mj_ar.link_body(
    name="eef_target",
    scale=1.0,                                           # Optional, defaults to 1.0 if not provided
    position_origin=np.array([0.0, 0.0, 0.0]),           # Optional, defaults to [0, 0, 0] if not provided
    rotation_origin=np.identity(3),                      # Optional, defaults to I(3) if not provided
    toggle_fn=my_toggle_function,                        # Optional, calls nothing if not provided
    button_fn=my_button_function,                        # Optional, calls nothing if not provided
    disable_pos=False,                                   # Optional, defaults to False if not provided
    disable_rot=False                                    # Optional, defaults to False if not provided
)

# Start the connector
mj_ar.start()
```

### Flexible Setup (works without MuJoCo):

You can retrieve the ARKit data including the position, rotation, button, and toggle states directly from a connected iOS device, making it flexible for usage in various applications beyond physics simulations. Try running ```mjpython demos/flexible_setup.py```.

```python
from teledex import TeledexConnector

# Initialize the connector with your desired parameters
connector = TeledexConnector()

# Start the connector
connector.start()

# Retrieve the latest AR data (after connecting the iOS device, see the guide below)
data = connector.get_latest_data()  # Returns {"position": (3, 1), "rotation": (3, 3), "button": bool, "toggle": bool}
```
## Additional Functions

```python
connector.vibrate(sharpness=0.8, intensity=0.4, duration=0.01) # Trigger a vibration on the connected device
connector.pause_updates()  # Temporarily stops receiving updates from the connected device.
connector.resume_updates() # Resumes receiving updates from the connected device.
connector.reset_position() # Resets the current position as the origin (0,0,0).

```

## FAQ

#### How can I reduce latency?

- If you're experiencing latency, try connecting your PC to your device's hotspot. This should significantly reduce latency if you're far from a router since the communication happens locally via WebSockets.
  
#### Can I use it for a non-MuJoCo application?

- Yes, check the [Flexible Setup](#flexible-setup-works-without-mujoco) out where you can retrive the pure ARKit position and rotation and use it as you wish. You wouldn't need to pass in the MuJoCo model and data in such a case.

## How to contribute?

Report any bugs you encounter to the issue tracker. Also, feel free to suggest any features. Those could be regarding the app (iOS/Android apps) or the python package.

## Acknowledgements

- Thanks to [@Lr-2002](https://www.github.com/Lr-2002) for creating an [Android app](https://github.com/Lr-2002/arcore-android-sdk/tree/main?tab=readme-ov-file) that works with the package.
- Thanks to  [@kevinzakka](https://www.github.com/kevinzakka) for the [mink](https://github.com/kevinzakka/mink) and [mjctrl](https://github.com/kevinzakka/mjctrl) libraries, which are used in the provided demos.

## Citation

If you use teledex in your research, please cite it as follows:

```bibtex
@software{rayyan2024teledex,
  author       = {Rayyan, Omar},
  title        = {{Teledex}: Phone Teleoperation for Robots},
  version      = {1.3.0},
  year         = {2024},
  month        = sep,
  doi          = {10.5281/zenodo.17111581},
  url          = {https://github.com/omarrayyann/teledex},
  license      = {MIT},
}
```



