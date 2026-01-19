# Simple Phone Pose Retrieval Setup (Teleop)

This guide explains how to run Teledex for retrieving pose data directly from your iOS device without requiring MuJoCo. Follow the steps below to set it up and get started in a few minutes.

![1127(3)](https://github.com/user-attachments/assets/77587d66-b9c7-4f0e-b7ae-97da8e7719f4)

---

## Installation

1. **Download the iOS App:**  
   Install the MuJoCo AR app from the App Store [here](https://apps.apple.com/ae/app/mujoco-ar/id6612039501).

2. **Install the Python Package:**  
   Run the following command to install the Teledex Python package:

   ```bash
   pip install teledex
   ```

---

## Usage: Retrieve Pose Data Without MuJoCo

### Step 1: Initialize the Connector

Create a connection between your Python program and the iOS device.

```python
from teledex import Session

# Initialize the connector
connector = Session(port=8888, debug=False)  # Customize the port if necessary
```

---

### Step 2: Start the Connection

Begin listening for data from your iOS device.

```python
connector.start()
```

---

### Step 3: Connect Your iOS Device

1. Open the **MuJoCo AR** app on your iOS device.
2. Enter the IP address and port displayed in your Python console.
3. Tap "Connect" on the app.

---

### Step 4: Retrieve Pose Data

Once connected, retrieve the latest position, rotation, button, and toggle states from the iOS device:

```python
data = connector.get_latest_data()
print(data)
# Output: {"position": [x, y, z], "rotation": [[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]], "button": True/False, "toggle": True/False}
```

---


### Latency Issues
- If you experience latency, connect your PC to your device's mobile hotspot. This establishes a local WebSocket connection, reducing delay.

### Inaccurate Tracking
- Avoid blocking the camera with your fingers.
- Adjust the app's tracking sensitivity through the `scale` parameter if using MuJoCo features later.

---

 **How can I quickly test it in simulation?**  
   Run a script using the provided example `demo/flexible_setup.py` to test device connectivity and data retrieval.

---
