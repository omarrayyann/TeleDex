import asyncio
import websockets
import json
import numpy as np
import os
import signal
import socket
import threading
import platform
import subprocess
from urllib.parse import urlencode
from scipy.spatial.transform import Rotation as R

try:
    import qrcode
    HAS_QRCODE = True
except ImportError:
    HAS_QRCODE = False

APP_CLIP_URL = "https://orayyan.com/teledex/clip/"


class Session:
    """
    A connector to receive position and rotation data from a connected AR application.
    Supports modular handlers that get called on each data update.
    """

    def __init__(self, port=8888, debug=False, show_qr=True, hand_offset=0.26, app_clip=False, delta=False):
        """
        Initialize the session.

        Args:
            port (int): The port on which the connector listens.
            debug (bool): Enable debug mode for verbose output.
            show_qr (bool): Whether to display QR code in terminal on startup.
            hand_offset (float): Distance in meters along the phone's local Z axis
                                 from the phone pose to the hand pose.
            app_clip (bool): Encode the QR code as a TeleDex App Clip link, so scanning it with
                             the iPhone Camera opens TeleDex and connects without installing the app.
                             Requires app version 2.2+ when scanning from inside the app.
            delta (bool): Track pose changes for relative controllers (OSC, velocity, delta IK).
                          Read them with `get_delta()` from your control loop.
        """
        self.ip_address = self._get_local_ip()
        self.port = port
        self.show_qr = show_qr
        self.app_clip = app_clip
        self.hand_offset = hand_offset
        self.latest_data = {
            "rotation": None,
            "position": None,
            "finger_angles": None,
            "button": None,
            "toggle": None,
            "position_hand": None,
            "rotation_hand": None,
            "landmarks": None,
            "world_landmarks": None,
        }
        self.server = None
        self.ping_interval = 20000000000
        self.ping_timeout = 10000000000
        self.connected_clients = set()
        self.debug = debug
        self.reset_position_values = np.array([0.0, 0.0, 0.0])
        self.get_updates = True
        self.position_limits = None

        # Delta tracking (opt-in): deltas are measured from an anchor pose that only advances
        # when get_delta() is called, so dropped messages or slow control loops never lose motion.
        self.delta = delta
        self.delta_jump_position = 0.1  # meters between consecutive messages treated as a discontinuity
        self.delta_jump_rotation = 0.6  # radians between consecutive messages treated as a discontinuity
        self._delta_lock = threading.Lock()
        self._delta_anchor = None  # (position, rotation) already reported through get_delta()
        self._delta_latest = None  # latest (position, rotation) received
        self._delta_rebase = False  # exclude the change into the next message (after reset/resume/connect)
        
        # Modular handler system
        self._handlers = []
        self._on_update_callbacks = []
        self._on_connect_callbacks = []
        self._on_disconnect_callbacks = []

    def add_handler(self, handler):
        """
        Add a handler that will be updated on each data frame.
        
        Handlers should have an `update(session, data)` method that gets called
        when new data arrives.
        
        Args:
            handler: An object with an `update(session, data)` method.
        """
        self._handlers.append(handler)
        return handler
    
    def on_update(self, callback):
        """
        Register a callback function to be called on each data update.
        
        Args:
            callback: A function that takes (session, data) as arguments.
        """
        self._on_update_callbacks.append(callback)
    
    def on_connect(self, callback):
        """
        Register a callback function to be called when a device connects.
        
        Args:
            callback: A function that takes (session) as argument.
        """
        self._on_connect_callbacks.append(callback)
    
    def on_disconnect(self, callback):
        """
        Register a callback function to be called when a device disconnects.
        
        Args:
            callback: A function that takes (session) as argument.
        """
        self._on_disconnect_callbacks.append(callback)

    def _get_local_ip(self):
        """
        Get the local IP address of this machine.
        Tries multiple methods to handle different network configurations.
        """
        # Method 1: Try connecting to an external address (works with internet)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(1)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except (OSError, socket.timeout):
            pass
        
        # Method 2: Try connecting to a local broadcast address (works without internet)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(1)
            s.connect(("10.255.255.255", 1))
            ip = s.getsockname()[0]
            s.close()
            if ip != "0.0.0.0":
                return ip
        except (OSError, socket.timeout):
            pass
        
        # Method 3: Get IP from hostname
        try:
            hostname = socket.gethostname()
            ip = socket.gethostbyname(hostname)
            if ip != "127.0.0.1":
                return ip
        except socket.error:
            pass
        
        # Method 4: Parse network interfaces (macOS/Linux)
        if platform.system() != "Windows":
            try:
                import subprocess
                result = subprocess.run(["ifconfig"], capture_output=True, text=True)
                for line in result.stdout.split('\n'):
                    if 'inet ' in line and '127.0.0.1' not in line:
                        parts = line.strip().split()
                        idx = parts.index('inet') + 1
                        if idx < len(parts):
                            ip = parts[idx]
                            if ip.startswith(('192.168.', '10.', '172.')):
                                return ip
            except Exception:
                pass
        
        # Fallback
        return "127.0.0.1"

    def _print_qr_code(self):
        """
        Print a QR code to the terminal containing the connection URL.
        """
        if not HAS_QRCODE:
            print("[INFO] Install 'qrcode' package for QR code display: pip install qrcode")
            return
        
        connection_url = self.connection_link() if self.app_clip else f"{self.ip_address}:{self.port}"
        
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            box_size=1,
            border=1,
        )
        qr.add_data(connection_url)
        qr.make(fit=True)
        
        if self.app_clip:
            print(f"\n[INFO] Scan this QR code with your iPhone Camera to open TeleDex ({connection_url}):\n")
        else:
            print(f"\n[INFO] Scan this QR code to connect ({connection_url}):\n")
        qr.print_ascii(invert=True)
        print()

    def connection_link(self):
        """
        Get the App Clip link that opens TeleDex and connects it to this session.
        """
        return f"{APP_CLIP_URL}?{urlencode({'ip': self.ip_address, 'port': self.port})}"

    async def _stop_server(self):
        """
        Stop the WebSocket server and close all active connections.
        """
        if self.server is not None:
            for websocket in list(self.connected_clients):
                try:
                    await websocket.close()
                except Exception:
                    pass

            self.server.close()
            await self.server.wait_closed()
            self.server = None
            print("[INFO] Session Stopped")
            asyncio.get_running_loop().stop()

    def stop(self):
        """
        Stop the Session: shut down the server, then join the thread.
        """
        self.loop.call_soon_threadsafe(
            lambda: asyncio.create_task(self._stop_server())
        )
        if hasattr(self, "_thread"):
            self._thread.join()
            
    def reset_position(self):
        """
        Reset the position to the current position, treating it as (0,0,0).
        """
        if self.latest_data["position"] is not None:
            self.reset_position_values += self.latest_data["position"]
            self._delta_rebase = True
            if self.debug:
                print(f"[INFO] Position reset to: {self.reset_position_values}")
    
    def pause_updates(self):
        """
        Pauses getting updates of data from the connected device.
        """
        self.get_updates = False
    
    def resume_updates(self):
        """
        Resumes getting updates of data from the connected device.
        """
        self._delta_rebase = True  # don't replay motion that happened while paused
        self.get_updates = True

    def _kill_process_using_port(self, port):
        """
        Kill the process using the given port on Unix-based systems.
        """
        if platform.system() != "Windows":
            try:
                command = f"lsof -t -i:{port}"
                pid = subprocess.check_output(command, shell=True).strip().decode()
                if pid:
                    os.kill(int(pid), signal.SIGKILL)
                    if self.debug:
                        print(f"[INFO] Killed process with PID {pid} using port {port}")
                else:
                    if self.debug:
                        print(f"[INFO] No process found using port {port}")
            except subprocess.CalledProcessError as e:
                if self.debug:
                    print(f"[ERROR] Failed to kill process using port {port}: {e}")
        else:
            try:
                command = f"netstat -ano | findstr :{port}"
                output = subprocess.check_output(command, shell=True).decode()
                lines = output.strip().splitlines()
                if lines:
                    pid = lines[0].strip().split()[-1]
                    os.system(f'taskkill /PID {pid} /F')
                    if self.debug:
                        print(f"[INFO] Killed process with PID {pid} using port {port}")
                else:
                    if self.debug:
                        print(f"[INFO] No process found using port {port}")
            except subprocess.CalledProcessError as e:
                if self.debug:
                    print(f"[ERROR] Failed to kill process using port {port}: {e}")

    def _process_data(self, data):
        """
        Process incoming data and update latest_data.
        """
        
        if 'rotation' in data and 'position' in data:

            rotation = np.array(data['rotation'])
            position = np.array(data['position'])
            self.latest_data["rotation"] = rotation.T
            self.latest_data["position"] = np.array([position[0], position[1], position[2]]).astype(float)
            
            if self.reset_position_values is not None and self.latest_data["position"].dtype == self.reset_position_values.dtype:
                self.latest_data["position"] -= self.reset_position_values

            if self.position_limits is not None:
                self.latest_data["position"][0] = np.clip(
                    self.latest_data["position"][0],
                    self.position_limits[0][0],
                    self.position_limits[0][1]
                )
                self.latest_data["position"][1] = np.clip(
                    self.latest_data["position"][1],
                    self.position_limits[1][0],
                    self.position_limits[1][1]
                )   
                self.latest_data["position"][2] = np.clip(
                    self.latest_data["position"][2],
                    self.position_limits[2][0],
                    self.position_limits[2][1]
                )

            self.latest_data["button"] = data.get('button', False)
            self.latest_data["toggle"] = data.get('toggle', False)

            if self.delta:
                self._track_delta(self.latest_data["position"], self.latest_data["rotation"])
        
            # Compute hand pose
            T_phone_hand = np.eye(4)
            T_phone_hand[:3, :3] = R.from_euler('z', 180, degrees=True).as_matrix()
            T_phone_hand[:3, 3] = np.array([0.0, 0.0, self.hand_offset])
            
            T_world_phone = np.eye(4)
            T_world_phone[:3, :3] = self.latest_data["rotation"].reshape(3, 3)
            T_world_phone[:3, 3] = self.latest_data["position"]
            
            T_world_hand = T_world_phone @ T_phone_hand
            
            self.latest_data["position_hand"] = T_world_hand[:3, 3]
            self.latest_data["rotation_hand"] = T_world_hand[:3, :3].flatten()

        if 'landmarks' in data:
            self.latest_data["landmarks"] = np.array(data["landmarks"])
        if 'world_landmarks' in data:
            self.latest_data["world_landmarks"] = np.array(data["world_landmarks"])

    def _track_delta(self, position, rotation):
        """
        Record the latest pose for get_delta(), excluding discontinuities such as the
        app's Reset button, so they never reach a relative controller as motion.
        """
        position = np.array(position, dtype=float)
        rotation = np.array(rotation, dtype=float).reshape(3, 3)

        with self._delta_lock:
            if self._delta_anchor is None:
                self._delta_anchor = (position.copy(), rotation.copy())
                self._delta_latest = (position, rotation)
                self._delta_rebase = False
                return

            prev_position, prev_rotation = self._delta_latest
            jump_angle = np.linalg.norm(R.from_matrix(rotation @ prev_rotation.T).as_rotvec())
            is_jump = (np.linalg.norm(position - prev_position) > self.delta_jump_position
                       or jump_angle > self.delta_jump_rotation)

            if self._delta_rebase or is_jump:
                # Shift the anchor so the pending (unreported) motion is kept but this step is dropped
                anchor_position, anchor_rotation = self._delta_anchor
                self._delta_anchor = (
                    anchor_position + (position - prev_position),
                    anchor_rotation @ prev_rotation.T @ rotation,
                )
                self._delta_rebase = False
                if self.debug and is_jump:
                    print("[INFO] Pose discontinuity ignored for deltas")

            self._delta_latest = (position, rotation)

    def get_delta(self, max_dpos=None, max_drot=None):
        """
        Get the pose change since the previous call (requires Session(delta=True)).

        Call once per control step. Motion is never lost: if the phone moved more than the
        limits allow, the remainder is returned on the following calls.

        Args:
            max_dpos (float): Optional cap on the translation returned per call, in meters.
            max_drot (float): Optional cap on the rotation returned per call, in radians.

        Returns:
            dict: "dpos" (3,) translation change, "drot" (3, 3) rotation change and
                  "drot_axis_angle" (3,) the same rotation as an axis-angle vector, all in the
                  session frame (x forward, y left, z up), such that rotation = drot @ previous_rotation.
                  Also includes the current "button" and "toggle" states.
        """
        if not self.delta:
            raise RuntimeError("Delta tracking is off. Create the session with Session(delta=True).")

        dpos = np.zeros(3)
        drot = np.eye(3)
        rotvec = np.zeros(3)

        with self._delta_lock:
            if self._delta_anchor is not None:
                anchor_position, anchor_rotation = self._delta_anchor
                latest_position, latest_rotation = self._delta_latest

                dpos = latest_position - anchor_position
                distance = np.linalg.norm(dpos)
                if max_dpos is not None and distance > max_dpos:
                    dpos = dpos * (max_dpos / distance)

                rotvec = R.from_matrix(latest_rotation @ anchor_rotation.T).as_rotvec()
                angle = np.linalg.norm(rotvec)
                if max_drot is not None and angle > max_drot:
                    rotvec = rotvec * (max_drot / angle)
                drot = R.from_rotvec(rotvec).as_matrix()

                self._delta_anchor = (anchor_position + dpos, drot @ anchor_rotation)

        return {
            "dpos": dpos,
            "drot": drot,
            "drot_axis_angle": rotvec,
            "button": self.latest_data["button"],
            "toggle": self.latest_data["toggle"],
        }

    def reset_delta(self):
        """
        Discard any motion not yet returned by get_delta(), e.g. when your own clutch
        disengages or the robot is reset.
        """
        with self._delta_lock:
            if self._delta_latest is not None:
                position, rotation = self._delta_latest
                self._delta_anchor = (position.copy(), rotation.copy())

    def _call_handlers(self):
        """
        Call all registered handlers and callbacks.
        """
        data_copy = self.latest_data.copy()
        
        # Call handlers
        for handler in self._handlers:
            should_vibrate = handler.update(self, data_copy)
            if should_vibrate:
                self.vibrate(duration=0.01, intensity=1.0, sharpness=0.5)
        
        # Call update callbacks
        for callback in self._on_update_callbacks:
            callback(self, data_copy)
            
    async def _handle_connection(self, websocket):
        """
        Handle incoming connections and messages from the application.
        """
        print("[INFO] Device connected successfully!")
        self.connected_clients.add(websocket)
        self._delta_rebase = True  # a (re)connecting phone may start from a different pose
        
        # Call connect callbacks
        for callback in self._on_connect_callbacks:
            callback(self)
        
        try:
            async for message in websocket:
                if self.get_updates:
                    data = json.loads(message)
                    self._process_data(data)
                    self._call_handlers()
                    
                    if self.debug:
                        print(f"[DATA] Rotation: {self.latest_data['rotation']}, Position: {self.latest_data['position']}, Button: {self.latest_data['button']}, Toggle: {self.latest_data['toggle']}")
        except websockets.ConnectionClosed as e:
            print(f"[INFO] Application disconnected: {e}")
        finally:
            self.connected_clients.remove(websocket)
            # Call disconnect callbacks
            for callback in self._on_disconnect_callbacks:
                callback(self)

    async def _start(self):
        """
        Start the connector.
        """
        attempt = 0
        max_attempts = 10
        while attempt < max_attempts:
            self._kill_process_using_port(self.port)
            await asyncio.sleep(0.1)
            try:
                print(f"[INFO] Session Starting on port {self.port}...")
                self.server = await websockets.serve(self._handle_connection, "0.0.0.0", self.port, ping_interval=self.ping_interval, ping_timeout=self.ping_timeout)
                print(f"[INFO] Session Started. Connect using: {self.ip_address}:{self.port}")
                print(f"[INFO] Waiting for a device to connect...")
                if self.show_qr:
                    self._print_qr_code()
                break
            except OSError as e:
                print(f"[WARNING] Port {self.port} is in use. Trying next port.")
                self.port += 1
                attempt += 1
        else:
            raise RuntimeError("Failed to start server on any port. Exceeded maximum attempts.")

        await self.server.wait_closed()
        
    def start(self):
        """
        Start the Session.
        """
        self.loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self._thread.start()

    def _run_event_loop(self):
        """
        Internal: run the asyncio event loop until stopped.
        """
        asyncio.set_event_loop(self.loop)
        self.loop.create_task(self._start())
        self.loop.run_forever()
        self.loop.close()

    def add_position_limit(self, position_limits):
        """
        Set the position limits for the latest_data['position'].

        Args:
            position_limits (list or numpy array): An array with shape (3, 2), where each row corresponds to
                                                   [min, max] limits for x, y, z positions.
        """
        self.position_limits = np.array(position_limits)

    def vibrate(self, duration=0.1, intensity=1.0, sharpness=1.0):
        """
        Vibrate the connected device.

        Args:
            duration (float): The duration of the vibration in seconds.
            intensity (float): The intensity of the vibration.
            sharpness (float): The sharpness of the vibration.
        """
        asyncio.run_coroutine_threadsafe(self._vibrate(sharpness, intensity, duration), self.loop)

    async def _vibrate(self, sharpness, intensity, duration):
        """
        Send a vibration command to all connected clients.
        """
        if self.connected_clients:
            message = json.dumps({"vibrate": True, "duration": duration, "intensity": intensity, "sharpness": sharpness})
            for client in self.connected_clients:
                await client.send(message)
                if self.debug:
                    print(f"[INFO] Sent vibration command to client: {message}")
        else:
            if self.debug:
                print("[INFO] No connected clients to send the vibration command.")

    def get_latest_data(self):
        """
        Get the latest received data.

        Returns:
            dict: The latest rotation, position, and other data.
        """
        return self.latest_data
